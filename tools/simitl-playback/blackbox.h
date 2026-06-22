#pragma once

/**
 * \file blackbox.h
 * \brief Header-only CSV parser for Betaflight Blackbox logs.
 *
 * Parses Betaflight blackbox CSV files (e.g. btfl_079.bbl.csv) containing
 * header metadata followed by data rows.  The \c debug[0-2] columns contain
 * raw/unfiltered gyro values (already in deg/s) used as the reference for
 * physics tuning.  The \c gyroADC[0-2] columns contain filtered gyro values
 * (also in deg/s).
 *
 * The simulation outputs angular velocity in rad/s, which must be converted
 * to deg/s (multiply by 180/pi) before comparing with blackbox values.
 *
 * Usage:
 *   BlackboxData bb = readBlackboxFile("btfl_079.bbl.csv");
 *   for (const auto& frame : bb.frames) {
 *     float rawGyroX_degs = frame.debug[0];  // already in deg/s
 *     float gyroX_degs    = frame.gyroADC[0]; // already in deg/s
 *     ...
 *   }
 */

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <unordered_map>

// ============================================================================
// Data structures
// ============================================================================

/// Parsed header metadata from a Betaflight blackbox CSV.
struct BbHeader {
  // -- Firmware info --------------------------------------------------------
  int         firmwareType        = 0;
  std::string firmware;             // e.g. "4.2"
  int         firmwarePatch        = 0;
  std::string firmwareVersion;      // e.g. "4.2.0"
  std::string firmwareRevision;     // e.g. "Betaflight 4.2.0 (8f2d21460) STM32F405"
  std::string firmwareDate;
  std::string boardInfo;            // e.g. "AIRB OMNIBUSF4"
  std::string craftName;            // e.g. "vtx-slayer"

  // -- Timing ---------------------------------------------------------------
  int    frameIntervalI        = 128;   // base loop time in μs (gyro loop)
  int    frameIntervalPNum     = 1;     // numerator for PID log decimation
  int    frameIntervalPDenom   = 8;     // denominator for PID log decimation
  int    looptime              = 125;   // gyro loop time in μs
  int    pidProcessDenom       = 2;      // PID runs every N gyro loops
  int    debugMode             = 6;      // DEBUG_GYRO_RAW

  // -- Motor / throttle -----------------------------------------------------
  int motorOutputMin     = 148;
  int motorOutputMax     = 2047;

  // -- Gyro / sensor --------------------------------------------------------
  double gyroScale       = 1.7453292519943295e-8;  // from header
  int    acc1G           = 2048;

  // -- Battery --------------------------------------------------------------
  int vbatscale              = 110;
  int vbatmincellvoltage     = 330;
  int vbatwarningcellvoltage = 350;
  int vbatmaxcellvoltage     = 430;
  int vbatref                = 1594;
  int currentMeterOffset     = 0;
  int currentMeterScale      = 400;

  // -- PID / rates ----------------------------------------------------------
  std::string rc_rates;      // "20,20,20"
  std::string rc_expo;       // "0,0,0"
  std::string rates;         // "50,50,50"
  std::string rate_limits;   // "1998,1998,1998"
  std::string rollPID;       // "35,59,32,21"
  std::string pitchPID;      // "39,63,35,23"
  std::string yawPID;        // "38,63,0,0"

  // -- Derived / computed ---------------------------------------------------

  /// Conversion factor: decoded CSV gyro value (deg/s) → radians per second.
  /// For the decoded .bbl.csv format, gyroScale = π/180 / 1,000,000, so the
  /// correct factor is gyroScale * 1,000,000 = π/180 = 0.0174533 rad/s per deg/s.
  /// NOTE: this field is NOT used by the playback code — the decoded CSV values
  /// are already in deg/s and are written directly into the output CSV.
  float gyroRawToRadps = 0.0174533f;

  /// Blackbox log rate in Hz (derived from frameIntervalI and decimation).
  float logRateHz = 500.0f;

  /// The effective time between log rows in seconds.
  float rowIntervalSec = 0.002f;  // ~2000μs = 500Hz
};

/// A single parsed data frame from the blackbox.
struct BbFrame {
  uint64_t loopIteration = 0;
  uint64_t time          = 0;   ///< microseconds (field from CSV)

  // -- RC inputs (what we feed into the sim) --------------------------------
  int16_t rcCommand[4] = {};     ///< [roll, pitch, yaw, throttle]
                                 ///  roll/pitch/yaw: ±500, throttle: 1000-2000

  // -- Reference outputs (what we compare against) --------------------------
  int16_t debug[4] = {};         ///< [0-2] = raw/unfiltered gyro (deg/s)
                                 ///  [3] = gyro calibration stddev

  // -- Secondary / validation data ------------------------------------------
  int16_t gyroADC[4]    = {};    ///< filtered gyro (deg/s)
  int16_t motor[4]      = {};    ///< DShot/PWM output values (148-2047)
  int16_t setpoint[4]   = {};    ///< FC's setpoint
  int16_t axisP[3]      = {};
  int16_t axisI[3]      = {};
  int16_t axisD[3]      = {};
  int16_t axisF[3]      = {};
  uint16_t vbatLatest   = 0;
  int16_t amperageLatest = 0;
  int16_t rssi          = 0;
  int16_t accSmooth[3]  = {};

  // -- Flags ----------------------------------------------------------------
  uint32_t flightModeFlags = 0;   ///< Bit 0 = ARM flag
  uint32_t stateFlags      = 0;   ///< Bit 0 = ARM flag (fallback)
  uint8_t  failsafePhase    = 0;
  uint8_t  rxSignalReceived = 0;
  uint8_t  rxFlightChannelsValid = 0;

  // -- Additional fields ----------------------------------------------------
  float    heading[3]     = {};   ///< estimated attitude (may be "" in CSV)
  int16_t  axisSum[3]     = {};
  int16_t  rcCommands[4]  = {};   ///< raw RC commands after RX
  int16_t  axisError[3]   = {};
  int16_t  motorLegacy[4] = {};

  /// Convenience: is the FC armed in this frame?
  bool isArmed() const {
    return (flightModeFlags & 1) != 0 || (stateFlags & 1) != 0;
  }
};

/// Top-level blackbox data.
struct BlackboxData {
  BbHeader            header;
  std::vector<BbFrame> frames;
};

// ============================================================================
// CSV Parser
// ============================================================================

namespace bb_detail {

/// Trim whitespace from both ends of a string.
inline std::string trim(const std::string& s) {
  size_t start = 0;
  while (start < s.size() && (s[start] == ' ' || s[start] == '\t'))
    start++;
  size_t end = s.size();
  while (end > start && (s[end-1] == ' ' || s[end-1] == '\t' || s[end-1] == '\r'))
    end--;
  return s.substr(start, end - start);
}

/// Parse a CSV line into fields, handling quoted strings.
/// Supports double-quoted fields with escaped quotes ("").
inline std::vector<std::string> parseCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  fields.reserve(60);

  size_t i = 0;
  while (i < line.size()) {
    // Skip whitespace before field
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t'))
      i++;

    if (i >= line.size()) {
      fields.push_back("");
      break;
    }

    if (line[i] == '"') {
      // Quoted field
      i++; // skip opening quote
      std::string field;
      while (i < line.size()) {
        if (line[i] == '"') {
          // Check for escaped quote ""
          if (i + 1 < line.size() && line[i + 1] == '"') {
            field += '"';
            i += 2;
          } else {
            // End of quoted field
            i++; // skip closing quote
            break;
          }
        } else {
          field += line[i];
          i++;
        }
      }
      fields.push_back(field);
      // Skip to next comma
      while (i < line.size() && line[i] != ',')
        i++;
      if (i < line.size() && line[i] == ',')
        i++;
    } else {
      // Unquoted field: read until comma or end
      size_t start = i;
      while (i < line.size() && line[i] != ',')
        i++;
      fields.push_back(line.substr(start, i - start));
      if (i < line.size() && line[i] == ',')
        i++;
    }
  }

  return fields;
}

/// Parse a numeric value from a CSV field string.
/// Returns 0 for empty/quoted-empty fields ("").
inline int parseIntField(const std::string& s) {
  std::string t = trim(s);
  if (t.empty() || t == "\"\"")
    return 0;
  // Remove surrounding quotes if present
  if (t.size() >= 2 && t.front() == '"' && t.back() == '"')
    t = t.substr(1, t.size() - 2);
  if (t.empty())
    return 0;
  return static_cast<int>(std::strtol(t.c_str(), nullptr, 10));
}

inline float parseFloatField(const std::string& s) {
  std::string t = trim(s);
  if (t.empty() || t == "\"\"")
    return 0.0f;
  if (t.size() >= 2 && t.front() == '"' && t.back() == '"')
    t = t.substr(1, t.size() - 2);
  if (t.empty())
    return 0.0f;
  return static_cast<float>(std::strtod(t.c_str(), nullptr));
}

inline uint32_t parseUintField(const std::string& s) {
  std::string t = trim(s);
  if (t.empty() || t == "\"\"")
    return 0;
  if (t.size() >= 2 && t.front() == '"' && t.back() == '"')
    t = t.substr(1, t.size() - 2);
  if (t.empty())
    return 0;
  return static_cast<uint32_t>(std::strtoul(t.c_str(), nullptr, 10));
}

/// Column index mapping entry.
struct ColMapEntry {
  int  index;      ///< column index in CSV
  bool valid;      ///< whether this column was found
};

/// Column name mapping for the data rows.
struct ColumnMap {
  ColMapEntry loopIteration   = {-1, false};
  ColMapEntry time            = {-1, false};
  ColMapEntry rcCommand[4]    = {{-1, false}, {-1, false}, {-1, false}, {-1, false}};
  ColMapEntry debug[4]        = {{-1, false}, {-1, false}, {-1, false}, {-1, false}};
  ColMapEntry gyroADC[3]      = {{-1, false}, {-1, false}, {-1, false}};
  ColMapEntry motor[4]        = {{-1, false}, {-1, false}, {-1, false}, {-1, false}};
  ColMapEntry setpoint[4]     = {{-1, false}, {-1, false}, {-1, false}, {-1, false}};
  ColMapEntry axisP[3]        = {{-1, false}, {-1, false}, {-1, false}};
  ColMapEntry axisI[3]        = {{-1, false}, {-1, false}, {-1, false}};
  ColMapEntry axisD[3]        = {{-1, false}, {-1, false}, {-1, false}};
  ColMapEntry axisF[3]        = {{-1, false}, {-1, false}, {-1, false}};
  ColMapEntry vbatLatest      = {-1, false};
  ColMapEntry amperageLatest  = {-1, false};
  ColMapEntry rssi            = {-1, false};
  ColMapEntry accSmooth[3]    = {{-1, false}, {-1, false}, {-1, false}};
  ColMapEntry flightModeFlags = {-1, false};
  ColMapEntry stateFlags      = {-1, false};
  ColMapEntry failsafePhase   = {-1, false};
  ColMapEntry rxSignalReceived = {-1, false};
  ColMapEntry rxFlightChannelsValid = {-1, false};
  ColMapEntry heading[3]      = {{-1, false}, {-1, false}, {-1, false}};
  ColMapEntry axisSum[3]      = {{-1, false}, {-1, false}, {-1, false}};
  ColMapEntry rcCommands[4]   = {{-1, false}, {-1, false}, {-1, false}, {-1, false}};
  ColMapEntry axisError[3]    = {{-1, false}, {-1, false}, {-1, false}};
  ColMapEntry motorLegacy[4]  = {{-1, false}, {-1, false}, {-1, false}, {-1, false}};
};

/// Build column index map from the CSV column header line.
inline ColumnMap buildColumnMap(const std::vector<std::string>& colHeaders) {
  ColumnMap cm;

  auto setIdx = [&](const std::string& name, int idx) {
    if      (name == "loopIteration")  cm.loopIteration = {idx, true};
    else if (name == "time")           cm.time          = {idx, true};
    else if (name == "rcCommand[0]")   cm.rcCommand[0]  = {idx, true};
    else if (name == "rcCommand[1]")   cm.rcCommand[1]  = {idx, true};
    else if (name == "rcCommand[2]")   cm.rcCommand[2]  = {idx, true};
    else if (name == "rcCommand[3]")   cm.rcCommand[3]  = {idx, true};
    else if (name == "debug[0]")       cm.debug[0]      = {idx, true};
    else if (name == "debug[1]")       cm.debug[1]      = {idx, true};
    else if (name == "debug[2]")       cm.debug[2]      = {idx, true};
    else if (name == "debug[3]")       cm.debug[3]      = {idx, true};
    else if (name == "gyroADC[0]")     cm.gyroADC[0]    = {idx, true};
    else if (name == "gyroADC[1]")     cm.gyroADC[1]    = {idx, true};
    else if (name == "gyroADC[2]")     cm.gyroADC[2]    = {idx, true};
    else if (name == "motor[0]")       cm.motor[0]      = {idx, true};
    else if (name == "motor[1]")       cm.motor[1]      = {idx, true};
    else if (name == "motor[2]")       cm.motor[2]      = {idx, true};
    else if (name == "motor[3]")       cm.motor[3]      = {idx, true};
    else if (name == "setpoint[0]")    cm.setpoint[0]   = {idx, true};
    else if (name == "setpoint[1]")    cm.setpoint[1]   = {idx, true};
    else if (name == "setpoint[2]")    cm.setpoint[2]   = {idx, true};
    else if (name == "setpoint[3]")    cm.setpoint[3]   = {idx, true};
    else if (name == "axisP[0]")       cm.axisP[0]      = {idx, true};
    else if (name == "axisP[1]")       cm.axisP[1]      = {idx, true};
    else if (name == "axisP[2]")       cm.axisP[2]      = {idx, true};
    else if (name == "axisI[0]")       cm.axisI[0]      = {idx, true};
    else if (name == "axisI[1]")       cm.axisI[1]      = {idx, true};
    else if (name == "axisI[2]")       cm.axisI[2]      = {idx, true};
    else if (name == "axisD[0]")       cm.axisD[0]      = {idx, true};
    else if (name == "axisD[1]")       cm.axisD[1]      = {idx, true};
    else if (name == "axisD[2]")       cm.axisD[2]      = {idx, true};
    else if (name == "axisF[0]")       cm.axisF[0]      = {idx, true};
    else if (name == "axisF[1]")       cm.axisF[1]      = {idx, true};
    else if (name == "axisF[2]")       cm.axisF[2]      = {idx, true};
    else if (name == "vbatLatest")     cm.vbatLatest    = {idx, true};
    else if (name == "amperageLatest") cm.amperageLatest = {idx, true};
    else if (name == "rssi")           cm.rssi          = {idx, true};
    else if (name == "accSmooth[0]")   cm.accSmooth[0]  = {idx, true};
    else if (name == "accSmooth[1]")   cm.accSmooth[1]  = {idx, true};
    else if (name == "accSmooth[2]")   cm.accSmooth[2]  = {idx, true};
    else if (name == "flightModeFlags") cm.flightModeFlags = {idx, true};
    else if (name == "stateFlags")     cm.stateFlags    = {idx, true};
    else if (name == "failsafePhase")  cm.failsafePhase = {idx, true};
    else if (name == "rxSignalReceived") cm.rxSignalReceived = {idx, true};
    else if (name == "rxFlightChannelsValid") cm.rxFlightChannelsValid = {idx, true};
    else if (name == "heading[0]")     cm.heading[0]    = {idx, true};
    else if (name == "heading[1]")     cm.heading[1]    = {idx, true};
    else if (name == "heading[2]")     cm.heading[2]    = {idx, true};
    else if (name == "axisSum[0]")     cm.axisSum[0]    = {idx, true};
    else if (name == "axisSum[1]")     cm.axisSum[1]    = {idx, true};
    else if (name == "axisSum[2]")     cm.axisSum[2]    = {idx, true};
    else if (name == "rcCommands[0]")  cm.rcCommands[0] = {idx, true};
    else if (name == "rcCommands[1]")  cm.rcCommands[1] = {idx, true};
    else if (name == "rcCommands[2]")  cm.rcCommands[2] = {idx, true};
    else if (name == "rcCommands[3]")  cm.rcCommands[3] = {idx, true};
    else if (name == "axisError[0]")   cm.axisError[0]  = {idx, true};
    else if (name == "axisError[1]")   cm.axisError[1]  = {idx, true};
    else if (name == "axisError[2]")   cm.axisError[2]  = {idx, true};
    else if (name == "motorLegacy[0]") cm.motorLegacy[0]= {idx, true};
    else if (name == "motorLegacy[1]") cm.motorLegacy[1]= {idx, true};
    else if (name == "motorLegacy[2]") cm.motorLegacy[2]= {idx, true};
    else if (name == "motorLegacy[3]") cm.motorLegacy[3]= {idx, true};
  };

  for (size_t i = 0; i < colHeaders.size(); i++) {
    std::string name = trim(colHeaders[i]);
    // Remove surrounding quotes if present
    if (name.size() >= 2 && name.front() == '"' && name.back() == '"')
      name = name.substr(1, name.size() - 2);
    setIdx(name, static_cast<int>(i));
  }

  return cm;
}

/// Parse a single data row into a BbFrame using the column map.
inline BbFrame parseFrame(const std::vector<std::string>& fields,
                          const ColumnMap& cm)
{
  BbFrame f;

  auto getInt = [&](const ColMapEntry& e) -> int16_t {
    if (!e.valid || e.index < 0 || e.index >= (int)fields.size())
      return 0;
    return static_cast<int16_t>(parseIntField(fields[e.index]));
  };

  auto getU32 = [&](const ColMapEntry& e) -> uint32_t {
    if (!e.valid || e.index < 0 || e.index >= (int)fields.size())
      return 0;
    return parseUintField(fields[e.index]);
  };

  auto getU16 = [&](const ColMapEntry& e) -> uint16_t {
    if (!e.valid || e.index < 0 || e.index >= (int)fields.size())
      return 0;
    return static_cast<uint16_t>(parseUintField(fields[e.index]));
  };

  auto getFloat = [&](const ColMapEntry& e) -> float {
    if (!e.valid || e.index < 0 || e.index >= (int)fields.size())
      return 0.0f;
    return parseFloatField(fields[e.index]);
  };

  auto getU8 = [&](const ColMapEntry& e) -> uint8_t {
    if (!e.valid || e.index < 0 || e.index >= (int)fields.size())
      return 0;
    return static_cast<uint8_t>(parseUintField(fields[e.index]));
  };

  if (cm.loopIteration.valid)
    f.loopIteration = static_cast<uint64_t>(getU32(cm.loopIteration));
  if (cm.time.valid)
    f.time = static_cast<uint64_t>(getU32(cm.time));

  for (int i = 0; i < 4; i++) f.rcCommand[i] = getInt(cm.rcCommand[i]);
  for (int i = 0; i < 4; i++) f.debug[i]     = getInt(cm.debug[i]);
  for (int i = 0; i < 3; i++) f.gyroADC[i]   = getInt(cm.gyroADC[i]);
  for (int i = 0; i < 4; i++) f.motor[i]     = getInt(cm.motor[i]);
  for (int i = 0; i < 4; i++) f.setpoint[i]  = getInt(cm.setpoint[i]);
  for (int i = 0; i < 3; i++) f.axisP[i]     = getInt(cm.axisP[i]);
  for (int i = 0; i < 3; i++) f.axisI[i]     = getInt(cm.axisI[i]);
  for (int i = 0; i < 3; i++) f.axisD[i]     = getInt(cm.axisD[i]);
  for (int i = 0; i < 3; i++) f.axisF[i]     = getInt(cm.axisF[i]);

  f.vbatLatest      = getU16(cm.vbatLatest);
  f.amperageLatest   = getInt(cm.amperageLatest);
  f.rssi            = getInt(cm.rssi);

  for (int i = 0; i < 3; i++) f.accSmooth[i] = getInt(cm.accSmooth[i]);

  f.flightModeFlags = getU32(cm.flightModeFlags);
  f.stateFlags      = getU32(cm.stateFlags);
  f.failsafePhase    = getU8(cm.failsafePhase);
  f.rxSignalReceived = getU8(cm.rxSignalReceived);
  f.rxFlightChannelsValid = getU8(cm.rxFlightChannelsValid);

  for (int i = 0; i < 3; i++) f.heading[i]   = getFloat(cm.heading[i]);
  for (int i = 0; i < 3; i++) f.axisSum[i]   = getInt(cm.axisSum[i]);
  for (int i = 0; i < 4; i++) f.rcCommands[i] = getInt(cm.rcCommands[i]);
  for (int i = 0; i < 3; i++) f.axisError[i]  = getInt(cm.axisError[i]);
  for (int i = 0; i < 4; i++) f.motorLegacy[i] = getInt(cm.motorLegacy[i]);

  return f;
}

/// Parse a header key-value line like "key","value"
inline bool parseHeaderLine(const std::string& line,
                            BbHeader& hdr,
                            std::unordered_map<std::string, std::string>& rawHeaders)
{
  auto fields = parseCsvLine(line);
  if (fields.size() < 2)
    return false;

  std::string key = trim(fields[0]);
  std::string val = trim(fields[1]);

  // Remove surrounding quotes
  if (key.size() >= 2 && key.front() == '"' && key.back() == '"')
    key = key.substr(1, key.size() - 2);
  if (val.size() >= 2 && val.front() == '"' && val.back() == '"')
    val = val.substr(1, val.size() - 2);

  rawHeaders[key] = val;

  // Parse known fields
  if      (key == "firmwareType")        hdr.firmwareType = std::atoi(val.c_str());
  else if (key == "firmware")            hdr.firmware = val;
  else if (key == "firmwarePatch")       hdr.firmwarePatch = std::atoi(val.c_str());
  else if (key == "firmwareVersion")     hdr.firmwareVersion = val;
  else if (key == "Firmware revision")   hdr.firmwareRevision = val;
  else if (key == "Firmware date")       hdr.firmwareDate = val;
  else if (key == "Board information")   hdr.boardInfo = val;
  else if (key == "Craft name")          hdr.craftName = val;
  else if (key == "frameIntervalI")      hdr.frameIntervalI = std::atoi(val.c_str());
  else if (key == "frameIntervalPNum")   hdr.frameIntervalPNum = std::atoi(val.c_str());
  else if (key == "frameIntervalPDenom") hdr.frameIntervalPDenom = std::atoi(val.c_str());
  else if (key == "motorOutput") {
    // Parse "min,max"
    auto comma = val.find(',');
    if (comma != std::string::npos) {
      hdr.motorOutputMin = std::atoi(val.substr(0, comma).c_str());
      hdr.motorOutputMax = std::atoi(val.substr(comma + 1).c_str());
    }
  }
  else if (key == "gyroScale")          hdr.gyroScale = std::strtod(val.c_str(), nullptr);
  else if (key == "acc_1G")             hdr.acc1G = std::atoi(val.c_str());
  else if (key == "looptime")           hdr.looptime = std::atoi(val.c_str());
  else if (key == "pid_process_denom")  hdr.pidProcessDenom = std::atoi(val.c_str());
  else if (key == "debug_mode")         hdr.debugMode = std::atoi(val.c_str());
  else if (key == "rc_rates")           hdr.rc_rates = val;
  else if (key == "rc_expo")            hdr.rc_expo = val;
  else if (key == "rates")              hdr.rates = val;
  else if (key == "rate_limits")        hdr.rate_limits = val;
  else if (key == "rollPID")            hdr.rollPID = val;
  else if (key == "pitchPID")           hdr.pitchPID = val;
  else if (key == "yawPID")             hdr.yawPID = val;
  else if (key == "vbatscale")          hdr.vbatscale = std::atoi(val.c_str());
  else if (key == "vbatmincellvoltage") hdr.vbatmincellvoltage = std::atoi(val.c_str());
  else if (key == "vbatwarningcellvoltage") hdr.vbatwarningcellvoltage = std::atoi(val.c_str());
  else if (key == "vbatmaxcellvoltage") hdr.vbatmaxcellvoltage = std::atoi(val.c_str());
  else if (key == "vbatref")            hdr.vbatref = std::atoi(val.c_str());
  else if (key == "currentMeterOffset") hdr.currentMeterOffset = std::atoi(val.c_str());
  else if (key == "currentMeterScale")  hdr.currentMeterScale = std::atoi(val.c_str());

  return true;
}

} // namespace bb_detail

// ============================================================================
// Public API
// ============================================================================

/**
 * \brief Read and parse a Betaflight blackbox CSV file.
 *
 * \param path  Path to the .bbl.csv file.
 * \return      Fully parsed BlackboxData with header + frames.
 * \throws std::runtime_error on I/O or parse errors.
 */
inline BlackboxData readBlackboxFile(const std::string& path) {
  namespace bbd = bb_detail;

  // Read entire file into memory
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file)
    throw std::runtime_error("Cannot open blackbox file: " + path);

  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::string buffer(static_cast<size_t>(size), '\0');
  if (!file.read(buffer.data(), size))
    throw std::runtime_error("Failed to read blackbox file: " + path);

  BlackboxData result;
  BbHeader& hdr = result.header;
  std::unordered_map<std::string, std::string> rawHeaders;

  // Temporary storage: lines
  std::vector<std::string> lines;
  lines.reserve(30000);

  // Split into lines
  size_t pos = 0;
  while (pos < buffer.size()) {
    size_t end = buffer.find('\n', pos);
    if (end == std::string::npos)
      end = buffer.size();
    // Skip trailing \r
    size_t lineEnd = end;
    if (lineEnd > pos && buffer[lineEnd - 1] == '\r')
      lineEnd--;
    lines.push_back(buffer.substr(pos, lineEnd - pos));
    pos = end + 1;
  }

  // Find the column header line (starts with "loopIteration")
  size_t colHeaderLine = 0;
  for (size_t i = 0; i < lines.size(); i++) {
    // Look for the start of the column header
    if (lines[i].find("loopIteration") != std::string::npos &&
        lines[i].find("rcCommand[0]") != std::string::npos)
    {
      colHeaderLine = i;
      break;
    }
    // Otherwise, try to parse as header key-value
    bbd::parseHeaderLine(lines[i], hdr, rawHeaders);
  }

  if (colHeaderLine == 0 && lines.size() > 0) {
    // Fallback: try line by line
    // Just proceed and try to detect
    for (size_t i = 0; i < lines.size(); i++) {
      if (lines[i].find("\"loopIteration\"") != std::string::npos) {
        colHeaderLine = i;
        break;
      }
    }
  }

  if (colHeaderLine == 0) {
    throw std::runtime_error("Could not find column header line in blackbox file");
  }

  // Parse column headers
  auto colHeaders = bbd::parseCsvLine(lines[colHeaderLine]);
  auto cm = bbd::buildColumnMap(colHeaders);

  // Compute derived header values
  // The decoded .bbl.csv stores gyro values already in deg/s.
  // The gyroScale header field (= π/180 / 1,000,000) converts stored value
  // to rad/s:  value_radps = stored_value * gyroScale * 1,000,000
  // Since gyroScale * 1,000,000 = π/180, stored_value is in deg/s.
  // gyroRawToRadps is not used anywhere — it is kept here for documentation.
  auto gyroIt = rawHeaders.find("gyroScale");
  if (gyroIt != rawHeaders.end() && hdr.gyroScale > 0.0) {
    hdr.gyroRawToRadps = static_cast<float>(hdr.gyroScale * 1000000.0);
  } else {
    // Fallback: standard conversion deg/s → rad/s
    hdr.gyroRawToRadps = 0.0174533f;
  }

  // Log rate: frameIntervalI * frameIntervalPDenom / frameIntervalPNum μs per row
  // = 128 * 8 / 1 = 1024 μs... but actual data shows ~2000 μs spacing
  // Compute from actual data if available, otherwise use 500 Hz default
  hdr.rowIntervalSec = 0.002f;  // ~2000μs as observed from data
  hdr.logRateHz = 500.0f;

  // Parse data rows
  result.frames.reserve(lines.size() - colHeaderLine);
  for (size_t i = colHeaderLine + 1; i < lines.size(); i++) {
    if (lines[i].empty())
      continue;

    auto fields = bbd::parseCsvLine(lines[i]);
    // Need at least ~40 fields for a valid frame
    if (fields.size() < 30)
      continue;

    try {
      BbFrame frame = bbd::parseFrame(fields, cm);
      result.frames.push_back(frame);
    } catch (...) {
      // Skip malformed rows
      continue;
    }
  }

  // Compute actual log rate from first few time values
  if (result.frames.size() >= 2) {
    uint64_t dt = result.frames[1].time - result.frames[0].time;
    if (dt > 0 && dt < 100000) {
      hdr.rowIntervalSec = static_cast<float>(dt) / 1000000.0f;
      hdr.logRateHz = 1.0f / hdr.rowIntervalSec;
    }
  }

  return result;
}
