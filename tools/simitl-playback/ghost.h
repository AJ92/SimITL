#pragma once

/**
 * \file Ghost.h
 * \brief Header-only JSON decoder for SimITL ghost recording files (ghost.json).
 *
 * Provides structured reading of ghost data files containing recorded
 * flight telemetry frames.
 *
 * Usage:
 *   GhostData data = readGhostFile("ghost.json");
 *   for (const auto& frame : data.samples) { ... }
 */

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cstdlib>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstring>

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// 3-component floating point vector (double precision).
struct GhostVec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/// 4-component floating point vector / quaternion (double precision).
struct GhostVec4 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 0.0;
};

/// A single recorded frame (sample) from a ghost recording.
struct GhostFrame {
  double time         = 0.0;   ///< Simulation time in seconds.
  int    ghostEvent   = 0;     ///< Event type identifier.
  double camAngle     = 0.0;   ///< Camera angle in degrees.

  GhostVec4 orientation;        ///< Quaternion orientation (x, y, z, w).
  GhostVec3 position;           ///< World-space position (x, y, z).

  float rcData[8]     = {};    ///< RC channel values in range [-1, 1].
  float motorRpm[4]   = {};    ///< Motor RPM values [m1..m4].
  float propellerDamage[4] = {}; ///< Propeller damage values [p1..p4].
};

/// Top-level ghost recording data.
struct GhostData {
  int trackId = 0;
  int quadId  = 0;
  std::vector<GhostFrame> samples;
};

// ---------------------------------------------------------------------------
// Parser implementation (detail namespace)
// ---------------------------------------------------------------------------

namespace detail {

// ---------------------------------------------------------------------------
// JSON tokeniser and parser – minimal, correct recursive-descent parser.
// ---------------------------------------------------------------------------

class JsonParser {
public:
  explicit JsonParser(const std::string& input)
    : m_input(input), m_pos(0) {}

  /// Parse the top-level GhostData object.
  GhostData parseGhost() {
    GhostData data;

    expect('{');

    while (m_pos < m_input.size()) {
      skipWhitespace();
      if (atEnd() || peek() == '}')
        break;

      std::string key = parseString();
      skipWhitespace();
      expect(':');
      skipWhitespace();

      if (key == "trackId") {
        data.trackId = parseAs<int>();
      } else if (key == "quadId") {
        data.quadId = parseAs<int>();
      } else if (key == "samples") {
        data.samples = parseSampleArray();
      } else {
        skipValue();
      }

      skipWhitespace();
      if (peek() == ',')
        consume();
    }

    expect('}');
    return data;
  }

private:
  const std::string& m_input;
  size_t m_pos = 0;

  // -- low-level helpers ---------------------------------------------------

  bool atEnd() const { return m_pos >= m_input.size(); }

  char peek() const {
    return atEnd() ? '\0' : m_input[m_pos];
  }

  char consume() {
    if (atEnd())
      throw std::runtime_error("JSON parser: unexpected end of input");
    return m_input[m_pos++];
  }

  void expect(char c) {
    skipWhitespace();
    char got = consume();
    if (got != c) {
      std::string msg = "JSON parser: expected '";
      msg += c;
      msg += "' but got '";
      msg += got;
      msg += "'";
      throw std::runtime_error(msg);
    }
  }

  void skipWhitespace() {
    while (!atEnd()) {
      char c = m_input[m_pos];
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r')
        ++m_pos;
      else
        break;
    }
  }

  // -- string parsing ------------------------------------------------------

  std::string parseString() {
    skipWhitespace();
    expect('"');

    std::string result;
    result.reserve(32);

    while (!atEnd()) {
      char c = consume();
      if (c == '"')
        break;

      if (c == '\\') {
        if (atEnd()) break;
        char esc = consume();
        switch (esc) {
          case '"':  result += '"';  break;
          case '\\': result += '\\'; break;
          case '/':  result += '/';  break;
          case 'b':  result += '\b'; break;
          case 'f':  result += '\f'; break;
          case 'n':  result += '\n'; break;
          case 'r':  result += '\r'; break;
          case 't':  result += '\t'; break;
          case 'u': {
            // Parse 4-digit hex unicode escape.
            std::string hex;
            for (int i = 0; i < 4 && !atEnd(); ++i)
              hex += consume();
            // Simple pass-through as UTF-8 replacement char.
            result += "\uFFFD";
            break;
          }
          default:
            result += esc;
            break;
        }
      } else {
        result += c;
      }
    }
    return result;
  }

  // -- number parsing ------------------------------------------------------

  /// Parse a JSON number and return as double.
  double parseDouble() {
    skipWhitespace();
    size_t start = m_pos;

    if (peek() == '-')
      consume();

    // Integral part.
    while (!atEnd() && std::isdigit(static_cast<unsigned char>(peek())))
      consume();

    // Fractional part.
    if (peek() == '.') {
      consume();
      while (!atEnd() && std::isdigit(static_cast<unsigned char>(peek())))
        consume();
    }

    // Exponent part.
    if (peek() == 'e' || peek() == 'E') {
      consume();
      if (peek() == '+' || peek() == '-')
        consume();
      while (!atEnd() && std::isdigit(static_cast<unsigned char>(peek())))
        consume();
    }

    if (start == m_pos)
      throw std::runtime_error("JSON parser: expected a number");

    // Copy substring into a null-terminated buffer for strtod.
    std::string numStr(m_input.data() + start, m_pos - start);
    char* end = nullptr;
    double value = std::strtod(numStr.c_str(), &end);
    if (end != numStr.c_str() + numStr.size())
      throw std::runtime_error("JSON parser: failed to parse number: " + numStr);
    return value;
  }

  /// Parse a number and cast to the requested arithmetic type.
  template <typename T>
  T parseAs() {
    return static_cast<T>(parseDouble());
  }

  // -- value skipping ------------------------------------------------------

  /// Skip over any JSON value (for unknown keys).
  void skipValue() {
    skipWhitespace();
    if (atEnd()) return;

    char c = peek();
    if (c == '"') {
      parseString();
    } else if (c == '{') {
      skipBracketed('{', '}');
    } else if (c == '[') {
      skipBracketed('[', ']');
    } else if (c == 't') {
      // true
      if (m_input.substr(m_pos, 4) == "true") m_pos += 4;
      else throw std::runtime_error("JSON parser: expected 'true'");
    } else if (c == 'f') {
      // false
      if (m_input.substr(m_pos, 5) == "false") m_pos += 5;
      else throw std::runtime_error("JSON parser: expected 'false'");
    } else if (c == 'n') {
      // null
      if (m_input.substr(m_pos, 4) == "null") m_pos += 4;
      else throw std::runtime_error("JSON parser: expected 'null'");
    } else {
      // number
      parseDouble();
    }
  }

  /// Skip a bracketed group { } or [ ], handling nesting and strings.
  void skipBracketed(char open, char close) {
    expect(open);
    int depth = 1;
    while (depth > 0 && !atEnd()) {
      char c = consume();
      if (c == open)
        ++depth;
      else if (c == close)
        --depth;
      else if (c == '"')
        parseString();  // skip string contents
    }
  }

  // -- structured parsing for ghost.json -----------------------------------

  /// Parse the "samples" array.
  std::vector<GhostFrame> parseSampleArray() {
    skipWhitespace();
    expect('[');

    std::vector<GhostFrame> frames;
    frames.reserve(512);

    while (m_pos < m_input.size()) {
      skipWhitespace();
      if (peek() == ']')
        break;

      frames.push_back(parseSampleObject());

      skipWhitespace();
      if (peek() == ',')
        consume();
    }

    expect(']');
    return frames;
  }

  /// Parse a single sample (frame) object.
  GhostFrame parseSampleObject() {
    GhostFrame frame;
    expect('{');

    while (m_pos < m_input.size()) {
      skipWhitespace();
      if (peek() == '}')
        break;

      std::string key = parseString();
      skipWhitespace();
      expect(':');
      skipWhitespace();

      if (key == "time") {
        frame.time = parseDouble();
      } else if (key == "ghostEvent") {
        frame.ghostEvent = parseAs<int>();
      } else if (key == "camAngle") {
        frame.camAngle = parseDouble();
      } else if (key == "orientation") {
        frame.orientation = parseVec4();
      } else if (key == "position") {
        frame.position = parseVec3();
      } else if (key == "rcData") {
        parseRcData(frame.rcData);
      } else if (key == "motorData") {
        parseMotorData(frame.motorRpm);
      } else if (key == "propellerData") {
        parsePropellerData(frame.propellerDamage);
      } else {
        skipValue();
      }

      skipWhitespace();
      if (peek() == ',')
        consume();
    }

    expect('}');
    return frame;
  }

  /// Parse a { "x": ..., "y": ..., "z": ... } object.
  GhostVec3 parseVec3() {
    GhostVec3 v{};
    expect('{');

    while (m_pos < m_input.size()) {
      skipWhitespace();
      if (peek() == '}')
        break;

      std::string key = parseString();
      skipWhitespace();
      expect(':');
      skipWhitespace();

      if (key == "x")       v.x = parseDouble();
      else if (key == "y")  v.y = parseDouble();
      else if (key == "z")  v.z = parseDouble();
      else                  skipValue();

      skipWhitespace();
      if (peek() == ',')
        consume();
    }

    expect('}');
    return v;
  }

  /// Parse a { "x": ..., "y": ..., "z": ..., "w": ... } object.
  GhostVec4 parseVec4() {
    GhostVec4 v{};
    expect('{');

    while (m_pos < m_input.size()) {
      skipWhitespace();
      if (peek() == '}')
        break;

      std::string key = parseString();
      skipWhitespace();
      expect(':');
      skipWhitespace();

      if (key == "x")       v.x = parseDouble();
      else if (key == "y")  v.y = parseDouble();
      else if (key == "z")  v.z = parseDouble();
      else if (key == "w")  v.w = parseDouble();
      else                  skipValue();

      skipWhitespace();
      if (peek() == ',')
        consume();
    }

    expect('}');
    return v;
  }

  /// Parse { "rc0": .., "rc1": .., ..., "rc7": .. } into an 8-float array.
  void parseRcData(float* out) {
    expect('{');

    while (m_pos < m_input.size()) {
      skipWhitespace();
      if (peek() == '}')
        break;

      std::string key = parseString();
      skipWhitespace();
      expect(':');
      skipWhitespace();

      double val = parseDouble();

      // Extract channel index from key like "rc0", "rc1", ...
      if (key.size() > 2 && key[0] == 'r' && key[1] == 'c') {
        int idx = std::atoi(key.c_str() + 2);
        if (idx >= 0 && idx < 8)
          out[idx] = static_cast<float>(val);
      }

      skipWhitespace();
      if (peek() == ',')
        consume();
    }

    expect('}');
  }

  /// Parse { "m1rpm": .., "m2rpm": .., "m3rpm": .., "m4rpm": .. }.
  void parseMotorData(float* out) {
    expect('{');

    while (m_pos < m_input.size()) {
      skipWhitespace();
      if (peek() == '}')
        break;

      std::string key = parseString();
      skipWhitespace();
      expect(':');
      skipWhitespace();

      double val = parseDouble();

      // "m1rpm" -> index 0, "m2rpm" -> index 1, etc.
      if (key.size() > 4 && key[0] == 'm' && key[2] == 'r') {
        int idx = key[1] - '1';  // '1' -> 0, '2' -> 1, ...
        if (idx >= 0 && idx < 4)
          out[idx] = static_cast<float>(val);
      }

      skipWhitespace();
      if (peek() == ',')
        consume();
    }

    expect('}');
  }

  /// Parse { "p1dmg": .., "p2dmg": .., "p3dmg": .., "p4dmg": .. }.
  void parsePropellerData(float* out) {
    expect('{');

    while (m_pos < m_input.size()) {
      skipWhitespace();
      if (peek() == '}')
        break;

      std::string key = parseString();
      skipWhitespace();
      expect(':');
      skipWhitespace();

      double val = parseDouble();

      // "p1dmg" -> index 0, "p2dmg" -> index 1, etc.
      if (key.size() > 4 && key[0] == 'p' && key[3] == 'd') {
        int idx = key[1] - '1';  // '1' -> 0, '2' -> 1, ...
        if (idx >= 0 && idx < 4)
          out[idx] = static_cast<float>(val);
      }

      skipWhitespace();
      if (peek() == ',')
        consume();
    }

    expect('}');
  }
};

} // namespace detail

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * \brief Read and decode a ghost.json recording file.
 *
 * \param path  Path to the ghost.json file.
 * \return      Fully parsed GhostData containing all frames.
 * \throws std::runtime_error on I/O or parse errors.
 */
inline GhostData readGhostFile(const std::string& path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file)
    throw std::runtime_error("Cannot open file: " + path);

  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  std::string buffer(static_cast<size_t>(size), '\0');
  if (!file.read(buffer.data(), size))
    throw std::runtime_error("Failed to read file: " + path);

  detail::JsonParser parser(buffer);
  return parser.parseGhost();
}
