#pragma once

/**
 * \file json.h
 * \brief Header-only minimal JSON parser for small config files.
 *
 * Provides a recursive-descent JSON parser that produces a tree of
 * \c JsonObject values.  Includes Vec3F/Vec4F accessor helpers for use
 * with the SimITL \c packets.h types.
 *
 * Usage:
 * \code
 *   auto val  = json::parseFile("config.json");
 *   double n  = val.get("key").asNumber(0.0);
 *   Vec3F  v  = val.get("vec").asVec3();
 *   std::string s = val.get("name").asString();
 * \endcode
 */

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <cstring>

#include "network/packets.h"

// ============================================================================
// json namespace – minimal JSON parser
// ============================================================================

namespace json {

// ---------------------------------------------------------------------------
// JsonObject – a recursive value (Null / Number / String / Object)
//
// Object members are stored in a vector-of-pairs to avoid the "incomplete
// type" problem with std::unordered_map<JsonObject>.  Config files are
// small (dozens of keys), so linear lookup is fine.
// ---------------------------------------------------------------------------
class JsonObject {
public:
  enum Type { Null, Number, String, Object };

  Type   type         = Null;
  double numberValue  = 0.0;
  std::string stringValue;
  std::vector<std::pair<std::string, JsonObject>> members;

  // -- query helpers -------------------------------------------------------

  bool has(const std::string& key) const {
    if (type != Object) return false;
    for (auto& m : members)
      if (m.first == key) return true;
    return false;
  }

  const JsonObject& get(const std::string& key) const {
    static const JsonObject kNull;
    if (type != Object) return kNull;
    for (auto& m : members)
      if (m.first == key) return m.second;
    return kNull;
  }

  void set(const std::string& key, JsonObject&& val) {
    members.emplace_back(key, std::move(val));
  }

  // -- scalar accessors ----------------------------------------------------

  double asNumber(double def = 0.0) const {
    return type == Number ? numberValue : def;
  }

  int asInt(int def = 0) const {
    return type == Number ? static_cast<int>(numberValue) : def;
  }

  std::string asString(const std::string& def = "") const {
    return type == String ? stringValue : def;
  }

  // -- SimITL vector accessors ---------------------------------------------

  Vec3F asVec3() const {
    Vec3F v{};
    if (type == Object) {
      v.x = static_cast<float>(get("x").asNumber(0.0));
      v.y = static_cast<float>(get("y").asNumber(0.0));
      v.z = static_cast<float>(get("z").asNumber(0.0));
    }
    return v;
  }

  Vec4F asVec4() const {
    Vec4F v{};
    if (type == Object) {
      v.x = static_cast<float>(get("x").asNumber(0.0));
      v.y = static_cast<float>(get("y").asNumber(0.0));
      v.z = static_cast<float>(get("z").asNumber(0.0));
      v.w = static_cast<float>(get("w").asNumber(0.0));
    }
    return v;
  }
};

// ---------------------------------------------------------------------------
// JsonParser – recursive descent parser
// ---------------------------------------------------------------------------
class JsonParser {
public:
  explicit JsonParser(const std::string& input)
    : m_input(input), m_pos(0) {}

  JsonObject parse() {
    skipWhitespace();
    if (peek() == '{') return parseObject();
    throw std::runtime_error("json: expected top-level object");
  }

private:
  const std::string& m_input;
  size_t m_pos = 0;

  bool atEnd() const { return m_pos >= m_input.size(); }
  char peek() const { return atEnd() ? '\0' : m_input[m_pos]; }
  char consume() {
    if (atEnd()) throw std::runtime_error("json: unexpected end of input");
    return m_input[m_pos++];
  }

  void expect(char c) {
    skipWhitespace();
    char got = consume();
    if (got != c) {
      std::string msg = "json: expected '";
      msg += c;
      msg += "' got '";
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

  std::string parseString() {
    skipWhitespace();
    expect('"');
    std::string result;
    result.reserve(32);
    while (!atEnd()) {
      char c = consume();
      if (c == '"') break;
      if (c == '\\') {
        if (atEnd()) break;
        char esc = consume();
        switch (esc) {
          case '"':  result += '"';  break;
          case '\\': result += '\\'; break;
          case '/':  result += '/';  break;
          case 'n':  result += '\n'; break;
          case 'r':  result += '\r'; break;
          case 't':  result += '\t'; break;
          default:   result += esc;  break;
        }
      } else {
        result += c;
      }
    }
    return result;
  }

  double parseNumber() {
    skipWhitespace();
    size_t start = m_pos;

    if (peek() == '-') consume();
    while (!atEnd() && std::isdigit(static_cast<unsigned char>(peek())))
      consume();
    if (peek() == '.') {
      consume();
      while (!atEnd() && std::isdigit(static_cast<unsigned char>(peek())))
        consume();
    }
    if (peek() == 'e' || peek() == 'E') {
      consume();
      if (peek() == '+' || peek() == '-') consume();
      while (!atEnd() && std::isdigit(static_cast<unsigned char>(peek())))
        consume();
    }

    if (start == m_pos)
      throw std::runtime_error("json: expected a number");

    std::string numStr(m_input.data() + start, m_pos - start);
    char* end = nullptr;
    double val = std::strtod(numStr.c_str(), &end);
    if (end != numStr.c_str() + numStr.size())
      throw std::runtime_error("json: failed to parse number: " + numStr);
    return val;
  }

  JsonObject parseValue() {
    skipWhitespace();
    if (atEnd()) return {};
    char c = peek();

    if (c == '{') return parseObject();
    if (c == '"') {
      JsonObject obj;
      obj.type        = JsonObject::String;
      obj.stringValue = parseString();
      return obj;
    }
    if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
      JsonObject obj;
      obj.type        = JsonObject::Number;
      obj.numberValue = parseNumber();
      return obj;
    }
    // boolean / null – not used in our configs, return null
    if (c == 't') { m_pos += 4; return {}; }  // true
    if (c == 'f') { m_pos += 5; return {}; }  // false
    if (c == 'n') { m_pos += 4; return {}; }  // null

    throw std::runtime_error("json: unexpected character");
  }

  JsonObject parseObject() {
    JsonObject obj;
    obj.type = JsonObject::Object;
    expect('{');

    while (!atEnd() && peek() != '}') {
      skipWhitespace();
      std::string key = parseString();
      skipWhitespace();
      expect(':');
      obj.set(key, parseValue());
      skipWhitespace();
      if (peek() == ',') consume();
    }
    expect('}');
    return obj;
  }
};

// ---------------------------------------------------------------------------
// Free helpers
// ---------------------------------------------------------------------------

/// Read a text file into a std::string.
inline std::string readFile(const std::string& path) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file)
    throw std::runtime_error("Cannot open file: " + path);
  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::string buf(static_cast<size_t>(size), '\0');
  if (!file.read(buf.data(), size))
    throw std::runtime_error("Failed to read file: " + path);
  return buf;
}

/// Parse a JSON file from disk and return the root JsonObject.
inline JsonObject parseFile(const std::string& path) {
  auto content = readFile(path);
  JsonParser p(content);
  return p.parse();
}

/// Parse a JSON string and return the root JsonObject.
inline JsonObject parseString(const std::string& input) {
  JsonParser p(input);
  return p.parse();
}

} // namespace json

// ============================================================================
// Generic JSON output utilities
// ============================================================================

/// Write a JSON number value to an output stream.
inline void writeJsonValue(std::ostream& os, double val, int precision = 16)
{
  os.precision(precision);
  os << val;
}

/// Write a Vec3F as a JSON array [x, y, z].
inline void writeVec3Json(std::ostream& os, const Vec3F& v)
{
  os << "[" << v.x << ", " << v.y << ", " << v.z << "]";
}

/// Write a Vec4F as a JSON array [x, y, z, w].
inline void writeVec4Json(std::ostream& os, const Vec4F& v)
{
  os << "[" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << "]";
}
