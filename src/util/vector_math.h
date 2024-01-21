#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

#include <array>
#include <cmath>

namespace SimITL {
  using vec2 = std::array<float, 2>;
  using vec3 = std::array<float, 3>;
  using vec4 = std::array<float, 4>;
  using mat3 = std::array<vec3, 3>;
  // w, x, y, z
  using quat = std::array<float, 4>;

  inline vec3 toVec3(const float arr[3]){
    return {arr[0], arr[1], arr[2]};
  }

  inline vec4 toVec4(const float arr[4]){
    return {arr[0], arr[1], arr[2], arr[3]};
  }

  constexpr mat3 identity = {vec3{1, 0, 0}, vec3{0, 1, 0}, vec3{0, 0, 1}};

  inline float dot(const vec3& v1, const vec3& v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
  }

  inline float dot(const vec4& v1, const vec4& v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2] + v1[3] * v2[3];
  }

  inline float length(const vec3& v) {
    return std::sqrt(dot(v, v));
  }

  inline float length(const vec4& v) {
    return std::sqrt(dot(v, v));
  }

  inline float length2(const vec3& v) {
    return dot(v, v);
  }

  inline float length2(const vec4& v) {
    return dot(v, v);
  }

  inline vec3 cross(const vec3& v1, const vec3& v2) {
    return {(v1[1] * v2[2]) - (v1[2] * v2[1]),
            (v1[2] * v2[0]) - (v1[0] * v2[2]),
            (v1[0] * v2[1]) - (v1[1] * v2[0])};
  }

  inline vec3 operator/(const vec3& v, float s) {
    return {v[0] / s, v[1] / s, v[2] / s};
  }

  inline vec3 operator/(float s, const vec3& v) {
    return {s / v[0], s / v[1], s / v[2]};
  }

  inline vec4 operator/(const vec4& v, float s) {
    return {v[0] / s, v[1] / s, v[2] / s, v[3] / s};
  }

  inline vec4 operator/(float s, const vec4& v) {
    return {s / v[0], s / v[1], s / v[2], s / v[3]};
  }

  inline vec3 operator/(const vec3& a, const vec3& b) {
    return {a[0] / b[0], a[1] / b[1], a[2] / b[2]};
  }

  inline vec4 operator/(const vec4& a, const vec4& b) {
    return {a[0] / b[0], a[1] / b[1], a[2] / b[2], a[3] / b[3]};
  }

  inline vec3 operator*(const vec3& v, float s) {
    return {v[0] * s, v[1] * s, v[2] * s};
  }

  inline vec4 operator*(const vec4& v, float s) {
    return {v[0] * s, v[1] * s, v[2] * s, v[3] * s};
  }

  inline vec3 operator*(float s, const vec3& v) {
    return {v[0] * s, v[1] * s, v[2] * s};
  }

  inline vec4 operator*(float s, const vec4& v) {
    return {v[0] * s, v[1] * s, v[2] * s, v[3] * s};
  }

  inline vec3 operator+(const vec3& a, float b) {
    return {a[0] + b, a[1] + b, a[2] + b};
  }

  inline vec4 operator+(const vec4& a, float b) {
    return {a[0] + b, a[1] + b, a[2] + b, a[3] + b};
  }

  inline vec3 operator+(const vec3& a, const vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
  }

  inline vec4 operator+(const vec4& a, const vec4& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]};
  }

  inline vec3 operator-(float a, const vec3& b) {
    return {a - b[0], a - b[1], a- b[2]};
  }

  inline vec4 operator-(float a, const vec4& b) {
    return {a - b[0], a - b[1], a - b[2], a - b[3]};
  }

  inline vec3 operator-(const vec3& a, float b) {
    return {a[0] - b, a[1] - b, a[2] - b};
  }

  inline vec4 operator-(const vec4& a, float b) {
    return {a[0] - b, a[1] - b, a[2] - b, a[3] - b};
  }

  inline vec3 operator-(const vec3& a, const vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
  }

  inline vec4 operator-(const vec4& a, const vec4& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]};
  }

  inline vec3 operator*(const vec3& a, const vec3& b) {
    return {a[0] * b[0], a[1] * b[1], a[2] * b[2]};
  }

  inline vec4 operator*(const vec4& a, const vec4& b) {
    return {a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3]};
  }

  inline vec3 normalize(const vec3& v) {
    const auto l = length(v);
    if (l == 0.0f) return {0, 0, 0};
    return v / l;
  }

  inline vec3 abs(const vec3& v) {
    return {fabsf(v[0]), fabsf(v[1]), fabsf(v[2])};
  }

  inline vec4 abs(const vec4& v) {
    return {fabsf(v[0]), fabsf(v[1]), fabsf(v[2]), fabsf(v[3])};
  }

  /// Returns a vector transformed (multiplied) by the transposed matrix. Note
  /// that this results in a multiplication by the inverse of the matrix only if
  /// it represents a rotation-reflection.
  inline vec3 xform_inv(const mat3& m, const vec3& v) {
    return {(m[0][0] * v[0]) + (m[1][0] * v[1]) + (m[2][0] * v[2]),
            (m[0][1] * v[0]) + (m[1][1] * v[1]) + (m[2][1] * v[2]),
            (m[0][2] * v[0]) + (m[1][2] * v[1]) + (m[2][2] * v[2])};
  }

  inline vec3 xform(const mat3& m, const vec3& v) {
    return {dot(m[0], v), dot(m[1], v), dot(m[2], v)};
  }

  inline vec3 get_axis(const mat3& m, int axis) {
    return {m[0][axis], m[1][axis], m[2][axis]};
  }

  inline quat mat3_to_quat(const mat3& m) {
    /* Allow getting a quaternion from an unnormalized transform */
    float trace = m[0][0] + m[1][1] + m[2][2];
    quat temp;

    if (trace > 0.0) {
      float s = std::sqrt(trace + 1.0);
      temp[3] = (s * 0.5);
      s = 0.5 / s;

      temp[0] = ((m[2][1] - m[1][2]) * s);
      temp[1] = ((m[0][2] - m[2][0]) * s);
      temp[2] = ((m[1][0] - m[0][1]) * s);
    } else {
      int i = m[0][0] < m[1][1] ? (m[1][1] < m[2][2] ? 2 : 1)
                                : (m[0][0] < m[2][2] ? 2 : 0);
      int j = (i + 1) % 3;
      int k = (i + 2) % 3;

      float s = std::sqrt(m[i][i] - m[j][j] - m[k][k] + 1.0);
      temp[i] = s * 0.5;
      s = 0.5 / s;

      temp[3] = (m[k][j] - m[j][k]) * s;
      temp[j] = (m[j][i] + m[i][j]) * s;
      temp[k] = (m[k][i] + m[i][k]) * s;
    }

    return temp;
  }

  inline mat3 transpose(const mat3& m) {
    mat3 ret;
    ret[0] = get_axis(m, 0);
    ret[1] = get_axis(m, 1);
    ret[2] = get_axis(m, 2);
    return ret;
  }

  inline mat3 operator*(const mat3& a, const mat3& b) {
    return {vec3{dot(get_axis(b, 0), a[0]),
                 dot(get_axis(b, 1), a[0]),
                 dot(get_axis(b, 2), a[0])},
            vec3{dot(get_axis(b, 0), a[1]),
                 dot(get_axis(b, 1), a[1]),
                 dot(get_axis(b, 2), a[1])},
            vec3{dot(get_axis(b, 0), a[2]),
                 dot(get_axis(b, 1), a[2]),
                 dot(get_axis(b, 2), a[2])}};
  }

  inline float sum(const vec3& a) {
    return a[0] + a[1] + a[2];
  }

  inline float sum(const vec4& a) {
    return a[0] + a[1] + a[2] + a[3];
  }

  inline vec3 maximum(const vec3& a, float b) {
    return {
      std::max(a[0], b),
      std::max(a[1], b),
      std::max(a[2], b)
    };
  }

  inline vec4 maximum(const vec4& a, float b) {
    return {
      std::max(a[0], b),
      std::max(a[1], b),
      std::max(a[2], b),
      std::max(a[3], b)
    };
  }

  inline vec3 minimum(const vec3& a, float b) {
    return {
      std::min(a[0], b),
      std::min(a[1], b),
      std::min(a[2], b)
    };
  }

  inline vec4 minimum(const vec4& a, float b) {
    return {
      std::min(a[0], b),
      std::min(a[1], b),
      std::min(a[2], b),
      std::min(a[3], b)
    };
  }

  inline float clamp(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
  }

  inline vec3 clamp(const vec3& a, float min, float max) {
    return {
      clamp(a[0], min, max),
      clamp(a[1], min, max),
      clamp(a[2], min, max)
    };
  }

  inline vec4 clamp(const vec4& a, float min, float max) {
    return {
      clamp(a[0], min, max),
      clamp(a[1], min, max),
      clamp(a[2], min, max),
      clamp(a[3], min, max)
    };
  }

  inline float interpolate(float a, float b, float i){
    return a + ((b - a) * i);
  }

  inline vec3 interpolate(const vec3& a,const vec3& b, float i){
    return a + ((b - a) * i);
  }

  inline vec3 interpolate(const vec3& a,const vec3& b, const vec3& i){
    return a + ((b - a) * i);
  }
}  // namespace SimITL

#endif // VECTOR_MATH_H