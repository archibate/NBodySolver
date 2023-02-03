#pragma once

#include <cmath>

using Real = double;
using Kilometers3PerSeconds2 = Real;
using Seconds = Real;
using JulianDays = Real;
using Degrees = Real;
using DegreesPreDay = Real;
using Kilometers = Real;
using Kilometers2 = Real;
using KilometersPerSecond = Real;
using KilometersPerSecond2 = Real;

template <class T>
struct Vector3 {
    T x, y, z;

    Vector3 operator+(Vector3 const &v) const {
        return {x + v.x, y + v.y, z + v.z};
    }

    Vector3 operator-(Vector3 const &v) const {
        return {x - v.x, y - v.y, z - v.z};
    }

    Vector3 operator*(T t) const {
        return {x * t, y * t, z * t};
    }

    T operator/(T t) {
        return *this * (T(1) / t);
    }

    friend Vector3 operator*(T t, Vector3 const &v) {
        return v * t;
    }
};

template <class T>
static Real binarySearch(T const &arr, typename T::value_type val) {
    auto it = std::lower_bound(arr.begin(), arr.end(), val);
    if (it == arr.end()) {
        return arr.size();
    } else if (it == arr.begin()) {
        return 0;
    }
    size_t index = it - arr.begin();
    Real offset = (val - arr[index]) / (arr[index + 1] - arr[index]);
    return (Real)index + offset;
}

template <class T>
static typename T::value_type linearInterpolate(T const &arr, Real val) {
    size_t index = (size_t)std::floor(val);
    if (index <= 0) {
        return arr.size() ? arr[0] : typename T::value_type{};
    } else if (index + 1 >= arr.size()) {
        return arr.size() ? arr.back() : typename T::value_type{};
    }
    Real offset = val - (Real)index;
    return arr[index] * (1.0 - offset) + arr[index + 1] * offset;
}

#ifdef M_PI
static constexpr Real kPI = M_PI;
#else
static constexpr Real kPI = 3.14159265358979323846;
#endif
static constexpr Real kDegrees = kPI / 180.0;
