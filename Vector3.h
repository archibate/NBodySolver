#pragma once

#include "MathUtils.h"

template <class T>
struct Vector3 {
    T x, y, z;

    Vector3 operator+() const {
        return *this;
    }

    Vector3 operator-() const {
        return {-x, -y, -z};
    }

    Vector3 operator+(Vector3 const &v) const {
        return {x + v.x, y + v.y, z + v.z};
    }

    Vector3 operator-(Vector3 const &v) const {
        return {x - v.x, y - v.y, z - v.z};
    }

    Vector3 operator*(T t) const {
        return {x * t, y * t, z * t};
    }

    Vector3 operator/(T t) const {
        return *this * (T(1) / t);
    }

    friend Vector3 operator*(T t, Vector3 const &v) {
        return v * t;
    }

    Vector3 &operator+=(Vector3 const &v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vector3 &operator-=(Vector3 const &v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Vector3 &operator*=(T t) {
        x *= t;
        y *= t;
        z *= t;
        return *this;
    }

    Vector3 &operator/=(T t) {
        return *this *= (T(1) / t);
    }

    // 点积
    T dot(Vector3 const &v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // 叉积
    Vector3 cross(Vector3 const &v) const {
        return {
            z * v.x - v.z * x,
            y * v.z - v.y * z,
            x * v.y - v.x * y,
        };
    }

    // 归一化（返回结果）
    Vector3 normalized() const {
        return *this * lengthInversed();
    }

    // 归一化（就地修改）
    Vector3 &normalize() {
        return *this *= lengthInversed();
    }

    // 长度的平方
    T lengthSquared() const {
        return x * x + y * y + z * z;
    }

    // 长度
    T length() const {
        return std::sqrt(lengthSquared());
    }

    // 长度倒数的平方
    T lengthInversedSquared() const {
        T len = lengthSquared();
        return len != T(0) ? T(1) / len : T(0);
    }

    // 长度的倒数
    T lengthInversed() const {
        T len = length();
        return len != T(0) ? T(1) / len : T(0);
    }

    // 改变长度，不改变方向
    Vector3 &setLength(T newLen) {
        normalize();
        return *this *= newLen;
    }

    // 设置指定法向上的分量，不改变投影方向上的分量
    Vector3 &setDot(Vector3 const &dir, T newDot) {
        return *this += ((newDot - dot(dir)) / dir.lengthSquared()) * dir;
    }

    // 沿指定法向翻转
    Vector3 flipDot(Vector3 const &dir) const {
        return *this -= (T(2) * dot(dir) / dir.lengthSquared()) * dir;
    }

    // 本矢量作为入射光方向，返回根据指定平面法向反射后的出射光方向
    Vector3 reflect(Vector3 const &dir) const {
        return deepCopy().flipDot(dir);
    }

    // TODO: refract

    // 修改 X 分量
    Vector3 &setX(T newX) {
        x = newX;
        return *this;
    }

    // 修改 Y 分量
    Vector3 &setY(T newY) {
        y = newY;
        return *this;
    }

    // 修改 Z 分量
    Vector3 &setZ(T newZ) {
        z = newZ;
        return *this;
    }

    // 修改 X 分量的符号
    Vector3 &flipX() {
        x = -x;
        return *this;
    }

    // 修改 Y 分量的符号
    Vector3 &flipY() {
        y = -y;
        return *this;
    }

    // 修改 Z 分量的符号
    Vector3 &flipZ() {
        z = -z;
        return *this;
    }

    // 设为零矢量
    Vector3 &setZero() {
        x = y = z = T(0);
        return *this;
    }

    // 从 X、Y、Z 分量构造矢量
    Vector3 &fromXYZ(T newX, T newY, T newZ) {
        x = newX;
        y = newY;
        z = newZ;
        return *this;
    }

    // 赤纬（度），以 Z 轴正方向为北天极
    Degrees declination() const {
        return std::atan2(z, std::sqrt(x * x + y * y)) / kDegrees;
    }

    // 赤经（度），以 X 轴正方向为0°度经线
    Degrees rightAscension() const {
        return std::atan2(y, x) / kDegrees;
    }

    // 旋转到新的赤纬（度），保持赤经和长度不变
    Vector3 &setDeclination(Degrees newDec) {
        fromSpherical(length(), newDec, rightAscension());
        return *this;
    }

    // 旋转到新的赤经（度），保持和赤纬长度不变
    Vector3 &setRightAscension(Degrees newRA) {
        fromSpherical(length(), declination(), newRA);
        return *this;
    }

    // 从长度、赤经和赤纬（度）构造矢量
    Vector3 &fromSpherical(T newLen, Degrees newDec, Degrees newRA) {
        z = newLen * std::sin(newDec * kDegrees);
        auto lenXY = newLen * std::cos(newDec * kDegrees);
        y = lenXY * std::sin(newRA * kDegrees);
        x = lenXY * std::cos(newRA * kDegrees);
        return *this;
    }

    // 沿X轴正方向旋转一定角度（度）
    Vector3 &rotateByX(Degrees angle) {
        auto cosAngle = std::cos(angle * kDegrees);
        auto sinAngle = std::sin(angle * kDegrees);
        auto tmp1 = y * cosAngle - z * sinAngle;
        auto tmp2 = z * cosAngle + y * sinAngle;
        y = tmp1;
        z = tmp2;
        return *this;
    }

    // 沿Y轴正方向旋转一定角度（度）
    Vector3 &rotateByY(Degrees angle) {
        auto cosAngle = std::cos(angle * kDegrees);
        auto sinAngle = std::sin(angle * kDegrees);
        auto tmp1 = z * cosAngle - x * sinAngle;
        auto tmp2 = x * cosAngle + z * sinAngle;
        z = tmp1;
        x = tmp2;
        return *this;
    }

    // 沿Z轴正方向旋转一定角度（度）
    Vector3 &rotateByZ(Degrees angle) {
        auto cosAngle = std::cos(angle * kDegrees);
        auto sinAngle = std::sin(angle * kDegrees);
        auto tmp1 = x * cosAngle - y * sinAngle;
        auto tmp2 = y * cosAngle + x * sinAngle;
        x = tmp1;
        y = tmp2;
        return *this;
    }

    // 沿指定轴向旋转一定角度（度）
    Vector3 &rotateByAxis(Vector3 const &axis, Degrees angle) {
        // 从 axis 到 Z 轴
        rotateByZ(-axis.rightAscension());
        rotateByY(axis.declination() - 90.0);
        // 绕 Z 轴旋转 angle 度
        rotateByZ(angle);
        // 从 Z 轴到 axis
        rotateByY(90.0 - axis.declination());
        rotateByZ(axis.rightAscension());
        return *this;
    }

    // 计算两个矢量之间的夹角
    Degrees angleBetween(Vector3 const &v) const {
        return std::acos(dot(v) * v.lengthInversed() * lengthInversed()) / kDegrees;
    }

    // 深拷贝本矢量
    Vector3 deepCopy() const {
        return *this;
    }
};
