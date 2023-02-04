#pragma once

#include "MathUtils.h"

// 球坐标
struct SphericalCoordinate {
    // 经度（度）
    Degrees rightAscension = 0.0;
    // 纬度（度）
    Degrees declination = 90.0;
    // 到中心天体的距离（千米）
    Kilometers distance = 0.0;

    // 笛卡尔坐标转球坐标
    static SphericalCoordinate fromCartesian(Vector3<Kilometers> const &position) {
        return {}; // TODO
    }

    // 球坐标转笛卡尔坐标
    Vector3<Kilometers> toCartesian() const {
        return {}; // TODO
    }
};
