#pragma once

#include "MathUtils.h"
#include "Vector3.h"

// 轨道六根数
struct KeplerianOrbit {
    // 轨道半长轴（千米）
    Kilometers semiMajorAxis = 0.0;
    // 轨道离心率
    Real eccentricity = 0.0;
    // 过近地点时刻（儒略日数）
    JulianDays preigeeTime = 0.0;
    // 升交点赤经（度）
    Degrees ascendingNodeRA = 0.0;
    // 近地点幅角（度）
    Degrees preigeeArgument = 0.0;
    // 轨道倾角（度）
    Degrees inclination = 0.0;

    struct PositionAndVelocity {
        Vector3<Kilometers> position;
        Vector3<KilometersPerSecond> velocity;
    };

    // 根据轨道六根数计算指定时间环绕指定质量中心天体时的位置和速度矢量
    PositionAndVelocity getPositionAndVelocity(Kilometers3PerSeconds2 gravitationalParameter, JulianDays instant) const {
        // https://blog.csdn.net/Lionel_Evans/article/details/109177480
        auto a = semiMajorAxis;
        auto e = eccentricity;
        auto M = (instant - preigeeTime) * std::sqrt(gravitationalParameter / (a * a * a));
        auto v = M + e * (2.0 - e * e / 4.0 + 5.0 * e * e * e * e / 96.0) * std::sin(M) + e * e * (5.0 / 4.0 + 11.0 * e * e / 24.0) * std::sin(2.0 * M) + e * e * e * (13.0 / 12.0 - 43.0 * e * e / 64.0) * std::sin(3.0 * M) + 103.0 * e * e * e * e * std::sin(4.0 * M) / 96.0 + 1097.0 * e * e * e * e * e * std::sin(5.0 * M) / 960.0;
        auto r = a * (1.0 - e * e) / (1.0 + e * std::cos(v));
        Vector3<Kilometers> pos{r * std::cos(v), r * std::sin(v), 0.0};
        pos.rotateByZ(-preigeeArgument);
        pos.rotateByY(-inclination);
        pos.rotateByZ(-ascendingNodeRA);
        auto c = std::sqrt(gravitationalParameter / (a * (1.0 - e * e)));
        Vector3<Kilometers> vel{-c * std::sin(v), c * (e + std::cos(v)), 0.0};
        vel.rotateByZ(-preigeeArgument);
        vel.rotateByY(-inclination);
        vel.rotateByZ(-ascendingNodeRA);
        return {pos, vel};
    }

    //static KeplerianOrbit fromPositionAndVelocity(Vector3<Kilometers3PerSeconds2> const &position, // todo
                                                  //Vector3<Kilometers3PerSeconds2> const &velocity,
                                                  //Kilometers3PerSeconds2 gravitationalParameter,
                                                  //JulianDays instant) {
    //}
};
