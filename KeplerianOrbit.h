#pragma once

#include "MathUtils.h"
#include "Vector3.h"

struct KeplerianOrbit {
    Vector3<Kilometers> getPosition(JulianDays instant) const {
        return {};
    }

    Vector3<KilometersPerSecond> getVelocity(JulianDays instant) const {
        return {};
    }
};
