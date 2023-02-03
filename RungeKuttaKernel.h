#pragma once

#include "SolarSystem.h"

struct RungeKuttaArgs {
    Real *xs;
    Real *ys;
    Real *zs;
    Real *vxs;
    Real *vys;
    Real *vzs;
};

size_t rungeKutta(BodyState *bodies, size_t numBodies, Real dt);
