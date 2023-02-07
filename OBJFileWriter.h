#pragma once

#include <vector>
#include <ostream>
#include <memory>
#include "MathUtils.h"
#include "Vector3.h"

struct OBJFileWriter {
    std::unique_ptr<std::ostream> oss;
    size_t vertCount = 0;

    void addComment(const char *msg) {
        *oss << "# " << msg << '\n';
    }

    void addObject(const char *name) {
        *oss << "o " << name << '\n';
    }

    void addCurve(std::vector<Vector3<Real>> const &points, Real scale) {
        size_t vertBase = vertCount;
        for (auto const &p: points) {
            vertCount++;
            *oss << 'v' << ' ' << p.x * scale << ' ' << p.y * scale << ' ' << p.z * scale << '\n';
        }
        for (size_t i = vertBase + 1; i < vertCount; i++) {
            *oss << 'f' << ' ' << i << ' ' << i + 1 << '\n';
        }
    }
};
