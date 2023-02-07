#pragma once

#include <vector>
#include <cassert>
#include <memory>
#include "MathUtils.h"

struct BakedAssocLaguerre {
    inline static std::vector<std::vector<std::vector<double>>> cache;

    static void bake(unsigned int maxN = 30, unsigned int maxI = 64) {
        for (unsigned int n = 0; n <= maxN; n++) {
            auto &column = cache.emplace_back();
            for (unsigned int m = 0; m <= n; m++) {
                auto &row = column.emplace_back();
                for (unsigned int i = 0; i <= maxI; i++) {
                    row.push_back(std::assoc_laguerre(n, m, (double)i / (double)maxI));
                }
            }
        }
    }

    static double call(unsigned int n, unsigned int m, double x) {
        assert(n < ys.size() && m < ys[n].size() && x >= 0.0 && x <= 1.0);
        return linearInterpolate(cache[n][m], x);
    }

};
