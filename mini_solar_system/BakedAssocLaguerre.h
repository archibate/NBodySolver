#pragma once

#include <vector>
#include <cassert>
#include <memory>
#include "MathUtils.h"

struct BakedAssocLaguerre {
    inline static std::vector<std::vector<std::vector<double>>> cache;

    static void bake(int maxN = 32, int maxI = 1024) {
        for (int n = 0; n <= maxN; n++) {
            auto &column = cache.emplace_back();
            for (int m = 0; m <= n; m++) {
                auto &row = column.emplace_back();
                for (int i = 0; i <= maxI; i++) {
                    row.push_back(std::assoc_laguerre(n, m, (double)i / (double)maxI));
                }
            }
        }
    }

    double operator()(int n, int m, double x) {
        if (n < 0 || n >= cache.size()) return 0.0;
        if (m < 0 || m > n) return 0.0;
        assert(n + 1 == cache[n].size());
        assert(x >= -1.0 && x <= 1.0);
        return linearInterpolate(cache[n][m], std::abs(x) * (cache[n][m].size() - 1));
    }
};
