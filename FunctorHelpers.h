#pragma once

#include <utility>
#include <type_traits>

struct NoReturnType {
    template <class T>
    operator T() const { return *static_cast<T const *>(reinterpret_cast<void const *>(1)); }
};

//#define FORWARD(v) (std::forward<decltype(v)>(v))
#define F_L0(...) (auto &&...) { return __VA_ARGS__; }
#define F_L1(v, ...) (auto &&v) { return __VA_ARGS__; }
#define F_L2(v1, v2, ...) (auto &&v1, auto &&v2) { return __VA_ARGS__; }
#define F_L3(v1, v2, v3, ...) (auto &&v1, auto &&v2, auto &&v3) { return __VA_ARGS__; }
#define F_LN(vs, ...) (auto &&...vs) { return __VA_ARGS__; }
#define F_L1N(v1, vs, ...) (auto &&v, auto &&...vs) { return __VA_ARGS__; }
#define F_LR(...) (auto &&__v) { return __v __VA_ARGS__; }
#define F_L0(...) (auto &&...) { return __VA_ARGS__; }
#define F_DIE(...) (auto &&...) -> NoReturnType { throw __VA_ARGS__; }
#define FF_L1(v) (auto &&v)
#define FF_L2(v1, v2) (auto &&v1, auto &&v2)
#define FF_L3(v1, v2, v3) (auto &&v1, auto &&v2, auto &&v3)
#define FF_LN(vs) (auto &&...vs)
#define FF_L1N(v1, vs) (auto &&v, auto &&...vs)

template <class F>
struct SelfKnowingFunctor {
    F f;

    template <class ...Ts>
    std::invoke_result_t<F, SelfKnowingFunctor, Ts...> operator()(Ts &&...ts) const {
        return f(*this, std::forward<Ts>(ts)...);
    }
};

template <class F>
SelfKnowingFunctor(F) -> SelfKnowingFunctor<F>;
