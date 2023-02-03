#pragma once

#include <optional>
#include <utility>

template <class T>
struct Optional {
    std::optional<T> o;

    template <class F, class = std::enable_if_t<!std::is_void_v<std::invoke_result_t<F, T>>>>
    Optional<std::invoke_result_t<F, T>> operator&(F const &f) const {
        if (o) return {f(*o)};
        else return std::nullopt;
    }

    template <class F, class = std::enable_if_t<std::is_void_v<std::invoke_result_t<F, T>>>>
    Optional const &operator&(F const &f) const {
        if (o) f(*o);
        return *this;
    }

    template <class F, class = std::enable_if_t<std::is_convertible_v<std::invoke_result_t<F>, T>>>
    T operator|(F const &f) const {
        if (o) return *o;
        else return f();
    }

    template <class F, class = std::enable_if_t<std::is_void_v<std::invoke_result_t<F>>>>
    Optional const &operator|(F const &f) const {
        if (!o) f();
        return *this;
    }

    Optional(T t) : o(std::move(t)) {
    }

    Optional(std::nullopt_t) : o(std::nullopt) {
    }
};

template <class T>
Optional(T) -> Optional<T>;
