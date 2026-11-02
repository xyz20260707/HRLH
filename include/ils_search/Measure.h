#ifndef PYVRP_MEASURE_H
#define PYVRP_MEASURE_H

#include <cassert>
#include <cmath>
#include <compare>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <type_traits>

namespace pyvrp
{
namespace measure_detail
{
template <typename T> constexpr bool add_overflows(T a, T b)
{
    constexpr auto lo = std::numeric_limits<T>::min();
    constexpr auto hi = std::numeric_limits<T>::max();
    if constexpr (std::is_signed_v<T>)
        return (b > 0 && a > hi - b) || (b < 0 && a < lo - b);
    else
        return a > hi - b;
}

template <typename T> constexpr bool sub_overflows(T a, T b)
{
    constexpr auto lo = std::numeric_limits<T>::min();
    constexpr auto hi = std::numeric_limits<T>::max();
    if constexpr (std::is_signed_v<T>)
        return (b < 0 && a > hi + b) || (b > 0 && a < lo + b);
    else
        return a < b;
}

template <typename T> constexpr bool mul_overflows(T a, T b)
{
    constexpr auto lo = std::numeric_limits<T>::min();
    constexpr auto hi = std::numeric_limits<T>::max();
    if (a == 0 || b == 0) return false;
    if constexpr (std::is_signed_v<T>)
    {
        if (a > 0) return b > 0 ? a > hi / b : b < lo / a;
        return b > 0 ? a < lo / b : a < hi / b;
    }
    else
        return a > hi / b;
}
}

enum class MeasureType
{
    COORD,
    DIST,
    DURATION,
    COST,
    LOAD,
};

template <typename T>
concept NumberType = std::is_arithmetic_v<T>;

template <MeasureType Type, NumberType Value> class Measure;

using Coordinate = Measure<MeasureType::COORD, double>;
using Cost = Measure<MeasureType::COST, int64_t>;
using Distance = Measure<MeasureType::DIST, int64_t>;
using Duration = Measure<MeasureType::DURATION, int64_t>;
using Load = Measure<MeasureType::LOAD, int64_t>;

template <MeasureType _, NumberType Value> class Measure
{
    Value value = 0;

public:

    Measure() = default;

    template <NumberType T>
    Measure(T const value) : value(static_cast<Value>(value))
    {
    }

    template <NumberType T> explicit operator T() const
    {
        return static_cast<T>(value);
    }

    template <MeasureType Other> explicit operator Measure<Other, Value>() const
    {
        return value;
    }

    [[nodiscard]] Value get() const;

    Measure &operator+=(Measure const rhs);
    Measure &operator-=(Measure const rhs);
    Measure &operator*=(Measure const rhs);
    Measure &operator/=(Measure const rhs);

    [[nodiscard]] bool operator==(Measure const other) const;
    [[nodiscard]] std::strong_ordering operator<=>(Measure const other) const;
};

template <MeasureType Type, NumberType Value>
Value Measure<Type, Value>::get() const
{
    return value;
}

template <MeasureType Type, NumberType Value>
Measure<Type, Value> &
Measure<Type, Value>::operator+=(Measure<Type, Value> const rhs)
{
    if constexpr (std::is_integral_v<Value>)
    {
        assert(!measure_detail::add_overflows(this->value, rhs.value));
    }

    this->value += rhs.value;
    return *this;
}

template <MeasureType Type, NumberType Value>
Measure<Type, Value> &
Measure<Type, Value>::operator-=(Measure<Type, Value> const rhs)
{
    if constexpr (std::is_integral_v<Value>)
    {
        assert(!measure_detail::sub_overflows(this->value, rhs.value));
    }

    this->value -= rhs.value;
    return *this;
}

template <MeasureType Type, NumberType Value>
Measure<Type, Value> &
Measure<Type, Value>::operator*=(Measure<Type, Value> const rhs)
{
    if constexpr (std::is_integral_v<Value>)
    {
        assert(!measure_detail::mul_overflows(this->value, rhs.value));
    }

    this->value *= rhs.value;
    return *this;
}

template <MeasureType Type, NumberType Value>
Measure<Type, Value> &
Measure<Type, Value>::operator/=(Measure<Type, Value> const rhs)
{
    this->value /= rhs.value;
    return *this;
}

template <MeasureType Type, NumberType Value>
bool Measure<Type, Value>::operator==(Measure<Type, Value> const other) const
{
    return value == other.value;
}

template <MeasureType Type, NumberType Value>
std::strong_ordering
Measure<Type, Value>::operator<=>(Measure<Type, Value> const other) const
{
    return value <=> other.value;
}

template <MeasureType Type, NumberType Value>
Measure<Type, Value> operator+(Measure<Type, Value> const lhs,
                               Measure<Type, Value> const rhs)
{
    if constexpr (std::is_integral_v<Value>)
    {
        assert(!measure_detail::add_overflows(lhs.get(), rhs.get()));
    }

    return lhs.get() + rhs.get();
}

template <MeasureType Type, NumberType Value>
Measure<Type, Value> operator+(Measure<Type, Value> const lhs)
{
    return +lhs.get();
}

template <MeasureType Type, NumberType Value>
Measure<Type, Value> operator-(Measure<Type, Value> const lhs,
                               Measure<Type, Value> const rhs)
{
    if constexpr (std::is_integral_v<Value>)
    {
        assert(!measure_detail::sub_overflows(lhs.get(), rhs.get()));
    }

    return lhs.get() - rhs.get();
}

template <MeasureType Type, NumberType Value>
Measure<Type, Value> operator-(Measure<Type, Value> const lhs)
{
    return -lhs.get();
}

template <MeasureType Type, NumberType Value>
Measure<Type, Value> operator*(Measure<Type, Value> const lhs,
                               Measure<Type, Value> const rhs)
{
    if constexpr (std::is_integral_v<Value>)
    {
        assert(!measure_detail::mul_overflows(lhs.get(), rhs.get()));
    }

    return lhs.get() * rhs.get();
}

template <MeasureType Type, NumberType Value>
Measure<Type, Value> operator/(Measure<Type, Value> const lhs,
                               Measure<Type, Value> const rhs)
{
    return lhs.get() / rhs.get();
}
}

template <pyvrp::MeasureType Type, pyvrp::NumberType Value>
class std::numeric_limits<pyvrp::Measure<Type, Value>>
{
public:
    static pyvrp::Measure<Type, Value> max()
    {
        return std::numeric_limits<Value>::max();
    }

    static pyvrp::Measure<Type, Value> min()
    {
        return std::numeric_limits<Value>::min();
    }
};

#endif
