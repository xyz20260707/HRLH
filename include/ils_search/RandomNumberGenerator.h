#ifndef PYVRP_RANDOMNUMBERGENERATOR_H
#define PYVRP_RANDOMNUMBERGENERATOR_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace pyvrp
{

class RandomNumberGenerator
{
    std::array<uint32_t, 4> state_;

public:
    typedef uint32_t result_type;

    explicit RandomNumberGenerator(uint32_t seed);

    static constexpr size_t min();

    static constexpr size_t max();

    result_type operator()();

    double rand();

    template <typename T> result_type randint(T high);

    template <typename RandomIt> void shuffle(RandomIt first, RandomIt last);

};

constexpr size_t RandomNumberGenerator::min()
{
    return std::numeric_limits<result_type>::min();
}

constexpr size_t RandomNumberGenerator::max()
{
    return std::numeric_limits<result_type>::max();
}

template <typename T>
RandomNumberGenerator::result_type RandomNumberGenerator::randint(T high)
{
    static_assert(std::is_integral<T>::value);
    return operator()() % high;
}

template <typename RandomIt>
void RandomNumberGenerator::shuffle(RandomIt first, RandomIt last)
{

    for (auto idx = last - first - 1; idx > 0; --idx)
        std::swap(first[idx], first[randint(idx + 1)]);
}
}

#endif
