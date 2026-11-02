#include "RandomNumberGenerator.h"

using pyvrp::RandomNumberGenerator;

RandomNumberGenerator::RandomNumberGenerator(uint32_t seed)
    : state_{seed, 123456789, 362436069, 521288629}
{
}

RandomNumberGenerator::result_type RandomNumberGenerator::operator()()
{

    uint32_t t = state_[3];

    uint32_t s = state_[0];
    state_[3] = state_[2];
    state_[2] = state_[1];
    state_[1] = s;

    t ^= t << 11;
    t ^= t >> 8;

    return state_[0] = t ^ s ^ (s >> 19);
}

double RandomNumberGenerator::rand()
{
    return operator()() / static_cast<double>(max());
}
