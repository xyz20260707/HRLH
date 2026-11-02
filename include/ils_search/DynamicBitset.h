#ifndef PYVRP_DYNAMICBITSET_H
#define PYVRP_DYNAMICBITSET_H

#include <bitset>
#include <climits>
#include <vector>

namespace pyvrp
{

class DynamicBitset
{
public:
    static constexpr size_t BLOCK_SIZE = sizeof(unsigned long long) * CHAR_BIT;
    using Block = std::bitset<BLOCK_SIZE>;

private:
    std::vector<Block> data_;

public:
    DynamicBitset(size_t numBits);
    DynamicBitset(std::vector<Block> data);

    [[nodiscard]] bool operator==(DynamicBitset const &other) const;

    [[nodiscard]] bool operator[](size_t idx) const;
    [[nodiscard]] Block::reference operator[](size_t idx);

    [[nodiscard]] std::vector<Block> const &data() const;

    [[nodiscard]] bool all() const;
    [[nodiscard]] bool any() const;

    [[nodiscard]] size_t count() const;
    [[nodiscard]] size_t size() const;

    DynamicBitset &operator&=(DynamicBitset const &other);
    DynamicBitset &operator|=(DynamicBitset const &other);
    DynamicBitset &operator^=(DynamicBitset const &other);

    [[nodiscard]] DynamicBitset operator|(DynamicBitset const &other) const;
    [[nodiscard]] DynamicBitset operator&(DynamicBitset const &other) const;
    [[nodiscard]] DynamicBitset operator^(DynamicBitset const &other) const;
    [[nodiscard]] DynamicBitset operator~() const;

    DynamicBitset &set();
    DynamicBitset &reset();
};
}

#endif
