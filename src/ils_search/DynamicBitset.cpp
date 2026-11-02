#include "DynamicBitset.h"

#include <cassert>

using pyvrp::DynamicBitset;

DynamicBitset::DynamicBitset(size_t numBits)

    : data_(numBits / BLOCK_SIZE + (numBits % BLOCK_SIZE != 0), 0)
{
}

DynamicBitset::DynamicBitset(std::vector<Block> data) : data_(std::move(data))
{
}

bool DynamicBitset::operator==(DynamicBitset const &other) const
{
    return data_ == other.data_;
}

bool DynamicBitset::operator[](size_t idx) const
{
    auto const q = idx / BLOCK_SIZE;
    auto const r = idx % BLOCK_SIZE;
    return data_[q][r];
}

DynamicBitset::Block::reference DynamicBitset::operator[](size_t idx)
{
    auto const q = idx / BLOCK_SIZE;
    auto const r = idx % BLOCK_SIZE;
    return data_[q][r];
}

std::vector<DynamicBitset::Block> const &DynamicBitset::data() const
{
    return data_;
}

bool DynamicBitset::all() const
{
    for (auto const &bitset : data_)
        if (!bitset.all())
            return false;
    return true;
}

bool DynamicBitset::any() const
{
    for (auto const &bitset : data_)
        if (bitset.any())
            return true;
    return false;
}

size_t DynamicBitset::count() const
{
    size_t count = 0;
    for (auto const &bitset : data_)
        count += bitset.count();
    return count;
}

size_t DynamicBitset::size() const { return BLOCK_SIZE * data_.size(); }

DynamicBitset &DynamicBitset::operator&=(DynamicBitset const &other)
{
    assert(size() == other.size());

    for (size_t idx = 0; idx != data_.size(); ++idx)
        data_[idx] &= other.data_[idx];

    return *this;
}

DynamicBitset &DynamicBitset::operator|=(DynamicBitset const &other)
{
    assert(size() == other.size());

    for (size_t idx = 0; idx != data_.size(); ++idx)
        data_[idx] |= other.data_[idx];

    return *this;
}

DynamicBitset &DynamicBitset::operator^=(DynamicBitset const &other)
{
    assert(size() == other.size());

    for (size_t idx = 0; idx != data_.size(); ++idx)
        data_[idx] ^= other.data_[idx];

    return *this;
}

DynamicBitset DynamicBitset::operator~() const
{
    std::vector<Block> copy(data_);
    for (size_t idx = 0; idx != data_.size(); ++idx)
        copy[idx] = ~data_[idx];
    return copy;
}

DynamicBitset DynamicBitset::operator|(DynamicBitset const &other) const
{
    std::vector<Block> copy(data_);
    return DynamicBitset(copy) |= other;
}

DynamicBitset DynamicBitset::operator&(DynamicBitset const &other) const
{
    std::vector<Block> copy(data_);
    return DynamicBitset(copy) &= other;
}

DynamicBitset DynamicBitset::operator^(DynamicBitset const &other) const
{
    std::vector<Block> copy(data_);
    return DynamicBitset(copy) ^= other;
}

DynamicBitset &DynamicBitset::set()
{
    for (auto &block : data_)
        block.set();
    return *this;
}

DynamicBitset &DynamicBitset::reset()
{
    for (auto &block : data_)
        block.reset();
    return *this;
}
