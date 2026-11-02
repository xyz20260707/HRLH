#pragma once

#include <cstddef>
#include <functional>
#include <list>
#include <unordered_map>

template <class Key, class Value, class Hash = std::hash<Key>>
class LruCache {
public:
    explicit LruCache(std::size_t capacity = 0) : capacity_(capacity) {}

    LruCache(const LruCache& other)
        : capacity_(other.capacity_),
          lru_order_(other.lru_order_)
    {
        for (auto it = lru_order_.begin(); it != lru_order_.end(); ++it)
            table_.emplace(*it, Entry{other.table_.at(*it).value, it});
    }
    LruCache& operator=(const LruCache& other)
    {
        if (this != &other) {
            LruCache copy(other);
            swap(copy);
        }
        return *this;
    }
    LruCache(LruCache&&) = default;
    LruCache& operator=(LruCache&&) = default;
    void swap(LruCache& other) noexcept
    {
        std::swap(capacity_, other.capacity_);
        lru_order_.swap(other.lru_order_);
        table_.swap(other.table_);
    }


    Value* get_mutable(const Key& key)
    {
        auto it = table_.find(key);
        if (it == table_.end()) return nullptr;
        lru_order_.splice(lru_order_.begin(), lru_order_, it->second.it);
        return &it->second.value;
    }

    void put(const Key& key, const Value& value)
    {
        if (capacity_ == 0) return;
        auto it = table_.find(key);
        if (it != table_.end()) {
            it->second.value = value;
            lru_order_.splice(lru_order_.begin(), lru_order_, it->second.it);
            return;
        }
        lru_order_.push_front(key);
        try {
            table_.emplace(key, Entry{value, lru_order_.begin()});
        } catch (...) {
            lru_order_.pop_front();
            throw;
        }
        shrink_to_capacity();
    }

private:
    struct Entry {
        Value value;
        typename std::list<Key>::iterator it;
    };

    void shrink_to_capacity()
    {
        while (table_.size() > capacity_) {
            Key old_key = lru_order_.back();
            lru_order_.pop_back();
            table_.erase(old_key);
        }
    }

    std::size_t capacity_ = 0;
    std::list<Key> lru_order_;
    std::unordered_map<Key, Entry, Hash> table_;
};
