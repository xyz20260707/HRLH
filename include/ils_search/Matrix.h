#ifndef PYVRP_MATRIX_H
#define PYVRP_MATRIX_H

#include <algorithm>
#include <cassert>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace pyvrp
{
template <typename T> class Matrix
{
    size_t cols_ = 0;
    size_t rows_ = 0;
    std::vector<T> data_ = {};

public:
    Matrix() = default;

    explicit Matrix(size_t nRows, size_t nCols, T value = {});

    explicit Matrix(std::vector<T> &&data, size_t nRows, size_t nCols);

    bool operator==(Matrix const &other) const = default;

    [[nodiscard]] decltype(auto) operator()(size_t row, size_t col);
    [[nodiscard]] decltype(auto) operator()(size_t row, size_t col) const;

    typename std::vector<T>::const_iterator begin() const;
    typename std::vector<T>::const_iterator end() const;

    typename std::vector<T>::iterator begin();
    typename std::vector<T>::iterator end();

    [[nodiscard]] T *data();
    [[nodiscard]] T const *data() const;

    [[nodiscard]] size_t numCols() const;

    [[nodiscard]] size_t numRows() const;

    [[nodiscard]] T max() const;

    [[nodiscard]] size_t size() const;
};

template <typename T>
Matrix<T>::Matrix(size_t nRows, size_t nCols, T value)
    : cols_(nCols), rows_(nRows), data_(nRows * nCols, value)
{
}

template <typename T>
Matrix<T>::Matrix(std::vector<T> &&data, size_t nRows, size_t nCols)
    : cols_(nCols), rows_(nRows), data_(std::forward<std::vector<T>>(data))
{
    assert(cols_ * rows_ == data_.size());
}

template <typename T>
decltype(auto) Matrix<T>::operator()(size_t row, size_t col)
{
    return data_[cols_ * row + col];
}

template <typename T>
decltype(auto) Matrix<T>::operator()(size_t row, size_t col) const
{
    return data_[cols_ * row + col];
}

template <typename T>
typename std::vector<T>::const_iterator Matrix<T>::begin() const
{
    return data_.begin();
}

template <typename T>
typename std::vector<T>::const_iterator Matrix<T>::end() const
{
    return data_.end();
}

template <typename T> typename std::vector<T>::iterator Matrix<T>::begin()
{
    return data_.begin();
}

template <typename T> typename std::vector<T>::iterator Matrix<T>::end()
{
    return data_.end();
}

template <typename T> T *Matrix<T>::data() { return data_.data(); }
template <typename T> T const *Matrix<T>::data() const { return data_.data(); }

template <typename T> size_t Matrix<T>::numCols() const { return cols_; }

template <typename T> size_t Matrix<T>::numRows() const { return rows_; }

template <typename T> T Matrix<T>::max() const
{
    return *std::max_element(data_.begin(), data_.end());
}

template <typename T> size_t Matrix<T>::size() const { return data_.size(); }
}

#endif
