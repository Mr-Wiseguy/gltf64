#ifndef __DYNAMIC_ARRAY_H__
#define __DYNAMIC_ARRAY_H__

#include <cstddef>
#include <memory>
#include <type_traits>
#include <initializer_list>
#include <fmt/core.h>

template <typename T, typename ...Ts>
inline constexpr bool areT_v = std::conjunction_v<std::is_same<T,Ts>...>;

template <typename T, typename ...Ts>
inline constexpr bool areConvertibleT_v = std::conjunction_v<std::is_convertible<Ts,T>...>;

template <typename T>
std::unique_ptr<T> make_unique_uninitialized(size_t size) {
    return std::unique_ptr<T>(new typename std::remove_extent<T>::type[size]);
}

template <typename T>
class dynamic_array {
public:
    using value_type             = T;
    using size_type              = std::size_t;
    using difference_type        = std::ptrdiff_t;
    using reference              = value_type&;
    using const_reference        = const value_type&;
    using pointer                = value_type*;
    using const_pointer          = const value_type*;
    using iterator               = value_type*;
    using const_iterator         = const value_type*;
    using reverse_iterator       = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    dynamic_array() = default;

    explicit dynamic_array(size_type size) :
        data_(std::unique_ptr<T[]>(new T[size]())),
        size_(size) {}

    explicit dynamic_array(size_type size, const T& value) :
        data_(std::unique_ptr<T[]>(new T[size])),
        size_(size)
    {
        std::fill(begin(), end(), value);
    }

    dynamic_array(std::initializer_list<T> list) :
        data_(std::unique_ptr<T[]>(new T[list.size()])),
        size_(list.size())
    {
        std::copy(list.begin(), list.end(), data_.get());
    }

    // Destructor (nothing needed due to unique_ptr)
    ~dynamic_array()
    {}

    // Copy constructor
    dynamic_array(const dynamic_array& rhs) :
        data_(std::unique_ptr<T[]>(new T[rhs.size_])),
        size_(rhs.size_)
    {
        std::copy(rhs.begin(), rhs.end(), data_.get());
    }
    
    // Move constructor
    dynamic_array(dynamic_array&& other) noexcept :
        data_(std::move(other.data_)),
        size_(std::exchange(other.size_, 0)) { }
    
    // Copy assignment
    dynamic_array& operator=(const dynamic_array& rhs)
    {
        if (this == &rhs) return *this;
        size_ = rhs.size_;
        data_ = std::unique_ptr<T[]>(new T[size_]);
        std::copy(rhs.begin(), rhs.end(), data_.get());
        return *this;
    }

    // Move assignment
    dynamic_array& operator=(dynamic_array&& rhs) noexcept
    {
        data_ = std::move(rhs.data_);
        size_ = std::exchange(rhs.size_, 0);
        return *this;
    };

    const T& operator[](size_type index) const noexcept { return data_[index]; }
    T&       operator[](size_type index)       noexcept { return data_[index]; }
    
    const T* data() const noexcept { return data_.get(); }
    T*       data()       noexcept { return data_.get(); }

    const_iterator begin() const noexcept { return &data_[0]; }
    iterator       begin()       noexcept { return &data_[0]; }

    const_iterator end() const noexcept { return &data_[size_]; }
    iterator       end()       noexcept { return &data_[size_]; }

    const_reverse_iterator rbegin() const noexcept { return &data_[size_ - 1]; }
    reverse_iterator       rbegin()       noexcept { return &data_[size_ - 1]; }

    const_reverse_iterator rend() const noexcept { return &data_[-1]; }
    reverse_iterator       rend()       noexcept { return &data_[-1]; }

    T* release() noexcept { size_ = 0; return data_.release(); }

    void truncate(size_type new_size) { if (new_size < size_) { size_ = new_size; }}

    size_type size() const noexcept { return size_; }
    bool empty() const noexcept { return size_ == 0; }
private:
    std::unique_ptr<T[]> data_;
    size_type size_;
};

#endif
