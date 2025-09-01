#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <cstddef>     // size_t
#include <memory>      // std::unique_ptr
#include <stdexcept>   // std::out_of_range
#include <utility>     // std::move

template <typename T>
class CircularBuffer {
public:
    // Constructor
    CircularBuffer() : CircularBuffer(10) {}       // Default capacity: 10
    explicit CircularBuffer(size_t capacity)
        : buffer_(std::make_unique<T[]>(check_cap_(capacity))),
          head_(0), tail_(0), capacity_(capacity), size_(0) {}

    // Rule of Five
    // Not allow copy to avoid shallow copy issues
    CircularBuffer(const CircularBuffer&) = delete;
    CircularBuffer& operator=(const CircularBuffer&) = delete;
    CircularBuffer(CircularBuffer&&) noexcept = default;
    CircularBuffer& operator=(CircularBuffer&&) noexcept = default;

    ~CircularBuffer() = default; // Smart pointer - automatically deallocates

    // Properties
    size_t capacity() const noexcept { return capacity_; }
    size_t size()     const noexcept { return size_; }
    bool   empty()    const noexcept { return size_ == 0; }
    bool   full()     const noexcept { return size_ == capacity_; }

    // Maintain memry, just reset indices
    void clear() noexcept { head_ = tail_ = size_ = 0; }

    // Do not overwrite when full
    bool push(const T& item) {
        if (full()) return false;
        buffer_[head_] = item;
        advance_head_();
        ++size_;
        return true;
    }
    bool push(T&& item) {
        if (full()) return false;
        buffer_[head_] = std::move(item);
        advance_head_();
        ++size_;
        return true;
    }

    // FIFO pop (Not return value) - Discard the oldest item and advance one step
    bool pop() {
        if (empty()) return false;
        advance_tail_();
        --size_;
        return true;
    }

    // 0=outdated data, size-1=recent data (Read-Only)
    T operator[](size_t index) const {
        if (index >= size_) throw std::out_of_range("Index out of range");
        size_t pos = (tail_ + index) % capacity_;
        return buffer_[pos]; // 필요하면 const T& 로 바꿔도 됨
    }

    // 디버그용
    size_t get_head() const noexcept { return head_; }
    size_t get_tail() const noexcept { return tail_; }

private:
    static size_t check_cap_(size_t c) {
        if (c < 1) throw std::invalid_argument("capacity must be >= 1");
        return c;
    }

    std::unique_ptr<T[]> buffer_;
    size_t head_;
    size_t tail_;
    size_t capacity_;
    size_t size_;

    void advance_head_() noexcept { head_ = (head_ + 1) % capacity_; }
    void advance_tail_() noexcept { tail_ = (tail_ + 1) % capacity_; }
};

#endif // CIRCULAR_BUFFER_H