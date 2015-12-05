//
// Created by sean on 18/11/15.
//

#ifndef PCL_MULTI_THREADED_PROCESSING_BUFFER_HPP
#define PCL_MULTI_THREADED_PROCESSING_BUFFER_HPP

#include <memory>
#include <condition_variable>
#include <boost/circular_buffer.hpp>

template <typename Type>
class Buffer {
private:
    boost::circular_buffer<Type> buffer_;
    std::mutex mutex_;
    std::condition_variable buff_empty_;
    Buffer(const Buffer&) = delete;
    Buffer& operator = (const Buffer&) = delete;
    static const int TIME_OUT_MS_DEFAULT = 10;
    static const int DEFAULT_BUFF_SIZE = 100;
    std::chrono::duration<int, std::milli> time_out_ms_;

public:
    Buffer(size_t buff_size, int time_out_ms) :
            time_out_ms_ (time_out_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.set_capacity(buff_size);
    }

    Buffer(size_t buff_size = DEFAULT_BUFF_SIZE) :
            Buffer(buff_size, TIME_OUT_MS_DEFAULT) {
    }

    inline bool isFull() {
        std::lock_guard<std::mutex> lock(mutex_);
        return (buffer_.full());
    }

    inline bool isEmpty() {
        std::lock_guard<std::mutex> lock(mutex_);
        return (buffer_.empty());
    }

    inline size_t getSize() {
        std::lock_guard<std::mutex> lock(mutex_);
        return (buffer_.size());
    }

    inline size_t getCapacity() const {
        return (buffer_.capacity());
    }

    bool pushBack(Type new_item) {
        bool overwrite = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(buffer_.full())
                overwrite = true;
            buffer_.push_back(new_item);
        }
        buff_empty_.notify_one();
        return overwrite;
    }

    Type getFront() {
        std::unique_lock<std::mutex> lock(mutex_);
        if(buff_empty_.wait_for(lock, time_out_ms_, [&](){ return !buffer_.empty();})) {
            Type front_item = buffer_.front();
            buffer_.pop_front();
            return front_item;
        }
        else
            return nullptr;
    }
};


#endif //PCL_MULTI_THREADED_PROCESSING_BUFFER_HPP
