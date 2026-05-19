#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <stop_token>
#include <utility>

namespace gs_livo {

template<typename T>
class ConcurrentQueue {
public:
    ConcurrentQueue() = default;
    ~ConcurrentQueue() { clear(); }

    void push(T item) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            queue_.push(std::move(item));
        }
        cv_.notify_one();
    }

    bool tryPop(T& out) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.empty()) return false;
        out = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    T waitPop(std::stop_token stoken) {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, stoken, [this] { return !queue_.empty(); });
        if (queue_.empty()) return T{};
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    T waitPop() {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this] { return !queue_.empty(); });
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.size();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mtx_);
        while (!queue_.empty()) queue_.pop();
    }

private:
    std::queue<T>               queue_;
    mutable std::mutex          mtx_;
    std::condition_variable_any cv_;
};

}  // namespace gs_livo
