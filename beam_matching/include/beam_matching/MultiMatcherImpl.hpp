#pragma once

#include <iostream>

namespace beam_matching {

template <class T, class R>
MultiMatcher<T, R>::~MultiMatcher() {
    // Destroy all the workers
    this->stop = true;
    this->ip_condition.notify_all();
    for (int id = 0; id < this->n_thread; ++id) {
        this->pool.at(id).join();
    }
}

template <class T, class R>
void MultiMatcher<T, R>::InitPool(R params) {
    this->config = params;
    for (int i = 0; i < this->n_thread; i++) {
        this->matchers.emplace_back(T(R(this->config)));
        this->pool.emplace_back(
          std::thread(&MultiMatcher<T, R>::Spin, this, i));
    }
}

template <class T, class R>
void MultiMatcher<T, R>::Spin(int threadid) {
    std::tuple<int, PointCloudPtr, PointCloudPtr> val;
    while (true) {
        {
            std::unique_lock<std::mutex> lock(this->ip_mutex);
            while (!this->stop && this->input.empty()) {
                this->ip_condition.wait(lock);
            }
            if (this->stop) {
                return;
            }
            val = this->input.front();
            this->input.pop();
            lock.unlock();
            this->ip_condition.notify_one();
        }
        this->matchers.at(threadid).SetRef(std::get<1>(val));
        this->matchers.at(threadid).SetTarget(std::get<2>(val));
        this->matchers.at(threadid).Match();
        this->matchers.at(threadid).EstimateInfo();
        {
            std::unique_lock<std::mutex> lockop(this->op_mutex);
            this->output.emplace(std::get<0>(val),
                                 this->matchers.at(threadid).GetResult(),
                                 this->matchers.at(threadid).GetInfo());
            {
                std::unique_lock<std::mutex> lockcnt(this->cnt_mutex);
                --(this->remaining_matches);
            }
            lockop.unlock();
            this->op_condition.notify_one();
        }
    }
}

template <class T, class R>
void MultiMatcher<T, R>::Insert(const int &id,
                                const PointCloudPtr &src,
                                const PointCloudPtr &target) {
    {
        std::unique_lock<std::mutex> lock(this->ip_mutex);
        while (this->input.size() >= static_cast<size_t>(this->queue_size)) {
            this->ip_condition.wait(lock);
        }
        this->input.emplace(id, src, target);
        {
            std::unique_lock<std::mutex> lockcnt(this->cnt_mutex);
            ++(this->remaining_matches);
        }
        lock.unlock();
        this->ip_condition.notify_one();
    }
}

template <class T, class R>
bool MultiMatcher<T, R>::Done() {
    {
        std::unique_lock<std::mutex> lockcnt(this->cnt_mutex);
        if (this->remaining_matches == 0) {
            return true;
        } else {
            return false;
        }
    }
}

}  // namespace beam_matching
