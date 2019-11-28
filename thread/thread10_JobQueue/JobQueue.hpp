#pragma once
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>

/**
 * @brief Job queue for the producer-consumer paradigm
 * @tparam T
 */
template <typename T>
class JobQueue {
  public:
    class Job {
      public:
        Job() : valid_(false) {}
        explicit Job(const T& data) : data_(data), valid_(true) {}

      public:
        // check whether the data is valid
        bool isValid() const { return valid_; }

        // get reference to the data
        T& data() { return data_; }
        const T& data() const { return data_; }

      private:
        T data_;
        bool valid_;
    };

  public:
    JobQueue();
    explicit JobQueue(const size_t& maxJobNums);
    ~JobQueue();

    /**
     * @brief Get the number of pushed but not popped jobs in the queue
     * @return The number of pushed but not poped jobs in the queue
     */
    std::size_t size() const;

    /**
     * @brief Return whether the job queue is stopped
     * @return True if the job queue is stopped, otherwise return false
     */
    inline bool isStop() const { return stop_; }

    /**
     * @brief Push a new job to the queue, waits if the number of jobs is exceeded
     * @param data New job data
     * @return True if push success, false for push fail
     */
    bool push(const T& data);

    /**
     * @brief Pop a job from the queue, wait if there is no job in the queue
     * @return Job popped from the queue
     */
    Job pop();

    /**
     * @brief Wait for all jobs to bo popped and then stop the queue
     */
    void wait();

    /**
     * @brief Stop the queue
     */
    void stop();

    /**
     * @brief Clear all pushed and not popped jobs from the queue
     */
    void clear();

  private:
    std::size_t maxJobNums_;
    std::atomic<bool> stop_;
    std::queue<T> jobs_;
    std::mutex mutex_;
    std::condition_variable pushCondition_;
    std::condition_variable popCondition_;
    std::condition_variable emptyCondition_;
};

/**************************************************************************************************
 * Implementation
 **************************************************************************************************/

template <typename T>
JobQueue<T>::JobQueue() : JobQueue(std::numeric_limits<std::size_t>::max()) {}

template <typename T>
JobQueue<T>::JobQueue(const size_t& maxJobNums) : maxJobNums_(maxJobNums), stop_(false) {}

template <typename T>
JobQueue<T>::~JobQueue() {
    stop();
}

// Get the number of pushed but not popped jobs in the queue
template <typename T>
std::size_t JobQueue<T>::size() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return jobs_.size();
}

// Push a new job to the queue, waits if the number of jobs is exceeded
template <typename T>
bool JobQueue<T>::push(const T& data) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (jobs_.size() >= maxJobNums_ && !stop_) {
        popCondition_.wait(lock);
    }

    if (stop_) {
        return false;
    } else {
        jobs_.emplace(data);
        pushCondition_.notify_one();
        return true;
    }
}

// Pop a job from the queue, wait if there is no job in the queue
template <typename T>
typename JobQueue<T>::Job JobQueue<T>::pop() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (jobs_.empty() && !stop_) {
        pushCondition_.wait(lock);
    }

    if (stop_) {
        return Job();
    } else {
        Job job(jobs_.front());
        jobs_.pop();
        popCondition_.notify_one();
        if (jobs_.empty()) {
            emptyCondition_.notify_all();
        }
        return job;
    }
}

// Wait for all jobs to bo popped and then stop the queue
template <typename T>
void JobQueue<T>::wait() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!jobs_.empty()) {
        emptyCondition_.wait(lock);
    }
}

// Stop the queue
template <typename T>
void JobQueue<T>::stop() {
    stop_ = true;
    pushCondition_.notify_all();
    popCondition_.notify_all();
}

// Clear all pushed and not popped jobs from the queue
template <typename T>
void JobQueue<T>::clear() {
    std::unique_lock<std::mutex> lock(mutex_);
    std::queue<T> emptyJobs;
    std::swap(jobs_, emptyJobs);
}
