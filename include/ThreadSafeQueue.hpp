#pragma once

#include <mutex>
#include <queue>
#include <condition_variable>

template<class T>
class ThreadSafeQueue
{
public:
	ThreadSafeQueue(size_t maximum_size) : _max_size(maximum_size) {};

	~ThreadSafeQueue()
	{
		std::scoped_lock<std::mutex> lock(_mutex);
		_queue.clear();
	}

	bool push_back(const T& item)
	{
		std::scoped_lock<std::mutex> lock(_mutex);

		if (_queue.size() < _max_size) {
			_queue.push_back(item);
			_cv.notify_one();
			return true;
		}

		return false;
	};

	bool pop_front(T* item, bool blocking = false)
	{
		std::unique_lock<std::mutex> lock(_mutex);

		if (blocking) {
			while (_queue.empty()) {
				_cv.wait(lock);
			}
		}

		if (_queue.size()) {
			*item = _queue.front();
			_queue.pop_front();
			return true;
		}

		return false;
	};

private:
	std::deque<T> _queue {};
	std::mutex _mutex {};
	std::condition_variable _cv {};
	size_t _max_size {};
};