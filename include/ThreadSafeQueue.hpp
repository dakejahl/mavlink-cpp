#pragma once

#include <mutex>
#include <queue>
#include <memory>

template<class T>
class ThreadSafeQueue
{
public:
	ThreadSafeQueue(size_t maximum_size) : _max_size(maximum_size) {};

	~ThreadSafeQueue()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_queue.clear();
	}

	bool push_back(const T& item)
	{
		std::lock_guard<std::mutex> lock(_mutex);

		if (_queue.size() < _max_size) {
			_queue.push_back(item);
			return true;
		}

		return false;
	};

	bool pop_front(T* item)
	{
		std::lock_guard<std::mutex> lock(_mutex);

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
	size_t _max_size {};
};