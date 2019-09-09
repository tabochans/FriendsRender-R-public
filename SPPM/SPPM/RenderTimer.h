#pragma once
#include <chrono>

class RenderTimerMillisecond {
private:

	std::chrono::time_point<std::chrono::system_clock> _GlobalTimePoint;

	std::chrono::time_point<std::chrono::system_clock> _StartTimePoint;
	std::chrono::time_point<std::chrono::system_clock> _RenderTimePoint;

	double _AverageTime;
	unsigned long long _NumCall;

public:

	static unsigned int GetGlobalTime() {
		return static_cast<unsigned int>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
	}

	void Reset() {
		return Start();
	}

	void Start() {
		_AverageTime = 0;
		_NumCall = 0;
		_StartTimePoint = std::chrono::system_clock::now();
		_RenderTimePoint = _StartTimePoint;
	}

	unsigned int PassedTime() {
		auto Now = std::chrono::system_clock::now();
		unsigned int duration = std::chrono::duration_cast<std::chrono::milliseconds>(Now - _StartTimePoint).count();
		return duration;
	}

	double RenderStamp() {
		auto now = std::chrono::system_clock::now();
		unsigned int duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - _RenderTimePoint).count();

		_AverageTime = (_AverageTime * _NumCall + duration) / double(_NumCall + 1);
		_NumCall++;

		_RenderTimePoint = now;

		return _AverageTime;
	}

	double AverageTime() {
		return _AverageTime;
	}

	RenderTimerMillisecond() : _AverageTime(0), _StartTimePoint(std::chrono::system_clock::now()), _NumCall(0), _GlobalTimePoint(std::chrono::system_clock::now()){
	}
	~RenderTimerMillisecond() {
	}


};