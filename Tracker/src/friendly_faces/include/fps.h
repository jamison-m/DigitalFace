#pragma once

#include <time.h>

class FPSCounter {
private:
	unsigned int frame_cnt;
	double fps;
	time_t start, end;
	void reset();
public:
	FPSCounter();
	void frame();
	double getFPS();
};
