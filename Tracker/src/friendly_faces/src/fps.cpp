#include "fps.h"

void FPSCounter::reset() {
	time(&start);
	frame_cnt = 0;
	time(&end);
}

FPSCounter::FPSCounter() {
	reset();
}

void FPSCounter::frame() {
	frame_cnt++;
	time(&end);
}

double FPSCounter::getFPS() {
	if (difftime(end, start) >= 1) {
		fps = frame_cnt;

		reset();
	}

	return fps;
}
