#pragma once

#include <vector>
#include <unordered_map>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class FFFrameProcessor {
private:
	CascadeClassifier face_cascade;
public:
	FFFrameProcessor();

	vector<Rect> process(Mat frame);
};
