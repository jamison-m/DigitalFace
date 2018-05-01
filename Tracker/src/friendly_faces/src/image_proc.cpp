#include "image_proc.h"

FFFrameProcessor::FFFrameProcessor() {
	if (!face_cascade.load("res/haarcascade_frontalface_default.xml")) {
		printf("error loading face cascade\n");
		return;
	}
}

int upscale;

vector<Rect> FFFrameProcessor::process(Mat frame) {
	upscale = frame.cols / 160;
	//haar cascades
	vector<Rect> FR_faces;

	Mat process;
	resize(frame, process, Size(160, 140));

	Mat frame_gray;
	cvtColor(process, frame_gray, COLOR_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	face_cascade.detectMultiScale(frame_gray, FR_faces, 1.1, 3, CV_HAAR_SCALE_IMAGE);
	
	
	for_each(FR_faces.begin(), FR_faces.end(), [](Rect& faceRect){
		faceRect.x *= upscale;
		faceRect.y *= upscale;
		faceRect.width *= upscale;
		faceRect.height *= upscale;
	});

	/*if (FR_faces.size() >= 1) {
		int x = FR_faces[0].width/2 + FR_faces[0].x;
		int y = FR_faces[0].height/2 + FR_faces[0].y;
		x *= upscale;
		y *= upscale;
		return Point2i(x,y);
	}*/

	return FR_faces;
}

