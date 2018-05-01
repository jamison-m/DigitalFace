#include <string>
#include <algorithm>
#include <stdio.h>
#include <cmath>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include "opencv2/opencv.hpp"
#include "ros/ros.h"

#include "image_proc.h"
#include "fps.h"
#include "net.h"

#include "visualization_msgs/Marker.h"

using namespace tf;
using namespace cv;
using namespace std;
using namespace ros;
using namespace cv_bridge;

//loop stuff
FFFrameProcessor frame_proc;
FPSCounter fps;
Mat frame;
UDPBroadcast bcast;
VideoCapture cap(0);
Point2i faces = Point2i(0,0);
int choose;

//3 rigid transformations <(eye1_pos, eye2_pos, head_pos), (eye1_rot, eye2_rot, head_rot)>
typedef pair<tuple<Vector3, Vector3, Vector3>, tuple<Vector3, Vector3, Vector3>> tri_rigid;

class SegbotProcessor {
private:
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;
	ros::Subscriber vis_sub;
	bool processing;

	void callback(const sensor_msgs::ImageConstPtr& msg) {
		CvImagePtr cv_ptr;
		try {
			cv_ptr = toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		run(cv_ptr->image.clone());
	}
	
	void vis_callback(const visualization_msgs::Marker& msg) {
		printf("%s\n", msg.ns.c_str());
	}
	
	void img_callback(const visualization_msgs::Marker& msg) {
		printf("%s\n", msg.ns.c_str());
	}
	
	//justin's math to convert virtual to physical coordinates
	Vector3 v2p(Vector3& v) {
		return v;
	}
	
	//math that converts physical coordinates to 3 rigid transformations (position & rotation)
	tri_rigid math(Vector3& p) {
		tri_rigid vv;
		return vv;
	}

public:
	void run(Mat frame) {
		if (frame.empty()) {
			return;
		}
		
		Mat outputFrame = frame.clone();
		vector<Rect> newFaces = frame_proc.process(frame);
		if (newFaces.size() >= 1) {
			faces = Point2i(newFaces[0].width/2 + newFaces[0].x, newFaces[0].height/2 + newFaces[0].y);
		}

		//display
		if (newFaces.size() <= 1) {
		    circle(outputFrame, faces, 5, Scalar(0, 255, 0));
		}
		else if (newFaces.size() == 2) {
			if (false) { //true for higher/lower, false for left/right
				Point2i higher;
				Point2i lower;
				if (newFaces[0].height/2 + newFaces[0].y < newFaces[1].height/2 + newFaces[1].y) {
					higher = Point2i(newFaces[0].width/2 + newFaces[0].x, newFaces[0].height/2 + newFaces[0].y);
					lower = Point2i(newFaces[1].width/2 + newFaces[1].x, newFaces[1].height/2 + newFaces[1].y);
				}
				else {
					higher = Point2i(newFaces[1].width/2 + newFaces[1].x, newFaces[1].height/2 + newFaces[1].y);
					lower = Point2i(newFaces[0].width/2 + newFaces[0].x, newFaces[0].height/2 + newFaces[0].y);
				}
				if (choose == 0) {
					faces = lower;
					printf("lower\n");
				}
				else {
					faces = higher;
					printf("higher\n");
				}
				circle(outputFrame, higher, 5, Scalar(255, 0, 0));
				circle(outputFrame, lower, 5, Scalar(0, 0, 255));
			}
			else {
				Point2i left;
				Point2i right;
				if (newFaces[0].width/2 + newFaces[0].x < newFaces[1].width/2 + newFaces[1].x) {
					left = Point2i(newFaces[0].width/2 + newFaces[0].x, newFaces[0].height/2 + newFaces[0].y);
					right = Point2i(newFaces[1].width/2 + newFaces[1].x, newFaces[1].height/2 + newFaces[1].y);
				}
				else {
					left = Point2i(newFaces[1].width/2 + newFaces[1].x, newFaces[1].height/2 + newFaces[1].y);
					right = Point2i(newFaces[0].width/2 + newFaces[0].x, newFaces[0].height/2 + newFaces[0].y);
				}
				if (choose == 0) {
					faces = left;
					printf("left: %d\n", choose);
				}
				else {
					faces = right;
					printf("right: %d\n", choose);
				}
				circle(outputFrame, left, 5, Scalar(255, 0, 0));
				circle(outputFrame, right, 5, Scalar(0, 0, 255));
			}
		}

		//UDP
		double nx = (2.0 * faces.x / outputFrame.cols) - 1;
		double ny = 1 - (2.0 * faces.y / outputFrame.rows);

		nx = -nx * nx * nx - 0.561516;
		ny = (ny * ny * ny) - 0.296296;

		nx *= 2;
		ny *= 2;

		

		string send = to_string(nx) + "," + to_string(ny);
		//printf("%s\n", send.c_str());
		bcast.send(send.c_str());

		//frame
		imshow("cam", outputFrame);
		
		if (waitKey(25) == 120) {
			processing = false;
			string send = "0,0";
			printf("%s\n", send.c_str());
			bcast.send(send.c_str());
			destroyAllWindows();
			shutdown();
		}
	}
	
	SegbotProcessor(NodeHandle& nh) : it(nh) {
		processing = true;
		//vis_sub = nh.subscribe("/image_raw", 1, &SegbotProcessor::vis_callback, this);
		//image_sub = it.subscribe("/image_raw", 1, &SegbotProcessor::callback, this);
	}
};

int main(int argc, char** argv) {
	init(argc, argv, "friendly_faces_runner");

	printf("CXX Standard:   %li\n", __cplusplus);
	printf("OpenCV Version: %s\n", CV_VERSION);

	namedWindow("cam");
	
	startWindowThread();

	NodeHandle nh;

	SegbotProcessor sp(nh);
	
	srand (time(NULL));
	choose = rand() % 2;
	
	while (ok()) {
		Mat frame;
		cap.read(frame);
		sp.run(frame);
		spinOnce();
	}

	return 0;
}
