#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "../tracking.hpp"
#include <iostream>
using namespace std;
int main(int argc, char* argv[])
{
	cv::VideoCapture cap;
	cap.open(0);
	wong::calibration_camera camera("../images/out_camera_data.yml");
	cv::Mat frame,out;
	for (;;){
		cap >> frame;
		camera.remap_image(frame,out);
		cv::imshow("src",out);
		if(cv::waitKey(5)>=0)
			break;
	}
	
}
