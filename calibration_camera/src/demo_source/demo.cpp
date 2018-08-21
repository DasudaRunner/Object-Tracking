#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "../../../tracking.hpp"
#include <iostream>
using namespace std;
int main(int argc, char* argv[])
{
	cv::VideoCapture cap;
	cap.open(0);
	wong::calibration_camera camera(argv[1]);
	cv::Mat frame,out;
	cout<<"Press any key to exit the program !"<<endl;
	for (;;){
		cap >> frame;
		camera.remap_image(frame,out);
		cv::imshow("src",frame);
		cv::imshow("corrected",out);
		if(cv::waitKey(5)>=0)
			break;
	}
	
}
