# Object-Tracking<br>
[![contributions welcome](https://img.shields.io/badge/contributions-welcome-brightgreen.svg?style=flat)](https://github.com/dwyl/esta/issues)<br>
## This object tracking library is optimized by using Eigen3 and openMP
- &emsp;&emsp;*Author: Haibo Wang*<br>
- &emsp;&emsp;*Email: dasuda2015@163.com*
- &emsp;&emsp;*This is an open source Object-Tracking library, The development language is C++, The classic and mainstream object tracking algorithms and related algorithms are reproduced(For example, RANSAC, least squares and so on). Of course, these are optimized by Eigen3 and openMP.*
## Hardware platform:
- &emsp;&emsp;*CPU: intel i7 6700K*<br>
- &emsp;&emsp;*Memory:8G x 2 DDR4*
## Performance:
- **RANSAC**: Linear Fitting, 600 sample point, 500 iteration, Time consuming:*100us *<br>
- **Kalman Filter**: Two dimensional coordinate tracking, System state variable is [x,y,dx,dy],prediction+update,Mean time consuming:*8us*<br>
- **MeanShift**:using kernel function,refactoring with Eigen3 and openMP.
## 1.Camera calibration(If you want to do something good, you mast sharpen it firstly)<br>
- Use opencv's own calibration tool to calculate the internal parameters of the camera, the chessboard picture has been given, fool-style operation, take a look.
### Location:<br>
		DasudaRunner/Object-Tracking/calibration_camera
### Requirments:<br>
		opencv: >=3.3.0 or >=3.0.0
		python: >=2.7
		python-opencv:
### Quick Start:<br>
#### &emsp;Step 1. Connecte Camera, get images containing checkerboard.
		./src$ python getChecker.py # press 's' to save image
#### &emsp;Step 2. I encapsulated these commands into a script file, and you can  run the following command.
		./src$ sudo sh calibration_camera.sh
&emsp;&emsp;&emsp;Now you can get out_camera_data.yml , this is a file that contains your camera's internal reference.
#### &emsp;Step 3. You can run ./demo, and get corrected images.
		./src$ ./demo
&emsp;&emsp;&emsp;Actually, we call tracking.hpp's interface remap_image().<br>
```cpp
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
		wong::calibration_camera camera("../out_camera_data.yml");
		cv::Mat frame,out;
		cout<<"Press any key to exit the program !"<<endl;
		for (;;){
			cap >> frame;
			camera.remap_image(frame,out);
			cv::imshow("src",out);
			if(cv::waitKey(5)>=0)
				break;
		}
	}
```
