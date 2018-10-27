//
// Created by dasuda on 18-10-25.
//

#ifndef TRACKING_CAMERA_HPP
#define TRACKING_CAMERA_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

/*
***************测试代码****************
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include "utils/Camera.hpp"

using namespace std;

int main()
{
    calibration_camera camera("-----/out_camera_data.yml");
    cv::VideoCapture cap(0);
    cv::Mat frame,remape_frame;
    while(true){
        cap>>frame;

        cv::imshow("src",frame);

        camera.remap_image(frame,remape_frame);

        cv::imshow("remape",remape_frame);

        if(cv::waitKey(5)>0)
        {
            break;
        }
    }
}

 */

namespace wong{
    class calibration_camera{
        private:
            cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
            cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
            cv::Size imageSize;
        public:
            //构造函数读取标定的参数文件
            calibration_camera(std::string filename){
                fprintf(stderr, "loadParams.\n");
                cv::FileStorage fs(filename, cv::FileStorage::READ);
                if (!fs.isOpened()) {
                    fprintf(stderr, "%s:%d:loadParams falied. 'camera.yml' does not exist\n", __FILE__, __LINE__);
                }
                char buf1[100];
                sprintf(buf1, "camera_matrix");
                char buf2[100];
                sprintf(buf2, "distortion_coefficients");
                fs[buf1] >> cameraMatrix;
                fs[buf2] >> distCoeffs;
                fs.release();
            }
            //对摄像头读取的每一帧进行校正
            void remap_image(cv::Mat &src,cv::Mat &out){
                cv::Mat view, rview, map1, map2;
                imageSize = src.size();
                cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
                                            imageSize, CV_16SC2, map1, map2);
                cv::remap(src, out, map1, map2, cv::INTER_LINEAR);
            }
    };
}
#endif //TRACKING_CAMERA_HPP
