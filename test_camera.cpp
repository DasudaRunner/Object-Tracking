//#include <fstream>
//#include <sstream>
//#include <algorithm>
//#include "opencv2/core.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui.hpp"
//#include "tracking.hpp"
//#include <Eigen/Dense>
//#include <opencv2/opencv.hpp>
//#include <string>
//#include <iostream>
//#include <vector>
//#include "tracking.hpp"
//#include "KCFlib/kcftracker.hpp"
//using namespace std;
//using Eigen::MatrixXd;
//cv::Rect selection;
//bool selectObject = false;
//cv::Point origin;
//cv::Mat frame,img,out,rect_image;
//bool flag = false;
//bool tpause = false;
//
//static void onMouse(int event, int x, int y, int, void*)
//{
//    if (!flag&&tpause)
//    {
//        if (selectObject)
//        {
//            selection.x = MIN(x, origin.x);
//            selection.y = MIN(y, origin.y);
//            selection.width = std::abs(x - origin.x);
//            selection.height = std::abs(y - origin.y);
//            selection &= cv::Rect(0, 0, frame.cols, frame.rows);
//        }
//
//        switch (event)
//        {
//            case CV_EVENT_LBUTTONDOWN:
//                origin = cv::Point(x, y);
//                selection = cv::Rect(x, y, 0, 0);
//                selectObject = true;
//                break;
//            case CV_EVENT_LBUTTONUP:
//                selectObject = false;
//                if (selection.width > 0 && selection.height > 0)
//                    cv::rectangle(img, selection, cv::Scalar(0, 255, 255));
//                flag = true;
//                break;
//        }
//    }
//}
//
//int main(int argc, char* argv[])
//{
//
//    haibo::MeanShift tracker;
//    cv::VideoCapture cap;
//
//    cap.open(0);
//    MatrixXd A(2,2);
//    haibo::KF kf;
//    kf.initKF();
//
//    cv::namedWindow("Image", 2);
//    cv::setMouseCallback("Image", onMouse, 0);
//    haibo::calibration_camera camera("/home/irim/david/camera/cpp_src/test/out_camera_data.yml");
//
//    int nFrames = 0;
//    for (;;)
//    {
//        if (!tpause)
//        {
//            cap >> out;
//            camera.remap_image(out,frame);
//            if (frame.empty())
//                break;
//            frame.copyTo(img);
//            if (flag)
//            {
////                result = tracker.update(img);
//                cout<<"-----------------------------------------"<<nFrames<<endl;
//                tracker.tracking_withKernel(img);
////                rectangle(img, cv::Point(result.x, result.y), cv::Point(result.x + result.width, result.y + result.height), cv::Scalar(0, 255, 255), 1, 8);
//                nFrames++;
////                cout<<"tracking: "<<nFrames<<endl;
//            }
//        }
//        else
//        {
//            if (flag&&nFrames == 0)
//            {
//                cout<<"into init"<<endl;
////                tracker.init(selection, img);//Rect(xMin, yMin, width, height)
//                tracker.initTracker_withKernel(img,selection);
//                cout<<"init ok"<<endl;
//            }
//        }
//
//        cv::imshow("Image", img);
//        char c = (char)cv::waitKey(10);
//        if (c == 27)
//            break;
//        switch (c)
//        {
//            case 'p':
//                tpause = !tpause;
//                break;
//            case 'r':
//                flag = false;
//                nFrames = 0;
//                frame.copyTo(img);
//                imshow("Image", img);
//                break;
//        }
//    }
//
//    return 0;
//
//}

//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include <iostream>
//#include "opencv2/core.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui.hpp"
//#include <opencv2/opencv.hpp>
//#include "cv-helpers.hpp"
//using namespace std;
//int main(int argc, char * argv[])
//{
////// Create a Pipeline - this serves as a top-level API for streaming and processing frames
//    rs2::pipeline p;
//    rs2::colorizer colorize;
//
//    colorize.set_option(RS2_OPTION_COLOR_SCHEME,2);
//// Configure and start the pipeline
//    rs2::pipeline_profile profile = p.start();
//
//    rs2::align align_to(RS2_STREAM_COLOR);
//
//    auto gen_element = [](int erosion_size)
//    {
//        return cv::getStructuringElement(cv::MORPH_RECT,
//                                         cv::Size(erosion_size + 1, erosion_size + 1),
//                                         cv::Point(erosion_size, erosion_size));
//    };
//
//    const int erosion_size = 3;
//    auto erode_less = gen_element(erosion_size);
//    auto erode_more = gen_element(erosion_size * 2);
//    auto create_mask_from_depth = [&](cv::Mat& depth, int thresh, cv::ThresholdTypes type)
//    {
//        cv::threshold(depth, depth, thresh, 255, type);
//        cv::dilate(depth, depth, erode_less);
//        cv::erode(depth, depth, erode_more);
//    };
//
//    while (true)
//    {
//        // Block program until frames arrive
//        rs2::frameset frames = p.wait_for_frames();
//
//        rs2::frameset aligned_set = align_to.process(frames);
//        rs2::frame depth = aligned_set.get_depth_frame();
//        auto color_mat = frame_to_mat(aligned_set.get_color_frame());
//
//        rs2::frame bw_depth = colorize(depth);
//
//        auto depth_mat = frame_to_mat(bw_depth);
//        cv::cvtColor(depth_mat, depth_mat, CV_BGR2GRAY);
//        create_mask_from_depth(depth_mat, 180, cv::THRESH_BINARY);
//
//        auto far = frame_to_mat(bw_depth);
//        cv::cvtColor(far, far, CV_BGR2GRAY);
//        far.setTo(255, far == 0);
//        create_mask_from_depth(far, 100, cv::THRESH_BINARY_INV);
//
//        cv::Mat mask;
//        mask.create(depth_mat.size(), CV_8UC1);
//        mask.setTo(cv::Scalar::all(cv::GC_BGD));
//        mask.setTo(cv::GC_PR_BGD, far == 0);
//        mask.setTo(cv::GC_FGD, depth_mat == 255);
//
//        cv::Mat bgModel, fgModel;
//        cv::grabCut(color_mat, mask, cv::Rect(), bgModel, fgModel, 1, cv::GC_INIT_WITH_MASK);
//        cv::Mat3b foreground = cv::Mat3b::zeros(color_mat.rows, color_mat.cols);
//        color_mat.copyTo(foreground, (mask == cv::GC_FGD) | (mask == cv::GC_PR_FGD));
//
////        cout<<depth_image_cv.size<<endl;
//        cv::imshow("color",color_mat);
//        cv::imshow("depth",depth_mat);
//
//        cv::imshow("foreground",foreground);
//
//
//        cv::waitKey(2);
//
////        break;
//
//    }
//
//}

#include <fstream>
#include <sstream>
#include <algorithm>
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "tracking.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <omp.h>
#include <typeinfo>
using namespace std;
using Eigen::MatrixXd;
using namespace Eigen;
cv::Rect selection;
cv::Point origin;
cv::Mat frame;
cv::Mat hist;
int flag=0;
//
//cv::Mat srcHSV,H_hist,H_channel;
//std::vector<cv::Mat> srcChannels;
//cv::MatND backProjection;
//int histBin = 180; //直方图中bin的个数
////    int allBins = histBin*histBin*histBin;
//float range[2] = {0,float(histBin)};
//const float * histRange = {range};
//haibo::utils util;
//static void onMouse(int event, int x, int y, int, void*)
//{
//    switch (event)
//    {
//        case CV_EVENT_LBUTTONDOWN:
//            origin = cv::Point(x, y);
//            selection = cv::Rect(x, y, 0, 0);
//            break;
//        case CV_EVENT_LBUTTONUP:
//            selection.x = MIN(x, origin.x);
//            selection.y = MIN(y, origin.y);
//            selection.width = std::abs(x - origin.x);
//            selection.height = std::abs(y - origin.y);
//            selection &= cv::Rect(0, 0, frame.cols, frame.rows);
//            if (selection.width > 0 && selection.height > 0)
//                cv::rectangle(frame, selection, cv::Scalar(0, 255, 255));
//
//////            cv::cvtColor(frame,srcHSV,CV_BGR2HSV);
////            cv::cvtColor(frame,srcHSV,CV_BGR2YCrCb);
////            cv::split(srcHSV,srcChannels);
////
//////            H_channel = srcChannels.at(0);
////            H_channel = srcChannels.at(1);
////            cv::Mat roi_frame = H_channel(selection);
////
////            cv::calcHist(&roi_frame,1,0,cv::Mat(),H_hist,1,&histBin,&histRange,true,false);
////
////            cv::normalize(H_hist, H_hist, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
//            util.prepareBackProject(frame,selection,0,H_hist,histBin,true);
//            flag=1;
//            break;
//    }
//}
double mouse_point[2];
static void onMouse(int event, int x, int y, int, void*)
{
    if(event==CV_EVENT_MOUSEMOVE){
        mouse_point[0]=x;
        mouse_point[1]=y;
    }

}

int main(int argc, char * argv[])
{
//    cv::VideoCapture cap;
//    cap.open(0);
//
//    cv::namedWindow("Image");
//    cv::setMouseCallback("Image", onMouse, 0);
//
//    haibo::KF kf;
//    cv::Point pre_point;
//    while(true){
//        cap>>frame;
//        pre_point = kf.predict(mouse_point,2);
//
//        cv::circle(frame,cv::Point(int(mouse_point[0]),int(mouse_point[1])),5,cv::Scalar(0,255,0),3);
//        cv::circle(frame,pre_point,5,cv::Scalar(255,0,0),3);
//
//        cv::imshow("Image",frame);
//        if(cv::waitKey(5)>0)
//        {
//            break;
//        }
//
//    }
//    double sum=0,sum_f;
//#pragma omp parallel for reduction(+:sum,sum_f)
//    for (int i = 0; i < 10; ++i) {
//        sum+=i;
//        sum_f+=2*i;
//    }
//    cout<<sum<<endl;
//    cout<<sum_f<<endl;

    std::vector<cv::Point2d> a;
    a.push_back(cv::Point2d(35.3,10.98));
    a.push_back(cv::Point2d(29.7,11.13));
    a.push_back(cv::Point2d(30.8,12.51));
    a.push_back(cv::Point2d(58.8,8.4));
    a.push_back(cv::Point2d(61.4,9.27));

    a.push_back(cv::Point2d(71.3,8.73));
    a.push_back(cv::Point2d(74.4,6.36));
    a.push_back(cv::Point2d(76.6,8.5));
    a.push_back(cv::Point2d(70.7,7.82));
    a.push_back(cv::Point2d(57.5,9.17));

    a.push_back(cv::Point2d(46.4,8.24));
    a.push_back(cv::Point2d(28.9,12.19));
    a.push_back(cv::Point2d(28.1,11.88));
    a.push_back(cv::Point2d(39.1,9.57));
    a.push_back(cv::Point2d(46.8,10.94));

    a.push_back(cv::Point2d(48.5,9.58));
    a.push_back(cv::Point2d(59.3,10.09));
    a.push_back(cv::Point2d(70.0,8.11));
    a.push_back(cv::Point2d(70.0,6.83));
    a.push_back(cv::Point2d(74.5,8.88));

    a.push_back(cv::Point2d(72.1,7.68));
    a.push_back(cv::Point2d(58.1,8.47));
    a.push_back(cv::Point2d(44.6,8.86));
    a.push_back(cv::Point2d(33.4,10.38));
    a.push_back(cv::Point2d(28.6,11.08));

    //
//    a.push_back(cv::Point2d(35.4,10.95));
//    a.push_back(cv::Point2d(29.8,11.12));
//    a.push_back(cv::Point2d(30.7,12.58));
//    a.push_back(cv::Point2d(58.6,8.9));
//    a.push_back(cv::Point2d(61.9,9.25));
//
//    a.push_back(cv::Point2d(71.9,8.72));
//    a.push_back(cv::Point2d(74.4,6.38));
//    a.push_back(cv::Point2d(76.6,8.5));
//    a.push_back(cv::Point2d(70.7,7.83));
//    a.push_back(cv::Point2d(57.5,9.14));
//
//    a.push_back(cv::Point2d(46.7,8.28));
//    a.push_back(cv::Point2d(28.7,12.15));
//    a.push_back(cv::Point2d(28.4,11.85));
//    a.push_back(cv::Point2d(39.4,9.55));
//    a.push_back(cv::Point2d(46.1,10.97));
//
//    a.push_back(cv::Point2d(48.7,9.55));
//    a.push_back(cv::Point2d(59.8,10.02));
//    a.push_back(cv::Point2d(70.2,8.16));
//    a.push_back(cv::Point2d(70.4,6.85));
//    a.push_back(cv::Point2d(74.5,8.89));
//
//    a.push_back(cv::Point2d(72.1,7.65));
//    a.push_back(cv::Point2d(58.8,8.41));
//    a.push_back(cv::Point2d(44.7,8.85));
//    a.push_back(cv::Point2d(33.4,10.37));
//    a.push_back(cv::Point2d(28.6,11.08));

    wong::utils util;
    wong::RANSAC line;

    cv::Mat image = cv::Mat::zeros(480,640,CV_8UC3);
    image.setTo(cv::Scalar(100,0,0));

    std::vector<cv::Point2d> points;
    line.getTestSamples(points,1000);
    for (int i = 0; i < points.size(); ++i) {
        cv::circle(image, points[i], 1, cv::Scalar(0, 0, 255), 1, 8, 0);
    }

    double K[2];
    for (int j = 0; j < 10; ++j) {
        util.clock_start();
        line.fit(points,K);
        util.print_time_us("01");
    }

//    cv::line(image,cv::Point(0,-K[1]/K[0]),cv::Point(1800,(1800-K[1])/K[0]),cv::Scalar(0,255,0),3);
//    cout<<K[0]<<endl;
//    cout<<K[1]<<endl;
    cout<<points.size()<<endl;
//    cv::imshow("image",image);
//    cv::waitKey(0);

}
