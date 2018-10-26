//
// Created by irim on 18-8-2.
//

#ifndef TRACKING_HPP
#define TRACKING_HPP

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <omp.h>
#include <time.h>

namespace wong {

    using namespace std;
    using Eigen::MatrixXd;
    using namespace Eigen;

    class utils{
        private:
            clock_t last_time;

        public:

            static void prepareBackProject(cv::Mat& image,const cv::Rect selections,const int mode,cv::Mat& hist,const int bins,bool enable_normalize){
                assert(mode>=0 && mode<=1);

                cv::Mat color_model,single_channel;
                std::vector<cv::Mat> split_src;
                float range[2] = {0,float(bins)};
                const float * histRange = {range};
                cv::Mat roi_image = image(selections);

                if(mode==0){
                    cv::cvtColor(roi_image,color_model,CV_BGR2HSV);
                    cv::split(color_model,split_src);
                    single_channel = split_src.at(0);
                }else {
                    cv::cvtColor(roi_image, color_model, CV_BGR2YCrCb);
                    cv::split(color_model, split_src);
                    single_channel = split_src.at(1);
                }

                cv::calcHist(&single_channel,1,0,cv::Mat(),hist,1,&bins,&histRange,true,false);

                if(enable_normalize){
                    cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
                }
            }

            static void getBackProject(const cv::Mat& src, const cv::Mat& hist,const int bins, cv::Mat& backProjectImage, const int mode){
                assert(mode>=0 && mode<=1);

                cv::Mat color_model,single_channel;
                std::vector<cv::Mat> split_src;
                float range[2] = {0,float(bins)};
                const float * histRange = {range};

                if(mode==0){
                    cv::cvtColor(src,color_model,CV_BGR2HSV);
                    cv::split(color_model,split_src);
                    single_channel = split_src.at(0);
                }else {
                    cv::cvtColor(src, color_model, CV_BGR2YCrCb);
                    cv::split(color_model, split_src);
                    single_channel = split_src.at(1);
                }

                cv::calcBackProject(&single_channel,1,0,hist,backProjectImage,&histRange,1,true);
            }

            template <typename T1,typename T2>
            static void ArrayToMatrix(T1& array,T2& matrix,int h,int w){
                matrix = Map<T2>(array,h,w);
            }

            //按列展开
            template <typename T1,typename T2>
            static void MatrixToArray(T1& matrix,T2& array){
                array = matrix.data();
            }

//            static void PointsToMatrix(std::vector<cv::Point2d>& points,MatrixXd matrix){
//                unsigned long n_points;
//                n_points = points.size();
//                for (int i = 0; i < n_points; ++i) {
//
//                }
//            }

            void clock_start(){
                last_time = clock();
            }

            void print_time_us(string ID){
                cout<<"Total time["<<ID<<"]: "<<((double)((clock()-last_time)*1000000.0)/CLOCKS_PER_SEC)<<"us"<<endl;
            }
            void print_time_ms(string ID){
                cout<<"Total time["<<ID<<"]: "<<((double)((clock()-last_time)*1000.0)/CLOCKS_PER_SEC)<<"ms"<<endl;
            }

            double getTime_ms(){
                return ((double)((clock()-last_time)*1000.0)/CLOCKS_PER_SEC);
            }
            double getTime_us(){
                return ((double)((clock()-last_time)*1000000.0)/CLOCKS_PER_SEC);
            }

    };

    class MeanShift{

        private:
            //画出矩形框
            void drawRect(cv::Mat& src,cv::Rect rec);
            //带有安全检查的方框移动
            cv::Point2d safeUpdate(cv::Point2d ori,cv::Point2d update,cv::Rect track_window,const int img_width,const int img_height);

            //计算直方图 返回为hist
//            int histBin = 16; //直方图中bin的个数
//            int allBins = histBin*histBin*histBin;
//            float range[2] = {0,float(histBin)};
//            const float * histRange = {range};
//
//            void getHist(const cv::Mat& src,cv::Mat& hist){
//                cv::calcHist(&src,1,0,cv::Mat(),hist,1,&histBin,&histRange,true,false);
//            }
//
//            //计算直方图的相关性
//            double histCorrelation(cv::Mat& src,cv::Mat& pattern){
//                cv::Mat srcHSV,patternHSV;
//                cv::cvtColor(src,srcHSV,CV_BGR2HSV);
//                cv::cvtColor(pattern,patternHSV,CV_BGR2HSV);
//
//                std::vector<cv::Mat> srcChannels,patternChannels;
//                cv::split(srcHSV,srcChannels);
//                cv::split(patternHSV,patternChannels);
//
//                cv::Mat srcHist,patternHist;
//                getHist(srcChannels[0],srcHist);
//                getHist(patternChannels[0],patternHist);
//
//                cv::normalize(srcHist,srcHist,0,1,cv::NORM_MINMAX);
//                cv::normalize(patternHist,patternHist,0,1,cv::NORM_MINMAX);
//
//                /*
//                 * CV_COMP_CORREL:相关性性 1相关性最大
//                 * CV_COMP_CHISQR：卡方法 0相关性最大
//                 * CV_COMP_INTERSECT：交集法 值越大相关性越大
//                 * CV_COMP_BHATTACHARYYA: BHATTACHARYYA距离法 0相关性最大
//                 */
//                return cv::compareHist(srcHist,patternHist,CV_COMP_CORREL);
//            }

            const int bin = 16;
            const int allBins = bin*bin*bin;

    //------------------tracking_withoutKernel--------------------
            int t_x,t_y,t_w,t_h; //追踪框的参数
            int band_width; //带宽

            double  *m_wei = (double* )malloc(sizeof(double)*50*50); //权值矩阵
            double C_ = 0.0; //归一化系数
            double  *hist1 = (double* )malloc(sizeof(double)*bin*bin*bin); //模板的直方图
            double  *hist2 = (double* )malloc(sizeof(double)*bin*bin*bin); //候选区的直方图
            double *w = (double* )malloc(sizeof(double)*bin*bin*bin);//权值
    //------------------tracking_withoutKernel--------------------

            //计算巴氏相关系数
            double Bhattacharyya(cv::Mat& Pu,cv::Mat& qu);

            //核函数相关的参数
            double *K;
            double *Gx;
            double *Gy;

            void EpanechnikovKernelF(const int H,const int W);

            //获得概率密度矩阵 计算模板和匹配区域的特征
            void getPu(cv::Mat& roi_image,cv::Mat& Mat_Pu);

            //权值矩阵
            void getWi(cv::Mat& roi_image,cv::Mat& pu, cv::Mat& qu ,cv::Mat& Mat_Wi);

            //更新Yi,追踪框的新位置
            cv::Point2d updateYi(const cv::Mat& roi_image,cv::Mat& Mat_Wi);

            cv::Rect trackWindow;//保存追踪框
            cv::Point2d point_Y0,point_Y1;//保存追踪框的原始点和候选框的点
            cv::Mat qu_hist,pu_hist; //追踪框和候选框的特征

            cv::Mat Mat_wi; //权值矩阵


        public:

            void initTracker_withoutKernel(const cv::Mat & src,cv::Rect tracking_window);

            //跟踪函数
            void tracking_withoutKernel(cv::Mat& frame);

            void initTracker_withKernel(const cv::Mat& src,cv::Rect rec);

            const double update_step = 1.4; //更新的步长
            double lastCorrelation=0.0;

            bool tracking_withKernel(cv::Mat& src);
    };



    class KF{

        private:
            const int dim_status = 4; //系统状态的维数

            const int dim_measure = 2; //系统观测的维数

    //        const int dim_control = 1; //系统控制输入的维数
            const double time_step=0.1; //时间周期

            MatrixXd Xhat = MatrixXd::Constant(dim_status,1,0); //k-1时刻系统状态的最优估计 ok  n*1
            MatrixXd Phat = MatrixXd::Constant(dim_status,dim_status,0); //k-1时刻最优估计的协方差 ok n*n

            MatrixXd Xhat_minus = MatrixXd::Constant(dim_status,1,0); //k-1时刻的预测值 ok n*1
            MatrixXd Phat_minus = MatrixXd::Constant(dim_status,dim_status,0); //k-1时刻最优估计的协方差 ok n*n

            MatrixXd A = MatrixXd::Constant(dim_status,dim_status,0); //上一时刻的最优估计 ok n*n
    //        MatrixXd B = MatrixXd::Constant(dim_control,dim_control,0); //控制->状态空间转换矩阵
            MatrixXd H = MatrixXd::Constant(dim_measure,dim_status,0); //测量矩阵 n*m

            MatrixXd Q = MatrixXd::Identity(dim_status,dim_status); //系统预测噪声 ok n*n
            MatrixXd R = MatrixXd::Identity(dim_measure,dim_measure); //系统测量噪声 ok m*m

            MatrixXd K = MatrixXd::Constant(dim_status,dim_measure,0);; //卡尔曼增益 ok

            MatrixXd I = MatrixXd::Identity(dim_status,dim_status); //对角矩阵 ok

        public:
            KF();
            cv::Point predict(double* measurement,int len);
    };

    class RANSAC{

        private:

            inline double getDistanceP2L(cv::Point2d a,double* K){
                return abs(K[0]*a.x+K[1]*a.y+K[2])/(sqrt(K[0]*K[0]+K[1]*K[1]));
            }

            void getDistanceP2L_Matrix(MatrixXd& points,double* K,MatrixXd& dis);

            template <typename T>
            inline double getDistanceP2P(const T ax, const T ay,const T bx,const T by){
                return sqrt(pow(ax-bx,2)+pow(ay-by,2));
            }

        public:
            RANSAC(){
                srand((unsigned)time(NULL));
            }
            const double inlier_error=2.99;//判断时候是否为内点的阈值
            //最小二乘法拟合直线
            void LeastSquares(std::vector<cv::Point2d> points,double* result);
            //两点确定一条直线
            void getLine(double ax,double ay,double bx, double by,double* K);

            //随机获得两个点
            bool getSamples(const int max,int* sam_id);

            //判断两个点的距离
            void fit(std::vector<cv::Point2d> points,double K_coefficient[2]);

            void getTestSamples(std::vector<cv::Point2d>& points,const int max_iter);

    };

}

#endif //TRACKING_HPP
