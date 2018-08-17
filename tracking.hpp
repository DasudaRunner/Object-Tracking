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
            void drawRect(cv::Mat& src,cv::Rect rec){
                cv::rectangle(src,rec.tl(),rec.br(),cv::Scalar(0,255,0),2);
            }

            //带有安全检查的方框移动
            cv::Point2d safeUpdate(cv::Point2d ori,cv::Point2d update,cv::Rect track_window,const int img_width,const int img_height){
                ori.x += update.x* update_step;
                ori.y += update.y* update_step;

                if(round(ori.x) +track_window.width >= img_width){
                    ori.x = img_width - track_window.width-1;
                }else if(ori.x < 0){
                    ori.x = 0;
                }

                if(round(ori.y)+track_window.height>=img_height){
                    ori.y = img_height - track_window.height-1;
                }else if(ori.y < 0){
                    ori.y=0;
                }

                return ori;
            }

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
            double Bhattacharyya(cv::Mat& Pu,cv::Mat& qu){  //范围0-1
                assert(Pu.rows==1 && qu.rows==1); //保证是行向量
                assert(Pu.rows==qu.rows); //保证尺寸是一样的

                double Bhatta=0;
                const double* data_Pu = Pu.ptr<double>(0);
                const double* data_q = qu.ptr<double>(0);
                for (int i = 0; i < Pu.cols; ++i) {
                    Bhatta += sqrt(data_Pu[i]*data_q[i]);
                }

                return Bhatta;
            }

            //核函数相关的参数
            double *K;
            double *Gx;
            double *Gy;
            void EpanechnikovKernelF(const int H,const int W){

                int radWidth = (int)sqrt(pow((double)W,2)+pow((double)H,2));
                int radHeight = radWidth;
                double pi = 3.141596;
                int d = 2;//当前维度
                double Cd = pi; //d维空间下单位球体的体积
                double kx=0,ky=0; //kernel funciton 的梯度

                K = new double[W*H];
                Gx = new double[W*H];
                Gy = new double[W*H];

                #pragma omp parallel for
                for (int i = 0; i < W*H; ++i) { //初始化
                    K[i]=0;
                    Gx[i]=0;
                    Gy[i]=0;
                }

                //计算函数值 保存在K中，但它只与位置有关
                for (int i = 0; i < H; ++i) {
                    for (int j = 0; j < W; ++j) {
                        kx = pow(double(i-0.5*H)/(double)radWidth,2);
                        ky = pow(double(j-0.5*W)/(double)radHeight,2);
                        if((kx+ky)<1){
                            K[i*W+j] = (0.5*(d+2)*(1-(kx+ky)))/Cd;
                        }else{
                            K[i*W+j] = 0;
                        }
                    }
                }

                //接下来计算密度
                for (int x = 0; x < W; ++x) {
                    for (int y = 0; y < H; ++y) {
                        //x方向的密度
                        if(x==0)
                            Gx[y*W+x] = -(K[y*W+x+1]-K[y*W+x]);
                        else if(x==(W-1))
                            Gx[y*W+x] = -(K[y*W+x]-K[y*W+x-1]);
                        else
                            Gx[y*W+x] = -(K[y*W+x+1]-K[y*W+x-1])/2.0;
                        //y方向的密度
                        if(y==0)
                            Gy[y*W+x] = -(K[(y+1)*W+x]-K[y*W+x]);
                        else if(y==(H-1))
                            Gy[y*W+x] = -(K[y*W+x]-K[(y-1)*W+x]);
                        else
                            Gy[y*W+x] = -(K[(y+1)*W+x]-K[(y-1)*W+x])/2.0;
                    }
                }

            }

            //获得概率密度矩阵 计算模板和匹配区域的特征
            void getPu(cv::Mat& roi_image,cv::Mat& Mat_Pu){
                double *pu = new double[allBins];

                #pragma omp parallel for
                for (int i = 0; i < allBins; ++i) {
                    pu[i]=0.0;
                }

                //求出了qu特征矩阵
                int bxi=0;
                double sum_pu = 0.0;
                for (int i = 0; i < roi_image.rows; ++i) {
                    const uchar* data = roi_image.ptr<uchar>(i);
                    for (int j = 0; j < roi_image.cols; j+=3) {
                        bxi = ((int)data[j+2]>>5)*bin*bin +
                                ((int)data[j+1]>>5)*bin +
                                ((int)data[j]>>5);
                        pu[bxi] += K[i*roi_image.cols+j];//有可能出错 它类似一个直方图
                        sum_pu += K[i*roi_image.cols+j];
                    }
                }

                //归一化的pu
                double sumQu=0;
                for (int i = 0; i < allBins; ++i) {
                    pu[i] /= sum_pu;
                    sumQu += pu[i]; //这个sumQu不是1吗？
                }

                cv::Mat pu_mat(1,allBins,CV_64FC1,pu);
                Mat_Pu.release();
                pu_mat.copyTo(Mat_Pu);
            }

            //权值矩阵
            void getWi(cv::Mat& roi_image,cv::Mat& pu, cv::Mat& qu ,cv::Mat& Mat_Wi){

                double* Wi = new double[roi_image.rows*roi_image.cols];

                #pragma omp parallel for
                for (int i = 0; i < roi_image.rows*roi_image.cols; ++i) {
                    Wi[i]=0.0;
                }

                //计算得到权值矩阵
                int bxi = 0;
                const double* data_pu = pu.ptr<double>(0);
                const double* data_qu = qu.ptr<double>(0);
//                #pragma omp parallel for
                for (int i = 0; i < roi_image.rows; ++i) {
                    const uchar* data_image = roi_image.ptr<uchar>(i);
                    for (int j = 0; j < roi_image.cols; j+=3) {
                        bxi = ((int)data_image[j+2]>>5)*bin*bin+
                              ((int)data_image[j+1]>>5)*bin+
                              ((int)data_image[j]>>5);
                        if(data_pu[bxi] != 0){
                            Wi[i*roi_image.cols+j] = sqrt(data_qu[bxi]/data_pu[bxi]);
                        }else{
                            Wi[i*roi_image.cols+j] = 0;
                        }
                    }
                }

                cv::Mat W_Mat(roi_image.rows,roi_image.cols,CV_64FC1,Wi);
                Mat_Wi.release();
                W_Mat.copyTo(Mat_Wi);
            }

            //更新Yi,追踪框的新位置
            cv::Point2d updateYi(const cv::Mat& roi_image,cv::Mat& Mat_Wi){

                double wi,g_x=0,g_y=0,g_xy=0,sum_wig=0;
                double yi_x=0,yi_y=0;
                double yi_dx=0,yi_dy=0;
                for (int i = 0; i < roi_image.rows; ++i) {
                    for (int j = 0; j < roi_image.cols; ++j) {
                        wi = Mat_Wi.at<double>(i,j);
                        g_x = Gx[i*roi_image.cols+j];
                        g_y = Gy[i*roi_image.cols+j];
                        g_xy = sqrt(g_x*g_x+g_y*g_y);

                        yi_dx += (i+1)*wi*g_x;
                        yi_dy += (j+1)*wi*g_y;

                        sum_wig += wi*g_xy;

//                        cout<<"-----------inner variable-------------"<<endl;
//                        cout<<"g_x: "<<g_x<<endl;
//                        cout<<"g_y: "<<g_y<<endl;
//                        cout<<"wi: "<<wi<<endl;
//                        cout<<"g_xy: "<<g_xy<<endl;
//                        cout<<"sum_wig: "<<sum_wig<<endl;

                    }
                }

                yi_x = yi_dx/sum_wig;
                yi_y = yi_dy/sum_wig;

//                cout<<"-----------inner variable-------------"<<endl;
//                cout<<"yi_dx: "<<yi_dx<<endl;
//                cout<<"yi_dy: "<<yi_dy<<endl;
//                cout<<"sum_wig: "<<sum_wig<<endl;

                return cv::Point2d(yi_x,yi_y);
            }

            cv::Rect trackWindow;//保存追踪框
            cv::Point2d point_Y0,point_Y1;//保存追踪框的原始点和候选框的点
            cv::Mat qu_hist,pu_hist; //追踪框和候选框的特征

            cv::Mat Mat_wi; //权值矩阵


        public:

            void initTracker_withoutKernel(const cv::Mat & src,cv::Rect tracking_window){

                double dist;//临时变量
                int q_r,q_g,q_b,q_temp;

                cv::Mat roi = src(tracking_window);

                cout<<"roi.size: "<<roi.size<<endl;

                t_x = tracking_window.x;
                t_y = tracking_window.y;
                t_w = tracking_window.width;
                t_h = tracking_window.height;

                band_width = pow((double)(t_w)/2.0,2)+pow((double)(t_h)/2.0,2);
                cout<<"band_width: "<<band_width<<endl;

//                memset(m_wei,0.0, sizeof(double)*t_w*t_h);
//                memset(hist1,0.0, sizeof(double)*bin*bin*bin);
                free(m_wei);
                m_wei = (double* )malloc(sizeof(double)*t_w*t_h);
                for (int i = 0; i < t_w*t_h; ++i) {
                    m_wei[i]=0.0;
                }

                for (int i = 0; i < 4096; ++i) {
                    hist1[i]=0.0;
                }

                C_ = 0.0;

                //初始化权值矩阵和归一化系数
//                #pragma omp parallel for
                for (int i = 0; i < t_h; ++i) {
                    for (int j = 0; j <t_w ; ++j) {
                        dist = pow(i-(double)t_h/2,2)+pow(j-(double)t_w/2,2); //求的是某一点到中点的欧式距离
                        m_wei[i*t_w + j] = 1 - dist/band_width; //这里求出了权值矩阵，直观的解释就是距离中心越近的像素权重越大,
                                                                // band_width相当于dist的最大值,而前面的1-说明了距离与权值是反比的
                        C_ += m_wei[i*t_w + j]; //权值矩阵求和
                    }
                }

                //计算权值直方图
//                #pragma omp parallel for
                for (int i = 0; i < t_h; ++i) {
                    uchar* data = roi.ptr<uchar>(i);
                    for (int j = 0; j < t_w; j+=3) {

                        q_b = data[j]/bin;
                        q_g = data[j+1]/bin;
                        q_r = data[j+2]/bin;

                        q_temp = q_r*256+q_g*16+q_b;
                        hist1[q_temp] += m_wei[i*t_w+j];
                    }
                }
                cout<<"q_g: "<<q_g<<endl;

                //归一化
//                #pragma omp parallel for
                for (int i = 0; i < 4096; ++i) {
                    hist1[i] /= C_;
                }
            }

            //跟踪函数
            void tracking_withoutKernel(cv::Mat& frame){

                int *q_temp = (int *)malloc(sizeof(int)*t_w*t_h),q_r,q_g,q_b;
                double sum_w = 0.0,x1=0,x2=0,y1=2.0,y2=2.0;
                int num=0;
                cv::Mat roi_frame,temp_frame;
                frame.copyTo(temp_frame);

                while ((pow(y2,2)+pow(y1,2)>0.8) && (num < 20)){
                    //控制迭代次数
                    ++num;

                    memset(q_temp,0,sizeof(int)*t_w*t_h);

                    for (int i = 0; i < 4096; ++i) {
                        hist2[i]=0.0;
                        w[i]=0.0;
                    }

                    //获得当前追踪框的roi图像
                    roi_frame = temp_frame(cv::Rect(t_x,t_y,t_w,t_h));

                    //画出跟踪框
                    drawRect(frame,cv::Rect(t_x,t_y,t_w,t_h));

//                    cout<<"ok1"<<endl;
                    #pragma omp parallel for
                    for (int i = 0; i < t_h; ++i) {
                        uchar* data = roi_frame.ptr<uchar>(i);
                        for (int j = 0; j < t_w; j+=3) {
                            q_b = data[j]/bin;
                            q_g = data[j+1]/bin;
                            q_r = data[j+2]/bin;
                            q_temp[i*t_w+j] = q_r*256+q_g*16+q_b;
                            hist2[q_temp[i*t_w+j]] += m_wei[i*t_w+j];

                        }
                    }
//                    cout<<"ok2"<<endl;
                    #pragma omp parallel for
                    for (int i = 0; i < bin*bin*bin; ++i) {
                        hist2[i] /= C_;

                        if (hist2[i] != 0)
                        {
                            w[i] = sqrt(hist1[i]/hist2[i]);
                            if(isnan(w[i])){
                                cout<<"w[i] is nan:" <<hist1[i]<<", "<<hist2[i]*C_<<", "<<hist2[i]<<endl;
//                                exit(0);
                                w[i]=0;
                            }
                        }else
                        {
                            w[i] = 0;
                        }
                    }

                    sum_w = 0.0;
                    x1 = 0.0;
                    x2 = 0.0;

                    //不能使用openMP 连加
//                    #pragma omp parallel for
                    for (int i = 0; i < t_h; ++i) {
                        for (int j = 0; j < t_w; ++j) {
                            sum_w += w[q_temp[i*t_w+j]];
                            x1 += w[q_temp[i*t_w+j]]*(i-t_h/2);
                            x2 += w[q_temp[i*t_w+j]]*(j-t_w/2);
                        }
                    }
//                    cout<<"ok4"<<endl;
                    //获得偏移量
                    y1 = x1 / sum_w;
                    y2 = x2 / sum_w;

                    //更新位置，这一步骤之前需要进行相关性判断
                    if((t_x+y2+t_w)<temp_frame.cols){
                        t_x += y2;
                    }else{
                        t_x = temp_frame.cols-t_w;
                    }

                    if((t_y+y1+t_h)<temp_frame.rows){
                        t_y += y1;
                    }else{
                        t_y = temp_frame.rows-t_h;
                    }

                    cout<<"-------track_windows-------"<<endl;
                    cout<<"C_: "<<C_<<endl;
                    cout<<"sum_w: "<<sum_w<<endl;
                    cout<<"y_bias: "<<y1<<endl;
                    cout<<"x_bias: "<<y2<<endl;
                    cout<<"t_x: "<<t_x<<endl;
                    cout<<"t_y: "<<t_y<<endl;
                    cout<<"t_w: "<<t_w<<endl;
                    cout<<"t_h: "<<t_h<<endl;

                    roi_frame.release(); //释放内存，每一循环都会得到新的roi区域

                }

                free(q_temp); //释放内存
            }

            void initTracker_withKernel(const cv::Mat& src,cv::Rect rec){
                trackWindow = rec;
                point_Y0.x = rec.x;
                point_Y0.y = rec.y;
                EpanechnikovKernelF(rec.height,rec.width);
                cv::Mat roi_image = src(rec);
                getPu(roi_image,qu_hist);
                qu_hist.copyTo(pu_hist);
            }

            const double update_step = 1.4; //更新的步长
            double lastCorrelation=0.0;
            bool tracking_withKernel(cv::Mat& src){
                int iter_count=0;
                double error=1.0,eps=0.3,dx=2.0,dy=2.0;
                cv::Mat roi_image;
                cv::Point2d Yi_bias;
                cv::Mat draw_mat;
                src.copyTo(draw_mat);

                double bhattacharyyaK = 0.0;

                while(error > eps && iter_count < 20){

                    ++iter_count;

                    roi_image = draw_mat(trackWindow);

                    getPu(roi_image,pu_hist); //获得候选区域的特征

                    getWi(roi_image,pu_hist,qu_hist,Mat_wi); //获得权值矩阵

                    bhattacharyyaK = Bhattacharyya(pu_hist,qu_hist);

                    Yi_bias = updateYi(roi_image,Mat_wi);

                    if(isnan(Yi_bias.x) || isnan(Yi_bias.y)){
                        continue;
                    }

                    if(lastCorrelation > bhattacharyyaK){
                        Yi_bias = Yi_bias / 2.0;
                    }

                    point_Y0 = safeUpdate(point_Y0,Yi_bias,trackWindow,draw_mat.cols,draw_mat.rows);

                    trackWindow.x = (int)round(point_Y0.x);
                    trackWindow.y = (int)round(point_Y0.y);

                    error = sqrt(pow((double)Yi_bias.x,2) + pow((double)Yi_bias.y,2));

                    cout<<"-------track_windows-------"<<endl;
                    cout<<"point_Y0.x: "<<point_Y0.x<<endl;
                    cout<<"point_Y0.y: "<<point_Y0.y<<endl;

                    cout<<"Yi_bias.x: "<<Yi_bias.x<<endl;
                    cout<<"Yi_bias.y: "<<Yi_bias.y<<endl;

                    cout<<"error: "<<error<<endl;

                    cout<<"bhattacharyya_: "<<bhattacharyyaK<<endl;

                }
                drawRect(src,trackWindow);
                return true;
            }
    };

    class CamShift{
//        public:

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
        KF(){

            //转换矩阵
            A << 1,0,1,0,
                0,1,0,1,
                0,0,1,0,
                0,0,0,1;
            H << 1,0,0,0,
                    0,1,0,0;
            //先确定R再去调节Q
            Q *= 1e-2;
            R *= 0.1;

        }
        cv::Point predict(double* measurement,int len){

            MatrixXd measure;
            double* prediction;

            utils::ArrayToMatrix(measurement,measure,len,1); //将测量值转换为matrix

            Xhat_minus = A * Xhat; //状态预测 公式1
            Phat_minus = A * Phat * A.transpose() + Q; //系统预测的协方差矩阵 公式2

            MatrixXd temp(dim_measure,dim_measure);
            temp = H * Phat_minus * H.transpose() + R;
            K = Phat_minus * H.transpose() * temp.inverse(); //求出卡尔曼增益 公式3

            Xhat = Xhat_minus + K * (measure - H * Xhat_minus); //更新系统状态 公式4

            Phat = (I - K * H) * Phat_minus; //更新估计值的协方差矩阵 公式5

            utils::MatrixToArray(Xhat_minus,prediction);

            return cv::Point(prediction[0],prediction[1]);
        }
    };

    class RANSAC{

    private:

        inline double getDistanceP2L(cv::Point2d a,double* K){
            return abs(K[0]*a.x+K[1]*a.y+K[2])/(sqrt(K[0]*K[0]+K[1]*K[1]));
        }

        void getDistanceP2L_Matrix(MatrixXd& points,double* K,MatrixXd& dis){
            MatrixXd line_ = MatrixXd::Constant(3,1,0);
            line_ << K[0],K[1],K[2];

            double temp2;
            temp2 = sqrt(K[0]*K[0]+K[1]*K[1]);

            MatrixXd dis_matrix;
            dis_matrix = points * line_ / temp2;
            ArrayXd dis_array;

            dis_array = dis_matrix.array();
            dis_array = dis_array.abs();
            dis = dis_array.matrix();
        }
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
        void LeastSquares(std::vector<cv::Point2d> points,double* result){
            double sumX_pow=0,sumX=0,sumXY=0,sumY=0;
            unsigned long N_points = points.size();

//            #pragma omp parallel for reduction(+:sumX_pow,sumX,sumXY,sumY)
            for (int i = 0; i < N_points; ++i) {
                sumX_pow += points[i].x * points[i].x;
                sumX += points[i].x;
                sumXY += points[i].x*points[i].y;
                sumY += points[i].y;
            }

            double temp;
            temp = N_points*sumX_pow-sumX*sumX;
            if(temp==0){
                result[0] = 1;
                result[1] = 0;
            }else{
                result[0] = (N_points*sumXY-sumX*sumY)/temp;
                result[1] = (sumX_pow*sumY-sumX*sumXY)/temp;
            }
//            cout<<result[0]<<endl;
        }
        //两点确定一条直线
        void getLine(double ax,double ay,double bx, double by,double* K){
            if(ax==bx && ay==by){
                cout<<"RANSAC error"<<endl;
                assert(false);
            }
            K[0] = by - ay; //A
            K[1] = ax - bx; //B
            K[2] = bx*ay - ax*by; //C
        }

        //随机获得两个点
        bool getSamples(const int max,int* sam_id){
            if(max<=2)return false;

            do{
                sam_id[0] = int((rand()/double(RAND_MAX))*(max-1));
                sam_id[1] = int((rand()/double(RAND_MAX))*(max-1));

            }while(sam_id[0] == sam_id[1]);

//            cout<<"sam_id[0]: "<<sam_id[0]<<endl;
//            cout<<"sam_id[1]: "<<sam_id[1]<<endl;
            return true;
        }

        //判断两个点的距离
        void fit(std::vector<cv::Point2d> points,double K_coefficient[2]){
            unsigned long num_points;
            num_points = points.size();
            MatrixXd m_points = MatrixXd::Constant(num_points,3,1);
//            #pragma omp parallel for
            for (int i = 0; i < num_points; ++i) {
                m_points(i,0) = points[i].x;
                m_points(i,1) = points[i].y;
            }
            int N = 500,sample_count=0,sample_id[2],max_inlier_counter=0;
            bool stop_flag=false;

            double K_abc[3];//由两点确定的直线参数
            int inlier_counter=0;//内点的个数

            std::vector<cv::Point2d> final_inlier_points;
            while(N > sample_count && !stop_flag){
                //获取两个样本
                if(!getSamples(num_points,sample_id)){
                    stop_flag = true;
                    continue;
                }
                //两个点之间的距离不能太小
                if(getDistanceP2P(m_points(sample_id[0],0),m_points(sample_id[0],1),
                                  m_points(sample_id[1],0),m_points(sample_id[1],1))<5.0)
                {
                    ++sample_count;
                    continue;
                }
                //通过选取的两个点计算出确定的直线方程
                getLine(m_points(sample_id[0],0),m_points(sample_id[0],1),
                        m_points(sample_id[1],0),m_points(sample_id[1],1),K_abc);
                //找出内点
                MatrixXd error_;
                getDistanceP2L_Matrix(m_points,K_abc,error_);

                std::vector<cv::Point2d> inlierPoints;

                for (int i = 0; i < num_points; ++i) {
                    if(error_(i,0) <= inlier_error){
                        inlierPoints.push_back(cv::Point2d(m_points(i,1),m_points(i,0)));
                    }
                }
                inlier_counter = inlierPoints.size();

                if(inlier_counter > max_inlier_counter){
                    max_inlier_counter = inlier_counter;
                    final_inlier_points = inlierPoints;
                }

                if(inlier_counter == 0)
                {
                    N=500;
                }else{
                    double epsilon = double(inlier_counter)/(double)num_points;
                    double p = 0.99;
                    double s = 2.0;
                    N = int(log(1.0-p)/log(1.0-pow(epsilon,s)));
                }
                ++sample_count;

            }
            double KKKK[2];
            LeastSquares(final_inlier_points,K_coefficient);
        }

        void getTestSamples(std::vector<cv::Point2d>& points,const int max_iter){
//            for (int i = 0; i < max_iter; i+=5) {
//                points.push_back(cv::Point2d(i,i*0.85+7+(rand()/double(RAND_MAX))*40));
//            }
//            for (int i = 0; i < max_iter; i+=50) {
//                points.push_back(cv::Point2d(i,i*0.85+((rand()-RAND_MAX)/double(RAND_MAX))*400));
//            }
            for (int i = 1; i < max_iter; i+=10) {
                points.push_back(cv::Point2d(i,i*0.85+7+((rand()-RAND_MAX)/double(RAND_MAX))*50));
            }

            for (int i = 1; i < max_iter; i+=2) {
                points.push_back(cv::Point2d(i,i*0.85+7+((rand()-RAND_MAX)/double(RAND_MAX))*10));
            }
        }

    };

}

#endif //TRACKING_HPP
