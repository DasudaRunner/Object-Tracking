//
// Created by dasuda on 18-10-26.
//

#include "../include/tracking.hpp"

void wong::MeanShift::drawRect(cv::Mat& src,cv::Rect rec)
{
    cv::rectangle(src,rec.tl(),rec.br(),cv::Scalar(0,255,0),2);
}

cv::Point2d wong::MeanShift::safeUpdate(cv::Point2d ori,
                                        cv::Point2d update,
                                        cv::Rect track_window,
                                        const int img_width,
                                        const int img_height)
{
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

double wong::MeanShift::Bhattacharyya(cv::Mat& Pu,cv::Mat& qu)
{
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

void wong::MeanShift::EpanechnikovKernelF(const int H,const int W){

    int radWidth = (int)sqrt(pow((double)W,2)+pow((double)H,2));
    int radHeight = radWidth;
    double pi = 3.141596;
    int d = 2;//当前维度
    double Cd = pi; //d维空间下单位球体的体积
    double kx=0,ky=0; //kernel funciton 的梯度

    K = new double[W*H];
    Gx = new double[W*H];
    Gy = new double[W*H];

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
void wong::MeanShift::getPu(cv::Mat& roi_image,cv::Mat& Mat_Pu){
    double *pu = new double[allBins];

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
void wong::MeanShift::getWi(cv::Mat& roi_image,cv::Mat& pu, cv::Mat& qu ,cv::Mat& Mat_Wi){

    double* Wi = new double[roi_image.rows*roi_image.cols];

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
cv::Point2d wong::MeanShift::updateYi(const cv::Mat& roi_image,cv::Mat& Mat_Wi){

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

void wong::MeanShift::initTracker_withoutKernel(const cv::Mat & src,cv::Rect tracking_window){

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
    for (int i = 0; i < t_h; ++i) {
        for (int j = 0; j <t_w ; ++j) {
            dist = pow(i-(double)t_h/2,2)+pow(j-(double)t_w/2,2); //求的是某一点到中点的欧式距离
            m_wei[i*t_w + j] = 1 - dist/band_width; //这里求出了权值矩阵，直观的解释就是距离中心越近的像素权重越大,
            // band_width相当于dist的最大值,而前面的1-说明了距离与权值是反比的
            C_ += m_wei[i*t_w + j]; //权值矩阵求和
        }
    }

    //计算权值直方图
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
    for (int i = 0; i < 4096; ++i) {
        hist1[i] /= C_;
    }
}

//跟踪函数
void wong::MeanShift::tracking_withoutKernel(cv::Mat& frame){

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


        for (int i = 0; i < t_h; ++i) {
            for (int j = 0; j < t_w; ++j) {
                sum_w += w[q_temp[i*t_w+j]];
                x1 += w[q_temp[i*t_w+j]]*(i-t_h/2);
                x2 += w[q_temp[i*t_w+j]]*(j-t_w/2);
            }
        }

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

void wong::MeanShift::initTracker_withKernel(const cv::Mat& src,cv::Rect rec){
    trackWindow = rec;
    point_Y0.x = rec.x;
    point_Y0.y = rec.y;
    EpanechnikovKernelF(rec.height,rec.width);
    cv::Mat roi_image = src(rec);
    getPu(roi_image,qu_hist);
    qu_hist.copyTo(pu_hist);
}

bool wong::MeanShift::tracking_withKernel(cv::Mat& src){
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