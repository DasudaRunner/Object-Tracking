//
// Created by dasuda on 18-10-26.
//

#include "../include/tracking.hpp"

void wong::RANSAC::getDistanceP2L_Matrix(MatrixXd& points,double* K,MatrixXd& dis){
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

//最小二乘法拟合直线
void wong::RANSAC::LeastSquares(std::vector<cv::Point2d> points,double* result){
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
void wong::RANSAC::getLine(double ax,double ay,double bx, double by,double* K){
    if(ax==bx && ay==by){
        cout<<"RANSAC error"<<endl;
        assert(false);
    }
    K[0] = by - ay; //A
    K[1] = ax - bx; //B
    K[2] = bx*ay - ax*by; //C
}

//随机获得两个点
bool wong::RANSAC::getSamples(const int max,int* sam_id){
    if(max<=2)return false;

    do{
        sam_id[0] = int((rand()/double(RAND_MAX))*(max-1));
        sam_id[1] = int((rand()/double(RAND_MAX))*(max-1));

    }while(sam_id[0] == sam_id[1]);

    return true;
}

void wong::RANSAC::fit(std::vector<cv::Point2d> points,double K_coefficient[2]){
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

void wong::RANSAC::getTestSamples(std::vector<cv::Point2d>& points,const int max_iter){
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