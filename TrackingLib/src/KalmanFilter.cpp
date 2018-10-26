//
// Created by dasuda on 18-10-26.
//

#include "../include/tracking.hpp"

wong::KF::KF(){

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

cv::Point wong::KF::predict(double* measurement,int len){

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