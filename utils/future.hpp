//
// Created by dasuda on 18-8-28.
//
#pragma once
#ifndef TRACKING_FUTURE_H
#define TRACKING_FUTURE_H

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

namespace wong{
    class GetTensorflowNet{
        using namespace std;
        using namespace cv;
    private:
        dnn::Net net;
        String _modelName,_inputBLobName,_outputBlobName;
        Size _imgSize;

        void getMaxClass_n(const Mat &probBlob, int *classId, double *classProb);
    public:
        //初始化模型
        GetTensorflowNet(String modelName,String inputBlobName, String outputBlobName, Size imgSize);
        //进行预测
        bool predict(const std::vector<Mat> & imgs, int nClass, int *classId, double *classProb_n);

    };
    /*
     * @details init the tensorflow model
     * @param modelName the model file path
     * @param inputBlobName the input tensor's name
     * @param outputBlobName the output tensor's name
     * @param imgSize input images's size
     * */
    GetTensorflowNet::GetTensorflowNet(String modelName, String inputBlobName, String outputBlobName, Size imgSize) {
        _modelName = modelName;
        _inputBLobName = inputBlobName;
        _outputBlobName = outputBlobName;
        _imgSize = imgSize;

        net = dnn::readNetFromTensorflow(_modelName);
        if(net.empty()){
            cerr<<"Plase Check the model path: "<<modelName<<endl;
        }
    }

    bool GetTensorflowNet::predict(const std::vector<cv::Mat> &imgs, int nClass, int *classId, double *classProb_n) {

        if(imgs.size()!=nClass){
            cerr<<"input images size is wrong "<<endl;
            return false;
        }

        Mat inputBlobs;
        for (size_t i = 0; i < imgs.size(); ++i) {
            Mat inputBlob = dnn::blobFromImage(imgs[i], 1.0f, _imgSize, Scalar(), false, false);
            inputBlobs.push_back(inputBlob);
        }

        net.setInput(inputBlobs,_inputBLobName);
        Mat result = net.forward(_outputBlobName);
        getMaxClass_n(result, classId, classProb_n);

        return true;
    }

    void GetTensorflowNet::getMaxClass_n(const cv::Mat &probBlob, int *classId, double *classProb) {
        for (size_t i = 0; i < 9; i++) {
            Mat probMat = probBlob.row(i).reshape(1, 1); //reshape the blob to 1x1000 matrix
            Point classNumber;

            double _classProb;
            minMaxLoc(probMat, NULL, &_classProb, NULL, &classNumber);
            classId[i] = classNumber.x + 1;
            classProb[i] = _classProb;
        }
    }
}

#endif //TRACKING_FUTURE_H
