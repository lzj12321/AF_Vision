#ifndef NCC_MATCH_H
#define NCC_MATCH_H

#include<opencv2/opencv.hpp>
#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<QTime>
#include<QBitArray>

typedef struct str_img_data{
    uint pixelNum;
    uint validPixelNum;
    ushort imgCols;
    ushort imgRows;
    double imgStandardDeviation;
    cv::Mat diffMat;
    cv::Mat maskMat;
}str_img_data;

typedef struct NccMatchResult{
    cv::Point matchPoint;
    short int matchAngle;
    float matchValue=-1;
}NccMatchResult;

class NCC_Match
{
public:
    QTime testTimer;
    NCC_Match();
    virtual ~NCC_Match();
    void setTemplateAndMask(cv::Mat& tmp,cv::Mat& mask,ushort downSampleTime=0);
    void generateTemplateAndMask(cv::Mat& tmp,cv::Mat& mask,ushort downSampleTime=0);
    NccMatchResult nccMatch(cv::Mat&src,float thresholdValue=0,int anleMatchRange=360,ushort matchNum=1);
    void rotateMat(cv::Mat& temp,cv::Mat&dst,float angle);
    cv::Mat resizeMat(cv::Mat& src, float i);
    ushort downSamplingNum=5;
private:
    cv::Mat templateMat;
    cv::Mat maskMat;
    int generateTempDataLayer=4;
    const int defaultDownSampleTime=4;
private:
    ushort rotateTempStepSize;
    ushort coarseMatchAngleStepSize;
    ushort preciseMatchAngleStepSize;
public:
    float* layerThreshold=nullptr;
    ushort* layerAngleMatchRange=nullptr;
    ushort* layerCoordinateMatchRange=nullptr;

    std::vector<std::vector<NccMatchResult>>VEC_matchPointsxx;
    std::vector<cv::Mat> vec_downSampleSrcImg;
    std::vector<cv::Mat> vec_downSampleTemp;
    std::vector<cv::Mat> vec_downSampleMask;
    std::vector<std::vector<str_img_data>>VEC_rotateTempData;
    std::vector<int>vec_matchTime;
private:
    void paramInitialize(ushort num);
    void dataClearBeforeMatch();
    void imgDownSampling(cv::Mat&src,cv::Mat&dst,ushort layerNum);
    void bestMatchPointFilter(std::vector<NccMatchResult>&src,NccMatchResult&dst);
    void generateRotateTemplate(ushort stepSize,cv::Mat&temp,std::vector<cv::Mat>&vec_dstMat,ushort startAngle=0);
    void generateDownSampleSrc(cv::Mat&src);
    void nccCoarseMatch(ushort matchLayer,std::vector<std::vector<NccMatchResult>>&VEC_matchPoints,ushort angleMatchRange=360);
    void nccPreciseMatch(int,std::vector<std::vector<NccMatchResult>>&VEC_matchPoints);
    void nccRotateMatch(ushort matchLayer,float&matchValue,cv::Point&bestMatchPoint,ushort&bestMatchAngle,ushort angleMatchRange,ushort stepSize=1);

    void calculateImgData(cv::Mat&tmp,cv::Mat&mask,str_img_data*imgData);
    inline void _calculateImgData(cv::Mat&tmp,cv::Mat&mask,str_img_data*imgData);
    inline float processNccValue(str_img_data&imgData1,str_img_data&imgData2);

    void initializeTemplate(cv::Mat&tmp,cv::Mat&mask,ushort initializeLayer=1);
    void generateTemplate(cv::Mat&tmp,cv::Mat&mask,ushort initializeLayer=1);
    inline void nccPointMatch(cv::Mat &src,str_img_data&tmpImgData, cv::Point& matchPoint,float*nccMatchValue);
private:
    NCC_Match operator=(const NCC_Match&);
};
#endif // NCC_MATCH_H
