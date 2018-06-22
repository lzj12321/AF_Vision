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

typedef struct str_matchPoint{
    cv::Point matchPoint;
    short int angle=0;
    float matchValue;
}str_matchPoint;

class NCC_Match
{
public:
    int stopMatchLayer=0;
    QTime testTimer;
    NCC_Match();
    virtual ~NCC_Match();
    virtual void preProcessTemplate(cv::Mat&temp);
    virtual void preProcessSource(cv::Mat&src);
    void setAngleMatchRange(unsigned short int);
    void setTemplateAndMask(cv::Mat& tmp,cv::Mat& mask,ushort downSampleTime=0);
    void generateTemplateAndMask(cv::Mat& tmp,cv::Mat& mask,ushort downSampleTime=0);
    void nccMatch(cv::Mat&src,float thresholdValue=0,bool maskFlag=true,bool rotateFlag=true,ushort matchNum=1);
    void rotateMat(cv::Mat& temp,cv::Mat&dst,float angle);
    void setRotateAndMaskFlag(bool,bool);
    cv::Mat resizeMat(cv::Mat& src, float i);
    ushort downSamplingNum=5;
    bool matchStatus=false;
    int reckonDownSampleTime(cv::Mat&src);
private:
    cv::Mat templateMat;
    cv::Mat maskMat;
    cv::Mat srcMat;
    int generateTempDataLayer=4;
private:
    bool enableMask;
    bool enableRotate;
    ushort rotateTempStepSize;
    ushort coarseMatchAngleStepSize;
    ushort preciseMatchAngleStepSize;
    ushort angleMatchRange=360;
public:
    cv::Point nccMatchPoint;
    float nccMatchScore;
    ushort nccMatchAngle;
    cv::Point getMatchPoint();
    float getMatchScore();
    int getMatchAngle();
public:
    float* layerThreshold=nullptr;
    ushort* layerAngleMatchRange=nullptr;
    ushort* layerCoordinateMatchRange=nullptr;

    std::vector<std::vector<str_matchPoint>>VEC_matchPointsxx;
    std::vector<cv::Mat> vec_downSampleSrcImg;
    std::vector<cv::Mat> vec_downSampleTemp;
    std::vector<cv::Mat> vec_downSampleMask;
    std::vector<std::vector<str_img_data>>VEC_rotateTempData;
    std::vector<int>vec_matchTime;
private:
    void dataInitialize(ushort num);
    void dataClearBeforeMatch();
    void imgDownSampling(cv::Mat&src,cv::Mat&dst,ushort layerNum);
    void bestMatchPointFilter(std::vector<str_matchPoint>&src,str_matchPoint&dst);
    void generateRotateTemplate(ushort stepSize,cv::Mat&temp,std::vector<cv::Mat>&vec_dstMat,ushort startAngle=0);
    void generateDownSampleSrc(cv::Mat&src);
    int nccCoarseMatch(ushort matchLayer,std::vector<std::vector<str_matchPoint>>&VEC_matchPoints);
    void nccPreciseMatch(int,std::vector<std::vector<str_matchPoint>>&VEC_matchPoints);
    void nccRotateMatch(ushort matchLayer,float&matchValue,cv::Point&bestMatchPoint,ushort&bestMatchAngle,ushort angleMatchRange,ushort stepSize=1);
    int nccUnRotateCoarseMatch(ushort matchLayer,std::vector<std::vector<str_matchPoint>>&VEC_matchPoints);
    int nccRotateCoarseMatch(ushort matchLayer,ushort angleRange,std::vector<std::vector<str_matchPoint>>&VEC_matchPoints);
    void nccRotatePreciseMatch(int,ushort angleRange,std::vector<std::vector<str_matchPoint>>&VEC_matchPoints);
    void nccUnRotatePreciseMatch(ushort,std::vector<std::vector<str_matchPoint>>&VEC_matchPoints);

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
