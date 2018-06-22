#ifndef CALIBRATE_H
#define CALIBRATE_H
#include<vector>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include"opencv2/imgproc.hpp"
#include<QString>

#define NON_EXCEPT 0
#define NEED_MORE_CALIBRATE_POINT 1
#define UNKNOW_CALIBRATE_ERROR 2
#define CALIBRATION_DATA_ERROR 3
#define LOAD_CALIBRATION_INFORMATION_ERROR 4
#define SAVE_CALIBRATION_INFORMATION_ERROR 5
#define INPUT_CALIBRATION_INFORMATION_ERROR 6

class calibrate
{
public:
    calibrate();
    void calibrateParamsIntialize(QString path);
private:
    ///////////////////根据图像上检测到的目标坐标和世界坐标计算旋转矩阵、偏移矩阵////////////////////////
    std::vector<cv::Point2f> imagePoints;							/* 图像上检测到的物体坐标 */
    std::vector<cv::Point3f> worldPoints;							/* 物体的世界坐标 */
    std::vector<std::vector<cv::Point2f>> vec_imagePoints;			/* 图像上检测到的物体坐标 */
    std::vector<std::vector<cv::Point3f>> vec_worldPoints;			/* 物体的世界坐标 */
    cv::Point2f resultPoint;

    cv::Mat intrinsic;                                          //相机内参数
    cv::Mat distortion_coeff;                                   //相机畸变参数
    std::vector<cv::Mat> rvecs;                                 //旋转向量
    std::vector<cv::Mat> tvecs;                                 //平移向量
    cv::Size imageSize;											//图片尺寸

    cv::Mat calibrateMsgMat;                                    //标定矩阵
    QString paramPath;
    bool calculateStatus=false;
private:
    cv::Mat calibrateParamRead(const std::string&);
    void calibrateParamWrite(const std::string&,const cv::Mat&,cv::FileStorage::Mode);
    void calibratePointsIntialize(std::vector<cv::Point2f>, std::vector<cv::Point3f>,cv::Size&);
    int calibrateResultSave();
    int calibrateResultLoad();
    void dataClear();
public:
    ////////////get the points data///////////////
    int calibrateTheCamera(std::vector<cv::Point2f>, std::vector<cv::Point3f>,cv::Size&);
    cv::Point2f calculateWorldPosition(const cv::Point2f&);
    double evaluateCalibrateResult();
    bool getCalibrateStatus();
    cv::Point2f getResultPoint();
    cv::Mat getCalibrateMat();
    cv::Size getCalibrateImgSize();
    void setCalibrateMat(cv::Mat&tmp);
};

#endif // CALIBRATE_H
