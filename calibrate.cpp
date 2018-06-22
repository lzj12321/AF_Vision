#include "calibrate.h"
#include<iostream>
#include<QFileInfo>
#include<QDebug>

using namespace cv;
calibrate::calibrate()
{
}

void calibrate::calibrateParamsIntialize(QString path)
{
    paramPath=path;
    intrinsic.create(3, 3, CV_64FC1);
    distortion_coeff.create(5, 1, CV_64FC1);
    calibrateResultLoad();
}

void calibrate::calibratePointsIntialize(std::vector<cv::Point2f> _imagePoints, std::vector<cv::Point3f> _worldPoints,cv::Size&imgSize)
{
    imagePoints=_imagePoints;
    worldPoints=_worldPoints;
    vec_imagePoints.push_back(_imagePoints);
    vec_worldPoints.push_back(_worldPoints);
    imageSize=imgSize;
}

int calibrate::calibrateTheCamera(std::vector<cv::Point2f> _imagePoints, std::vector<cv::Point3f> _worldPoints,cv::Size&imgSize)
{
    calculateStatus=false;
    if(_imagePoints.size()<4||_worldPoints.size()<4)
        return NEED_MORE_CALIBRATE_POINT;
    for(auto iter=_imagePoints.cbegin();iter!=_imagePoints.cend();iter++)
    {
        if((*iter).x>imgSize.width||(*iter).y>imgSize.height)
            return INPUT_CALIBRATION_INFORMATION_ERROR;
    }
    dataClear();
    calibratePointsIntialize(_imagePoints,_worldPoints,imgSize);
    cv::calibrateCamera(vec_worldPoints, vec_imagePoints, imageSize, intrinsic, distortion_coeff, rvecs, tvecs, 0);
    calibrateResultSave();
    calculateStatus=true;
    return NON_EXCEPT;
}

cv::Point2f calibrate::calculateWorldPosition(const cv::Point2f &srcPoint)
{
    std::vector<cv::Point2f> vec_srcPoint;
    std::vector<cv::Point2f> vec_resultPoint;
    if(rvecs.size()==0)
        calculateStatus=false;
    vec_srcPoint.push_back(srcPoint);
    //////////矫正//////////////////////
    cv::undistortPoints(vec_srcPoint, vec_resultPoint, intrinsic, distortion_coeff, intrinsic);
    /********图像坐标到世界坐标*******/
    cv::Mat rotation_Matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0));
    //将旋转矩阵转换为旋转矢量
    cv::Rodrigues(rvecs[0], rotation_Matrix);
    cv::Mat H(3, 3, CV_32FC1, cv::Scalar::all(0));
    cv::Mat translation_ve;
    tvecs[0].copyTo(translation_ve);
    rotation_Matrix.copyTo(H);
    H.at<double>(0, 2) = translation_ve.at<double>(0, 0);
    H.at<double>(1, 2) = translation_ve.at<double>(1, 0);
    H.at<double>(2, 2) = translation_ve.at<double>(2, 0);
    cv::Mat hu;
    hu = intrinsic*H;
    cv::Mat hu2 = hu.inv();
    double a1, a2, a3, a4, a5, a6, a7, a8, a9;
    a1 = hu2.at<double>(0, 0);
    a2 = hu2.at<double>(0, 1);
    a3 = hu2.at<double>(0, 2);
    a4 = hu2.at<double>(1, 0);
    a5 = hu2.at<double>(1, 1);
    a6 = hu2.at<double>(1, 2);
    a7 = hu2.at<double>(2, 0);
    a8 = hu2.at<double>(2, 1);
    a9 = hu2.at<double>(2, 2);

    resultPoint.x = (a1*vec_resultPoint[0].x + a2*vec_resultPoint[0].y + a3) / (a7*vec_resultPoint[0].x + a8*vec_resultPoint[0].y + a9);//世界坐标中x值
    resultPoint.y = (a4*vec_resultPoint[0].x + a5*vec_resultPoint[0].y + a6) / (a7*vec_resultPoint[0].x + a8*vec_resultPoint[0].y + a9);//世界坐标中Y值
    return resultPoint;
}

double calibrate::evaluateCalibrateResult()
{
    double err=-1;
    std::vector<cv::Point2f> tmpImagePoints;
    /****     通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/
    cv::projectPoints(worldPoints, rvecs[0], tvecs[0], intrinsic, distortion_coeff, tmpImagePoints);

    /* 计算新的投影点和旧的投影点之间的误差 */
    cv::Mat sourceImagePointsMat = cv::Mat(1, imagePoints.size(), CV_32FC2);
    cv::Mat compareImagePointMat = cv::Mat(1, imagePoints.size(), CV_32FC2);
    for (int j = 0; j < imagePoints.size(); j++)
    {
        sourceImagePointsMat.at<cv::Vec2f>(0, j) = cv::Vec2f(imagePoints[j].x, imagePoints[j].y);
        compareImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tmpImagePoints[j].x, tmpImagePoints[j].y);
    }
    err = cv::norm(compareImagePointMat, sourceImagePointsMat, cv::NORM_L2);
    return err;
}

bool calibrate::getCalibrateStatus()
{
    return calculateStatus;
}

cv::Point2f calibrate::getResultPoint()
{
    return resultPoint;
}

cv::Mat calibrate::getCalibrateMat()
{
    return calibrateMsgMat;
}

Size calibrate::getCalibrateImgSize()
{
    return imageSize;
}

void calibrate::setCalibrateMat(cv::Mat&tmp)
{
    calibrateMsgMat=tmp.clone();
}

cv::Mat calibrate::calibrateParamRead(const std::string&matName)
{
    cv::FileStorage fs(paramPath.toStdString(), cv::FileStorage::READ);
    cv::Mat tmpMat;
    if(fs.isOpened())
        fs[matName] >> tmpMat;
    fs.release();
    return tmpMat;
}

void calibrate::calibrateParamWrite(const std::string&matName, const cv::Mat &tmpMat,cv::FileStorage::Mode writeMode)
{
    cv::FileStorage fs(paramPath.toStdString(), writeMode);
    if(fs.isOpened())
    {
        fs<<matName<<tmpMat;
    }
    fs.release();
}

int calibrate::calibrateResultSave()
{
    qDebug()<<"calibrateResultSave calibrateResultSave"<<endl;
    try
    {
        calibrateParamWrite("intrinsic",intrinsic,cv::FileStorage::Mode::WRITE);
        calibrateParamWrite("distortion_coeff",distortion_coeff,cv::FileStorage::Mode::APPEND);
        calibrateParamWrite("rvecs",rvecs[0],cv::FileStorage::Mode::APPEND);
        calibrateParamWrite("tvecs",tvecs[0],cv::FileStorage::Mode::APPEND);
        calibrateParamWrite("calibrateMsgMat",calibrateMsgMat,cv::FileStorage::Mode::APPEND);

        cv::FileStorage fs(paramPath.toStdString(), cv::FileStorage::APPEND);
        if(fs.isOpened())
        {
            fs<<"imageSize"<<imageSize;
        }
        fs.release();
        return NON_EXCEPT;
    }
    catch(cv::Exception&ex)
    {
        return SAVE_CALIBRATION_INFORMATION_ERROR;
    }
}

int calibrate::calibrateResultLoad()
{
    try
    {
        QFileInfo  fileInfo(paramPath);
        if(fileInfo.exists())
        {
            calibrateMsgMat=calibrateParamRead("calibrateMsgMat");
            intrinsic=calibrateParamRead("intrinsic");
            distortion_coeff=calibrateParamRead("distortion_coeff");
            cv::Mat rvecs0=calibrateParamRead("rvecs");
            rvecs.clear();
            rvecs.push_back(rvecs0);
            cv::Mat tvecs0=calibrateParamRead("tvecs");
            tvecs.clear();
            tvecs.push_back(tvecs0);
            cv::FileStorage fs(paramPath.toStdString(), cv::FileStorage::READ);
            if(fs.isOpened())
            {
                fs["imageSize"]>>imageSize;
            }
            fs.release();
            calculateStatus=true;
            return NON_EXCEPT;
        }
        else
        {
            calculateStatus=false;
            return LOAD_CALIBRATION_INFORMATION_ERROR;
        }
    }
    catch(cv::Exception&ex)
    {
        calculateStatus=false;
        return LOAD_CALIBRATION_INFORMATION_ERROR;
    }
}

void calibrate::dataClear()
{
    rvecs.clear();
    tvecs.clear();
    imagePoints.clear();
    worldPoints.clear();
    vec_imagePoints.clear();
    vec_worldPoints.clear();
}
