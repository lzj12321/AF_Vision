#ifndef CVANDUI_H
#define CVANDUI_H
#include<QLabel>
#include<opencv2/opencv.hpp>
#include<opencv2/core.hpp>

class cvAndUi
{
public:
    cvAndUi();
    double adjustMatSizeToShow(cv::Mat&mat,cv::Mat&dst, QLabel *label);
    cv::Mat resizeMat(cv::Mat& src, float i);
    void showMatOnDlg(cv::Mat &mat, QLabel*label,double* ratioPtr=nullptr);
};

#endif // CVANDUI_H
