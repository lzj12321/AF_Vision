#include "cvandui.h"
#include<QImage>
#include<QMessageBox>
#include <QFileDialog>
#include <QTextCodec>
#include<QDebug>

cvAndUi::cvAndUi()
{
}

double cvAndUi::adjustMatSizeToShow(cv::Mat&mat,cv::Mat&dst, QLabel *label)
{
    double ratio=0;
    double colRatio=mat.cols/(double)label->width();
    double rowRatio=mat.rows/(double)label->height();
    ratio=colRatio>rowRatio?colRatio:rowRatio;
    dst=resizeMat(mat,ratio);
    return ratio;
}

cv::Mat cvAndUi::resizeMat(cv::Mat &src, float i)
{
    cv::Mat dst;
    cv::resize(src, dst, cv::Size(cvRound( src.cols / i),cvRound( src.rows / i)));
    return dst;
}

void cvAndUi::showMatOnDlg(cv::Mat &mat, QLabel *label,double* ratioPtr)
{
    if(mat.empty())
    {
        qDebug()<<"mat.empty()";
        return;
    }
    QImage imageShow;
    cv::Mat resizeMat;
    double r=0;
    r=adjustMatSizeToShow(mat,resizeMat,label);
    if(ratioPtr!=nullptr)
        *ratioPtr=r;
    if(resizeMat.type() == CV_8UC1)
    {
        QImage image(resizeMat.cols, resizeMat.rows, QImage::Format_Indexed8);
        image.setColorCount(256);
        for(int i = 0; i < 256; i++)
        {
            image.setColor(i, qRgb(i, i, i));
        }
        uchar *pSrc = resizeMat.data;
        for(int row = 0; row < resizeMat.rows; row ++)
        {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, resizeMat.cols);
            pSrc += resizeMat.step;
        }
        imageShow=image;
    }
    else if(resizeMat.type() == CV_8UC3)
    {
        const uchar *pSrc = (const uchar*)resizeMat.data;
        QImage image(pSrc, resizeMat.cols, resizeMat.rows, resizeMat.step, QImage::Format_RGB888);
        imageShow=image.rgbSwapped();
    }
    else if(resizeMat.type() == CV_8UC4)
    {
        const uchar *pSrc = (const uchar*)resizeMat.data;
        QImage image(pSrc, resizeMat.cols, resizeMat.rows, resizeMat.step, QImage::Format_ARGB32);
        imageShow=image;
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setText( "ERROR: Mat could not be converted to QImage.");
        msgBox.exec();
    }
    label->setPixmap(QPixmap::fromImage(imageShow));
    label->show();
}
