#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <QDialog>
#include<QLineEdit>
#include<QLabel>
#include"calibrate.h"
#include"vision.h"

namespace Ui {
class calibration;
}

class calibration : public QDialog
{
    Q_OBJECT

public:
    explicit calibration(QWidget *parent = 0);
    virtual ~calibration();
    void calibrateInitialize(QString path);
    cv::Point2f computePoint(cv::Point2f&point);
    bool computePoint(STR_Vision_Result&result);
    bool getCalibrateStatus();
private slots:
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();
    void on_pushButton_6_clicked();
private:
    int calibrateIndex;
    bool calibrateStatus;
    calibrate calibrateObj;
    int positionShowPositionX=95;
    int positionShowPositionY=60;
    int lableLength=100;
    int lableWidth=20;
    cv::Size imgSize;
    Ui::calibration *ui;
    std::vector<QLineEdit*>inputPosition_XLineEdit;
    std::vector<QLineEdit*>inputPosition_YLineEdit;
    std::vector<QLineEdit*>inputPosition2_XLineEdit;
    std::vector<QLineEdit*>inputPosition2_YLineEdit;
    std::vector<QLabel*>inputPositionNumFlag;
private:
    ///////////////////根据图像上检测到的目标坐标和世界坐标计算旋转矩阵、偏移矩阵////////////////////////////
    std::vector<cv::Point2f> calibrateImagePoints;							/* 图像上检测到的物体坐标 */
    std::vector<cv::Point3f> calibrateWorldPoints;							/* 物体的世界坐标 */
private:
    void enableCalibrateUiIni();
public slots:
    void slot_calibrateCamera();
};

#endif // CALIBRATION_H
