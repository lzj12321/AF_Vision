#include "calibration.h"
#include "ui_calibration.h"
#include<QMessageBox>
#include<QDebug>

calibration::calibration(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::calibration)
{
    ui->setupUi(this);
}

calibration::~calibration()
{
    auto iter=inputPosition_XLineEdit.begin();
    for(;iter!=inputPosition_XLineEdit.end();iter++)
    {
        delete *iter;
    }
    iter=inputPosition_YLineEdit.begin();
    for(;iter!=inputPosition_YLineEdit.end();iter++)
    {
        delete *iter;
    }
    auto iter2=inputPositionNumFlag.begin();
    for(;iter2!=inputPositionNumFlag.end();iter2++)
    {
        delete *iter2;
    }
    delete ui;
}

void calibration::calibrateInitialize(QString path)
{
    qDebug()<<"calibrate path:"<<path<<endl;
    calibrateObj.calibrateParamsIntialize(path);
    calibrateStatus=calibrateObj.getCalibrateStatus();
    enableCalibrateUiIni();
}

cv::Point2f calibration::computePoint(cv::Point2f &point)
{
    return calibrateObj.calculateWorldPosition(point);
}

bool calibration::computePoint(STR_Vision_Result &result)
{
    cv::Point2f point(result.x,result.y);
    cv::Point2f tempPoint=calibrateObj.calculateWorldPosition(point);
    result.x=tempPoint.x;
    result.y=tempPoint.y;
    return true;
}

bool calibration::getCalibrateStatus()
{
    return calibrateStatus;
}

void calibration::on_pushButton_clicked()
{
    if(inputPosition_XLineEdit.size()>8)
        return;

    int x=inputPosition_XLineEdit.size()+1;
    QLabel* newLable=new QLabel(this);
    inputPositionNumFlag.push_back(newLable);
    newLable->setText(QString::number(x,10));
    ui->gridLayout_2->addWidget(newLable);

    QLineEdit* newLineEdit=new QLineEdit(this);
    newLineEdit->setValidator(new QDoubleValidator(-100000.0,100000.0,3,this));
    inputPosition_XLineEdit.push_back(newLineEdit);

    QLineEdit* newLineEdit2=new QLineEdit(this);
    newLineEdit2->setValidator(new QDoubleValidator(-100000.0,100000.0,3,this));
    inputPosition_YLineEdit.push_back(newLineEdit2);

    ui->gridLayout_2->addWidget(newLineEdit);
    ui->gridLayout_2->addWidget(newLineEdit2);

    QLineEdit* newLineEdit3=new QLineEdit(this);
    newLineEdit3->setValidator(new QDoubleValidator(-100000.0,100000.0,3,this));
    inputPosition2_XLineEdit.push_back(newLineEdit3);

    QLineEdit* newLineEdit4=new QLineEdit(this);
    newLineEdit4->setValidator(new QDoubleValidator(-100000.0,100000.0,3,this));
    inputPosition2_YLineEdit.push_back(newLineEdit4);

    ui->gridLayout_2->addWidget(newLineEdit3);
    ui->gridLayout_2->addWidget(newLineEdit4);

    newLineEdit->setGeometry(QRect(positionShowPositionX,positionShowPositionY*x,lableLength,lableWidth));
    newLineEdit->setAlignment(Qt::AlignCenter);
    newLineEdit->show();

    newLineEdit2->setGeometry(QRect(positionShowPositionX+110,positionShowPositionY*x,lableLength,lableWidth));
    newLineEdit2->setAlignment(Qt::AlignCenter);
    newLineEdit2->show();

    newLineEdit3->setGeometry(QRect(positionShowPositionX+250,positionShowPositionY*x,lableLength,lableWidth));
    newLineEdit3->setAlignment(Qt::AlignCenter);
    newLineEdit3->show();

    newLineEdit4->setGeometry(QRect(positionShowPositionX+360,positionShowPositionY*x,lableLength,lableWidth));
    newLineEdit4->setAlignment(Qt::AlignCenter);
    newLineEdit4->show();

    newLable->setGeometry(QRect(positionShowPositionX-30,positionShowPositionY*x,lableLength-80,lableWidth));
    newLable->show();
}

void calibration::on_pushButton_2_clicked()
{
    if(inputPosition_XLineEdit.size()>0)
    {
        auto iter=--inputPosition_XLineEdit.end();
        QLineEdit* tmpLineEdit=*iter;
        tmpLineEdit->setVisible(true);
        delete tmpLineEdit;
        inputPosition_XLineEdit.pop_back();

        iter=--inputPosition_YLineEdit.end();
        QLineEdit* tmpLineEdit2=*iter;
        tmpLineEdit2->setVisible(true);
        delete tmpLineEdit2;
        inputPosition_YLineEdit.pop_back();

        iter=--inputPosition2_XLineEdit.end();
        QLineEdit* tmpLineEdit3=*iter;
        tmpLineEdit3->setVisible(true);
        delete tmpLineEdit3;
        inputPosition2_XLineEdit.pop_back();

        iter=--inputPosition2_YLineEdit.end();
        QLineEdit* tmpLineEdit4=*iter;
        tmpLineEdit4->setVisible(true);
        delete tmpLineEdit4;
        inputPosition2_YLineEdit.pop_back();

        auto iter2=--inputPositionNumFlag.end();
        QLabel* tmpLable=*iter2;
        tmpLable->setVisible(true);
        delete tmpLable;
        inputPositionNumFlag.pop_back();
    }
}

void calibration::on_pushButton_3_clicked()
{
    int cols=ui->lineEdit_5->text().toInt();
    int rows=ui->lineEdit_6->text().toInt();
    imgSize=cv::Size(cols,rows);

    calibrateWorldPoints.clear();
    calibrateImagePoints.clear();
    if(inputPosition_XLineEdit.size()<4)
    {
        ui->textEdit->setTextColor(Qt::red);
        ui->textEdit->setText(QString("At least four valid point is needed to calibrate the camera!"));
        return;
    }
    auto iter1=inputPosition_XLineEdit.begin();
    auto iter2=inputPosition_YLineEdit.begin();
    auto iter3=inputPosition2_XLineEdit.begin();
    auto iter4=inputPosition2_YLineEdit.begin();
    QLineEdit* tmpLineEditPtr[4];
    while(iter1!=inputPosition_XLineEdit.end())
    {
        tmpLineEditPtr[0]=*iter1;
        tmpLineEditPtr[1]=*iter2;
        tmpLineEditPtr[2]=*iter3;
        tmpLineEditPtr[3]=*iter4;

        if(tmpLineEditPtr[0]->text().isEmpty()||tmpLineEditPtr[1]->text().isEmpty()||tmpLineEditPtr[2]->text().isEmpty()||tmpLineEditPtr[3]->text().isEmpty())
            break;
        cv::Point2f tmpImagePoint;
        cv::Point3f tmpWorldPoint;
        tmpImagePoint.x=tmpLineEditPtr[0]->text().toFloat();
        tmpImagePoint.y=tmpLineEditPtr[1]->text().toFloat();
        calibrateImagePoints.push_back(tmpImagePoint);

        tmpWorldPoint.x=tmpLineEditPtr[2]->text().toFloat();
        tmpWorldPoint.y=tmpLineEditPtr[3]->text().toFloat();
        tmpWorldPoint.z=0;
        calibrateWorldPoints.push_back(tmpWorldPoint);

        iter1++;
        iter2++;
        iter3++;
        iter4++;
    }
    if(calibrateImagePoints.size()<4)
    {
        ui->textEdit->setTextColor(Qt::red);
        ui->textEdit->setText(QString("At least four valid point is needed to calibrate the camera!"));
        return;
    }
    cv::Mat tmpCalibrateMat=cv::Mat::eye(cv::Size(2,inputPosition2_XLineEdit.size()),CV_64FC2);
    for(int i=0;i<inputPosition_XLineEdit.size();i++)
    {
        double d1=inputPosition_XLineEdit[i]->text().toDouble();
        double d2=inputPosition_YLineEdit[i]->text().toDouble();
        if(d1>imgSize.width||d2>imgSize.height)
        {
            ui->textEdit->setTextColor(Qt::red);
            ui->textEdit->setText(QString("the input coodinate out range!"));
            calibrateStatus=false;
            return;
        }
        tmpCalibrateMat.at<cv::Vec2d>(i,0)[0]=d1;
        tmpCalibrateMat.at<cv::Vec2d>(i,0)[1]=d2;
        tmpCalibrateMat.at<cv::Vec2d>(i,1)[0]=inputPosition2_XLineEdit[i]->text().toDouble();
        tmpCalibrateMat.at<cv::Vec2d>(i,1)[1]=inputPosition2_YLineEdit[i]->text().toDouble();
    }

    calibrateObj.setCalibrateMat(tmpCalibrateMat);
    calibrateObj.calibrateTheCamera(calibrateImagePoints,calibrateWorldPoints,imgSize);

    float evaluateResult=calibrateObj.evaluateCalibrateResult();
    if(evaluateResult==-1)
    {
        ui->textEdit->setTextColor(Qt::red);
        ui->textEdit->setText(QString("Failed to calibrate the camera!"));
        calibrateStatus=false;
    }
    else
    {
        ui->textEdit->setTextColor(Qt::blue);
        ui->textEdit->setText(QString("The calibrate result has ")+QString::number(evaluateResult,10,3)+QString(" pixels error!"));
        calibrateStatus=true;
    }
}

void calibration::on_pushButton_4_clicked()
{
    if(ui->lineEdit->text().isEmpty()||ui->lineEdit_2->text().isEmpty())
        return;
    cv::Point2f srcPoint,dstPoint;
    srcPoint.x=ui->lineEdit->text().toDouble();
    srcPoint.y=ui->lineEdit_2->text().toDouble();
    dstPoint=calibrateObj.calculateWorldPosition(srcPoint);
    if(calibrateObj.getCalibrateStatus())
    {
        ui->lineEdit_3->setText(QString::number(dstPoint.x,10,3));
        ui->lineEdit_4->setText(QString::number(dstPoint.y,10,3));
    }
    else
    {
        ui->textEdit->setTextColor(Qt::red);
        ui->textEdit->setText(QString("calculate statues error!"));
    }
}

void calibration::slot_calibrateCamera()
{
    calibrateWorldPoints.clear();
    calibrateImagePoints.clear();
    if(inputPosition_XLineEdit.size()<4)
    {
        ui->textEdit->setTextColor(Qt::red);
        ui->textEdit->setText(QString("At least four valid point is needed to calibrate the camera!"));
        return;
    }
    auto iter1=inputPosition_XLineEdit.begin();
    auto iter2=inputPosition_YLineEdit.begin();
    auto iter3=inputPosition2_XLineEdit.begin();
    auto iter4=inputPosition2_YLineEdit.begin();
    QLineEdit* tmpLineEditPtr[4];
    while(iter1!=inputPosition_XLineEdit.end())
    {
        tmpLineEditPtr[0]=*iter1;
        tmpLineEditPtr[1]=*iter2;
        tmpLineEditPtr[2]=*iter3;
        tmpLineEditPtr[3]=*iter4;

        if(tmpLineEditPtr[0]->text().isEmpty()||tmpLineEditPtr[1]->text().isEmpty()||tmpLineEditPtr[2]->text().isEmpty()||tmpLineEditPtr[3]->text().isEmpty())
            break;
        cv::Point2f tmpImagePoint;
        cv::Point3f tmpWorldPoint;
        tmpImagePoint.x=tmpLineEditPtr[0]->text().toFloat();
        tmpImagePoint.y=tmpLineEditPtr[1]->text().toFloat();
        calibrateImagePoints.push_back(tmpImagePoint);

        tmpWorldPoint.x=tmpLineEditPtr[2]->text().toFloat();
        tmpWorldPoint.y=tmpLineEditPtr[3]->text().toFloat();
        tmpWorldPoint.z=0;
        calibrateWorldPoints.push_back(tmpWorldPoint);

        iter1++;
        iter2++;
        iter3++;
        iter4++;
    }
    if(calibrateImagePoints.size()<4)
    {
        ui->textEdit->setTextColor(Qt::red);
        ui->textEdit->setText(QString("At least four valid point is needed to calibrate the camera!"));
        return;
    }
    cv::Mat tmpCalibrateMat=cv::Mat::eye(cv::Size(2,inputPosition2_XLineEdit.size()),CV_64FC2);
    for(int i=0;i<inputPosition_XLineEdit.size();i++)
    {
        tmpCalibrateMat.at<cv::Vec2d>(i,0)[0]=inputPosition_XLineEdit[i]->text().toDouble();
        tmpCalibrateMat.at<cv::Vec2d>(i,0)[1]=inputPosition_YLineEdit[i]->text().toDouble();
        tmpCalibrateMat.at<cv::Vec2d>(i,1)[0]=inputPosition2_XLineEdit[i]->text().toDouble();
        tmpCalibrateMat.at<cv::Vec2d>(i,1)[1]=inputPosition2_YLineEdit[i]->text().toDouble();
    }
    calibrateObj.calibrateTheCamera(calibrateImagePoints,calibrateWorldPoints,imgSize);
    calibrateObj.setCalibrateMat(tmpCalibrateMat);

    float evaluateResult=calibrateObj.evaluateCalibrateResult();
    if(evaluateResult==-1)
    {
        ui->textEdit->setTextColor(Qt::red);
        ui->textEdit->setText(QString("Failed to calibrate the camera!"));
        calibrateStatus=false;
    }
    else
    {
        ui->textEdit->setTextColor(Qt::blue);
        ui->textEdit->setText(QString("The calibrate result has ")+QString::number(evaluateResult,10,3)+QString(" pixels error!"));
        calibrateStatus=true;
    }
}

void calibration::on_pushButton_6_clicked()
{
    close();
}

void calibration::enableCalibrateUiIni()
{
    ui->lineEdit_3->setReadOnly(true);
    ui->lineEdit_4->setReadOnly(true);
    ui->textEdit->setReadOnly(true);
    ui->lineEdit->setAlignment(Qt::AlignCenter);
    ui->lineEdit_2->setAlignment(Qt::AlignCenter);
    ui->lineEdit_3->setAlignment(Qt::AlignCenter);
    ui->lineEdit_4->setAlignment(Qt::AlignCenter);

    cv::Mat tmpCalibrateMat=calibrateObj.getCalibrateMat();
    imgSize=calibrateObj.getCalibrateImgSize();
    try
    {
        for(int i=0;i<tmpCalibrateMat.rows;i++)
        {
            on_pushButton_clicked();
        }
        for(int i=0;i<tmpCalibrateMat.rows;i++)
        {
            inputPosition_XLineEdit[i]->setText(QString::number(tmpCalibrateMat.at<cv::Vec2d>(i,0)[0],10,3));
            inputPosition_YLineEdit[i]->setText(QString::number(tmpCalibrateMat.at<cv::Vec2d>(i,0)[1],10,3));
            inputPosition2_XLineEdit[i]->setText(QString::number(tmpCalibrateMat.at<cv::Vec2d>(i,1)[0],10,3));
            inputPosition2_YLineEdit[i]->setText(QString::number(tmpCalibrateMat.at<cv::Vec2d>(i,1)[1],10,3));
        }
        ui->lineEdit_5->setText(QString::number(imgSize.width,10));
        ui->lineEdit_6->setText(QString::number(imgSize.height,10));
    }
    catch(cv::Exception ex)
    {
        QMessageBox msg;
        msg.setText(QString(ex.what()));
        msg.exec();
    }
}
