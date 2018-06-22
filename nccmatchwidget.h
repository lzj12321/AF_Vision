#ifndef NCCMATCHWIDGET_H
#define NCCMATCHWIDGET_H

#include <QWidget>
#include<QMouseEvent>
#include"cvandui.h"
#include"ncc_match.h"
#include<vision.h>
#include<QCloseEvent>

namespace Ui {
class nccMatchWidget;
}

class nccMatchWidget : public QWidget,cvAndUi
{
    Q_OBJECT
public:
    explicit nccMatchWidget(QWidget *parent = 0);
    ~nccMatchWidget();
    STR_Vision_Result ncc_Match(cv::Mat&mat,int model=-1);
    STR_Vision_Result getNccMatchResultFormat();
    cv::Mat getResultImg(cv::Mat&src);
    cv::Point2f getMatchPoint();
    float getMatchAngle();
    void pointTransmit(cv::Point2f&p);
    void nccInitialize(int model=-1);
    void setParamPath(QString path);
    void nccWidgetIni(cv::Mat&t);
    int currentMatchModel=-1;
    int getCurrentMatchModel();
protected:
    void closeEvent(QCloseEvent *event);
private slots:
    void on_pushButton_4_clicked();
    void on_pushButton_6_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_7_clicked();
    void on_pushButton_clicked();
    void on_pushButton_5_clicked();
    void on_pushButton_8_clicked();
    void on_pushButton_9_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_10_clicked();
    void on_comboBox_currentIndexChanged(int index);
    void on_pushButton_11_clicked();
    void on_lineEdit_6_textEdited(const QString &arg1);
    void on_lineEdit_2_textEdited(const QString &arg1);
    void on_lineEdit_textEdited(const QString &arg1);

private:
    int modelNum=0;
    int comboxCurrentIndex=-1;
    std::string paramPath;
    Ui::nccMatchWidget *ui;
    cv::Mat sourceMat;
    cv::Mat templateMat,maskMat,tempMaskShowMat;
    cv::Mat temporyTempMat,temporyMaskMat,temporyTempMaskShowMat;

    bool isEnableMask=true;
    bool isEnableRotate=true;

    int roiX=0;
    int roiY=0;
    int roiWidth=0;
    int roiHeight=0;

    int clickX=0;
    int clickY=0;
    int releaseX=0;
    int releaseY=0;
    bool pRoiStatus=false;
    bool pTempStatus=false;
    bool pMaskStatus=false;
    bool drawMaskStatus=false;
    double ratio=0;
    NCC_Match nccmatch;
    int nccMatchTime=0;
    bool pencileStatus=false;
    bool eraserStatus=false;

    std::vector<QString> vecModelName;
    std::vector<int>vecRoiX;
    std::vector<int>vecRoiY;
    std::vector<int>vecRoiWidth;
    std::vector<int>vecRoiHeight;
    std::vector<int>vecDownSampleTime;
    std::vector<int>vecAngleMatchRange;
    std::vector<QString>vecMatchThreshold;
    std::vector<int>vecThresholdValue;
private:
    bool nccGenerateTemplate(cv::Mat&temp,cv::Mat&mask,bool isEnableMask,bool isEnableRotate);
    bool nccGenerateTemplate(cv::Mat&temp,cv::Mat&mask,NCC_Match&nccmatch,bool isEnableMask,bool isEnableRotate,uint downSampleTime=0);
    void outPutMatchResult();
    void outPutMatchResult(NCC_Match&nccmatch);
    cv::Mat drawResultOnMat();
    cv::Mat drawResultOnMat(NCC_Match&nccmatch);
    void getRoiMat(cv::Mat&src,cv::Mat&mat,int x,int y,int w,int h);
    void getRoiMat(cv::Mat&src,cv::Mat&mat);
    void restorePaintingStatus();
    void uiInitialize(int model);
    void imgLoad(int index=0);
    void imgSave(int index);
    void paramSave();
    void paramLoad();
    void setModelParam(int model);
    void drawRoiOnMat(cv::Mat&src);
    void drawRoiOnMat(cv::Mat &src,int x,int y,int w,int h);
    void addNewModel(const QString& newModelName);
    void delModel(int index);
    void delModelParam(int index);
    void delModelImg(int index);
    void preProcessTemplate(cv::Mat&temp,int thresholdValue=0);
    void preProcessSource(cv::Mat&src,int thresholdValue=0);
private:
    cv::Point matchPoint;
    float matchScore=-2;
    ushort matchAngle=0;
    bool matchStatus=false;
    bool nccStatus=false;
protected:
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
};

#endif // NCCMATCHWIDGET_H
