#ifndef VISION_H
#define VISION_H
#include<QString>

#define NON_ERROR 0                     ///////无错误
#define CAMERA_ERROR 1                  //////相机错误
#define NCC_MATCH_ERROR 2               //////匹配错误
#define CALIBRATE_ERROR 3               //////标定错误
#define STATION_OR_MODEL_ERROR 4        //////工位或者型号数据错误
#define UNKNOW_ERROR 5                  ///////未知错误


typedef struct strVisionParam{
    QString stationName;
    std::string cameraSerialNum=std::string("null");
    int cameraExposeTime=0;
    bool isEnableCalibrate=false;
    bool isSaveRecord=false;
    int maxImgSaveNum=0;
}strVisionParam;

typedef struct STR_Vision_Result{
    int runStation=-1;
    int runModel=-1;

    double x=-1;
    int x_addr=-1;

    double y=-1;
    int y_addr=-1;

    double z=-1;
    int z_addr=-1;

    double angle=-1;
    int angle_addr=-1;

    int model=-1;
    int model_addr=-1;

    int errorFlag=NON_ERROR;
    int errorFlag_addr=-1;

    double u=-1;
    double v=-1;
    bool matchStatus=false;
    double socore=-2;

    bool x_flag=false;
    bool y_flag=false;
    bool z_flag=false;
    bool model_flag=false;
    bool angle_flag=false;
    bool u_flag=false;
    bool v_flag=false;
    bool errorFlag_flag=false;
    bool socore_flag=false;
    bool matchStatus_flag;
}STR_Vision_Result;

#endif // VISION_H
