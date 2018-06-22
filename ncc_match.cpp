#include "ncc_match.h"
#include<iostream>
#include<QTime>
#include<QBitArray>
#include<omp.h>
#include<QDebug>

using namespace std;
#define PI 3.141592654

NCC_Match::NCC_Match()
{
    rotateTempStepSize=1;
    coarseMatchAngleStepSize=3;
    preciseMatchAngleStepSize=1;
}

NCC_Match::~NCC_Match()
{
    if(layerThreshold!=nullptr){
        delete[] layerThreshold;
        layerThreshold=nullptr;
    }
    if(layerAngleMatchRange!=nullptr){
        delete[] layerAngleMatchRange;
        layerAngleMatchRange=nullptr;
    }
    if(layerCoordinateMatchRange!=nullptr){
        delete[] layerCoordinateMatchRange;
        layerCoordinateMatchRange=nullptr;
    }
}

void NCC_Match::paramInitialize(ushort num)
{
    if(layerThreshold!=nullptr){
        delete[] layerThreshold;
        layerThreshold=nullptr;
    }
    if(layerAngleMatchRange!=nullptr){
        delete[] layerAngleMatchRange;
        layerAngleMatchRange=nullptr;
    }
    if(layerCoordinateMatchRange!=nullptr){
        delete[] layerCoordinateMatchRange;
        layerCoordinateMatchRange=nullptr;
    }

    layerThreshold=new float[num+1];
    layerAngleMatchRange=new ushort[num+1];
    layerCoordinateMatchRange=new ushort[num+1];
    for(int i=num;i>=0;i--){
        layerThreshold[i]=0.5;
        layerAngleMatchRange[i]=3;
        layerCoordinateMatchRange[i]=2;
        if(i==1||i==0){
            layerAngleMatchRange[i]=1;
            layerCoordinateMatchRange[i]=1;
        }
    }
}

void NCC_Match::setTemplateAndMask(cv::Mat &tmp,cv::Mat &mask,ushort downSampleTime)
{
    if(tmp.cols!=mask.cols||tmp.rows!=mask.rows||tmp.empty()||mask.empty()){
        cout<<"tmp.cols!=mask.cols||tmp.rows!=mask.rows||tmp.empty()||mask.empty()"<<endl;
        return;
    }
    QTime timer;
    timer.start();
    if(tmp.type()!=CV_8UC1){
        cv::cvtColor(tmp,templateMat,cv::COLOR_RGB2GRAY);
    }
    else
        templateMat=tmp.clone();

    if(mask.type()!=CV_8UC1){
        cv::cvtColor(mask,maskMat,cv::COLOR_RGB2GRAY);
    }
    else
        maskMat=mask.clone();

    if(downSampleTime==0)
    {
        downSamplingNum=defaultDownSampleTime;
    }
    else
    {
        downSamplingNum=downSampleTime;
    }
    cout<<"AI choose downsample time:"<<downSamplingNum<<endl;
    paramInitialize(downSamplingNum);
    initializeTemplate(tmp,mask,downSamplingNum);
    cout<<"initialize template time:"<<timer.elapsed()<<endl;
}

void NCC_Match::generateTemplateAndMask(cv::Mat &tmp, cv::Mat &mask, ushort sampleTime)
{
    if(tmp.cols!=mask.cols||tmp.rows!=mask.rows||tmp.empty()||mask.empty()){
        cout<<"tmp.cols!=mask.cols||tmp.rows!=mask.rows||tmp.empty()||mask.empty()"<<endl;
        return;
    }
    QTime timer;
    timer.start();
    if(tmp.type()!=CV_8UC1){
        cv::cvtColor(tmp,templateMat,cv::COLOR_RGB2GRAY);
    }
    else
        templateMat=tmp.clone();

    if(mask.type()!=CV_8UC1){
        cv::cvtColor(mask,maskMat,cv::COLOR_RGB2GRAY);
    }
    else
        maskMat=mask.clone();

    if(sampleTime==0)
    {
        downSamplingNum=defaultDownSampleTime;
    }
    else
    {
        downSamplingNum=sampleTime;
    }
    paramInitialize(downSamplingNum);
    generateTemplate(tmp,mask,downSamplingNum);
}

void NCC_Match::initializeTemplate(cv::Mat &tmp, cv::Mat &mask,ushort initializeLayer)
{
    VEC_rotateTempData.reserve(downSamplingNum+1);
    for(int i=0;i<VEC_rotateTempData.size();++i){
        std::vector<str_img_data>().swap(VEC_rotateTempData[i]);
    }
    std::vector<std::vector<str_img_data>>().swap(VEC_rotateTempData);
    mask=mask/255;
    for(ushort i=0;i<=initializeLayer;++i){
        cv::Mat downSampleTmp;
        imgDownSampling(tmp,downSampleTmp,i);
        cv::Mat downSampleMask;
        imgDownSampling(mask,downSampleMask,i);
        std::vector<cv::Mat> _vec_rotateTempImg;
        std::vector<cv::Mat> _vec_rotateMaskImg;
        _vec_rotateTempImg.reserve(360);
        _vec_rotateMaskImg.reserve(360);
        generateRotateTemplate(rotateTempStepSize,downSampleTmp,_vec_rotateTempImg);
        generateRotateTemplate(rotateTempStepSize,downSampleMask,_vec_rotateMaskImg);
        std::vector<str_img_data> _vec_imgData;
        _vec_imgData.reserve(360);
        for(short int j=0;j<_vec_rotateTempImg.size();++j){
            str_img_data imgData;
            calculateImgData(_vec_rotateTempImg[j],_vec_rotateMaskImg[j],&imgData);
            _vec_imgData.push_back(imgData);
        }
        VEC_rotateTempData.push_back(_vec_imgData);
    }
}

void NCC_Match::generateTemplate(cv::Mat &tmp, cv::Mat &mask, ushort DownSampleTime)
{
    for(int i=0;i<VEC_rotateTempData.size();++i){
        VEC_rotateTempData[i].clear();
    }
    VEC_rotateTempData.clear();
    vec_downSampleTemp.clear();
    vec_downSampleMask.clear();

    vec_downSampleTemp.push_back(tmp);
    vec_downSampleMask.push_back(mask);
    for(int i=0;i<=DownSampleTime;++i){
        cv::Mat downSampleTmp;
        imgDownSampling(vec_downSampleTemp[i],downSampleTmp,1);
        vec_downSampleTemp.push_back(downSampleTmp);
        cv::Mat downSampleMask;
        imgDownSampling(vec_downSampleMask[i],downSampleMask,1);
        vec_downSampleMask.push_back(downSampleMask);
    }

    for(int i=DownSampleTime;i>DownSampleTime-generateTempDataLayer;--i){
        std::vector<cv::Mat> _vec_rotateTempImg;
        std::vector<cv::Mat> _vec_rotateMaskImg;
        _vec_rotateTempImg.reserve(360);
        _vec_rotateMaskImg.reserve(360);
        generateRotateTemplate(rotateTempStepSize,vec_downSampleTemp[i],_vec_rotateTempImg);
        generateRotateTemplate(rotateTempStepSize,vec_downSampleMask[i],_vec_rotateMaskImg);
        std::vector<str_img_data> _vec_imgData;
        _vec_imgData.reserve(360);
        for(short int j=0;j<_vec_rotateTempImg.size();++j){
            str_img_data imgData;
            calculateImgData(_vec_rotateTempImg[j],_vec_rotateMaskImg[j],&imgData);
            _vec_imgData.push_back(imgData);
        }
        VEC_rotateTempData.push_back(_vec_imgData);
    }
}

void NCC_Match::calculateImgData(cv::Mat&temp, cv::Mat &mask,str_img_data* imgData)
{
    imgData->imgCols=temp.cols;
    imgData->imgRows=temp.rows;
    imgData->diffMat=cv::Mat(cv::Size(temp.cols,temp.rows),CV_32FC1,cv::Scalar::all(0));

    cv::Mat tempMat32f;
    imgData->maskMat=mask.clone();
    double sum=0;
    if(temp.type()!=CV_32FC1){
        temp.convertTo(tempMat32f,CV_32FC1, 1.0/255, 0);
        sum=temp.dot(mask);
    }else{
        temp.copyTo(tempMat32f);
        cv::Mat t;;
        mask.convertTo(t,CV_32FC1, 1.0/255, 0);
        sum=tempMat32f.dot(t);
    }
    imgData->validPixelNum=imgData->maskMat.dot(imgData->maskMat);
    float mean=sum/imgData->validPixelNum;
    cv::Mat meanMat=cv::Mat(cv::Size(temp.cols,temp.rows),CV_32FC1,cv::Scalar::all(mean));
    imgData->diffMat=tempMat32f-meanMat;
    imgData->imgStandardDeviation=sqrt(imgData->diffMat.dot(imgData->diffMat));
}

inline void NCC_Match::_calculateImgData(cv::Mat &temp, cv::Mat &mask, str_img_data *imgData)
{
    imgData->diffMat=cv::Mat(cv::Size(temp.cols,temp.rows),CV_32FC1,cv::Scalar::all(0));

    cv::Mat tempMat32f;
    double sum=0;
    if(temp.type()!=CV_32FC1){
        temp.convertTo(tempMat32f,CV_32FC1, 1.0/255, 0);
        sum=temp.dot(mask);
    }else{
        temp.copyTo(tempMat32f);
        cv::Mat t;;
        mask.convertTo(t,CV_32FC1, 1.0/255, 0);
        sum=tempMat32f.dot(t);
    }
    imgData->validPixelNum=mask.dot(mask);
    float mean=sum/imgData->validPixelNum;
    cv::Mat meanMat=cv::Mat(cv::Size(temp.cols,temp.rows),CV_32FC1,cv::Scalar::all(mean));
    imgData->diffMat=tempMat32f-meanMat;
    imgData->imgStandardDeviation=sqrt(imgData->diffMat.dot(imgData->diffMat));
}

NccMatchResult NCC_Match::nccMatch(cv::Mat &src, float thresholdValue,int anleMatchRange, ushort matchNum)
{
    NccMatchResult bestMatchPoint;
    dataClearBeforeMatch();

    if(VEC_rotateTempData.size()==0||src.empty()||src.cols<VEC_rotateTempData[0][0].imgCols||src.rows<VEC_rotateTempData[0][0].imgRows)
    {
        qDebug()<<"VEC_rotateTempData.size()==0||src.empty()||src.cols<VEC_rotateTempData[0][0].imgCols||src.rows<VEC_rotateTempData[0][0].imgRows";
        VEC_matchPointsxx.clear();
        return bestMatchPoint;
    }

    generateDownSampleSrc(src);
    std::vector<std::vector<NccMatchResult>>VEC_matchPoints;
    ///////////corase match return the stop layer///////////////
    nccCoarseMatch(downSamplingNum,VEC_matchPoints,anleMatchRange);
    if(VEC_matchPoints[0].size()==0)
    {
        qDebug()<<"VEC_matchPoints[0].size()==0";
        return bestMatchPoint;
    }
    ///////////precise match start match form the stop layer //
    int layer=downSamplingNum;
    nccPreciseMatch(layer,VEC_matchPoints);

//    if(VEC_matchPoints.size()<=layer){
//        qDebug()<<"VEC_matchPoints.size()<=(startMatchLayer-stopMatchLayer)";
//        qDebug()<<"VEC_matchPoints.size():"<<VEC_matchPoints.size();
//        return bestMatchPoint;
//    }

    ///////////filtrate best match point//////////////////////
    bestMatchPointFilter(VEC_matchPoints[downSamplingNum],bestMatchPoint);
    if(bestMatchPoint.matchValue<thresholdValue){
        qDebug()<<"bestMatchPoint.matchValue<thresholdValue";
        return bestMatchPoint;
    }
    VEC_matchPointsxx=VEC_matchPoints;
    return bestMatchPoint;
}

void NCC_Match::rotateMat(cv::Mat&temp,cv::Mat&dst, float rotateAngle)
{
    double width=sqrt(pow(temp.rows,2)+pow(temp.cols,2));
    width=cvRound(width);
    cv::Size2f rotateResultImgSize=cv::Size2f(width,width);
    cv::Mat rotateTemp;
    rotateTemp=cv::Mat(rotateResultImgSize,CV_8UC1,cv::Scalar::all(0));

    ushort offsetX=cvRound(width/2.0-temp.cols/2.0);
    ushort offsetY=cvRound(width/2.0-temp.rows/2.0);
    cv::Mat temporyMat=rotateTemp(cv::Rect(offsetX,offsetY,temp.cols,temp.rows));
    temp.copyTo(temporyMat);

    cv::Point rotateTempCenter=cv::Point(cvRound(rotateTemp.cols/2.0),cvRound(rotateTemp.rows/2.0));
    cv::Mat rotateTempMat = cv::getRotationMatrix2D(rotateTempCenter,rotateAngle, 1.0);
    cv::warpAffine(rotateTemp, rotateTemp, rotateTempMat,rotateResultImgSize,1,0,0);

    double angle=rotateAngle*PI/180.0;
    double sinx=fabs(sin(angle));
    double cosx=fabs(cos(angle));
    ushort h=cvRound(temp.cols*sinx+temp.rows*cosx);
    ushort w=cvRound(temp.cols*cosx+temp.rows*sinx);
    cv::Size2i sizex=cv::Size2i(w,h);
    dst=cv::Mat(sizex,CV_8UC1,cv::Scalar::all(0));

    cv::Point startPoint=cv::Point(cvRound(rotateTempCenter.x-w/2.0),cvRound(rotateTempCenter.y-h/2.0));
    rotateTemp(cv::Rect(startPoint.x,startPoint.y,w,h)).copyTo(dst);
}

void NCC_Match::generateRotateTemplate(ushort stepSize,cv::Mat &temp,std::vector<cv::Mat>&vec_dstMat,ushort startAngle)
{
    ushort i=startAngle;
    while(i<360){
        cv::Mat rotateTemp;
        rotateMat(temp,rotateTemp,(float)i);
        vec_dstMat.push_back(rotateTemp);
        i+=stepSize;
    }
}

void NCC_Match::generateDownSampleSrc(cv::Mat &src)
{
    vec_downSampleSrcImg.push_back(src);
    for(ushort i=1;i<=downSamplingNum;++i){
        cv::Mat mat;
        ushort cols=vec_downSampleSrcImg[i-1].cols;
        ushort rows=vec_downSampleSrcImg[i-1].rows;
        cols=cvRound(cols/2.0);
        rows=cvRound(rows/2.0);
        cv::pyrDown(vec_downSampleSrcImg[i-1],mat,cv::Size(cols,rows));
        vec_downSampleSrcImg.push_back(mat);
    }
}

void NCC_Match::nccCoarseMatch(ushort matchLayer,std::vector<std::vector<NccMatchResult>>&VEC_matchPoints,ushort angleMatchRange)
{       
    testTimer.restart();
    //////////////匹配范围由模板较小的那条边的一半决定//////////////////////
    ushort startMatchX=VEC_rotateTempData[downSamplingNum-matchLayer][0].imgCols>>1;
    ushort startMatchY=VEC_rotateTempData[downSamplingNum-matchLayer][0].imgRows>>1;

    ushort minValue=startMatchX>startMatchY?startMatchY:startMatchX;
    ushort colRange=vec_downSampleSrcImg[matchLayer].cols-minValue;
    ushort rowRange=vec_downSampleSrcImg[matchLayer].rows-minValue;
    std::vector<NccMatchResult> matchPoints;
    short int i=minValue;
    for(i=minValue;i<colRange;++i){
#pragma omp parallel for
        for(short int j=minValue;j<rowRange;++j){
            cv::Point point(i,j);
            ushort angle=0;
            float value=-2;
            nccRotateMatch(matchLayer,value,point,angle,angleMatchRange,coarseMatchAngleStepSize);
#pragma omp critical
            if(value>layerThreshold[matchLayer])
            {
                NccMatchResult tmpMatchPointData;
                tmpMatchPointData.matchAngle=angle;
                tmpMatchPointData.matchPoint=point;
                tmpMatchPointData.matchValue=value;
                matchPoints.push_back(tmpMatchPointData);
            }
        }
    }
    VEC_matchPoints.push_back(matchPoints);
    vec_matchTime.push_back(testTimer.elapsed());
}


void NCC_Match::nccPreciseMatch(int layer,std::vector<std::vector<NccMatchResult>>&VEC_matchPoints)
{
    for(short int i=layer-1;i>=0;--i){
        testTimer.restart();
        int srcCol=vec_downSampleSrcImg[i].cols;
        int srcRow=vec_downSampleSrcImg[i].rows;
        int num=srcCol*srcRow*360;
        QBitArray flagBitArray(num);
        flagBitArray.fill(false);

        std::vector<NccMatchResult> validMatchPoints;
        int index=layer-i-1;

        int m=layerAngleMatchRange[i];
        int n=layerCoordinateMatchRange[i];
        int vecSize=VEC_matchPoints[index].size();

        NccMatchResult layerMaxMatchPoint;
        layerMaxMatchPoint.matchValue=-2;
        for(short int j=0;j<vecSize;++j){
            short int x=(VEC_matchPoints[index][j].matchPoint.x<<1)-n;
            short int y=(VEC_matchPoints[index][j].matchPoint.y<<1)-n;
            x=(x>0)?x:0;
            y=(y>0)?y:0;
            //////////different layer has different match range/////////////////
            ushort pointMatchRange=n<<1;
            ushort angleMatchRange=m<<1;

            NccMatchResult maxMatchPointData;
            maxMatchPointData.matchValue=-2;
            short int offsetX=0;
            for(offsetX=0;offsetX<=pointMatchRange;++offsetX){
                short int offsetY=0;
#pragma omp parallel for private(offsetY)// schedule(dynamic) /*shared(_angle,_x,_y,_value)*/
                for(offsetY=0;offsetY<=pointMatchRange;++offsetY){
                    uint xx=x+offsetX;
                    uint yy=y+offsetY;
                    ushort matchAngle=(VEC_matchPoints[index][j].matchAngle-m+360)%360;
                    cv::Point point(xx,yy);
                    int tempNum=xx*srcRow*360+yy*360+matchAngle;
                    if(flagBitArray.at(tempNum))
                        continue;
                    short int offsetAngle=0;
                    for(offsetAngle=0;offsetAngle<=angleMatchRange;++offsetAngle){
                        ///////point match//////////////////
                        float value=-2;
                        if(i<=downSamplingNum-generateTempDataLayer){
                            str_img_data tempData;
                            cv::Mat rotateTemp,rotateMask;
                            rotateMat(vec_downSampleTemp[i],rotateTemp,matchAngle);
                            rotateMat(vec_downSampleMask[i],rotateMask,matchAngle);
                            calculateImgData(rotateTemp,rotateMask,&tempData);
                            nccPointMatch(vec_downSampleSrcImg[i],tempData,point,&value);
                        }else
                            nccPointMatch(vec_downSampleSrcImg[i],VEC_rotateTempData[downSamplingNum-i][matchAngle],point,&value);
#pragma omp critical
                        {
                            if(value>=layerThreshold[i]&&value>maxMatchPointData.matchValue)
                            {
                                maxMatchPointData.matchAngle=matchAngle;
                                maxMatchPointData.matchPoint=point;
                                maxMatchPointData.matchValue=value;
                            }
                        }
                        matchAngle=(matchAngle+preciseMatchAngleStepSize)%360;
                    }
                    flagBitArray.setBit(tempNum,true);
                }
            }
            if(maxMatchPointData.matchValue!=-2){
                validMatchPoints.push_back(maxMatchPointData);
            }
        }
        if(validMatchPoints.size()==0){
            cout<<"this layer no match point! "<<i<<endl;
            cout<<"this layer threshold:"<<layerThreshold[i]<<endl;
            break;
        }
        if(i<=4){
            bestMatchPointFilter(validMatchPoints,layerMaxMatchPoint);
            std::vector<NccMatchResult>().swap(validMatchPoints);
            validMatchPoints.push_back(layerMaxMatchPoint);
        }

        VEC_matchPoints.push_back(validMatchPoints);
        vec_matchTime.push_back(testTimer.elapsed());
    }
}

void NCC_Match::nccRotateMatch(ushort matchLayer,float&matchValue,cv::Point&matchPoint,ushort&bestMatchAngle,ushort angleMatchRange,ushort matchStepSize)
{
    ushort num=angleMatchRange<<1;
    ushort angle=360-angleMatchRange;
    for(ushort i=0;i<=num;i+=matchStepSize){
        ushort index=(i+angle)%360;
        float tmpValue=-2;
        nccPointMatch(vec_downSampleSrcImg[matchLayer],VEC_rotateTempData[downSamplingNum-matchLayer][index],matchPoint,&tmpValue);
        if(tmpValue>matchValue){
            bestMatchAngle=index;
            matchValue=tmpValue;
        }
    }
}

inline void NCC_Match::nccPointMatch(cv::Mat &src, str_img_data &tmpImgData, cv::Point& matchPoint,float*value)
{
    /////////////////////////在大模板的情况下旋转会导致上下都超出界限导致运算错误///////////////////////////
    bool isTmpOutRange=false;
    ushort validTmpX,validTmpY,validWidth,validHeight;
    ushort validSrcX,validSrcY;
    short int x;
    short int y;
    x=cvRound(matchPoint.x-tmpImgData.imgCols/2.0);
    y=cvRound(matchPoint.y-tmpImgData.imgRows/2.0);
    if(x>=0){
        validTmpX=0;
        validSrcX=x;
        validWidth=tmpImgData.imgCols;
    }else{
        validWidth=tmpImgData.imgCols+x;
        isTmpOutRange=true;
        validTmpX=abs(x);
        validSrcX=0;
    }

    if(y>=0){
        validTmpY=0;
        validSrcY=y;
        validHeight=tmpImgData.imgRows;
    }else{
        validHeight=tmpImgData.imgRows+y;
        isTmpOutRange=true;
        validTmpY=abs(y);
        validSrcY=0;
    }

    ///////temp col out range////////////
    if((validSrcX+validWidth)>src.cols){
        isTmpOutRange=true;
        validWidth=src.cols-validSrcX;
    }

    ///////temp row out range////////////
    if((validSrcY+validHeight)>src.rows){
        isTmpOutRange=true;
        validHeight=src.rows-validSrcY;
    }

    if(isTmpOutRange){
        str_img_data tmpData,srcData;
        cv::Mat srcMat;
        src(cv::Rect(validSrcX,validSrcY,validWidth,validHeight)).copyTo(srcMat);
        if((validTmpX+validWidth)>tmpImgData.imgCols||(validTmpY+validHeight)>tmpImgData.imgRows){
            *value=-2;
            return;
        }

        cv::Mat tempDiffMat,tempMaskMat;
        tmpImgData.diffMat(cv::Rect(validTmpX,validTmpY,validWidth,validHeight)).copyTo(tempDiffMat);
        tmpImgData.maskMat(cv::Rect(validTmpX,validTmpY,validWidth,validHeight)).copyTo(tempMaskMat);
        _calculateImgData(tempDiffMat,tempMaskMat,&tmpData);
        _calculateImgData(srcMat,tempMaskMat,&srcData);
        *value=processNccValue(srcData,tmpData);
    }else{
        cv::Mat srcMat;
        str_img_data srcData;
        src(cv::Rect(validSrcX,validSrcY,validWidth,validHeight)).copyTo(srcMat);
        _calculateImgData(srcMat,tmpImgData.maskMat,&srcData);
        *value=processNccValue(srcData,tmpImgData);
    }
}

NCC_Match NCC_Match::operator=(const NCC_Match&)
{
    return *this;
}

inline float NCC_Match::processNccValue(str_img_data &imgData1, str_img_data &imgData2)
{
    double value=imgData1.diffMat.dot(imgData2.diffMat);
    return value/(imgData1.imgStandardDeviation*imgData2.imgStandardDeviation);
}


void NCC_Match::imgDownSampling(cv::Mat&src,cv::Mat&dst,ushort layerNum)
{
    dst=src;
    if(layerNum==0)return;
    ushort cols=src.cols;
    ushort rows=src.rows;
    for(ushort i=0;i<layerNum;++i){
        cols=cvRound(cols/2.0);
        rows=cvRound(rows/2.0);
        cv::pyrDown(dst,dst,cv::Size(cols,rows));
    }
}

void NCC_Match::bestMatchPointFilter(std::vector<NccMatchResult>&src,NccMatchResult&dst)
{
    float value=-2;
    int index=-1;
    for(int i=0;i<src.size();++i){
        if(src[i].matchValue>value){
            value=src[i].matchValue;
            index=i;
        }
    }
    if(index>=0)
        dst=src[index];
}

void NCC_Match::dataClearBeforeMatch()
{
    vec_downSampleSrcImg.clear();
    vec_matchTime.clear();
}

cv::Mat NCC_Match::resizeMat(cv::Mat& src, float i)
{
    cv::Mat dst;
    cv::resize(src, dst, cv::Size(cvRound( src.cols / i),cvRound( src.rows / i)));
    return dst;
}
