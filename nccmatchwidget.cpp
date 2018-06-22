#include "nccmatchwidget.h"
#include "ui_nccmatchwidget.h"
#include<QMessageBox>
#include <QFileDialog>
#include <QTextCodec>
#include<QSettings>
#include<QTextStream>
#include<QDebug>
#include<QInputDialog>

#define roiColour cv::Scalar(255,200,50)

nccMatchWidget::nccMatchWidget(QWidget *parent):
    QWidget(parent),
    ui(new Ui::nccMatchWidget)
{
    ui->setupUi(this);
}

nccMatchWidget::~nccMatchWidget()
{
    delete ui;
}

STR_Vision_Result nccMatchWidget::ncc_Match(cv::Mat &srcMat,int model)
{
    STR_Vision_Result str_nccMatchResult;
    --model;
    matchScore=-2;
    matchAngle=0;
    matchStatus=false;

    if(model<0||modelNum==0||model>=modelNum){
        str_nccMatchResult.errorFlag=STATION_OR_MODEL_ERROR;
        return str_nccMatchResult;
    }
    if(srcMat.empty())return str_nccMatchResult;
    if(model!=currentMatchModel)
        nccInitialize(model);

    if(srcMat.type()!=CV_8UC1){
        cv::cvtColor(srcMat,srcMat,cv::COLOR_RGB2GRAY);
    }
    cv::Mat roiMat;
    getRoiMat(srcMat,roiMat);

    nccmatch.nccMatch(roiMat,vecMatchThreshold[model].toDouble(),true,true);
    matchStatus=nccmatch.matchStatus;
    if(matchStatus){
        str_nccMatchResult.x=nccmatch.nccMatchPoint.x+roiX;
        str_nccMatchResult.y=nccmatch.nccMatchPoint.y+roiY;
        str_nccMatchResult.angle=nccmatch.nccMatchAngle;
        str_nccMatchResult.socore=nccmatch.nccMatchScore;
        str_nccMatchResult.model=model+1;
        str_nccMatchResult.errorFlag=1;
        str_nccMatchResult.matchStatus=matchStatus;

        matchPoint=nccmatch.nccMatchPoint;
        matchAngle=nccmatch.nccMatchAngle;
        matchScore=nccmatch.nccMatchScore;
    }else{
        str_nccMatchResult.model=0;
        str_nccMatchResult.errorFlag=0;
        str_nccMatchResult.matchStatus=matchStatus;
    }
    return str_nccMatchResult;
}

STR_Vision_Result nccMatchWidget::getNccMatchResultFormat()
{
    STR_Vision_Result resultFormat;
    resultFormat.angle_flag=true;
    resultFormat.x_flag=true;
    resultFormat.errorFlag_flag=true;
    resultFormat.matchStatus_flag=true;
    resultFormat.model_flag=true;
    resultFormat.z_flag=false;
    resultFormat.y_flag=true;
    resultFormat.v_flag=false;
    resultFormat.u_flag=false;
    resultFormat.socore_flag=true;
    return resultFormat;
}

cv::Mat nccMatchWidget::getResultImg(cv::Mat&src)
{
    if(src.type()!=CV_8UC3){
        cv::cvtColor(src,src,cv::COLOR_GRAY2BGR);
    }
    sourceMat=src.clone();
    return drawResultOnMat();
}

cv::Point2f nccMatchWidget::getMatchPoint()
{
    cv::Point2f p(matchPoint.x,matchPoint.y);
    return p;
}

float nccMatchWidget::getMatchAngle()
{
    return matchAngle;
}

void nccMatchWidget::on_pushButton_4_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,tr("Open Image"),".",tr("Image File (*.jpg *.png *.bmp *.jpeg)"));
    QTextCodec *code = QTextCodec::codecForName("gb18030");
    std::string name = code->fromUnicode(filename).data();
    if(!name.empty())
    {
        cv::Mat src=cv::imread(name,1);
        if(!src.empty())
        {
            sourceMat=src.clone();
            if(sourceMat.type()!=CV_8UC3)
                cv::cvtColor(sourceMat,sourceMat,cv::COLOR_GRAY2BGR);
            cv::Mat s=sourceMat.clone();
            // preProcessSource(s);
            drawRoiOnMat(s,roiX,roiY,roiWidth,roiHeight);
            showMatOnDlg(s,ui->label_6,&ratio);
            restorePaintingStatus();
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("Image Data Is Null");
            msgBox.exec();
        }
    }
}

void nccMatchWidget::on_pushButton_6_clicked()
{
    restorePaintingStatus();
    pRoiStatus=true;
    ui->textEdit->clear();
    ui->textEdit->append(QString("In the process of set ROI"));
    showMatOnDlg(sourceMat,ui->label_6,&ratio);
}

void nccMatchWidget::on_pushButton_2_clicked()
{
    restorePaintingStatus();
    pTempStatus=true;
    ui->textEdit->clear();
    ui->textEdit->append(QString("In the process of set template"));
    showMatOnDlg(sourceMat,ui->label_6,&ratio);
}

void nccMatchWidget::mousePressEvent(QMouseEvent *event)
{
    if(event->x()>=ui->label_6->x()&&event->x()<=(ui->label_6->x()+ui->label_6->width())&&event->y()>=ui->label_6->y()&&event->y()<=(ui->label_6->y()+ui->label_6->height())){
        QCursor cursor;
        QPixmap pixmap(QCoreApplication::applicationDirPath()+"/settings/cursor.png");
        cursor = QCursor(pixmap,-1,-1);
        setCursor(cursor);
        int w=cvRound((event->x()-ui->label_6->x())*ratio);
        int h=cvRound((event->y()-ui->label_6->y())*ratio);
        ui->label_10->setText("x:"+QString::number(w,10)+" y:"+QString::number(h,10));
        if(pRoiStatus){
            clickX=event->x();
            clickY=event->y();
        }
        if(pTempStatus){
            clickX=event->x();
            clickY=event->y();
        }
        if(pMaskStatus){
            drawMaskStatus=true;
        }
    }
}

void nccMatchWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->x()>=ui->label_6->x()&&event->x()<=(ui->label_6->x()+ui->label_6->width())&&event->y()>=ui->label_6->y()&&event->y()<=(ui->label_6->y()+ui->label_6->height())){
        setCursor(QCursor(Qt::ArrowCursor));
        int w=cvRound((event->x()-ui->label_6->x())*ratio);
        int h=cvRound((event->y()-ui->label_6->y())*ratio);
        ui->label_10->setText("x:"+QString::number(w,10)+" y:"+QString::number(h,10));
        if(pRoiStatus){
            releaseX=event->x();
            releaseY=event->y();
            cv::Mat t=sourceMat.clone();
            int x=cvRound((clickX-ui->label_6->x())*ratio);
            int y=cvRound((clickY-ui->label_6->y())*ratio);

            cv::rectangle(t,cv::Point(x,y),cv::Point(w,h),cv::Scalar(0,255,255),3);
            showMatOnDlg(t,ui->label_6,&ratio);
            roiX=x;
            roiY=y;
            roiWidth=abs(w-x);
            roiHeight=abs(h-y);
            pRoiStatus=false;
        }
        if(pTempStatus){
            releaseX=event->x();
            releaseY=event->y();
            cv::Mat t=sourceMat.clone();
            int x=cvRound((clickX-ui->label_6->x())*ratio);
            int y=cvRound((clickY-ui->label_6->y())*ratio);

            cv::rectangle(t,cv::Point(x,y),cv::Point(w,h),cv::Scalar(0,255,255),3);
            showMatOnDlg(t,ui->label_6,&ratio);

            tempMaskShowMat=sourceMat(cv::Rect(std::min(x,w),std::min(y,h),abs(w-x),abs(h-y))).clone();
            temporyTempMat=tempMaskShowMat.clone();
            temporyTempMaskShowMat=tempMaskShowMat.clone();
            temporyMaskMat=cv::Mat(tempMaskShowMat.rows,tempMaskShowMat.cols,CV_8UC1,cv::Scalar::all(255));

            showMatOnDlg(tempMaskShowMat,ui->label_5);

            pTempStatus=false;
        }
        if(pMaskStatus){
            showMatOnDlg(temporyTempMaskShowMat,ui->label_5);
            drawMaskStatus=false;
        }
    }
}

void nccMatchWidget::mouseMoveEvent(QMouseEvent *event)
{
    if(event->x()>=ui->label_6->x()&&event->x()<=(ui->label_6->x()+ui->label_6->width())&&event->y()>=ui->label_6->y()&&event->y()<=(ui->label_6->y()+ui->label_6->height()))
    {
        int w=cvRound((event->x()-ui->label_6->x())*ratio);
        int h=cvRound((event->y()-ui->label_6->y())*ratio);
        ui->label_10->setText("x:"+QString::number(w,10)+" y:"+QString::number(h,10));
        if(pRoiStatus){
            cv::Mat t=sourceMat.clone();
            int x=cvRound((clickX-ui->label_6->x())*ratio);
            int y=cvRound((clickY-ui->label_6->y())*ratio);

            cv::rectangle(t,cv::Point(x,y),cv::Point(w,h),roiColour,3);
            showMatOnDlg(t,ui->label_6,&ratio);
        }
        if(pTempStatus){
            cv::Mat t=sourceMat.clone();
            int x=cvRound((clickX-ui->label_6->x())*ratio);
            int y=cvRound((clickY-ui->label_6->y())*ratio);

            cv::rectangle(t,cv::Point(x,y),cv::Point(w,h),cv::Scalar(0,255,255),3);
            showMatOnDlg(t,ui->label_6,&ratio);
        }
        if(pMaskStatus&&drawMaskStatus){
            if(w<tempMaskShowMat.cols&&h<tempMaskShowMat.rows){
                int l=ui->spinBox->text().toInt();
                if(pencileStatus){
                    cv::circle(temporyTempMaskShowMat,cv::Point(w,h),l,cv::Scalar(255,0,0),-1);
                    cv::circle(temporyMaskMat,cv::Point(w,h),l,cv::Scalar(0),-1);
                }
                if(eraserStatus){
                    int validX,validY,validHeight,validWidth;
                    short int x;
                    short int y;
                    x=w-l;
                    y=h-l;
                    if(x>=0){
                        validX=x;
                        validWidth=l*2;
                    }else{
                        validWidth=l*2+x;
                        validX=0;
                    }

                    if(y>=0){
                        validY=y;
                        validHeight=l*2;
                    }else{
                        validHeight=l*2+y;
                        validY=0;
                    }
                    ///////temp col out range////////////
                    if((validX+validWidth)>temporyMaskMat.cols){
                        validWidth=temporyMaskMat.cols-validX;
                    }

                    ///////temp row out range////////////
                    if((validY+validHeight)>temporyMaskMat.rows){
                        validHeight=temporyMaskMat.rows-validY;
                    }
                    if(validWidth==0||validHeight==0)
                        return;
                    cv::Mat r=cv::Mat(validHeight,validWidth,CV_8UC1,cv::Scalar::all(255));
                    r.copyTo(temporyMaskMat(cv::Rect(validX,validY,validWidth,validHeight)));
                    temporyTempMat(cv::Rect(validX,validY,validWidth,validHeight)).copyTo(temporyTempMaskShowMat(cv::Rect(validX,validY,validWidth,validHeight)));
                }
                showMatOnDlg(temporyTempMaskShowMat,ui->label_6,&ratio);
            }
        }
    }
    else
        setCursor(Qt::ArrowCursor);
}

void nccMatchWidget::on_pushButton_7_clicked()
{
    if(ui->comboBox->count()==0||modelNum==0){
        close();
        return;
    }
    int index=comboxCurrentIndex;

    vecAngleMatchRange[index]=ui->lineEdit_6->text().toInt();
    vecMatchThreshold[index]=ui->lineEdit_2->text();
    vecDownSampleTime[index]=ui->lineEdit->text().toInt();

    nccmatch.setAngleMatchRange(vecAngleMatchRange[index]);
    nccmatch.setRotateAndMaskFlag(true,true);

    vecRoiX[index]=roiX;
    vecRoiHeight[index]=roiHeight;
    vecRoiWidth[index]=roiWidth;
    vecRoiY[index]=roiY;
    if(!temporyMaskMat.empty())
        maskMat=temporyMaskMat.clone();
    if(!temporyTempMat.empty())
        templateMat=temporyTempMat.clone();
    if(!temporyTempMaskShowMat.empty())
        tempMaskShowMat=temporyTempMaskShowMat.clone();

    nccStatus=nccGenerateTemplate(templateMat,maskMat,isEnableMask,isEnableRotate);
    if(!nccStatus){
        ui->textEdit->clear();
        ui->textEdit->append(QString("generate template failed!"));
    }
    if(currentMatchModel!=index)
    {
        QMessageBox::StandardButton button;
        button = QMessageBox::question(this, "Caution",
                                       QString("Are you sure to change the current match model?"),
                                       QMessageBox::Yes | QMessageBox::No);
        if(button == QMessageBox::Yes){
            currentMatchModel=index;
        }
    }
    paramSave();
    imgSave(index);
    close();
}

void nccMatchWidget::on_pushButton_clicked()
{
    bool isOK;
    QString modelName = QInputDialog::getText(NULL, "Input Dialog",
                                              "Please input your model name",
                                              QLineEdit::Normal,
                                              "Your new model name",
                                              &isOK);
    if(isOK){
        QMessageBox::StandardButton button;
        button = QMessageBox::question(this, "Caution",
                                       QString("new model name "+modelName+"?"),
                                       QMessageBox::Yes | QMessageBox::No);
        if(button == QMessageBox::Yes){
            if(ui->comboBox->findText(modelName)!=-1){
                QMessageBox msgBox;
                msgBox.setText("model name repeat,create failed!");
                msgBox.exec();
            }else{
                if(modelName.isEmpty())
                {
                    QMessageBox msgBox;
                    msgBox.setText("model name cant be empty!");
                    msgBox.exec();
                    return;
                }
                addNewModel(modelName);
                ui->comboBox->addItem(modelName);
                paramSave();
                ui->comboBox->setCurrentIndex(ui->comboBox->count()-1);
            }
        }
    }
}

void nccMatchWidget::on_pushButton_5_clicked()
{
    NCC_Match temporyNccMatch;
    float threshold=ui->lineEdit_2->text().toFloat();
    if(threshold<=0)
        threshold=0.5;

    int angleMatchRange=ui->lineEdit_6->text().toInt();
    if(angleMatchRange<0)
        angleMatchRange=360;

    restorePaintingStatus();
    maskMat=temporyMaskMat.clone();
    templateMat=temporyTempMat.clone();

    if(templateMat.empty()||maskMat.empty()||templateMat.size!=maskMat.size){
        QMessageBox msgBox;
        msgBox.setText("templateMat.empty()||maskMat.empty()||templateMat.size!=maskMat.size");
        msgBox.exec();
        return;
    }


    tempMaskShowMat=temporyTempMaskShowMat.clone();
    int downSampleTime=ui->lineEdit->text().toInt();

    cv::Mat temp=templateMat.clone();
    nccGenerateTemplate(temp,maskMat,temporyNccMatch,isEnableMask,isEnableRotate,downSampleTime);
    temporyNccMatch.setAngleMatchRange(angleMatchRange);

    QTime nccMatchTimer;
    nccMatchTime=0;
    nccMatchTimer.start();
    cv::Mat src;
    getRoiMat(sourceMat,src,roiX,roiY,roiWidth,roiHeight);
    if(src.type()!=CV_8UC1){
        cv::cvtColor(src,src,cv::COLOR_RGB2GRAY);
    }
    temporyNccMatch.nccMatch(src,threshold,isEnableMask,isEnableRotate);

    nccMatchTime=nccMatchTimer.elapsed();
    cv::Mat resultMat;
    resultMat=drawResultOnMat(temporyNccMatch);
    showMatOnDlg(resultMat,ui->label_6,&ratio);
    outPutMatchResult(temporyNccMatch);
    showMatOnDlg(tempMaskShowMat,ui->label_5);
}

bool nccMatchWidget::nccGenerateTemplate(cv::Mat&temp,cv::Mat&mask,bool isEnableMask,bool isEnableRotate)
{
    if(temp.empty()||mask.empty()||temp.size!=mask.size)
    {
        qDebug()<<"temp.empty()||mask.empty()||temp.size!=mask.size"<<endl;
        return false;
    }
    nccmatch.setRotateAndMaskFlag(isEnableRotate,isEnableMask);
    cv::Mat t,m;
    if(temp.type()!=CV_8UC1){
        cv::cvtColor(temp,t,cv::COLOR_RGB2GRAY);
    }else{
        t=temp;
    }
    if(mask.type()!=CV_8UC1){
        cv::cvtColor(mask,m,cv::COLOR_RGB2GRAY);
    }else{
        m=mask;
    }
    nccmatch.generateTemplateAndMask(t,m,vecDownSampleTime[comboxCurrentIndex]);
    return true;
}

bool nccMatchWidget::nccGenerateTemplate(cv::Mat &temp, cv::Mat &mask, NCC_Match &nccmatch,bool isEnableMask,bool isEnableRotate,uint downSampleTime)
{
    if(temp.empty()||mask.empty()||temp.size!=mask.size)return false;
    nccmatch.setRotateAndMaskFlag(isEnableRotate,isEnableMask);
    cv::Mat t,m;
    if(temp.type()!=CV_8UC1){
        cv::cvtColor(temp,t,cv::COLOR_RGB2GRAY);
    }else{
        t=temp;
    }

    if(mask.type()!=CV_8UC1){
        cv::cvtColor(mask,m,cv::COLOR_RGB2GRAY);
    }else{
        m=mask;
    }
    if(downSampleTime==0)
        nccmatch.generateTemplateAndMask(t,m,vecDownSampleTime[comboxCurrentIndex]);
    else
        nccmatch.generateTemplateAndMask(t,m,downSampleTime);
    return true;
}

void nccMatchWidget::outPutMatchResult()
{
    ui->textEdit->clear();
    int matchUseTime=0;
    for(ushort i=0;i<nccmatch.vec_matchTime.size();i++){
        matchUseTime+=nccmatch.vec_matchTime[i];
    }
    for(int i=0;i<nccmatch.VEC_matchPointsxx.size();i++){
        ui->textEdit->append(""+QString::number(nccmatch.downSamplingNum-i,10)+QString("nd layer threshold:%1").arg(nccmatch.layerThreshold[nccmatch.downSamplingNum-i]));
        ui->textEdit->append(""+QString::number(nccmatch.downSamplingNum-i,10)+"nd layer match time:"+QString::number(nccmatch.vec_matchTime[i],10));
        ui->textEdit->append(QString::number((long)nccmatch.VEC_matchPointsxx[i].size(),10)+ " match points:");
        for(int j=0;j<nccmatch.VEC_matchPointsxx[i].size();j++){
            ui->textEdit->append("Point:("+QString::number((long)nccmatch.VEC_matchPointsxx[i][j].matchPoint.x,10)+","+QString::number((long)nccmatch.VEC_matchPointsxx[i][j].matchPoint.y,10)+")");
            ui->textEdit->append("Angle:"+ QString::number((long)nccmatch.VEC_matchPointsxx[i][j].angle,10));
            ui->textEdit->append(QString("Value:%1").arg(nccmatch.VEC_matchPointsxx[i][j].matchValue));
            ui->textEdit->append(" ");
        }
        ui->textEdit->append(" ");
    }
    ui->textEdit->append("Total used time:"+QString::number(nccMatchTime,10));
    ui->textEdit->append("ncc match used time:"+QString::number(matchUseTime,10));
    ui->textEdit->append("ncc match point:("+QString::number((long)nccmatch.nccMatchPoint.x,10)+","+QString::number((long)nccmatch.nccMatchPoint.y,10)+")");
    ui->textEdit->append("ncc match angle:"+QString::number(nccmatch.nccMatchAngle,10));
    ui->textEdit->append(QString("ncc match value:%1").arg(nccmatch.nccMatchScore));

    QFile recordFile("record.txt");
    if(recordFile.open(QFile::WriteOnly|QIODevice::Append)){
        QTextStream out(&recordFile);
        for(int i=0;i<nccmatch.VEC_matchPointsxx.size();i++){
            out<<"The "+QString::number(nccmatch.downSamplingNum-i,10)+"nd layer match used time:"+QString::number(nccmatch.vec_matchTime[i],10)<<"\r\n";
            out<<QString::number((long)nccmatch.VEC_matchPointsxx[i].size(),10)+ " match points:"<<"\r\n";
        }
        out<<"Img size("<<QString::number(roiWidth,10)+","<<QString::number(roiHeight,10)<<")"<<"\r\n";
        out<<"Total used time:"+QString::number(nccMatchTime,10)<<"\r\n";
        out<<"ncc match used time:"+QString::number(matchUseTime,10)<<"\r\n";
        out<<"ncc match point:("+QString::number((long)nccmatch.nccMatchPoint.x,10)+","+QString::number((long)nccmatch.nccMatchPoint.y,10)+")"<<"\r\n";
        out<<"ncc match angle:"+QString::number(nccmatch.nccMatchAngle,10)<<"\r\n";
        out<<QString("ncc match value:%1").arg(nccmatch.nccMatchScore)<<"\r\n";
        out<<"\r\n";
    }
}

void nccMatchWidget::outPutMatchResult(NCC_Match &nccmatch)
{
    ui->textEdit->clear();
    int matchUseTime=0;
    for(ushort i=0;i<nccmatch.vec_matchTime.size();i++){
        matchUseTime+=nccmatch.vec_matchTime[i];
    }
    for(int i=0;i<nccmatch.VEC_matchPointsxx.size();i++){
        ui->textEdit->append(""+QString::number(nccmatch.downSamplingNum-i,10)+QString("nd layer threshold:%1").arg(nccmatch.layerThreshold[nccmatch.downSamplingNum-i]));
        ui->textEdit->append(""+QString::number(nccmatch.downSamplingNum-i,10)+"nd layer match time:"+QString::number(nccmatch.vec_matchTime[i],10));
        ui->textEdit->append(QString::number((long)nccmatch.VEC_matchPointsxx[i].size(),10)+ " match points:");
        for(int j=0;j<nccmatch.VEC_matchPointsxx[i].size();j++){
            ui->textEdit->append("Point:("+QString::number((long)nccmatch.VEC_matchPointsxx[i][j].matchPoint.x,10)+","+QString::number((long)nccmatch.VEC_matchPointsxx[i][j].matchPoint.y,10)+")");
            ui->textEdit->append("Angle:"+ QString::number((long)nccmatch.VEC_matchPointsxx[i][j].angle,10));
            ui->textEdit->append(QString("Value:%1").arg(nccmatch.VEC_matchPointsxx[i][j].matchValue));
            ui->textEdit->append(" ");
        }
        ui->textEdit->append(" ");
    }
    ui->textEdit->append("Total used time:"+QString::number(nccMatchTime,10));
    ui->textEdit->append("ncc match used time:"+QString::number(matchUseTime,10));
    ui->textEdit->append("ncc match point:("+QString::number((long)nccmatch.nccMatchPoint.x,10)+","+QString::number((long)nccmatch.nccMatchPoint.y,10)+")");
    ui->textEdit->append("ncc match angle:"+QString::number(nccmatch.nccMatchAngle,10));
    ui->textEdit->append(QString("ncc match value:%1").arg(nccmatch.nccMatchScore));

    QFile recordFile("record.txt");
    if(recordFile.open(QFile::WriteOnly|QIODevice::Append)){
        QTextStream out(&recordFile);
        for(int i=0;i<nccmatch.VEC_matchPointsxx.size();i++){
            out<<"The "+QString::number(nccmatch.downSamplingNum-i,10)+"nd layer match used time:"+QString::number(nccmatch.vec_matchTime[i],10)<<"\r\n";
            out<<QString::number((long)nccmatch.VEC_matchPointsxx[i].size(),10)+ " match points:"<<"\r\n";
        }
        out<<"Img size("<<QString::number(roiWidth,10)+","<<QString::number(roiHeight,10)<<")"<<"\r\n";
        out<<"Total used time:"+QString::number(nccMatchTime,10)<<"\r\n";
        out<<"ncc match used time:"+QString::number(matchUseTime,10)<<"\r\n";
        out<<"ncc match point:("+QString::number((long)nccmatch.nccMatchPoint.x,10)+","+QString::number((long)nccmatch.nccMatchPoint.y,10)+")"<<"\r\n";
        out<<"ncc match angle:"+QString::number(nccmatch.nccMatchAngle,10)<<"\r\n";
        out<<QString("ncc match value:%1").arg(nccmatch.nccMatchScore)<<"\r\n";
        out<<"\r\n";
    }
}

cv::Mat nccMatchWidget::drawResultOnMat()
{
    cv::Mat resultMat;
    if(sourceMat.empty())return resultMat;
    if(sourceMat.type()==CV_8UC1)
        cv::cvtColor(sourceMat,resultMat,cv::COLOR_GRAY2BGR);
    else
        resultMat=sourceMat.clone();
    if(nccmatch.matchStatus)
        drawRoiOnMat(resultMat);

    if(nccmatch.matchStatus){
        int x=roiX;
        int y=roiY;
        int w=roiWidth;
        int h=roiHeight;
        if(roiX+roiWidth>resultMat.cols){
            x=0;
            w=resultMat.cols;
        }
        if(roiY+roiHeight>resultMat.rows){
            y=0;
            h=resultMat.rows;
        }
        cv::Mat r=resultMat(cv::Rect(x,y,w,h));
        cv::RotatedRect rotate_rect=cv::RotatedRect(nccmatch.nccMatchPoint,cv::Size2f(templateMat.cols,templateMat.rows),360-nccmatch.nccMatchAngle);
        cv::Point2f vertices[4];
        rotate_rect.points(vertices);
        for (int j = 0; j < 4; ++j){
            cv::line(r, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0),2);
        }
        cv::circle(r,nccmatch.nccMatchPoint,4,cv::Scalar(0,0,255),-1);

        cv::Mat t=resultMat(cv::Rect(x,y,w,h));
        if(r.cols==t.cols&&r.rows==t.rows)
            r.copyTo(t);
    }

    char positionText[50]="Point:Null!";
    cv::Point textPosition = cv::Point(70, 120);
    if(nccmatch.matchStatus)
    {
        sprintf(positionText, "Point:(%d,%d)",nccmatch.nccMatchPoint.x+roiX,nccmatch.nccMatchPoint.y+roiY);
    }
    putText(resultMat, positionText, textPosition, 2, 1.5, CV_RGB(255, 200, 100),2);

    char positionText2[50]="Angle:Null!";
    textPosition = cv::Point(70, 220);
    if(nccmatch.matchStatus)
    {
        sprintf(positionText2, "Angle:%d", matchAngle);
    }
    putText(resultMat, positionText2, textPosition, 2, 1.5, CV_RGB(255, 200, 100),2);

    char positionText3[50]="Value:Null";
    textPosition = cv::Point(70, 320);
    if(nccmatch.matchStatus)
    {
        sprintf(positionText3, "Value::%.3f",matchScore);
        putText(resultMat, positionText3, textPosition, 2, 1.5, CV_RGB(255, 200, 100),2);
    }
    return resultMat;
}

cv::Mat nccMatchWidget::drawResultOnMat(NCC_Match&nccmatch)
{
    cv::Mat resultMat;
    if(sourceMat.empty())return resultMat;
    if(sourceMat.type()==CV_8UC1)
        cv::cvtColor(sourceMat,resultMat,cv::COLOR_GRAY2BGR);
    else
        resultMat=sourceMat.clone();

    int x=roiX;
    int y=roiY;
    int w=roiWidth;
    int h=roiHeight;
    if(roiX+roiWidth>resultMat.cols){
        x=0;
        w=resultMat.cols;
    }
    if(roiY+roiHeight>resultMat.rows){
        y=0;
        h=resultMat.rows;
    }
    if(roiX==0&&roiY==0&&roiWidth==0&&roiHeight==0){
        w=resultMat.cols;
        h=resultMat.rows;
    }
    cv::Mat r=resultMat(cv::Rect(x,y,w,h));
    cv::RotatedRect rotate_rect=cv::RotatedRect(nccmatch.nccMatchPoint,cv::Size2f(templateMat.cols,templateMat.rows),360-nccmatch.nccMatchAngle);
    cv::Point2f vertices[4];
    rotate_rect.points(vertices);
    for (int j = 0; j < 4; ++j){
        cv::line(r, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0),2);
    }
    cv::circle(r,nccmatch.nccMatchPoint,4,cv::Scalar(0,0,255),-1);

    cv::Mat t=resultMat(cv::Rect(x,y,w,h));
    if(r.cols==t.cols&&r.rows==t.rows)
        r.copyTo(t);
    drawRoiOnMat(resultMat,roiX,roiY,roiWidth,roiHeight);

    char positionText[50]="Point:Null!";
    cv::Point textPosition = cv::Point(70, 120);
    if(nccmatch.matchStatus)
    {
        sprintf(positionText, "Point:(%d,%d)",nccmatch.nccMatchPoint.x+roiX,nccmatch.nccMatchPoint.y+roiY);
    }
    putText(resultMat, positionText, textPosition, 2, 1.5, CV_RGB(255, 200, 100),2);

    char positionText2[50]="Angle:Null!";
    textPosition = cv::Point(70, 220);
    if(nccmatch.matchStatus)
    {
        sprintf(positionText2, "Angle:%d",nccmatch.nccMatchAngle);
    }
    putText(resultMat, positionText2, textPosition, 2, 1.5, CV_RGB(255, 200, 100),2);

    char positionText3[50]="Value:Null";
    textPosition = cv::Point(70, 320);
    if(nccmatch.matchStatus)
    {
        sprintf(positionText3, "Value::%.3f",nccmatch.nccMatchScore);
        putText(resultMat, positionText3, textPosition, 2, 1.5, CV_RGB(255, 200, 100),2);
    }
    return resultMat;
}

void nccMatchWidget::getRoiMat(cv::Mat&src,cv::Mat&mat,int x,int y,int w,int h)
{
    if(x+w<=0||y+h<=0){
        mat=src.clone();
        return;
    }
    if((x+w)>src.cols){
        x=0;
        w=src.cols;
    }
    if((y+h)>src.rows){
        y=0;
        h=src.rows;
    }
    src(cv::Rect(x,y,w,h)).copyTo(mat);
}

void nccMatchWidget::getRoiMat(cv::Mat &src, cv::Mat &mat)
{
    int x,y,w,h;
    x=roiX;
    y=roiY;
    w=roiWidth;
    h=roiHeight;
    if(roiX+roiWidth<=0||roiY+roiHeight<=0){
        mat=src.clone();
        return;
    }
    if((roiX+roiWidth)>src.cols){
        x=0;
        w=src.cols;
    }
    if((roiY+roiHeight)>src.rows){
        y=0;
        h=src.rows;
    }
    src(cv::Rect(x,y,w,h)).copyTo(mat);
}

void nccMatchWidget::restorePaintingStatus()
{
    pRoiStatus=false;
    pTempStatus=false;
    pMaskStatus=false;
    drawMaskStatus=false;
}

void nccMatchWidget::uiInitialize(int model)
{
    if(model<0)return;
    ui->textEdit->setReadOnly(true);
    ui->lineEdit_6->setValidator(new QIntValidator(0,360,this));
    ui->lineEdit_2->setValidator(new QDoubleValidator(0.000,0.999,3,this));
    ui->pushButton_2->setToolTip("Caution:Please confirm the picture has the right size!");
    ui->pushButton_6->setToolTip("Caution:Please confirm the picture has the right size!");
    ui->pushButton_7->setToolTip("Caution:Push this to close the window will auto save the change!");
    ui->pushButton_8->setToolTip("Caution:Push this to close the window will not save the change!");
    ui->lineEdit_6->setText(QString::number(vecAngleMatchRange[model],10));
    ui->label_10->setStyleSheet("color:red;");

    if(modelNum==0)return;
    if(!tempMaskShowMat.empty()){
        showMatOnDlg(tempMaskShowMat,ui->label_5);
    }
    else
    {
        cv::Mat t=cv::Mat(ui->label_5->width(),ui->label_5->height(),CV_8UC1,cv::Scalar::all(255));

        showMatOnDlg(t,ui->label_5);
    }
    if(!sourceMat.empty()){
        cv::Mat s=sourceMat.clone();
        drawRoiOnMat(s);
        showMatOnDlg(s,ui->label_6,&ratio);
    }
    else
    {
        cv::Mat t=cv::Mat(ui->label_6->width(),ui->label_6->height(),CV_8UC1,cv::Scalar::all(255));
        showMatOnDlg(t,ui->label_6,&ratio);
    }
    ui->spinBox->setValue(3);
    ui->spinBox->setMinimum(1);
    ui->spinBox->setMaximum(20);

    for(int i=0;i<vecModelName.size();++i){
        if(ui->comboBox->findText(vecModelName[i])==-1)
            ui->comboBox->addItem(vecModelName[i]);
    }

    ui->lineEdit_6->setText(QString::number(vecAngleMatchRange[model],10));
    ui->lineEdit_2->setText(vecMatchThreshold[model]);
    ui->lineEdit->setText(QString::number(vecDownSampleTime[model],10));

    ui->comboBox->setCurrentIndex(model);
}

void nccMatchWidget::imgLoad(int model)
{
    if(modelNum==0)return;
    if(model==-1)return;
    templateMat=cv::imread(paramPath+std::string("/")+vecModelName[model].toStdString()+std::string("temp.bmp"),1);
    temporyTempMat=templateMat.clone();

    tempMaskShowMat=cv::imread(paramPath+std::string("/")+vecModelName[model].toStdString()+std::string("tempMask.bmp"),1);
    temporyTempMaskShowMat=tempMaskShowMat.clone();

    maskMat=cv::imread(paramPath+std::string("/")+vecModelName[model].toStdString()+std::string("mask.bmp"),0);
    temporyMaskMat=maskMat.clone();
}

void nccMatchWidget::imgSave(int index)
{
    if(modelNum==0)return;
    if(index==-1)return;
    cv::imwrite(paramPath+std::string("/")+ui->comboBox->itemText(index).toStdString()+std::string("temp.bmp"),templateMat);
    cv::imwrite(paramPath+std::string("/")+ui->comboBox->itemText(index).toStdString()+std::string("mask.bmp"),maskMat);
    cv::imwrite(paramPath+std::string("/")+ui->comboBox->itemText(index).toStdString()+std::string("tempMask.bmp"),tempMaskShowMat);
}

void nccMatchWidget::paramSave()
{
    QSettings paramWrite(QString::fromStdString(paramPath)+"/nccParam.ini",QSettings::IniFormat);
    paramWrite.setValue("ModelNum/num",modelNum);
    for(int i=0;i<modelNum;++i){
        paramWrite.setValue("Model"+QString::number(i+1,10)+"NccMatchParam/roiX",vecRoiX[i]);
        paramWrite.setValue("Model"+QString::number(i+1,10)+"NccMatchParam/roiY",vecRoiY[i]);
        paramWrite.setValue("Model"+QString::number(i+1,10)+"NccMatchParam/roiWidth",vecRoiWidth[i]);
        paramWrite.setValue("Model"+QString::number(i+1,10)+"NccMatchParam/roiHeight",vecRoiHeight[i]);
        paramWrite.setValue("Model"+QString::number(i+1,10)+"NccMatchParam/modelName",vecModelName[i]);
        paramWrite.setValue("Model"+QString::number(i+1,10)+"NccMatchParam/angleMatchRange",vecAngleMatchRange[i]);
        paramWrite.setValue("Model"+QString::number(i+1,10)+"NccMatchParam/matchThreshold",vecMatchThreshold[i]);
        paramWrite.setValue("Model"+QString::number(i+1,10)+"NccMatchParam/downSampleTime",vecDownSampleTime[i]);
        paramWrite.setValue("Model"+QString::number(i+1,10)+"NccMatchParam/thresholdValue",vecThresholdValue[i]);
    }
}

void nccMatchWidget::paramLoad()
{
    QSettings paramRead(QString::fromStdString(paramPath)+"/nccParam.ini",QSettings::IniFormat);
    modelNum=paramRead.value("ModelNum/num").toInt();
    for(int i=0;i<modelNum;++i){
        vecModelName.push_back(paramRead.value("Model"+QString::number(i+1,10)+"NccMatchParam/modelName").toString());
        vecRoiX.push_back(paramRead.value("Model"+QString::number(i+1,10)+"NccMatchParam/roiX").toInt());
        vecRoiY.push_back(paramRead.value("Model"+QString::number(i+1,10)+"NccMatchParam/roiY").toInt());
        vecRoiWidth.push_back(paramRead.value("Model"+QString::number(i+1,10)+"NccMatchParam/roiWidth").toInt());
        vecRoiHeight.push_back(paramRead.value("Model"+QString::number(i+1,10)+"NccMatchParam/roiHeight").toInt());

        vecAngleMatchRange.push_back(paramRead.value("Model"+QString::number(i+1,10)+"NccMatchParam/angleMatchRange").toInt());
        vecMatchThreshold.push_back(paramRead.value("Model"+QString::number(i+1,10)+"NccMatchParam/matchThreshold").toString());
        vecDownSampleTime.push_back(paramRead.value("Model"+QString::number(i+1,10)+"NccMatchParam/downSampleTime").toInt());
        vecThresholdValue.push_back(paramRead.value("Model"+QString::number(i+1,10)+"NccMatchParam/thresholdValue").toInt());
    }
}

void nccMatchWidget::setModelParam(int model)
{
    roiX=0;
    roiY=0;
    roiWidth=0;
    roiHeight=0;
    if(model==-1)return;
    if(modelNum==0)return;
    roiX=vecRoiX[model];
    roiY=vecRoiY[model];
    roiWidth=vecRoiWidth[model];
    roiHeight=vecRoiHeight[model];
}

void nccMatchWidget::nccInitialize(int model)
{
    paramLoad();
    if(model<0||modelNum<=model)
        return;
    currentMatchModel=model;
    imgLoad(model);
    setModelParam(model);
    uiInitialize(model);

    nccmatch.setAngleMatchRange(vecAngleMatchRange[model]);
    nccmatch.setRotateAndMaskFlag(true,true);
    if(templateMat.empty()||maskMat.empty()){
        nccStatus=false;
        return;
    }
    else
        nccStatus=true;
    cv::Mat t,m;
    if(templateMat.type()!=CV_8UC1){
        cv::cvtColor(templateMat,t,cv::COLOR_RGB2GRAY);
    }else
        t=templateMat;
    if(maskMat.type()!=CV_8UC1){
        cv::cvtColor(maskMat,m,cv::COLOR_RGB2GRAY);
    }else
        m=maskMat;
    //preProcessTemplate(t,vecThresholdValue[model]);
    nccmatch.generateTemplateAndMask(t,m,vecDownSampleTime[model]);
}

void nccMatchWidget::setParamPath(QString path)
{
    paramPath=path.toStdString();
}

void nccMatchWidget::nccWidgetIni(cv::Mat &t)
{
    imgLoad(comboxCurrentIndex);
    setModelParam(comboxCurrentIndex);
    showMatOnDlg(templateMat,ui->label_5);

    if(t.type()!=CV_8UC3)
        cv::cvtColor(t,sourceMat,cv::COLOR_GRAY2BGR);
    else
        sourceMat=t.clone();
    if(!sourceMat.empty()){
        cv::Mat s=sourceMat.clone();
        drawRoiOnMat(s);
        showMatOnDlg(s,ui->label_6,&ratio);
    }
    uiInitialize(comboxCurrentIndex);
}

int nccMatchWidget::getCurrentMatchModel()
{
    return currentMatchModel;
}

void nccMatchWidget::closeEvent(QCloseEvent *event)
{
    uiInitialize(currentMatchModel);
}

void nccMatchWidget::drawRoiOnMat(cv::Mat &src)
{
    int x,y,w,h;
    x=roiX;
    y=roiY;
    w=roiWidth;
    h=roiHeight;

    if(roiX+roiWidth<=0||roiY+roiHeight<=0){
        return;
    }
    if((roiX+roiWidth)>src.cols){
        x=0;
        w=src.cols;
    }
    if((roiY+roiHeight)>src.rows){
        y=0;
        h=src.rows;
    }
    cv::rectangle(src,cv::Rect(x,y,w,h),roiColour,4);
}

void nccMatchWidget::drawRoiOnMat(cv::Mat &src, int x, int y, int w, int h)
{
    if(x+w<=0||y+h<=0){
        cv::rectangle(src,cv::Rect(0,0,src.cols,src.rows),roiColour,4);
        return;
    }
    if((x+w)>src.cols){
        x=0;
        w=src.cols;
    }
    if((y+h)>src.rows){
        y=0;
        h=src.rows;
    }
    cv::rectangle(src,cv::Rect(x,y,w,h),roiColour,4);
}

void nccMatchWidget::addNewModel(const QString& newModelName)
{
    vecModelName.push_back(newModelName);
    vecRoiX.push_back(0);
    vecRoiY.push_back(0);
    vecRoiWidth.push_back(0);
    vecRoiHeight.push_back(0);
    vecDownSampleTime.push_back(5);

    vecAngleMatchRange.push_back(0);
    vecMatchThreshold.push_back(0);
    vecThresholdValue.push_back(0);
    ++modelNum;
}

void nccMatchWidget::delModel(int index)
{
    delModelParam(index);
    delModelImg(index);
    --modelNum;
}

void nccMatchWidget::delModelParam(int index)
{
    if(index>vecModelName.size())return;

    auto iter0=vecModelName.begin();
    vecModelName.erase(iter0+index);

    auto iter1=vecRoiX.begin();
    vecRoiX.erase(iter1+index);

    auto iter2=vecRoiY.begin();
    vecRoiY.erase(iter2+index);

    auto iter3=vecRoiHeight.begin();
    vecRoiHeight.erase(iter3+index);

    auto iter4=vecRoiWidth.begin();
    vecRoiWidth.erase(iter4+index);

    auto iter5=vecAngleMatchRange.begin();
    vecAngleMatchRange.erase(iter5+index);

    auto iter8=vecMatchThreshold.begin();
    vecMatchThreshold.erase(iter8+index);

    auto iter9=vecThresholdValue.begin();
    vecThresholdValue.erase(iter9+index);
}

void nccMatchWidget::delModelImg(int index)
{
    QString modelName=ui->comboBox->itemText(index);
    QString tempPath=QString::fromStdString(paramPath)+"/"+modelName+"temp.bmp";
    QFile tempImg(tempPath);
    if(tempImg.exists())tempImg.remove();

    QString maskPath=QString::fromStdString(paramPath)+"/"+modelName+"mask.bmp";
    QFile maskImg(maskPath);
    if(maskImg.exists())maskImg.remove();

    QString tempMaskPath=QString::fromStdString(paramPath)+"/"+modelName+"tempMask.bmp";
    QFile tempMaskImg(tempMaskPath);
    if(tempMaskImg.exists())tempMaskImg.remove();
}

void nccMatchWidget::preProcessTemplate(cv::Mat &temp,int thresholdValue)
{
    if(thresholdValue==0)thresholdValue=vecThresholdValue[comboxCurrentIndex];
    cv::threshold(temp,temp,thresholdValue,255,0);
    cv::Mat dilateKernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(15,15));
    cv::dilate(temp,temp,dilateKernel);
}

void nccMatchWidget::preProcessSource(cv::Mat &src,int thresholdValue)
{
    if(thresholdValue==0)thresholdValue=vecThresholdValue[comboxCurrentIndex];
    cv::threshold(src,src,thresholdValue,255,0);
    cv::Mat erodeKernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
    cv::erode(src,src,erodeKernel);
    cv::Mat dilateKernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(20,20));
    cv::dilate(src,src,dilateKernel);
}

void nccMatchWidget::on_pushButton_8_clicked()
{
    paramLoad();
    setModelParam(comboxCurrentIndex);
    uiInitialize(comboxCurrentIndex);
    close();
}

void nccMatchWidget::on_pushButton_9_clicked()
{
    restorePaintingStatus();
    pMaskStatus=true;
    temporyTempMaskShowMat=tempMaskShowMat.clone();
    showMatOnDlg(temporyTempMaskShowMat,ui->label_6,&ratio);
    showMatOnDlg(temporyTempMaskShowMat,ui->label_5);
    ui->textEdit->clear();
    ui->textEdit->append(QString("In the process of set mask"));
}

void nccMatchWidget::on_pushButton_3_clicked()
{
    pencileStatus=true;
    eraserStatus=false;
    ui->pushButton_3->setEnabled(false);
    ui->pushButton_10->setEnabled(true);
}

void nccMatchWidget::on_pushButton_10_clicked()
{
    pencileStatus=false;
    eraserStatus=true;
    ui->pushButton_3->setEnabled(true);
    ui->pushButton_10->setEnabled(false);
}

void nccMatchWidget::on_comboBox_currentIndexChanged(int index)
{
    comboxCurrentIndex=index;
    if(index==-1){
        ui->lineEdit_6->setText("");
        ui->lineEdit_2->setText("");
        ui->lineEdit->setText("");
        return;
    }
    setModelParam(index);
    imgLoad(index);
    uiInitialize(index);
}

void nccMatchWidget::on_pushButton_11_clicked()
{
    QMessageBox::StandardButton button;
    button = QMessageBox::question(this, "Caution",
                                   QString("Are you sure to delete current model:"+ui->comboBox->currentText()+"?"),
                                   QMessageBox::Yes | QMessageBox::No);
    if(button == QMessageBox::Yes){
        delModel(comboxCurrentIndex);
        ui->comboBox->removeItem(comboxCurrentIndex);
        if(ui->comboBox->count()==0)return;
        ui->comboBox->setCurrentIndex(ui->comboBox->count()-1);
    }
}

void nccMatchWidget::on_lineEdit_6_textEdited(const QString &arg1)
{
    if(modelNum==0)return;
    vecAngleMatchRange[comboxCurrentIndex]=arg1.toInt();
    paramSave();
}

void nccMatchWidget::on_lineEdit_2_textEdited(const QString &arg1)
{
    if(modelNum==0)return;
    vecMatchThreshold[comboxCurrentIndex]=arg1;
    paramSave();
}

void nccMatchWidget::on_lineEdit_textEdited(const QString &arg1)
{
    if(modelNum==0)return;
    vecDownSampleTime[comboxCurrentIndex]=arg1.toInt();
    paramSave();
}
