#include "calibratemachine.h"
#include <vector>
#include <algorithm>


#include "yuan_pattern.h"

CalibrateMachine::CalibrateMachine()
{
    objectPoints.clear();
    cameraPoints.clear();
    dlpPoints.clear();

    boardSize.width = 11;
    boardSize.height = 9;
}

CalibrateMachine::~CalibrateMachine()
{

}

bool CalibrateMachine::findCircleBoardCorners(cv::Mat &image, cv::Size boardSize, std::vector<cv::Point2f> &realCorners)
{
    if(image.channels()==3){
        cv::cvtColor(image,image,CV_BGR2GRAY);
    }
    cv::medianBlur(image,image,5);

    cv::Mat img(image.rows,image.cols,CV_8UC1);
    cv::subtract(cv::Scalar(255),image,img);

    cv::threshold(img,img,100,255,cv::THRESH_BINARY | cv::THRESH_OTSU);

    std::vector<std::vector< cv::Point> > contours;
    cv::findContours(img,contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

    //二值处理后
    cv::Mat findContoursMat(img.size(),CV_8UC1,cv::Scalar(0));
    cv::drawContours(findContoursMat,contours,-1,cv::Scalar(255),2);
    //cv::imshow("findContoursMat",findContoursMat);

    //移除过长和过短的轮廓
    int cmin = 50;
    int cmax = 2000;

    int realmin = 50000;
    int realmax = 0;

    std::vector<std::vector<cv::Point>>:: const_iterator itContour = contours.begin();

    while(itContour != contours.end()){

        if(realmin > (int)(*itContour).size())
        {
            realmin = (int)(*itContour).size();
        }
        if(realmax < (int)(*itContour).size())
        {
            realmax = (int)(*itContour).size();
        }

        if((int)(*itContour).size()< cmin ||
                (int)(*itContour).size()> cmax)
        {
            itContour = contours.erase(itContour);
        }
        else
        {
            ++itContour;
        }


    }
    //std::cout<<"the circles size, realmin : "<<realmin<<" , realmax : "<<realmax<<std::endl;

    //移除后
    cv::Mat selectContours(img.size(),CV_8U,cv::Scalar(0));
    cv::drawContours(selectContours,contours,-1,cv::Scalar(255),2);
    //cv::imshow("selectContours",selectContours);

    std::vector<std::vector<cv::Point>>::const_iterator itRect = contours.begin();

    while( itRect != contours.end()){

        cv::RotatedRect rect = cv::minAreaRect(cv::Mat(*itRect));

        float scale = 0;

        if( rect.size.width > rect.size.height ){

            scale = rect.size.width/ rect.size.height;
        }else{

            scale = rect.size.height/ rect.size.width;
        }

        cv::Point2f rectPoints[4];
        rect.points(rectPoints);

        if( scale > 1.3
                || rectPoints[0].x < 0.1*img.cols || rectPoints[1].x < 0.1*img.cols
                || rectPoints[2].x < 0.1*img.cols || rectPoints[3].x < 0.1*img.cols
                || rectPoints[0].y < 0.1*img.rows || rectPoints[1].y < 0.1*img.rows
                || rectPoints[2].y < 0.1*img.rows || rectPoints[3].y < 0.1*img.rows

                || rectPoints[0].x > 0.9*img.cols || rectPoints[1].x > 0.9*img.cols
                || rectPoints[2].x > 0.9*img.cols || rectPoints[3].x > 0.9*img.cols
                || rectPoints[0].y > 0.9*img.rows || rectPoints[1].y > 0.9*img.rows
                || rectPoints[2].y > 0.9*img.rows || rectPoints[3].y > 0.9*img.rows)
        {
            itRect = contours.erase(itRect);
        }else{
            ++itRect;
        }
    }
    //移除后
    cv::Mat pickContours(img.size(),CV_8UC1,cv::Scalar(0));
    cv::drawContours(pickContours,contours,-1,cv::Scalar(255),2);
    //cv::imshow("select_scale_Contours_filtered",pickContours);

    //半径容器、圆心容器
    std::vector<float> radius_v;
    std::vector<cv::Point2f> centers_v;

    cv::Point2f center;
    float radius;

    std::vector<std::vector<cv::Point>>:: const_iterator itSort = contours.begin();

    while(itSort != contours.end() ){

        cv::minEnclosingCircle((*itSort),center,radius);//最小包围圆

        radius_v.push_back(radius);
        centers_v.push_back(center);

        ++itSort;
    }
    if(centers_v.size()<3){
        std::cout<<"Error: the number of sircle is not adequate!\n";
        return false;
    }

    //半径排序
    for(int i = 0; i < centers_v.size(); i++)
    {
        for(int j = i+1; j < centers_v.size(); j++)
        {
            if(radius_v.at(i)< radius_v.at(j))
            {

                std::swap(radius_v[i],radius_v[j]);
                std::swap(centers_v[i],centers_v[j]);
            }
        }
    }
    //半径排序，希尔排序
//    for(int gap = centers_v.size(); gap >= 0; gap /=2)
//    {
//        for(int i = gap; i < centers_v.size(); ++i)
//        {
//            float insertR = radius_v[i];
//            cv::Point2f insertP = centers_v[i];
//            int j = i - gap;
//            while(j >= 0 && insertR < radius_v[j])
//            {
//                radius_v[j+gap] = radius_v[j];
//                centers_v[j+gap] = centers_v[j];
//                j -= gap;
//            }
//            radius_v[j+gap] = insertR;
//            centers_v[j+gap] = insertP;
//        }
//    }

    cv::Mat mark(img.size(),CV_8U,cv::Scalar(255));

    std::vector<cv::Point2f> mark_point;

    for(int i = 0; i<5; i++)
    {
        mark_point.push_back(centers_v[i]);
        cv::circle(mark,centers_v[i],radius_v[i],cv::Scalar(0),3);
    }
    //cv::imshow("mark_point",mark);


    //各点距离
    std::vector<float> distance;

    for(unsigned int i = 0; i < 5; ++i){
        for(unsigned int j = i+1; j < 5; ++j){

            float d = sqrt(pow(centers_v[i].x - centers_v[j].x , 2) + pow(centers_v[i].y - centers_v[j].y , 2) );
            distance.push_back(d);
        }
    }

    float maxD=0;
    int maxN;

    //找出最大值
    for(int i=0;i<10;i++)
    {
        if(maxD< distance.at(i))
        {
            maxD= distance.at(i);
            maxN= i;
        }

    }

    float minD=maxD;
    int minN;

    //找出最小值
    for(int i=0;i<10;i++)
    {
        if(minD> distance.at(i))
        {
            minD= distance.at(i);
            minN= i;
        }

    }

//    std::cout<<"the maxD is : "<<maxD<<" , the minD is : "<<minD<<std::endl;
//    std::cout<<"the index_max is : "<<maxN<<" , the index_min is : "<<minN<<std::endl;

    cv::Point2f longP_1,longP_2;
    cv::Point2f shortP_1,shortP_2;
    cv::Point2f upP,downP,leftP,rightP,upleftP;

    int flag = 10;

    switch(maxN)
    {
        case 0:
        {
            longP_1= centers_v.at(0);
            longP_2= centers_v.at(1);

            flag -=0;
            flag -=1;

        }

            break;
        case 1:
        {
            longP_1= centers_v.at(0);
            longP_2= centers_v.at(2);

            flag -=0;
            flag -=2;
        }

            break;
        case 2:
        {
            longP_1= centers_v.at(0);
            longP_2= centers_v.at(3);

            flag -=0;
            flag -=3;
        }

            break;
        case 3:
        {
            longP_1= centers_v.at(0);
            longP_2= centers_v.at(4);

            flag -=0;
            flag -=4;
        }

            break;

        case 4:
        {
            longP_1= centers_v.at(1);
            longP_2= centers_v.at(2);

            flag -=1;
            flag -=2;
        }

            break;
        case 5:
        {
            longP_1= centers_v.at(1);
            longP_2= centers_v.at(3);

            flag -=1;
            flag -=3;
        }

            break;
        case 6:
        {
            longP_1= centers_v.at(1);
            longP_2= centers_v.at(4);

            flag -=1;
            flag -=4;
        }

            break;
        case 7:
        {
            longP_1= centers_v.at(2);
            longP_2= centers_v.at(3);

            flag -=2;
            flag -=3;
        }

            break;

        case 8:
        {
            longP_1= centers_v.at(2);
            longP_2= centers_v.at(4);

            flag -=2;
            flag -=4;
        }

            break;
        case 9:
        {
            longP_1= centers_v.at(3);
            longP_2= centers_v.at(4);

            flag -=3;
            flag -=4;
        }

            break;
    }

    switch(minN)
    {
        case 0:
        {
            shortP_1= centers_v.at(0);
            shortP_2= centers_v.at(1);

            flag -=0;
            flag -=1;

        }

            break;
        case 1:
        {
            shortP_1= centers_v.at(0);
            shortP_2= centers_v.at(2);

            flag -=0;
            flag -=2;
        }

            break;
        case 2:
        {
            shortP_1= centers_v.at(0);
            shortP_2= centers_v.at(3);

            flag -=0;
            flag -=3;
        }

            break;
        case 3:
        {
            shortP_1= centers_v.at(0);
            shortP_2= centers_v.at(4);

            flag -=0;
            flag -=4;
        }

            break;

        case 4:
        {
            shortP_1= centers_v.at(1);
            shortP_2= centers_v.at(2);

            flag -=1;
            flag -=2;
        }

            break;
        case 5:
        {
            shortP_1= centers_v.at(1);
            shortP_2= centers_v.at(3);

            flag -=1;
            flag -=3;
        }

            break;
        case 6:
        {
            shortP_1= centers_v.at(1);
            shortP_2= centers_v.at(4);

            flag -=1;
            flag -=4;
        }

            break;
        case 7:
        {
            shortP_1= centers_v.at(2);
            shortP_2= centers_v.at(3);

            flag -=2;
            flag -=3;
        }

            break;

        case 8:
        {
            shortP_1= centers_v.at(2);
            shortP_2= centers_v.at(4);

            flag -=2;
            flag -=4;
        }

            break;
        case 9:
        {
            shortP_1= centers_v.at(3);
            shortP_2= centers_v.at(4);

            flag -=3;
            flag -=4;
        }

            break;
    }

    downP = mark_point.at(flag);//除了最大、最小距离，剩下的是downP

    float d_1 = abs(sqrt(pow(longP_1.x - shortP_1.x, 2) + pow(longP_1.y - shortP_1.y, 2)) -
        sqrt(pow(longP_2.x - shortP_1.x, 2) + pow(longP_2.y - shortP_1.y, 2)));

    float d_2 = abs(sqrt(pow(longP_1.x - shortP_2.x, 2) + pow(longP_1.y - shortP_2.y, 2)) -
        sqrt(pow(longP_2.x - shortP_2.x, 2) + pow(longP_2.y - shortP_2.y, 2)));

    if (d_1 > d_2)
    {
        upP = shortP_2;
        upleftP = shortP_1;

    }
    else {

        upP = shortP_1;
        upleftP = shortP_2;
    }

    float d_11 = abs(sqrt(pow(upleftP.x - downP.x, 2) + pow(upleftP.y - downP.y, 2)) -
        sqrt(pow(upleftP.x - longP_1.x, 2) + pow(upleftP.y - longP_1.y, 2)));

    float d_22 = abs(sqrt(pow(upleftP.x - downP.x, 2) + pow(upleftP.y - downP.y, 2)) -
        sqrt(pow(upleftP.x - longP_2.x, 2) + pow(upleftP.y - longP_2.y, 2)));

    if (d_11 > d_22)
    {
        leftP = longP_1;
        rightP = longP_2;
    }
    else {

        leftP = longP_2;
        rightP = longP_1;
    }

    //四对点
    cv::Point2f srcTriangle[4];
    cv::Point2f dstTriangle[4];

    cv::Mat tosrcWarpMat(3, 3, CV_32FC1);
    cv::Mat todstWarpMat(3, 3, CV_32FC1);

    cv::Mat dstImage = cv::Mat::zeros(img.rows, img.cols, img.type());

    cv::Mat testImage(img.size(), CV_8U, cv::Scalar(255));

    cv::circle(testImage, upP, radius_v[0], cv::Scalar(0), 2);
    cv::circle(testImage, downP, radius_v[0], cv::Scalar(0), 5);
    cv::circle(testImage, leftP, radius_v[0], cv::Scalar(0), 8);
    cv::circle(testImage, rightP, radius_v[0], cv::Scalar(0), 11);

    static int num = 0;
    cv::imwrite("../ScanData/board_image/circleDetect"+QString::number(num).toStdString()+".bmp",testImage);

    srcTriangle[0] = upP;
    srcTriangle[1] = downP;
    srcTriangle[2] = leftP;
    srcTriangle[3] = rightP;

    dstTriangle[0] = cv::Point2f(static_cast<float>(mark.cols*0.5), static_cast<float>(mark.rows*0.5 - 2 * minD));
    dstTriangle[1] = cv::Point2f(static_cast<float>(mark.cols*0.5 ), static_cast<float>(mark.rows*0.5 + 2*minD));
    dstTriangle[2] = cv::Point2f(static_cast<float>(mark.cols*0.5 - 3 * minD), static_cast<float>(mark.rows*0.5));
    dstTriangle[3] = cv::Point2f(static_cast<float>(mark.cols*0.5 + 3 * minD), static_cast<float>(mark.rows*0.5));

    todstWarpMat = cv::getPerspectiveTransform(srcTriangle, dstTriangle);

    tosrcWarpMat = cv::getPerspectiveTransform(dstTriangle, srcTriangle);

    //摆正标定板
    cv::Mat showMat = image.clone();
    cv::warpPerspective(showMat, dstImage, todstWarpMat, dstImage.size(), cv::INTER_CUBIC);
    //cv::imshow("Affine",dstImage);
    cv::imwrite("../ScanData/board_image/warpAffine"+QString::number(num).toStdString()+".bmp",dstImage);

    //找圆
    std::vector<cv::Point2f> imageCorners;

    cv::Mat dstImage1 = dstImage.clone();
    cv::subtract(cv::Scalar(255),dstImage,dstImage1);

    //圆形标定法，找圆心
    bool found = cv::findCirclesGrid(dstImage1, boardSize, imageCorners,
                        cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);

    cv::drawChessboardCorners(dstImage,boardSize,imageCorners,found);

    cv::imwrite("../ScanData/board_image/imageCorners"+QString::number(num).toStdString()+".bmp",dstImage);

//    cv::imshow("pattern",dstImage);

    if(!found)
    {
        return found;
    }

    for (int i = 0; i< imageCorners.size(); i++)
    {
        float x = imageCorners.at(i).x;
        float y = imageCorners.at(i).y;

        float realX = x*tosrcWarpMat.at<double>(0, 0) + y*tosrcWarpMat.at<double>(0, 1) + tosrcWarpMat.at<double>(0, 2);
        float realY = x*tosrcWarpMat.at<double>(1, 0) + y*tosrcWarpMat.at<double>(1, 1) + tosrcWarpMat.at<double>(1, 2);
        float perZ  =  1 / (x*tosrcWarpMat.at<double>(2, 0) + y*tosrcWarpMat.at<double>(2, 1) + tosrcWarpMat.at<double>(2, 2));

        realX *= perZ;
        realY *= perZ;

        realCorners.push_back(cv::Point2f(realX, realY));
    }

    cv::Mat src = image.clone();
    cv::drawChessboardCorners(src,boardSize,realCorners,found);
    cv::imwrite("../ScanData/board_image/realCorners"+QString::number(num).toStdString()+".bmp",src);

    num++;
    return found;
    cv::waitKey(0);
}

bool CalibrateMachine::get_cameraPoints(cv::Mat &boardImage,std::vector<cv::Point2f>& cameraP)
{

    if(1!= boardImage.channels())
    {
        cv::cvtColor(boardImage,boardImage,CV_BGR2GRAY);
    }

    bool found= findCircleBoardCorners(boardImage,boardSize,cameraP);

    if(!found)
    {
        qDebug()<<"Error: Can not find the circle points!";
        return false;
    }

    cameraPoints.push_back(cameraP);
    std::cout<<"cameraCirclePoints size: "<<cameraP.size()<<std::endl;
    return true;

}

void CalibrateMachine::pointsCameraToDlp(const std::vector<cv::Mat> &phaseX, const std::vector<cv::Mat> &phaseY, int period,
                                         std::vector<std::vector<cv::Point2f> > &cameraPoint,
                                         std::vector<std::vector<cv::Point2f> > &dlpPoint)
{

    double periodper2PI = period/(2*CV_PI);

    std::vector<cv::Point2f> temp;

    for(int n = 0; n < cameraPoint.size(); ++n){

        temp.clear();

        for(int m = 0; m < cameraPoint[n].size(); ++m){

            cv::Point2f point = cameraPoint[n][m];

            int xleft  = floor(point.x);
            int xright = ceil(point.x);
            int yUP    = floor(point.y);
            int yDOWN  = ceil(point.y);

            double xphaseleft  = phaseX[n].at<double>((int)point.y , xleft);
            double xphaseright = phaseX[n].at<double>((int)point.y , xright);
            double xphase      = xphaseleft + (xphaseright - xphaseleft)*(point.x - xleft)/(xright-xleft);

            double yphaseUP  = phaseY[n].at<double>(yUP  ,(int)point.x);
            double yphaseDOWN = phaseY[n].at<double>(yDOWN,(int)point.x);
            double yphase    = yphaseUP + (yphaseDOWN - yphaseUP)*(point.y-yUP)/(yDOWN-yUP);

            //            int x = point.x;
            //            int y = point.y;

            //            double xphase = phaseX[n].at<double>(y,x);
            //            double yphase = phaseY[n].at<double>(y,x);

            double dlp_x_value = xphase * periodper2PI;
            double dlp_y_value = yphase * periodper2PI;

            temp.push_back(cv::Point2f(dlp_x_value,dlp_y_value));
        }

        dlpPoint.push_back(temp);
    }

}

double CalibrateMachine::singleCamera_system_calibrate(std::vector<cv::Mat> &imagelist,int boardNum)//相对相位图
{
    if(imagelist.empty()){
        qDebug()<<"Error: the imagelist is empty and singleCamera_system_calibrate failed ";
        return 100;
    }
    QTime t1;
    t1.start();

    //相机标定
    for (int t = 0; t < boardNum; t++)
    {
        std::vector<cv::Point3f> tempPointSet;
        for (int i = 0; i<boardSize.height; i++)
        {
            for (int j = 0; j<boardSize.width; j++)
            {
                cv::Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.x = i*squareSize.width;
                realPoint.y = j*squareSize.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        objectPoints.push_back(tempPointSet);
    }

    camera_intrinsic = cv::Mat(3,3,CV_64FC1,cv::Scalar::all(0));
    camera_distortion = cv::Mat(1,5,CV_64FC1,cv::Scalar::all(0));

    std::vector<cv::Mat> camera_rvecs;
    std::vector<cv::Mat> camera_tvecs;

    cv::Mat Gmat = cv::Mat::zeros(3,3,CV_64FC1);

    std::cout<<"objectPoints size : "<<objectPoints.size()<<" , cameraPoints size : "<<cameraPoints.size()<<std::endl;
    if(objectPoints.size()!= cameraPoints.size() || objectPoints.empty() || cameraPoints.empty()){
        qDebug()<<"Error: the objectPoint size is not equal to cameraPoint size!";
        return 100;
    }

    cv::Size cameraSize = cv::Size(imagelist[0].cols,imagelist[1].rows);
    double cameraError= cv::calibrateCamera(objectPoints,
                               cameraPoints,
                               cameraSize,
                               camera_intrinsic,
                               camera_distortion,
                               camera_rvecs,camera_tvecs,
                               0,cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));

    std::cout<<"calibrateCamera error : "<<cameraError<<std::endl;

    int boardImage_num = camera_rvecs.size();
    //内参的逆
    cv::Mat camera_intrinsicInv = camera_intrinsic.inv();

    for(int n = 0; n < boardImage_num; n++){

        cv::Mat tempExtrinsic = cv::Mat::zeros(3,3,CV_64FC1);
        cv::Mat rotation_matrix;
        cv::Rodrigues(camera_rvecs[n], rotation_matrix);
        rotation_matrix.colRange(0,2).copyTo(tempExtrinsic.colRange(0,2));
        camera_tvecs[n].copyTo(tempExtrinsic.col(2));
        //求G矩阵
        Gmat = tempExtrinsic.inv()*camera_intrinsicInv;

        phaseValuelist.reserve(boardSize.width*boardSize.height+1);
        worldPoints.reserve(boardSize.width*boardSize.height+1);

        cv::Mat oneMat = cv::Mat::ones(3,3,CV_64FC1);//全1

        for(unsigned int i = 0; i < 90; ++i){

            cv::Point3f temppoint;
            double scale = 1/(cameraPoints[n][i].x * Gmat.at<double>(2,0)+ cameraPoints[n][i].y * Gmat.at<double>(2,1)
                             + Gmat.at<double>(2,2) );

            cv::Mat tempH;
            cv::multiply(camera_intrinsicInv,oneMat,tempH,scale);//乘

            cv::Mat imagepoint = cv::Mat(3,1,CV_64FC1);

            imagepoint.at<double>(0,0) = cameraPoints[n][i].x;
            imagepoint.at<double>(1,0) = cameraPoints[n][i].y;
            imagepoint.at<double>(2,0) = 1;

            cv::Mat objpointMat = tempH*imagepoint;

            temppoint.x = objpointMat.at<double>(0,0);
            temppoint.y = objpointMat.at<double>(1,0);
            temppoint.z = objpointMat.at<double>(2,0);

            worldPoints.push_back(temppoint);
            phaseValuelist.push_back(imagelist[n].at<double>( cameraPoints[n][i].x , cameraPoints[n][i].y ) );
        }
    }

    std::cout<<"worldPoints size : "<<worldPoints.size()<<" , phaseValuelist size : "<<phaseValuelist.size()<<std::endl;

    std::vector<cv::Point3f>::const_iterator it_world = worldPoints.begin();
    std::vector<double>::const_iterator it_phase = phaseValuelist.begin();

    int Num = worldPoints.size();

    cv::Mat coefficient = cv::Mat::zeros(Num,8,CV_64FC1);

    aMat = cv::Mat::zeros(8,1,CV_64FC1);

    for(unsigned int i = 0 ; i < Num; ++i){

        double* Ptr = coefficient.ptr<double>(i);

        Ptr[0] = (*it_world ).x ;
        Ptr[1] = (*it_world ).y ;
        Ptr[2] = (*it_world ).z ;
        Ptr[3] = 1 ;
        Ptr[4] = -(*it_phase )* (*it_world ).x ;
        Ptr[5] = -(*it_phase )* (*it_world ).y ;
        Ptr[6] = -(*it_phase )* (*it_world ).z ;
        Ptr[7] = -(*it_phase ) ;

        ++it_world;
        ++it_phase;
    }

    cv::SVD::solveZ(coefficient,aMat );

    saveSingleCameraCalibrateResult(cameraError);

    std::cout<<"singleCamera_system_calibrate consumed : "<<t1.elapsed()<<" ms\n";
    return cameraError;
}

double CalibrateMachine::doubleCamera_system_calibrate(std::vector<cv::Mat> &phaseX,std::vector<cv::Mat> &phaseY,int boardNum)
{

    if(phaseX.empty() || phaseY.empty() ){
        qDebug()<<"Error: the imagelist is empty and doubleCamera_system_calibrate failed ";
        return 1000000;
    }
    QTime t1;
    t1.start();

    //物点坐标
    for (int t = 0; t < boardNum; ++t)
    {
        std::vector<cv::Point3f> tempPointSet;
        for (int i = 0; i < boardSize.height; ++i)
        {
            for (int j = 0 ; j < boardSize.width; ++j)
            {
                cv::Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.x = j*squareSize.width;
                realPoint.y = i*squareSize.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        objectPoints.push_back(tempPointSet);
    }
    //求取dlp坐标
    pointsCameraToDlp(phaseX,phaseY,20,cameraPoints,dlpPoints);

    cv::Size cameraSize = cv::Size(1928,1448);
    double cameraError= cv::calibrateCamera(objectPoints,
                               cameraPoints,
                               cameraSize,
                               camera_intrinsic,
                               camera_distortion,
                               camera_rvecs,camera_tvecs,
                               0,cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));

    qDebug()<<"\nthe num of groups : "<<cameraPoints.size();

    qDebug()<<"cameraError: "<<cameraError;

    cv::Size dlpSize = cv::Size(1280,800);
    double dlpError= cv::calibrateCamera(objectPoints,
                               dlpPoints,
                               dlpSize,
                               dlp_intrinsic,
                               dlp_distortion,
                               dlp_rvecs,dlp_tvecs,
                               0,cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));

     qDebug()<<"dlpError: "<<dlpError;

     double stereoError = cv::stereoCalibrate(objectPoints, cameraPoints,  dlpPoints,camera_intrinsic, camera_distortion,dlp_intrinsic, dlp_distortion,
             cameraSize, _R, _T, _E, _F,
                                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, DBL_EPSILON)
                                             ,/*cv::CALIB_FIX_INTRINSIC*/ cv::CALIB_USE_INTRINSIC_GUESS /*+ cal_flags*/);

     qDebug()<<"stereoError: "<<stereoError;

     saveDoubleCameraCalibrateResult(cameraError,dlpError,stereoError);

     return stereoError;
}

void CalibrateMachine::saveSingleCameraCalibrateResult(double cameraError)
{

    cv::FileStorage fs("../ScanData/SingleCalibResult.yml", cv::FileStorage::WRITE); //填入写操作
    time_t rawtime;
    time(&rawtime);

    fs<<"calibrationDate"<<asctime(localtime(&rawtime));

    fs<<"Camera Calib - reprojection error"<<cameraError;
    fs<<"cameraIntrinsic"<<camera_intrinsic;
    fs<<"cameraDistCoeffs"<<camera_distortion;
    fs<<"aMatrix"<<aMat;

    fs.release();

    std::cout<<"saveSingleCameraCalibrateResult successfully ... \n";
}

void CalibrateMachine::saveDoubleCameraCalibrateResult(double cameraError,double dlpError,double stereoError)
{

    cv::FileStorage fs("../ScanData/DoubleCalibResult.yml", cv::FileStorage::WRITE); //填入写操作
    time_t rawtime;
    time(&rawtime);

    fs<<"calibrationDate"<<asctime(localtime(&rawtime));

    fs<<"Camera Calib - reprojection error"<<cameraError;
    fs<<"cameraM"<<camera_intrinsic;
    fs<<"cameraKc"<<camera_distortion;
    fs<<"Project Calib - reprojection error"<<dlpError;
    fs<<"proM"<<dlp_intrinsic;
    fs<<"proKc"<<dlp_distortion;
    fs<<"R"<<_R;
    fs<<"T"<<_T;
    fs<<"Stereo Calib - reprojection error"<<stereoError;
    fs.release();

    std::cout<<"saveDoubleCameraCalibrateResult successfully ... \n";

}

void CalibrateMachine::setCalibrateBoardImage(cv::Mat &img)
{
    calibrateBoardImg = img.clone();

    start();
}

void CalibrateMachine::setBoardSize(const int height, const int width)
{
    this->boardSize.width = width;
    this->boardSize.height = height;
}

void CalibrateMachine::setSquareSize(const int height, const int width)
{
    this->squareSize.width = width;
    this->squareSize.height = height;
}

void CalibrateMachine::clearCalibrateVector()
{
    this->objectPoints.clear();
    this->cameraPoints.clear();
    this->dlpPoints.clear();
    this->worldPoints.clear();
    this->phaseValuelist.clear();
}

void CalibrateMachine::run()
{
    std::vector<cv::Point2f> imageCorners;

    bool found = findCircleBoardCorners(calibrateBoardImg,boardSize,imageCorners);

    send_found_circle(found);

}
