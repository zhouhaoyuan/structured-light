#ifndef CALIBRATEMACHINE_H
#define CALIBRATEMACHINE_H

#include <QObject>
#include <QImage>
#include <QThread>
#include <QDebug>
#include <QTime>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

class CalibrateMachine : public QThread
{
    Q_OBJECT
public:
    CalibrateMachine();
    ~CalibrateMachine();

    bool findCircleBoardCorners(cv::Mat &image, cv::Size boardSize, std::vector<cv::Point2f> &realCorners);   

    bool get_cameraPoints(cv::Mat &boardImage,std::vector<cv::Point2f>& cameraP);

    void pointsCameraToDlp(const std::vector<cv::Mat> &phaseX,const std::vector<cv::Mat> &phaseY,int period,
                           std::vector<std::vector<cv::Point2f>> &cameraPoint,
                           std::vector<std::vector<cv::Point2f>> &dlpPoint);

    double singleCamera_system_calibrate(std::vector<cv::Mat> &imagelist,int boardNum);//相对相位图
    double doubleCamera_system_calibrate(std::vector<cv::Mat> &phaseX,std::vector<cv::Mat> &phaseY,int boardNum);

    void saveSingleCameraCalibrateResult(double cameraError);
    void saveDoubleCameraCalibrateResult(double cameraError,double dlpError,double stereoError);

    /****************/
    void setCalibrateBoardImage(cv::Mat &img);

    void setBoardSize(const int height,const int width);
    void setSquareSize(const int height,const int width);

    void clearCalibrateVector();
signals:
    void send_found_circle(bool flag);

protected:
    void run();

private:
    //投影仪当相机的双目标定
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> cameraPoints;
    std::vector<std::vector<cv::Point2f>> dlpPoints;

    //新系统单目标定
    std::vector<cv::Point3f> worldPoints;
    std::vector<double> phaseValuelist;
    cv::Mat aMat;

    cv::Size boardSize;
    cv::Size squareSize;

    cv::Mat _R, _T, _E, _F;
    cv::Mat camera_intrinsic,dlp_intrinsic;
    cv::Mat camera_distortion,dlp_distortion;

    std::vector<cv::Mat> camera_rvecs , camera_tvecs;
    std::vector<cv::Mat> dlp_rvecs , dlp_tvecs;

    cv::Mat E_1,E_2,M_1,M_2;

    cv::Mat calibrateBoardImg;

};

#endif // CALIBRATEMACHINE_H
