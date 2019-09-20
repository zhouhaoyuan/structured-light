#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H

#include <QObject>
#include <QThread>
#include <QDebug>
#include <QImage>
#include <QCoreApplication>
#include <QTime>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dlp_sdk.hpp>

class CaptureThread : public QThread
{
    Q_OBJECT
public:
    CaptureThread();

    void set_Camera(dlp::PG_FlyCap2_C* camera);

    void capture_Image();

    void setCaptureflag(bool flag);

signals:
    void send_capture_mat(cv::Mat &src);


protected:
    void run();

private:
    bool capture_flag;
    bool connectFlag;
    dlp::PG_FlyCap2_C camera;
    dlp::PG_FlyCap2_C* cameraPoint;
};

#endif // CAPTURETHREAD_H
