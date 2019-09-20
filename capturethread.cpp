#include "capturethread.h"

CaptureThread::CaptureThread()
{

}

void CaptureThread::set_Camera(dlp::PG_FlyCap2_C *camera)
{
    cameraPoint = camera;
}

void CaptureThread::capture_Image()
{
    capture_flag= true;

    // Check that camera is NOT null
     if(!cameraPoint) return;

     // Check if camera is connected
     if(!cameraPoint->isConnected()){
         dlp::CmdLine::Print("Camera NOT connected! \n");
         return;
     }

     dlp::Image          camera_frame;

     if(cameraPoint->Start().hasErrors()){
         dlp::CmdLine::Print("Could NOT start camera! \n");
         return;
     }

     dlp::Time::Sleep::Microseconds(100000);

        while(capture_flag){

            camera_frame.Clear();               // Clear the image object
            cameraPoint->GetFrame(&camera_frame);    // Grab the latest camera frame

            cv::Mat show_mat;
            //camera_frame.ConvertToRGB();
            camera_frame.GetOpenCVData(&show_mat);
            if(show_mat.channels()==3){
                cv::cvtColor(show_mat,show_mat,CV_RGB2BGR);
            }

            emit send_capture_mat(show_mat);

            dlp::Time::Sleep::Microseconds(50000);

        }

        cameraPoint->Stop();
        dlp::Time::Sleep::Microseconds(100000);
}

void CaptureThread::setCaptureflag(bool flag)
{
    capture_flag = flag;
}

void CaptureThread::run()
{
    capture_Image();
}
