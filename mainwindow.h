#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <QMainWindow>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <QTextBrowser>
#include <QDebug>
#include <QFileDialog>
#include <QTime>
#include <QMetaType>
#include <QLabel>
#include <QDesktopWidget>
#include <QApplication>
#include <QCheckBox>
#include <QFile>
#include <QTextStream>
#include <QTime>
#include <QImage>
#include <QMessageBox>
#include <QButtonGroup>
#include <QThread>
#include <QDir>
#include <QCoreApplication>
#include <QMenu>
#include <QAction>
#include <QListWidgetItem>
#include <common/returncode.hpp>
#include <common/debug.hpp>
#include <common/parameters.hpp>
#include <common/capture/capture.hpp>
#include <common/pattern/pattern.hpp>
#include <common/disparity_map.hpp>
#include <common/pattern/pattern.hpp>
#include <dlp_sdk.hpp>
#include <structured_light/structured_light.hpp>
#include <structured_light/gray_code/gray_code.hpp>
#include <structured_light/three_phase/three_phase.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <vtkRenderWindow.h>


#include <math.h>

#include "imageitem.h"
#include "capturethread.h"
#include "calibratemachine.h"
#include "yuan_pattern.h"
#include "pointcloud.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    QGraphicsScene* originalScene;
    ImageItem imageItem;



private slots:
    void zoomIn(int level = 1);
    void zoomOut(int level = 1);
    void resetView();
    void setupMatrix();
    void setResetButtonEnabled();
    void setCameraExposure();
    void setCamera_Resolution_FrameRate();

    void menu_loadAction_triggered();//重载
    void menu_prepareAction_triggered();//生成图片
    void on_projectWhiteBtn_clicked();//点亮
    void receive_cameraImage_slot(cv::Mat &img);//相机显示
    void get_space_key_press();//获取空格指令
    void get_calibrate_status_slot(bool flag);//标定拍摄
    void on_captureBtn_clicked();//拍摄
    void on_CalibrateBtn_clicked();//标定
    void on_reconstructionBtn_clicked();//重构

    void show_ply_cloud_file(QListWidgetItem* item);


signals:
    void send_key_press(int key);

protected:
    bool eventFilter(QObject *watched, QEvent *event);

private:

    void initial_fileList();

    void show_cloudFileInfoList(QFileInfoList& list);

    void initialVtkWidget();

    void connect_Device();

    void reconnect_Device();

    void projectSignalPattern( dlp::DLP_Platform *projector, int num);//点亮

    void prepareScanPatterns(bool updateFlag);//准备生成图片

    void generateProjectionPatterns(dlp::Pattern::Orientation orientation, int squence_count, dlp::Pattern::Sequence& scan_sequence);//生成图片

    void SavePatternImageData(const dlp::Pattern::Sequence &sequence,  const std::string basename);//DLP保存图片

    void highSpeedSynchronousShoot(dlp::Camera *camera, dlp::DLP_Platform *projector, const bool &cam_proj_hw_Synchronous,
                                   int startNum, int captureNum, std::vector<cv::Mat> &mat_list);//DLP同步拍摄
    bool CommProjCapture(std::vector<QImage> &Qimagelist,int start,int end);//普通投影仪拍摄

    void capture_calibrateImages();//拍摄标定板

    /*******************decode****************/
    void decode_multiWavelength(std::vector<cv::Mat> &inlist,std::vector<cv::Mat> &outlist,cv::Mat &mask);
    void decode_multiFrequency(std::vector<cv::Mat> &inlist,std::vector<cv::Mat> &outlist,cv::Mat &mask);
    void decode_color_dualThreeStep_dualFrequency(std::vector<cv::Mat> &inlist,std::vector<cv::Mat> &outlist,cv::Mat &mask);

    cv::Mat QImageToCvMat(QImage image);//格式转换

    void cvMatToQImage(cv::Mat &mat, QImage &qimage);//格式转换

    std::string removeSymbol(std::string& str,const std::string& mark);//去除标点符号

    void add_ply_cloud(PointCloud::Ptr pclPCloud,std::string filename);//添加点云
    void delete_ply_cloud(QListWidgetItem* item);//去除点云
    void show_reconstructionResult(std::vector<cv::Point3f>& pCloud, PointCloud::Ptr pclPCloud,std::string filename);//显示重建结果

    void setCameraParameter();

    void delaymsec(int msec);//延时
    void closeEvent( QCloseEvent *event);

private:

    QMenu *menu_loadORprepare;
    QAction *loadAction;
    QAction *prepareAction;
    QLabel *projector_labbel;

    //普通投影图片集
    std::vector<cv::Mat> project_image_list;
    std::vector<QImage> projectQimage;
    std::vector<cv::Mat> scan_mat_list;

    cv::Mat resultMat;
    QTime time;

    //相机控制类
     bool connectCameraFlag;
     //硬件触发标志位
     bool triggerFlag;
     //保存图片序号
     int captureNum;
     int scanNum;
     //采集标定图片标志
     bool getCalibrateImageFlag;
     bool scanFlag;

     int saveCalibrateImageNum;
     int getNumber;

     cv::Mat CalibrateImage;

     QString saveDir;//总的保存目录
     QString currentSaveDir;//现存目录
     QString board_image_path,calib_image_path,measure_path,cloud_path;//棋盘格、标定图像、投影图
     QString savePath;//保存路径

    /****************************************************************************************************************************/
    //DLP Camera
    dlp::LCr4500        projector_4500;
    dlp::PG_FlyCap2_C   camera_pg;
    dlp::Parameters settings;

    std::atomic_bool captureFlag;
    CaptureThread camera_capture_thread;
    CalibrateMachine calibrate_machine;
    dlp::yuan_pattern *pattern_class;

    cv::Mat show_mat;//界面图像
    cv::Mat maskImage; //模板图像
    cv::Mat board_mat; //棋盘格图像
    cv::Size boardSize;//棋盘格尺寸
    int imageWidth = 1280;
    int imageHeight = 800;

    int projectorMode = 0;//0为普通，1位可编程
    bool patterns_prepare;
    bool showFlag;
    bool projecting;
    bool singleCalibrateFlag = false;

    //点云
    QFileInfoList cloudList;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    pointCloud pointCloudClass;
};

#endif // MAINWINDOW_H
