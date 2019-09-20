#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QFrame>
#include <QPainter>
#include <QToolButton>
#include <QSlider>
#include <QScrollBar>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QDebug>
#include <QProcess>
#include <QMetaType>
#include <QDesktopWidget>
#include <QFileInfoList>
#include <QWidget>

#include <QListWidget>
#include <QCheckBox>

#include <qmath.h>
#include <iostream>
#include <atomic>
#include <cctype>
#include <string>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //生成文件夹
    initial_fileList();

    /************显示点云文件*****************/
    ui->reconstruction_listWidget->setMovement(QListView::Static);//设置单元项不可拖动，（Static、Free、Snap）
    ui->reconstruction_listWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);//设置垂直滚动条显示方式（ScrollBarAsNeeded：按需显示，
                                                                                //ScrollBarAlwaysOff：隐藏，ScrollBarAlwaysOn：常显）

    connect(ui->reconstruction_listWidget,SIGNAL(itemDoubleClicked(QListWidgetItem*)),
            this,SLOT(show_ply_cloud_file(QListWidgetItem*)));

    QDir dir_cloud(cloud_path);
    QStringList string;
    string<<"*";
    cloudList = dir_cloud.entryInfoList(string,QDir::Files,QDir::Name);
    show_cloudFileInfoList(cloudList);


    //初始化点云窗口
    initialVtkWidget();

    //标定板角点参数
    ui->CornerHeightspinBox->setValue(9);
    ui->CornerWidthspinBox->setValue(11);
    ui->CornerWidthspinBox->setRange(0,50);
    ui->CornerHeightspinBox->setRange(0,50);
    ui->CornerWidthlengthspinBox->setValue(10);
    ui->CornerHeightlengthspinBox->setValue(10);
    ui->CornerWidthlengthspinBox->setRange(0,100);
    ui->CornerHeightlengthspinBox->setRange(0,100);

    //相机参数设置
    //setCameraParameter();
    /*************************设置界面***************************/

    menu_loadORprepare = new QMenu();
    loadAction = new QAction(menu_loadORprepare);
    prepareAction = new QAction(menu_loadORprepare);
    menu_loadORprepare->addAction(loadAction);
    menu_loadORprepare->addAction(prepareAction);
    loadAction->setText(tr("Load pattern"));
    prepareAction->setText(tr("Prepare pattern"));
    menu_loadORprepare->setStyleSheet("font: 10pt");
    ui->loadpatternBtn->setMenu(menu_loadORprepare);
    connect(loadAction,SIGNAL(triggered()),this,SLOT(menu_loadAction_triggered()));
    connect(prepareAction,SIGNAL(triggered()),this,SLOT(menu_prepareAction_triggered()));

    ui->graphicsView->setFrameStyle( QFrame::Sunken | QFrame::StyledPanel);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing,false);//反锯齿
    ui->graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);//交互小手
    ui->graphicsView->setOptimizationFlags(QGraphicsView::DontSavePainterState);
    ui->graphicsView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    ui->graphicsView->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);


    //主界面
    originalScene =new QGraphicsScene(this);
   // QPixmap* gdutPixmap=new QPixmap("GDUT.jpg");
   // originalScene->addPixmap(*gdutPixmap);
    originalScene->addItem(&imageItem);
    ui->graphicsView->setScene(originalScene);
    ui->graphicsView->show();

    QPixmap gdutImage;
    gdutImage.load(":/systemImage/GDUT.jpg");
    imageItem.setPixmap(gdutImage);

    int size = style()->pixelMetric(QStyle::PM_ToolBarIconSize);
    QSize iconSize(size, size);

    ui->zoomSlider->setMinimum(0);
    ui->zoomSlider->setMaximum(500);
    ui->zoomSlider->setValue(250);
    ui->zoomSlider->setTickPosition(QSlider::TicksRight);

    ui->ZoomInToolBtn->setAutoRepeat(true);
    ui->ZoomInToolBtn->setAutoRepeatInterval(33);
    ui->ZoomInToolBtn->setAutoRepeatDelay(0);
    QIcon ZoomInToolBtn(":/systemImage/zoomin.png");
    ui->ZoomInToolBtn->setIcon(ZoomInToolBtn);
    ui->ZoomInToolBtn->setIconSize(iconSize);

    ui->ZoomOutTBtn->setAutoRepeat(true);
    ui->ZoomOutTBtn->setAutoRepeatInterval(33);
    ui->ZoomOutTBtn->setAutoRepeatDelay(0);
    QIcon ZoomOutToolBtn(":/systemImage/zoomout.png");
    ui->ZoomOutTBtn->setIcon(ZoomOutToolBtn);
    ui->ZoomOutTBtn->setIconSize(iconSize);

    ui->ResetTBtn->setText(tr("0"));
    ui->ResetTBtn->setEnabled(false);

    connect(ui->ResetTBtn, SIGNAL(clicked()), this, SLOT(resetView()));
    connect(ui->zoomSlider, SIGNAL(valueChanged(int)), this, SLOT(setupMatrix()));
    connect(ui->ZoomInToolBtn, SIGNAL(clicked(bool)), this, SLOT(zoomIn()));
    connect(ui->ZoomOutTBtn, SIGNAL(clicked(bool)), this, SLOT(zoomOut()));
    connect(ui->graphicsView->verticalScrollBar(), SIGNAL(valueChanged(int)),
            this, SLOT(setResetButtonEnabled()));
    connect(ui->graphicsView->horizontalScrollBar(), SIGNAL(valueChanged(int)),
            this, SLOT(setResetButtonEnabled()));

    //相机传输图片
    qRegisterMetaType<cv::Mat>("cv::Mat&");
    connect(&camera_capture_thread,SIGNAL(send_capture_mat(cv::Mat&)),this,SLOT(receive_cameraImage_slot(cv::Mat&)));
    //找圆心信号
    connect(&calibrate_machine,SIGNAL(send_found_circle(bool)),this,SLOT(get_calibrate_status_slot(bool)));
    //空格按下信号
    connect(this,SIGNAL(send_key_press(int)),this,SLOT(get_space_key_press()));
    //安装时间过滤器
    ui->graphicsView->installEventFilter(this);

    /***********************************************************/
    triggerFlag= false;

    getCalibrateImageFlag= false;

    saveCalibrateImageNum= 0;

    scanFlag= false;

    scanNum= 0;

    getNumber= 0;

    patterns_prepare= false;

    showFlag= true;



    boardSize.width  = ui->CornerWidthspinBox->value();
    boardSize.height = ui->CornerHeightspinBox->value();

    /***************************************************/
    QDir dir_calibration(calib_image_path);
    QStringList dir_list_calib;//标定文件夹的绝对路径
    QStringList name_list_calib;//标定文件夹的代号
    foreach(QFileInfo fileInfo,dir_calibration.entryInfoList(QDir::Dirs | QDir::Files))
    {
        QString strName = fileInfo.fileName();
        if( (strName == QString(".")) || (strName == QString("..")) )
            continue;
        if(fileInfo.isDir()){
            QString str = fileInfo.absolutePath() + "/" + strName + "/";
            dir_list_calib.push_back(str);
            name_list_calib.push_back(strName);
        }
    }
    saveCalibrateImageNum = dir_list_calib.size();
    qDebug()<<"saveCalibrateImageNum : "<<saveCalibrateImageNum;

    pattern_class = new dlp::yuan_pattern;
    connect_Device();
    projector_labbel = new QLabel;
}

MainWindow::~MainWindow()
{
    delete ui;   
    deletre_thread.wait();
    }

    if(calibrate_machine.isRunning()){
        calibrate_machine.quit();
        calibrate_machine.wait();
    }
}
//放大
void MainWindow::zoomIn(int level)
{
    ui->zoomSlider->setValue( ui->zoomSlider->value() + level);

}
//缩小
void MainWindow::zoomOut(int level)
{
    ui->zoomSlider->setValue( ui->zoomSlider->value() - level);

}
//重置
void MainWindow::resetView()
{
    ui->zoomSlider->setValue(250);
    setupMatrix();
    ui->graphicsView->ensureVisible(QRectF(0, 0, 0, 0));
    ui->ResetTBtn->setEnabled(false);
}
//截面表换矩阵
void MainWindow::setupMatrix()
{
    //qreal默认为double
    double scale = qPow(qreal(2), (ui->zoomSlider->value() - 250) / qreal(50));
    QMatrix matrix;
    matrix.scale(scale, scale);
    ui->graphicsView->setMatrix(matrix);
    setResetButtonEnabled();
}

void MainWindow::setResetButtonEnabled()
{
    ui->ResetTBtn->setEnabled(true);
}

void MainWindow::setCameraExposure()
{

}

void MainWindow::setCamera_Resolution_FrameRate()
{
    switch(ui->resolutionComboBox->currentIndex())
    {
    case 0:
    {

    }
        break;
    case 1:
    {

    }
        break;
    }
}

void MainWindow::menu_loadAction_triggered()
{
    QMessageBox::StandardButton ok = QMessageBox::warning(this,"Warning","Do You want to Update Firmware?",QMessageBox::Yes |QMessageBox::No, QMessageBox::No);
    if(ok== QMessageBox::Yes)
    {
        prepareScanPatterns(true);
        patterns_prepare= true;
        reconnect_Device();
    }
    qDebug()<<"Please wait! Loading pattern ... ";
}

void MainWindow::menu_prepareAction_triggered()
{
    qDebug()<<"Prepare pattern ... ";
    prepareScanPatterns(false);
    patterns_prepare= true;  
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->graphicsView )
    {
        if (event->type() == QEvent::KeyPress)
        {
           QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

           if (keyEvent->key() == Qt::Key_Space){

                emit send_key_press(keyEvent->key());
           }
           keyEvent->accept();
           return true;
        }
        else if(event->type() == QEvent::Wheel)
        {
            QWheelEvent *wheelEvent = static_cast<QWheelEvent *>(event);

            if(wheelEvent->delta() > 0){
                zoomIn(6);
            }else{
                zoomOut(6);
            }

            wheelEvent->accept();
            return true;
        }else {
           // standard event processing
           return QObject::eventFilter(watched, event);
       }
    }
}

void MainWindow::initial_fileList()
{
    /**********************************************/
    //文件夹生成
    saveDir = "../ScanData/";
    QDir dir(saveDir);
    if(!dir.exists()){
        bool ok = dir.mkdir(saveDir);//只创建一级目录，必须保证上级目录存在
        qDebug()<<"create dir : "<<saveDir<<" "<<ok;
    }
    saveDir = dir.absolutePath();

    board_image_path= saveDir+"/board_image/";
    QDir board_image_dir(board_image_path);
    if(!board_image_dir.exists())
    {
       bool ok = board_image_dir.mkdir(board_image_path);//只创建一级子目录，即必须保证上级目录存在
       qDebug()<<"create dir : "<<board_image_path<<" "<<ok;
    }
    board_image_path= board_image_dir.absolutePath();

    calib_image_path= saveDir+"/calib_image/";
    QDir calib_image_dir(calib_image_path);
    if(!calib_image_dir.exists())
    {
       bool ok = calib_image_dir.mkdir(calib_image_path);//只创建一级子目录，即必须保证上级目录存在
       qDebug()<<"create dir: "<<calib_image_path<<" "<<ok;
    }
    calib_image_path= calib_image_dir.absolutePath();

    measure_path= saveDir+"/measure_image/";
    QDir measure_image_dir(measure_path);
    if(!measure_image_dir.exists())
    {
       bool ok = measure_image_dir.mkdir(measure_path);//只创建一级子目录，即必须保证上级目录存在
       qDebug()<<"create dir : "<<measure_path<<" "<<ok;
    }
    measure_path = measure_image_dir.absolutePath();

    cloud_path = saveDir+"/cloud_File/";
    QDir cloud_path_dir(cloud_path);
    if(!cloud_path_dir.exists())
    {
        bool ok = cloud_path_dir.mkdir(cloud_path);
        qDebug()<<"create dir : "<<cloud_path<<" "<<ok;
    }
}

void MainWindow::show_cloudFileInfoList(QFileInfoList& list)
{
    for(unsigned int i = 0; i < list.count(); ++i)
    {
        QFileInfo tmpFileInfo = list.at(i);
        if(tmpFileInfo.isFile())
        {
            QIcon icon(":/systemImage/ply.png");
            QString fileName = "   " + tmpFileInfo.fileName();
            QListWidgetItem *tmp = new QListWidgetItem(icon,fileName);
            QCheckBox* box = new QCheckBox();
            box->setCheckable(true);
            box->setChecked(false);
            ui->reconstruction_listWidget->addItem(tmp);
            ui->reconstruction_listWidget->setItemWidget(tmp,box);
        }
    }
}

void MainWindow::initialVtkWidget()
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer",false));

    viewer->addPointCloud(cloud, "cloud");
    viewer->setBackgroundColor(0.1,0.1,0.1);
    viewer->addCoordinateSystem(1.0);
    //viewer->initCameraParameters ();

    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());

    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());

    ui->qvtkWidget->update();

}

void MainWindow::connect_Device()
{
    DLP_NEW_PARAMETERS_ENTRY(ConnectIdProjector,           "CONNECT_ID_PROJECTOR",  std::string, "0");
    DLP_NEW_PARAMETERS_ENTRY(ConnectIdCamera,              "CONNECT_ID_CAMERA",     std::string, "0");

    DLP_NEW_PARAMETERS_ENTRY(ConfigFileProjector,           "CONFIG_FILE_PROJECTOR",                std::string, "config/config_projector.txt");
    DLP_NEW_PARAMETERS_ENTRY(ConfigFileCamera,              "CONFIG_FILE_CAMERA",                   std::string, "config/config_camera.txt");

    DLP_NEW_PARAMETERS_ENTRY(ConfigFileStructuredLight1,    "CONFIG_FILE_STRUCTURED_LIGHT_1",       std::string, "config/algorithm_vertical.txt");
    DLP_NEW_PARAMETERS_ENTRY(ConfigFileStructuredLight2,    "CONFIG_FILE_STRUCTURED_LIGHT_2",       std::string, "config/algorithm_horizontal.txt");
    DLP_NEW_PARAMETERS_ENTRY(ConfigFileCalibProjector,      "CONFIG_FILE_CALIBRATION_PROJECTOR",    std::string, "config/calibration_projector.txt");

    DLP_NEW_PARAMETERS_ENTRY(OutputNameImageCameraCalibBoard, "OUTPUT_NAME_IMAGE_CAMERA_CALIBRATION_BOARD", std::string, "camera_calibration_board");
    DLP_NEW_PARAMETERS_ENTRY(OutputNameImageCameraCalib,    "OUTPUT_NAME_IMAGE_CAMERA_CALIBRATION", std::string, "camera_calibration_capture_");

    DLP_NEW_PARAMETERS_ENTRY(ConfigFileCalibCamera,         "CONFIG_FILE_CALIBRATION_CAMERA",       std::string, "config/calibration_camera.txt");
    DLP_NEW_PARAMETERS_ENTRY(CalibDataFileCamera,           "CALIBRATION_DATA_FILE_CAMERA",         std::string, "calibration/data/camera.xml");
    DLP_NEW_PARAMETERS_ENTRY(DirCameraCalibImageOutput,     "DIRECTORY_CAMERA_CALIBRATION_IMAGE_OUTPUT",    std::string, "calibration/camera_images/");

    ConnectIdProjector          connect_id_projector;
    ConnectIdCamera             connect_id_camera;

    ConfigFileProjector         config_file_projector;
    ConfigFileCamera            config_file_camera;


    ConfigFileCalibCamera       config_file_calib_camera;
    CalibDataFileCamera         calib_data_file_camera;
    DirCameraCalibImageOutput   dir_camera_calib_image_output;
    OutputNameImageCameraCalib  output_name_image_camera_calib;

    OutputNameImageCameraCalibBoard output_name_image_camera_calib_board;
    ConfigFileStructuredLight1  config_file_structured_light_1;
    ConfigFileStructuredLight2  config_file_structured_light_2;
    ConfigFileCalibProjector    config_file_calib_projector;

    // Variables used throughout code
    dlp::ReturnCode             ret;
   // dlp::StructuredLight*       module;
  //  dlp::Pattern::Sequence      sequence;
  //  dlp::Pattern::Color         color       = dlp::Pattern::Color::WHITE;   // Since all patterns are greyscale this does not matter for the saved images
 //  dlp::Pattern::Orientation   orientation = dlp::Pattern::Orientation::VERTICAL;
//    unsigned int width  = 912;
//    unsigned int height = 1140;
//    unsigned int pattern_resolution = width;
//    std::string basename = "pattern_";
//    std::string image_name = "image";

    // Load the settings
    dlp::Parameters settings;
    settings.Load("DLP_LightCrafter_4500_3D_Scan_Application_Config.txt");

    settings.Get(&connect_id_projector);
    settings.Get(&config_file_projector);
    settings.Get(&config_file_calib_projector);
    settings.Get(&config_file_structured_light_1);
    settings.Get(&config_file_structured_light_2);

    settings.Get(&output_name_image_camera_calib_board);
    settings.Get(&connect_id_camera);
    settings.Get(&config_file_camera);

    settings.Get(&config_file_calib_camera);
    settings.Get(&calib_data_file_camera);
    settings.Get(&dir_camera_calib_image_output);
    settings.Get(&output_name_image_camera_calib);

    // Connect camera and projector
    ret = dlp::DLP_Platform::ConnectSetup(projector_4500,connect_id_projector.Get(),config_file_projector.Get(),true);
    if(ret.hasErrors()) {
        dlp::CmdLine::Print("\n\nPlease resolve the LightCrafter connection issue before proceeding to next step...\n");
        projectorMode  = 0;//普通投影仪
    }else{
        projectorMode = 1;//可编程
    }
    ui->textBrowser->append("ProjectorMode is "+QString::number(projectorMode));

    ret = dlp::Camera::ConnectSetup(camera_pg,connect_id_camera.Get(),config_file_camera.Get(),true);
    if(ret.hasErrors()) {
        dlp::CmdLine::Print("\n\nPlease resolve the camera connection issue before proceeding to next step...\n");
    }


}

void MainWindow::reconnect_Device()
{
    camera_pg.Disconnect();

    projector_4500.Disconnect();

    // Reconnect system objects
    connect_Device();
}

void MainWindow::projectSignalPattern(dlp::DLP_Platform *projector, int num)
{
    if(camera_capture_thread.isRunning()){
        QMessageBox::warning(this,"Warning","Error: The camera is running!");
        return;
    }

    camera_capture_thread.set_Camera(&camera_pg);
    camera_capture_thread.start();
    /*******************************************************************************************************/
     // Check that projector is NOT null
    switch(projectorMode)
    {

    case 1:
    {
         if(!projector) return;

         // Check if projector is connected
         if(!projector->isConnected()){
             dlp::CmdLine::Print("Projector NOT connected! \n");
             return;
         }
         switch(num)
         {
         case 0:
         {
             projector->ProjectSolidBlackPattern();
         }
             break;
         case 1:
         {
             projector->ProjectSolidWhitePattern();
         }
             break;
         }
    }
        break;
    case 0:
    {
        QDesktopWidget *deskWgt = QApplication::desktop();
        int screen_count = deskWgt->screenCount();
        ui->textBrowser->append("projector number : "+QString::number(screen_count));

        QRect rect = deskWgt->screenGeometry(0);//主窗口
        if(screen_count == 1)
        {
            imageWidth = rect.width()/2;
            imageHeight = rect.height()/2;
            projector_labbel->move(rect.width()/4,rect.height()/4);
        }else{
            QRect rect1 = deskWgt->screenGeometry(1);
            imageWidth = rect1.width();
            imageHeight = rect1.height();
            projector_labbel->move(rect.width(),0);
            projector_labbel->showFullScreen();
        }
        QImage whiteQimage;
        cv::Mat whiteMat = cv::Mat(imageHeight,imageWidth,CV_8UC3,cv::Scalar(255,255,255));
        cvMatToQImage(whiteMat,whiteQimage);
        projector_labbel->setPixmap(QPixmap::fromImage(whiteQimage));
        projector_labbel->show();
    }
        break;
    }

}

void MainWindow::prepareScanPatterns(bool updateFlag)
{
    switch(projectorMode)
    {
        case 1 ://可编程
    {
        if(camera_capture_thread.isRunning()){
            camera_pg.Disconnect();
            projector_4500.Disconnect();
            dlp::Time::Sleep::Microseconds(500000);
        }

        dlp::Pattern::Sequence sequence;
        dlp::Parameters force_preparation;
        dlp::ReturnCode ret;

        //生成图片
        int patterns_count = 12;
        generateProjectionPatterns(dlp::Pattern::Orientation::VERTICAL,patterns_count,sequence);

        force_preparation.Set(dlp::DLP_Platform::Parameters::SequencePrepared(!updateFlag));
        projector_4500.Setup(force_preparation);

        // If the firmware needs to be uploaded, display debug messages so progress can be observed
    //    projector->SetDebugEnable(!previously_prepared);
        projector_4500.SetDebugEnable(updateFlag);

        // Prepare projector
        dlp::CmdLine::Print("Preparing projector for calibration and structured light...");
        ret = projector_4500.PreparePatternSequence(sequence);
        projector_4500.SetDebugEnable(false);

        if( ret.hasErrors()){
            dlp::CmdLine::Print("Projector prepare sequence FAILED: ");
            dlp::CmdLine::Print(ret.ToString());
        }
    }
        break;

    case 0://普通
    {
        project_image_list.clear();
        pattern_class->set_ImageSize(imageHeight,imageWidth);
        //多频
#if 0
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::VERTICAL,40,4);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::VERTICAL,46,4);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::VERTICAL,350,4);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::HORIZONTAL,40,4);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::HORIZONTAL,46,4);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::HORIZONTAL,350,4);
#endif

        //多波长
#if 1
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::VERTICAL,2000,3);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::VERTICAL,2000,3,CV_PI/3);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::VERTICAL,960,4);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::VERTICAL,320,4);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::VERTICAL,80,4);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::VERTICAL,20,4);

        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::HORIZONTAL,2000,3);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::HORIZONTAL,2000,3,CV_PI/3);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::HORIZONTAL,960,4);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::HORIZONTAL,320,4);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::HORIZONTAL,80,4);
        pattern_class->generate_sinusoidal_fringe(project_image_list,dlp::Pattern::Orientation::HORIZONTAL,20,4);

#endif
        //彩色双频双三步
#if 1
        pattern_class->generate_color_dual_three_step_sin_pattern(project_image_list,80,85,3,CV_PI/3,dlp::Pattern::Orientation::VERTICAL);
#endif
    }
        break;
    }
        //////做标记
    //        for(unsigned int num = 0; num < project_image_list.size(); ++num ){

    //            cv::Mat image = cv::Mat::zeros(cv::Size(100, 100), CV_8UC1);
    //            //设置蓝色背景
    //            image.setTo(cv::Scalar(255));

    //            //设置绘制文本的相关参数
    //            std::string text = QString::number(num+1).toStdString();
    //            int font_face = cv::FONT_HERSHEY_COMPLEX;
    //            double font_scale = 3;
    //            int thickness = 3;
    //            int baseline;
    //            //获取文本框的长宽
    //            cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);

    //            //将文本框居中绘制
    //            cv::Point origin;
    //            origin.x = image.cols / 2 - text_size.width / 2;
    //            origin.y = image.rows / 2 + text_size.height / 2;
    //            cv::putText(image, text, origin, font_face, font_scale, cv::Scalar(0), thickness, 8, 0);

    //            cv::Mat ROIimage = project_image_list[num](cv::Rect(350,250,100,100));
    //            image.copyTo(ROIimage);

    //            cv::imwrite("../ScanData/measure_image/"+QString::number(num).toStdString()+".bmp",project_image_list[num]);

    //        }

    ////保存生成的图片
//        for(int num = 0; num< project_image_list.size();++num){
//            cv::imwrite("../ScanData/measure_image/"+QString::number(num).toStdString()+".bmp",project_image_list[num]);
//        }
    ////转换为QImage
    projectQimage.clear();
    for(unsigned int num = 0; num < project_image_list.size(); ++num ){

        QImage temp;
        cvMatToQImage(project_image_list[num],temp);
        projectQimage.push_back(temp);
    }

    patterns_prepare= true;
    dlp::CmdLine::Print("Patterns prepared successfully!");

}

void MainWindow::generateProjectionPatterns(dlp::Pattern::Orientation orientation, int squence_count, dlp::Pattern::Sequence &scan_sequence)
{
    dlp::Pattern::Sequence  sub_sequence;
    dlp::ReturnCode             ret;

    dlp::Pattern::Color         color       = dlp::Pattern::Color::WHITE;   // Since all patterns are greyscale this does not matter for the saved images
    unsigned int width  = 912;
    unsigned int height = 1140;

    dlp::Parameters settings;
    settings.Clear();

    // Setup the module
    do{
        unsigned int pixels_per_period = 0;
        unsigned int bit_depth  = 8;
        dlp::Pattern::Bitdepth pattern_bitdepth;

        // Convert integer to pattern bitdepth
        switch (bit_depth) {
        case 1:
            pattern_bitdepth = dlp::Pattern::Bitdepth::MONO_1BPP;
            break;
        case 2:
            pattern_bitdepth = dlp::Pattern::Bitdepth::MONO_2BPP;
            break;
        case 3:
            pattern_bitdepth = dlp::Pattern::Bitdepth::MONO_3BPP;
            break;
        case 4:
            pattern_bitdepth = dlp::Pattern::Bitdepth::MONO_4BPP;
            break;
        case 5:
            pattern_bitdepth = dlp::Pattern::Bitdepth::MONO_5BPP;
            break;
        case 6:
            pattern_bitdepth = dlp::Pattern::Bitdepth::MONO_6BPP;
            break;
        case 7:
            pattern_bitdepth = dlp::Pattern::Bitdepth::MONO_7BPP;
            break;
        case 8:
            pattern_bitdepth = dlp::Pattern::Bitdepth::MONO_8BPP;
            break;
        default:
            pattern_bitdepth = dlp::Pattern::Bitdepth::MONO_8BPP;
            break;
        }

        settings.Set(dlp::StructuredLight::Parameters::PatternColor(color));
        settings.Set(dlp::StructuredLight::Parameters::PatternOrientation(orientation));
        settings.Set(dlp::StructuredLight::Parameters::PatternColumns(width));
        settings.Set(dlp::StructuredLight::Parameters::PatternRows(height));

        settings.Set(dlp::yuan_pattern::Parameters::Bitdepth(pattern_bitdepth));
        settings.Set(dlp::yuan_pattern::Parameters::PixelsPerPeriod(pixels_per_period));
        settings.Set(dlp::yuan_pattern::Parameters::SequenceCount(squence_count));

        // Setup the module
        ret = pattern_class->Setup(settings);

        // Print the result
        std::cout << "Setting up structured light module..." << ret.ToString() << std::endl;
    }while(ret.hasErrors());

    // Generate the patterns
    std::cout << "Generating structured light module patterns..." << std::endl;
    pattern_class->set_ImageSize(1140,912);
    pattern_class->GeneratePatternSequence(&sub_sequence);
    scan_sequence.Add(sub_sequence);
    // Clear variables and deallocate memory
    settings.Clear();

}

void MainWindow::SavePatternImageData(const dlp::Pattern::Sequence &sequence, const std::string basename)
{
    unsigned int count = 0;
    count = sequence.GetCount();

    for(unsigned int iPattern = 0; iPattern < count; ++iPattern){

        dlp::Pattern temp_pattern;
        // Get the pattern
        sequence.Get(iPattern, &temp_pattern);
        // Save the image data
        std::cout << "Saving image " << basename + "_" +dlp::Number::ToString(iPattern) + ".bmp" << std::endl;

        if(iPattern< 10)
        {
            temp_pattern.image_data.Save(basename + "_0" +dlp::Number::ToString(iPattern) + ".bmp");
        }
        else
        {
            temp_pattern.image_data.Save(basename + "_" +dlp::Number::ToString(iPattern) + ".bmp");
        }

    }

    return;
}

void MainWindow::highSpeedSynchronousShoot(dlp::Camera *camera, dlp::DLP_Platform *projector, const bool &cam_proj_hw_Synchronous, int startNum, int captureNum, std::vector<cv::Mat> &mat_list)
{
    //check the camera is not null
    if(!camera){
        qDebug()<<"the camera is NULL ... ";
        return;
    }
    //check whether the camera is connected
    if(!camera->isConnected()){
        dlp::CmdLine::Print("Camera NOT connected! \n");
        return;
    }
    // Check the projector is NOT null
    if(!projector){
        qDebug()<<"the camera is NULL ... ";
        return;
    }
    // Check whether the projector is connected
    if(!projector->isConnected()){
        dlp::CmdLine::Print("Projector NOT connected! \n");
        return;
    }

    // Variables used during scan loop
    int scan_count = 0;
    std::atomic_bool continue_scanning(true);
    dlp::Time::Chronograph  timer;
    dlp::Capture::Sequence  capture_scan;

    // Get the camera frame rate (This assumes the camera triggers the projector!)
    float frame_rate;
    camera->GetFrameRate(&frame_rate);

    // Determine the total scan time
    unsigned int period_us = 1000000 / frame_rate;

    unsigned int capture_time = 0;
    unsigned int pattern_start = startNum; // There is one calibration image so start with offse
    unsigned int pattern_count = captureNum;

    capture_time += period_us * pattern_count;

    // Enter the scanning loop
    while(continue_scanning){

        dlp::CmdLine::Print("\nStarting scan ", scan_count, "...");

        dlp::Capture::Sequence sequence_scan;

        //Peform images capture when both camera and projector are connected
        //via HW trigger signal for synchronization
        if(cam_proj_hw_Synchronous == true) {

            timer.Reset();
            // Start capturing images from the camera
            if(camera->Start().hasErrors()){
                dlp::CmdLine::Print("Could NOT start camera! \n");
                return;
            }
            // Scan the object
            dlp::ReturnCode sequence_return;
            sequence_return = projector->StartPatternSequence(pattern_start,pattern_count ,false);
            if(sequence_return.hasErrors()){
                dlp::CmdLine::Print("Sequence failed! Exiting scan routine ...");
                if(camera->Stop().hasErrors()){
                    dlp::CmdLine::Print("Camera failed to stop! Exiting scan routine ...");
                }
                dlp::CmdLine::Print("Sequence failed..." + sequence_return.ToString());
                return;
            }

            // Give camera time to start capturing images
//            dlp::Time::Sleep::Milliseconds(100);

            dlp::CmdLine::Print("DLP and Camera start time... \t \t", timer.Lap(), "ms");

            /**************************************************************************************************************/

            int loop_num= captureNum/3 +1;

            float period= 2.80;

            int reload_time= 78000;

            timer.Reset();
            camera->ControlStrobe(2);//设置存储序列，解锁
            dlp::CmdLine::Print("open Capture...\t\t", timer.Lap(), "ms");

            for(int i_capture= 0;i_capture< loop_num;i_capture++)
            {

                //open strobe
                camera->ControlStrobe(1);
                //capture sequence
                dlp::Time::Sleep::Microseconds(period*period_us);
                //close strobe
                camera->ControlStrobe(0);
                //reload sequence
                dlp::Time::Sleep::Microseconds(reload_time/3);
                //close store sequence
                camera->ControlStrobe(3);
                //reload sequence
                dlp::Time::Sleep::Microseconds(2*reload_time/3);
                //open store sequence
                camera->ControlStrobe(2);

            }
            //open strobe
            camera->ControlStrobe(1);

            // Stop grabbing images from the camera
            if(camera->Stop().hasErrors()){
                dlp::CmdLine::Print("Camera failed to stop! Exiting scan routine...");
                return;
            }

            dlp::CmdLine::Print("Pattern sequence capture completed in...\t", timer.Lap(), "ms");
            projector->StopPatternSequence();

            // Grab all of the images from the buffer to find the pattern sequence
            bool            min_images = false;
            dlp::ReturnCode ret;
            dlp::Capture    capture;
            dlp::Image      capture_image;

            std::vector<double> capture_sums;

            while(!min_images){

                capture_image.Clear();
                ret = camera->GetFrameBuffered(&capture_image);
                capture_image.ConvertToMonochrome();//！！！转灰度

                if(ret.hasErrors()){
                    min_images = true;
                    dlp::CmdLine::Print("Error: camera GetFrameBuffered has errors ... ;");
                }
                else{

                    double sum = 0;
                    capture_image.GetSum(&sum);
                    capture_sums.push_back(sum);

                    // Add the frame to the sequence
                    capture.image_data = capture_image;
                    capture.data_type  = dlp::Capture::DataType::IMAGE_DATA;
                    //capture_scan is a sequence
                    capture_scan.Add(capture);
                    capture_image.Clear();
                    capture.image_data.Clear();

                }

            }

            dlp::CmdLine::Print("Images retreived from buffer in...\t\t", timer.Lap(), "ms");

            sequence_scan.Clear();

            double previous_sum = capture_sums.at(0);

            timer.Lap();
            mat_list.clear();

            for(unsigned int iScan = 0; iScan < capture_scan.GetCount(); iScan++)
            {

                //get all buffer images
                dlp::Capture temp_buffer;
                capture_scan.Get(iScan,&temp_buffer);

                // Retrieve the image sum value
                double sum = capture_sums.at(iScan);

                //save capture sequence
                if(sum> previous_sum*1.05)
                {

                    dlp::Capture temp;
                    capture_scan.Get(iScan,&temp);

                    cv::Mat capture_mat;
                    temp.image_data.GetOpenCVData(&capture_mat);
                    mat_list.push_back(capture_mat);

                    temp.image_data.Clear();
                }

            }
        }

        capture_scan.Clear();
        dlp::CmdLine::Print("Patterns sorted in...\t\t\t\t", timer.Lap(), "ms");
/*************************************************************************************************************************************/

        if(camera->Stop().hasErrors()){
            dlp::CmdLine::Print("Camera failed to stop! Exiting scan routine...");
        }else{
            dlp::CmdLine::Print("camera->Stop()...\t\t", timer.Lap(), "ms");
            return;
        }

    }

}

bool MainWindow::CommProjCapture(std::vector<QImage> &Qimagelist,int start,int end)
{
       camera_capture_thread.setCaptureflag(false);
       dlp::Time::Sleep::Microseconds(200000);

       scan_mat_list.clear();

       if(!camera_pg.isConnected()){
           dlp::CmdLine::Print("Error: Could NOT connect camera! \n");
           return false;
       }
       if(camera_pg.Start().hasErrors()){
           ui->textBrowser->append("Error: Could NOT start camera!");
           return false;
       }
       ui->textBrowser->append("open Capture ......");
       ui->textBrowser->append("The Qimagelist size : "+QString::number(Qimagelist.size()));
       QTime time1;

       camera_pg.ControlStrobe(2);

       dlp::Time::Sleep::Microseconds(50000);
       time1.start();

       float frameRate;
       camera_pg.GetFrameRate(&frameRate);
       ui->textBrowser->append("Camera frameRate : "+QString::number(frameRate));
       unsigned int period_ms = ceil(1000 / frameRate);

       dlp::Image captureImage;
       dlp::Capture capture;
       dlp::Capture::Sequence capture_sequence;

       for(int n = start;n < end; ++n)
       {          
           projector_labbel->setPixmap(QPixmap::fromImage(Qimagelist[n]));

           ui->textBrowser->append(QString::number(n));         
           if(n==start)delaymsec(100);
           camera_pg.ControlStrobe(1);
           delaymsec(period_ms+150);//45帧90,26帧150

           camera_pg.ControlStrobe(0);

           camera_pg.ControlStrobe(3);
           camera_pg.ControlStrobe(2);

           camera_pg.GetFrame(&captureImage);

           //captureImage.ConvertToMonochrome();
           capture.image_data = captureImage;
           capture.data_type  = dlp::Capture::DataType::IMAGE_DATA;

           capture_sequence.Add(capture);

           captureImage.Clear();
           capture.image_data.Clear();

       }

       ui->textBrowser->append("\nPattern sequence capture completed in ... "+QString::number(time1.restart())+" ms");

       if(camera_pg.Stop().hasErrors()){
           dlp::CmdLine::Print("Error : Camera failed to stop!");
           return false;
       }

       ui->textBrowser->append("Images retreived from buffer in ... "+QString::number(time1.restart())+" ms");

       for(unsigned int iScan = 0; iScan < capture_sequence.GetCount(); ++iScan){

           dlp::Capture temp;
           capture_sequence.Get(iScan,&temp);

           cv::Mat capture_mat;
           temp.image_data.GetOpenCVData(&capture_mat);
           if(capture_mat.channels()==3){
               cv::cvtColor(capture_mat,capture_mat,CV_RGB2BGR);
           }
           scan_mat_list.push_back(capture_mat);

       }
       QString path;
       for(int i= 0;i< scan_mat_list.size();i++)
       {
           if(i< 10)
           {
               path= measure_path + "/capture_0"+ QString::number(i)+ ".bmp";
           }else{
               path= measure_path + "/capture_"+ QString::number(i)+ ".bmp";
           }
           cv::imwrite(path.toStdString(),scan_mat_list[i]);
       }


       QImage whiteQimage;
       cv::Mat whiteMat = cv::Mat(imageHeight,imageWidth,CV_8UC3,cv::Scalar(255,255,255));
       cvMatToQImage(whiteMat,whiteQimage);
       projector_labbel->setPixmap(QPixmap::fromImage(whiteQimage));

       if(camera_pg.Stop().hasErrors()){
           ui->textBrowser->append("\nCamera failed to stop! Exiting scan routine...");
           return false;
       }
       return true;
}

void MainWindow::capture_calibrateImages()
{
    board_mat = show_mat.clone();

    if(board_mat.data){

        camera_capture_thread.setCaptureflag(false);

        dlp::Time::Sleep::Microseconds(200000);

        projector_4500.StopPatternSequence();

        dlp::Time::Sleep::Microseconds(100000);

        calibrate_machine.setBoardSize(ui->CornerWidthspinBox->value(),ui->CornerHeightspinBox->value());
        calibrate_machine.setCalibrateBoardImage(board_mat);
    }
}

void MainWindow::decode_multiWavelength(std::vector<cv::Mat> &inlist, std::vector<cv::Mat> &outlist, cv::Mat &mask)
{
    if(inlist.empty()){
        std::cout<<"Error: decode_multiWavelength : inlist is empty!\n";
        return;
    }
    std::vector<cv::Mat> images1,images2,images3,images4,images5;
    std::vector<cv::Mat> wrapPhase,wrapPhaseX,wrapPhaseY;

    for(unsigned int n = 0; n < inlist.size(); n += 22)
    {
        images1.clear();
        images2.clear();
        images3.clear();
        images4.clear();
        images5.clear();
        images1.insert(images1.begin(),inlist.begin() + n,inlist.begin() + n+6);
        images2.insert(images2.begin(),inlist.begin() + n+6,inlist.begin() + n+10);
        images3.insert(images3.begin(),inlist.begin() + n+10,inlist.begin() + n+14);
        images4.insert(images4.begin(),inlist.begin() + n+14,inlist.begin() + n+18);
        images5.insert(images5.begin(),inlist.begin() + n+18,inlist.begin() + n+22);

        cv::Mat wrapphase1 = cv::Mat::zeros(inlist[0].rows,inlist[0].cols,CV_64FC1);
        cv::Mat wrapphase2 = cv::Mat::zeros(inlist[0].rows,inlist[0].cols,CV_64FC1);
        cv::Mat wrapphase3 = cv::Mat::zeros(inlist[0].rows,inlist[0].cols,CV_64FC1);
        cv::Mat wrapphase4 = cv::Mat::zeros(inlist[0].rows,inlist[0].cols,CV_64FC1);
        cv::Mat wrapphase5 = cv::Mat::zeros(inlist[0].rows,inlist[0].cols,CV_64FC1);

        pattern_class->decode_dualStepPhaseShift(mask,images1,wrapphase1,3);
        pattern_class->decode_phaseShifting_LookUpTable(mask,images2,wrapphase2,4);
        pattern_class->decode_phaseShifting_LookUpTable(mask,images3,wrapphase3,4);
        pattern_class->decode_phaseShifting_LookUpTable(mask,images4,wrapphase4,4);
        pattern_class->decode_phaseShifting_LookUpTable(mask,images5,wrapphase5,4);

        wrapPhase.push_back(wrapphase1);
        wrapPhase.push_back(wrapphase2);
        wrapPhase.push_back(wrapphase3);
        wrapPhase.push_back(wrapphase4);
        wrapPhase.push_back(wrapphase5);

    }
//    for(unsigned int n = 0; n < 1; ++n){

//        cv::Mat temp = wrapPhaseX[n].clone();
//        cv::normalize(temp,temp,1.0,0.0,cv::NORM_MINMAX);
//        temp.convertTo(temp,CV_8UC1,255,0);
//        cv::imwrite((measure_path+"/"+QString::number(n)+"X.bmp").toStdString(),temp);

//    }

    std::ofstream out("out.txt");
    for(size_t i = 0; i < wrapPhase[1].cols; ++i)
    {
        out<<(double)wrapPhase[2].at<double>(400,i)<<"\r\n";
    }


    cv::Mat unWrapphaseX = cv::Mat::zeros(scan_mat_list[0].rows,scan_mat_list[0].cols,CV_64FC1);
    cv::Mat unWrapphaseY = cv::Mat::zeros(scan_mat_list[0].rows,scan_mat_list[0].cols,CV_64FC1);
    wrapPhaseX.insert(wrapPhaseX.begin(),wrapPhase.begin(),wrapPhase.begin()+5);
    pattern_class->decode_multiWavelength(wrapPhaseX,mask,unWrapphaseX,dlp::Pattern::Orientation::VERTICAL);

    if(wrapPhase.size()>5){

        wrapPhaseY.insert(wrapPhaseY.begin(),wrapPhase.begin()+5,wrapPhase.end());
        pattern_class->decode_multiWavelength(wrapPhaseY,mask,unWrapphaseY,dlp::Pattern::Orientation::HORIZONTAL);
    }

    cv::Mat wrapx;
    cv::normalize(unWrapphaseX,wrapx,1.0,0.0,cv::NORM_MINMAX);
    wrapx.convertTo(wrapx,CV_8UC1,255,0);
    cv::imwrite("../ScanData/unwrapX.bmp",wrapx);

//    cv::Mat wrapy;
//    cv::normalize(unwrapPhaseX[1],wrapy,1.0,0.0,cv::NORM_MINMAX);
//    wrapy.convertTo(wrapy,CV_8UC1,255,0);
//    cv::imwrite("../ScanData/unwrapY.bmp",wrapy);

    outlist.push_back(unWrapphaseX);
    outlist.push_back(unWrapphaseY);
}

void MainWindow::decode_multiFrequency(std::vector<cv::Mat> &inlist, std::vector<cv::Mat> &outlist, cv::Mat &mask)
{
    if(inlist.empty()){
        std::cout<<"Error: decode_multiWavelength : inlist is empty!\n";
        return;
    }

    int nr = inlist[0].rows;
    int nc = inlist[0].cols;

    std::vector<cv::Mat> images;
    std::vector<cv::Mat> wrapped;

    for(unsigned int n = 0; n < inlist.size(); n += 4 )
    {
        images.clear();
        images.insert(images.begin(),inlist.begin() + n,inlist.begin()+n+4);

        cv::Mat wrapphase = cv::Mat::zeros(nr,nc,CV_64FC1);

        pattern_class->decode_phaseShifting_LookUpTable(mask,images,wrapphase,4);

        wrapped.push_back(wrapphase);
    }

    cv::Mat unWrapphaseX = cv::Mat::zeros(nr,nc,CV_64FC1);
    pattern_class->decode_multiFrequencyHeterodyning(wrapped,mask,unWrapphaseX);

    outlist.push_back(unWrapphaseX);
}

void MainWindow::decode_color_dualThreeStep_dualFrequency(std::vector<cv::Mat> &inlist, std::vector<cv::Mat> &outlist, cv::Mat &mask)
{
    if(inlist.empty()){
        std::cout<<"Error: decode_color_dualThreeStep_dualFrequency : inlist is empty!\n";
        return;
    }

    int nr = inlist[0].rows;
    int nc = inlist[0].cols;

    std::vector<cv::Mat> wrapped;

    for(unsigned int n = 0; n < inlist.size(); n += 3)
    {
        std::vector<cv::Mat> images1,images2,images3;
        std::vector<cv::Mat> images;
        cv::split(inlist[n], images1);
        cv::split(inlist[n+1], images2);
        cv::split(inlist[n+2], images3);

        images.push_back(images1[0]);
        images.push_back(images2[0]);
        images.push_back(images3[0]);
        images.push_back(images1[2]);
        images.push_back(images2[2]);
        images.push_back(images3[2]);

        cv::Mat wrapphase = cv::Mat::zeros(nr,nc,CV_64FC1);

        pattern_class->decode_dualStepPhaseShift(mask,images,wrapphase,3);

        wrapped.push_back(wrapphase);
    }

    cv::Mat unWrapphaseX = cv::Mat::zeros(nr,nc,CV_64FC1);
    pattern_class->decode_dualFrequencyHeterodyning(wrapped,80,85,mask,unWrapphaseX);

    outlist.push_back(unWrapphaseX);
}

cv::Mat MainWindow::QImageToCvMat(QImage image)
{
    if(image.isNull()){
        ui->textBrowser->append("Error : QImageToCvMat ,the QImage is null!");
        return cv::Mat::zeros(1928,1448,CV_8UC1);
    }

    cv::Mat mat;
    switch(image.format())
    {
    case QImage::Format_ARGB32:
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32_Premultiplied:
        mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), image.bytesPerLine());
        break;
    case QImage::Format_RGB888:
        mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());
        cv::cvtColor(mat, mat, CV_RGB2BGR);
        break;
    case QImage::Format_Grayscale8:
        mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), image.bytesPerLine());
        break;
    }
    return mat;
}

void MainWindow::cvMatToQImage(cv::Mat &mat, QImage &qimage)
{
    if(mat.empty()){
        ui->textBrowser->append("Error : cvMatToQImage, the Mat is empty!");
        return;
    }
    switch(mat.channels())
    {
    case 1:
        qimage =QImage((const unsigned char*)(mat.data),
                          mat.cols,mat.rows,
                          mat.cols*mat.channels(),
                          QImage::Format_Grayscale8);
        break;
    case 3:
        qimage =QImage((const unsigned char*)(mat.data),
                          mat.cols,mat.rows,
                          mat.cols*mat.channels(),
                          QImage::Format_RGB888);
        break;
    }

}

std::string MainWindow::removeSymbol(std::string &str, const std::string &mark)
{
    std::string str_ = str;
    int index = 0;
    while( (index = str_.find(mark,index)) != std::string::npos)
    {
       str_.erase(index,1);
    }
    return str_;
}

void MainWindow::add_ply_cloud(PointCloud::Ptr pclPCloud, std::string filename)
{
    float resolution = pointCloudClass.computeResolution(pclPCloud);

    int num = pointCloudClass.StatisticalOutlierRemoval_Filter(cloud,cloud,30,1.0);

    ui->textBrowser->append("the point-cloud name : "+QString::fromStdString(filename));
    ui->textBrowser->append("Vertex : "+ QString::number(num) );
    ui->textBrowser->append("Resolution : "+ QString::number(resolution) );

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    pointCloudClass.removeCentroidPoint(cloud, centroid);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud,0,255,0);
    viewer->addPointCloud(cloud,color,filename);
    viewer->resetCamera();
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, filename);

    viewer->addCoordinateSystem(1.0);

    ui->qvtkWidget->update ();
}

void MainWindow::setCameraParameter()
{
    ui->resolutionComboBox->addItem(QWidget::tr("1928 x 1448"));
    ui->resolutionComboBox->addItem(QWidget::tr("800 x 600"));

    ui->exposureSlider->setMinimum(5);
    ui->exposureSlider->setMaximum(38);
    ui->exposureSlider->setValue(38);
    ui->exposureSlider->setSingleStep(1);

    ui->exposureSpinBox->setValue(38);
    ui->exposureSpinBox->setMinimum(5);
    ui->exposureSpinBox->setMaximum(38);

    QObject::connect(ui->resolutionComboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(setCamera_Resolution_FrameRate()));

    QObject::connect(ui->exposureSlider,SIGNAL(valueChanged(int)),ui->exposureSpinBox,SLOT(setValue(int)));
    QObject::connect(ui->exposureSpinBox,SIGNAL(valueChanged(int)),ui->exposureSlider,SLOT(setValue(int)));

    QObject::connect(ui->exposureSlider,SIGNAL(valueChanged(int)),this,SLOT(setCameraExposure()));

}
//非阻塞延时
void MainWindow::delaymsec(int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);

     while( QTime::currentTime() < dieTime )

         QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void MainWindow::closeEvent(QCloseEvent *event)
{

    delete projector_labbel;
}


void MainWindow::on_projectWhiteBtn_clicked()
{

    projectSignalPattern(&projector_4500,1);

}

void MainWindow::receive_cameraImage_slot(cv::Mat &img)
{
    if(img.empty()){
        std::cout<<"Error: receive_cameraImage_slot , the Mat is empty ... \n";
        return;
    }

    show_mat = img.clone();

    QImage image;

    cv::Mat temp(img.size(),CV_8UC3);

    if(img.channels()==1)
    {       
        cv::cvtColor(img,temp,CV_GRAY2RGB);

        for(unsigned int i = 0;i < temp.rows; ++i){

            uchar* imgPtr = img.ptr<uchar>(i);
            uchar* ptr = temp.ptr<uchar>(i);
            for(unsigned int j = 0; j < temp.cols; ++j){

                if(imgPtr[j] > 250){
                    ptr[j*3] = 255;
                    ptr[j*3+1] = 0;
                    ptr[j*3+2] = 0;
                }
            }
        }
    }else{
        temp = img.clone();

        for(unsigned int i = 0;i < temp.rows; ++i){

            uchar* ptr = temp.ptr<uchar>(i);
            for(unsigned int j = 0; j < temp.cols; ++j){

                if(ptr[j*3] > 250 || ptr[j*3+1] > 250 || ptr[j*3+2] > 250){
                    ptr[j*3] = 255;
                    ptr[j*3+1] = 0;
                    ptr[j*3+2] = 0;
                }
            }
        }
    }

    image =QImage((const unsigned char*)(temp.data),
                      temp.cols,temp.rows,
                      temp.cols*temp.channels(),
                      QImage::Format_RGB888);

    imageItem.setPixmap(QPixmap::fromImage(image));

}

void MainWindow::get_space_key_press()
{
    if(!camera_capture_thread.isRunning()){

        ui->textBrowser->append("Error: Camera is not running!");
        return;
    }
    board_mat = show_mat.clone();

    if(board_mat.data)
    {

        calibrate_machine.setCalibrateBoardImage(board_mat);
    }
}

void MainWindow::get_calibrate_status_slot(bool flag)
{
    if(flag){

        if(!patterns_prepare)
        {
            prepareScanPatterns(false);
        }

        switch(projectorMode)
        {
        case 1:
        {
            //相机、投影仪停止
            camera_capture_thread.setCaptureflag(false);

            dlp::Time::Sleep::Microseconds(200000);

            projector_4500.StopPatternSequence();

            dlp::Time::Sleep::Microseconds(100000);

            highSpeedSynchronousShoot(&camera_pg,&projector_4500,true,0,24,scan_mat_list);
        }
            break;
        case 0:
        {
            if(projectQimage.empty()){
                ui->textBrowser->append("Error : projectQimage is empty!");
                return;
            }

            CommProjCapture(projectQimage,0,projectQimage.size());

        }
            break;
        }

        std::cout<<"scan_mat_list.size(): "<<scan_mat_list.size()<<std::endl;

        QString showStr= "Save Calibrate Image number: "+ QString::number(saveCalibrateImageNum);

        std::cout<<showStr.toStdString()<<std::endl;

        QString path;

        if(saveCalibrateImageNum < 10)
        {
            path = board_image_path+ "/board_0"+ QString::number(saveCalibrateImageNum)+ ".bmp";
        }else{
            path = board_image_path+ "/board_"+ QString::number(saveCalibrateImageNum)+ ".bmp";
        }

        bool save = cv::imwrite(path.toStdString(),board_mat);

        qDebug()<<"save board: "<<path<<" "<<save;

        //创建标定文件夹
        QString temp_saveDir;
        if(saveCalibrateImageNum < 10)
        {
            temp_saveDir = calib_image_path+"/0"+QString::number(saveCalibrateImageNum)+ "/";
        }else{
            temp_saveDir = calib_image_path+"/"+QString::number(saveCalibrateImageNum)+ "/";
        }

        QDir temp_dir(temp_saveDir);
        if(!temp_dir.exists()){

            bool ok = temp_dir.mkdir(temp_saveDir);
            qDebug()<<"create dir: "<<temp_saveDir<<" "<<ok;
        }

        //当前采集图片存储目录
        currentSaveDir = temp_saveDir;

        path = currentSaveDir+ "/board.bmp";

        save = cv::imwrite(path.toStdString(),board_mat);
        qDebug()<<"save board: "<<path<<" "<<save;

        for(int i= 0;i< scan_mat_list.size();i++)
        {
            if(i< 10)
            {
                path= currentSaveDir+ "capture_0"+ QString::number(i)+ ".bmp";
            }else{
                path= currentSaveDir+ "capture_"+ QString::number(i)+ ".bmp";
            }

            cv::imwrite(path.toStdString(),scan_mat_list.at(i));

        }

        saveCalibrateImageNum++;

        qDebug()<<"saveCalibrateImageNum: " << saveCalibrateImageNum;

        projectSignalPattern(&projector_4500,1);

    }else{

        qDebug()<<"Error: Couldn't find the circle in the boardImage";
    }

}

void MainWindow::on_captureBtn_clicked()
{
    if(!camera_capture_thread.isRunning()){
        QMessageBox::warning(this,"Warning","Please press the projectWhite button first!");
        return ;
    }

    if(!patterns_prepare)
    {
        prepareScanPatterns(false);
    }

    switch(projectorMode)
    {
    case 1://可编程
    {
        camera_capture_thread.setCaptureflag(false);

        dlp::Time::Sleep::Microseconds(200000);

        projector_4500.StopPatternSequence();

        dlp::Time::Sleep::Microseconds(200000);

        int capture_count= 1;
        while(capture_count-- >0)
        {
            scan_mat_list.clear();

            highSpeedSynchronousShoot(&camera_pg,&projector_4500,true,0,24,scan_mat_list);

            qDebug()<<capture_count<<" scan_mat_list: "<<scan_mat_list.size();
        }
    }
        break;
    case 0://普通
    {
        if(project_image_list.empty()){
            ui->textBrowser->append("Error : project_image_list is empty!");
            return;
        }

        CommProjCapture(projectQimage,0,projectQimage.size());
    }
        break;
    }

}

void MainWindow::on_CalibrateBtn_clicked()
{

    if(camera_capture_thread.isRunning()){
        camera_capture_thread.setCaptureflag(false);
    }

    QString dir_name = QFileDialog::getExistingDirectory(
                                this,
                                "Select Calibration Folder",
                                "../ScanData");

    if(dir_name.isEmpty())
    {
        std::cout<<"Error: Can not find the calibration file ... "<<std::endl;
        return;
    }

    QDir dir(dir_name);
    QStringList dir_list;
    //dir.entryInfoList()此函数的返回值是一个QFileInfoList对象，包含当前目录下的所有文件和子目录。
    foreach (QFileInfo fileInfo, dir.entryInfoList(QDir::Dirs | QDir::Files))
    {
        QString strName = fileInfo.fileName();

        if ((strName == QString(".")) || (strName == QString("..")))
            continue;
        if (fileInfo.isDir())
        {

            QString str = fileInfo.absolutePath() + "/" + strName + "/";
            dir_list.push_back(str);
        }
    }

    calibrate_machine.clearCalibrateVector();

    calibrate_machine.setBoardSize(ui->CornerHeightspinBox->value(),ui->CornerWidthspinBox->value());
    calibrate_machine.setSquareSize(ui->CornerHeightlengthspinBox->value(),ui->CornerWidthlengthspinBox->value());

    std::vector<cv::Mat> unwrapphaselistX,unwrapphaselistY;

    qDebug()<<"Start to read and decode ... ";

    double periodper2PI = 20 / (2*CV_PI);

    int board_num = dir_list.size();

    for(int dir_i = 0; dir_i < dir_list.size(); dir_i++)
    {
        QDir dir(dir_list.at(dir_i));

        dir.setFilter( QDir::Files | QDir::NoSymLinks );//QDir::NoSymLinks - 不列出符号连接
                                                        //QDir::Files - 只列出文件
        dir.setSorting(QDir::Name);

        QStringList filter;
        filter<<QString("*.bmp");
        QFileInfoList  path =  dir.entryInfoList(filter);

        QStringList absoluteList;

        for (int i = 0; i < path.size(); i++) {

             QString filename = path.at(i).absoluteFilePath();

             absoluteList.push_back(filename);

         }

         QTime time1;
         time1.start();

         std::vector<cv::Mat> image_mat;

         for(int i= 0;i< absoluteList.size();i++)
         {
             cv::Mat srcImage=cv::imread(absoluteList.at(i).toStdString(),0);
             image_mat.push_back(srcImage);
         }
         std::cout<<"Group "<<QString::number(dir_i+1).toStdString()<<" : "<<std::endl;
         std::cout<<"image_mat size : "<<image_mat.size()<<std::endl;
         std::cout<<"read mat time: "<<time1.restart()<<" ms\n";

        //检测角点并设置像素坐标
         std::vector<cv::Point2f> cameraP;
        if(!calibrate_machine.get_cameraPoints(image_mat[0],cameraP))
        {
            qDebug()<<"Error: get_cameraPoints failed!";
            continue;
        }
        //求解绝对相位
        ////多波长
        std::vector<cv::Mat> wrapPhaseX;

        cv::Mat mask = cv::Mat(image_mat[0].size(),CV_8UC1,cv::Scalar(255)); 
        cv::threshold(image_mat[0],mask,50,255,cv::THRESH_BINARY | cv::THRESH_OTSU);

        std::vector<cv::Mat> images1,images2,images3,images4,images5;

        int imageNum = 0;
        if(singleCalibrateFlag){
            imageNum = image_mat.size()/2;
        }else{
            imageNum = image_mat.size();
        }

        for(unsigned int n = 1; n < imageNum; n += 22)
        {
            images1.clear();
            images2.clear();
            images3.clear();
            images4.clear();
            images5.clear();
            images1.insert(images1.begin(),image_mat.begin() + n,image_mat.begin() + n+6);
            images2.insert(images2.begin(),image_mat.begin() + n+6,image_mat.begin() + n+10);
            images3.insert(images3.begin(),image_mat.begin() + n+10,image_mat.begin() + n+14);
            images4.insert(images4.begin(),image_mat.begin() + n+14,image_mat.begin() + n+18);
            images5.insert(images5.begin(),image_mat.begin() + n+18,image_mat.begin() + n+22);

            cv::Mat wrapphase1 = cv::Mat::zeros(image_mat[0].rows,image_mat[0].cols,CV_64FC1);
            cv::Mat wrapphase2 = cv::Mat::zeros(image_mat[0].rows,image_mat[0].cols,CV_64FC1);
            cv::Mat wrapphase3 = cv::Mat::zeros(image_mat[0].rows,image_mat[0].cols,CV_64FC1);
            cv::Mat wrapphase4 = cv::Mat::zeros(image_mat[0].rows,image_mat[0].cols,CV_64FC1);
            cv::Mat wrapphase5 = cv::Mat::zeros(image_mat[0].rows,image_mat[0].cols,CV_64FC1);

            pattern_class->decode_dualStepPhaseShift(mask,images1,wrapphase1,3);
            pattern_class->decode_phaseShifting_LookUpTable(mask,images2,wrapphase2,4);
            pattern_class->decode_phaseShifting_LookUpTable(mask,images3,wrapphase3,4);
            pattern_class->decode_phaseShifting_LookUpTable(mask,images4,wrapphase4,4);
            pattern_class->decode_phaseShifting_LookUpTable(mask,images5,wrapphase5,4);

            wrapPhaseX.push_back(wrapphase1);
            wrapPhaseX.push_back(wrapphase2);
            wrapPhaseX.push_back(wrapphase3);
            wrapPhaseX.push_back(wrapphase4);
            wrapPhaseX.push_back(wrapphase5);

        }

        for( int n = 0 ; n < wrapPhaseX.size() ; n += 5 )
        {

            std::vector<cv::Mat> temp;
            temp.insert(temp.begin(),wrapPhaseX.begin()+n,wrapPhaseX.begin() + n +5);
            cv::Mat unWrapphase = cv::Mat::zeros(image_mat[0].rows,image_mat[0].cols,CV_64FC1);
            if(n==0){

                pattern_class->decode_multiWavelength(temp,mask,unWrapphase,dlp::Pattern::Orientation::VERTICAL);
                unwrapphaselistX.push_back(unWrapphase);
            }
            else{

                pattern_class->decode_multiWavelength(temp,mask,unWrapphase,dlp::Pattern::Orientation::HORIZONTAL);
                unwrapphaselistY.push_back(unWrapphase);
            }

            cv::Mat wrap = cv::Mat::zeros(unWrapphase.rows,unWrapphase.cols,CV_64FC1);
            cv::normalize(unWrapphase,wrap,1.0,0.0,cv::NORM_MINMAX);
            wrap.convertTo(wrap,CV_8UC1,255,0);

            if(n==0)
                cv::imwrite(dir_list.at(dir_i).toStdString()+"unwrapX.png",wrap);
            else
                cv::imwrite(dir_list.at(dir_i).toStdString()+"unwrapY.png",wrap);

        }

        //投影
        std::vector<cv::Point2f> dlpP;

        for(int m = 0; m < cameraP.size(); ++m){

            cv::Point2f point = cameraP[m];

            int xleft  = floor(point.x);
            int xright = ceil(point.x);
            int yUP    = floor(point.y);
            int yDOWN  = ceil(point.y);

            double xphaseleft  = unwrapphaselistX[dir_i].at<double>((int)point.y , xleft);
            double xphaseright = unwrapphaselistX[dir_i].at<double>((int)point.y , xright);
            double xphase      = xphaseleft + (xphaseright - xphaseleft)*(point.x - xleft)/(xright-xleft);

            double yphaseUP  = unwrapphaselistY[dir_i].at<double>(yUP  ,(int)point.x);
            double yphaseDOWN = unwrapphaselistY[dir_i].at<double>(yDOWN,(int)point.x);
            double yphase    = yphaseUP + (yphaseDOWN - yphaseUP)*(point.y-yUP)/(yDOWN-yUP);

            //            int x = point.x;
            //            int y = point.y;

            //            double xphase = phaseX[n].at<double>(y,x);
            //            double yphase = phaseY[n].at<double>(y,x);

            double dlp_x_value = xphase * periodper2PI;
            double dlp_y_value = yphase * periodper2PI;

            dlpP.push_back(cv::Point2f(dlp_x_value,dlp_y_value));
        }
        cv::Size size_ = cv::Size(ui->CornerHeightspinBox->value(),ui->CornerWidthspinBox->value());
        cv::drawChessboardCorners(image_mat[0],size_,dlpP,true);
        cv::line(image_mat[0],cv::Point(0,799),cv::Point(1279,799),cv::Scalar(255),5);
        cv::line(image_mat[0],cv::Point(1279,799),cv::Point(1279,0),cv::Scalar(255),5);
        cv::imwrite("../ScanData/board_image/projector_"+QString::number(dir_i).toStdString()+".bmp",image_mat[0]);
    }

    //标定
    double error = 0;
    if(singleCalibrateFlag){

        error = calibrate_machine.singleCamera_system_calibrate(unwrapphaselistX,board_num);
    }else{

        error = calibrate_machine.doubleCamera_system_calibrate(unwrapphaselistX,unwrapphaselistY,board_num);
    }

}

void MainWindow::on_reconstructionBtn_clicked()
{

    if(!camera_capture_thread.isRunning()){
        QMessageBox::warning(this,"Warning","Please press the projectWhite button first!");
        return ;
    }

    QTime t1;
    t1.start();

    if(camera_capture_thread.isRunning()){

        maskImage = show_mat.clone();

        cv::threshold(maskImage,maskImage,25,255,  cv::THRESH_BINARY);

       // maskImage = cv::Mat(show_mat.size(),CV_8UC1,cv::Scalar(255));

        cv::imwrite("../ScanData/mask.bmp",maskImage);

    }else{

        maskImage = cv::imread("../ScanData/mask.bmp",0);
    }

    if(!patterns_prepare)
    {
        prepareScanPatterns(false);
    }

    switch(projectorMode)
    {
    case 1:
    {
        camera_capture_thread.setCaptureflag(false);

        dlp::Time::Sleep::Microseconds(200000);

        projector_4500.StopPatternSequence();

        dlp::Time::Sleep::Microseconds(200000);

        int capture_count= 1;
        while(capture_count-- >0)
        {
            scan_mat_list.clear();

            highSpeedSynchronousShoot(&camera_pg,&projector_4500,true,0,12,scan_mat_list);

            qDebug()<<capture_count<<" scan_mat_list: "<<scan_mat_list.size();
        }
    }
        break;
    case 0:
    {
        if(projectQimage.empty()){
            ui->textBrowser->append("Error : projectQimage is empty!");
            return;
        }
        ////投影拍摄
        CommProjCapture(projectQimage,0,projectQimage.size()/2);

    }
        break;
    }

    //////仿真
#if 0
    scan_mat_list.insert(scan_mat_list.begin(),project_image_list.begin(),project_image_list.begin()+22);
    maskImage = cv::Mat(scan_mat_list[0].size(),CV_8UC1,cv::Scalar(255));
#endif

    ////解码
    std::vector<cv::Mat> unwrapPhaseX;
    //多波长
    decode_multiWavelength(scan_mat_list,unwrapPhaseX,maskImage);
    //多频
    //decode_multiFrequency(scan_mat_list,unwrapPhaseX,maskImage);
    //彩色双频双三步
    //decode_color_dualThreeStep_dualFrequency(scan_mat_list,unwrapPhaseX,maskImage);

    ////重建
    std::vector<cv::Point3f> pCloud;
    pCloud.reserve(3000000);

    //pattern_class->reconstruct3D_by_singleCameraSystem(unwrapPhaseX[0],pCloud,maskImage);

    cv::Mat unwrapPhaseY = unwrapPhaseX[0].clone();
    std::vector<cv::Point2f> cameraP,dlpP;
    pattern_class->createCorrectPoints(unwrapPhaseX[0],unwrapPhaseY,20,cameraP,dlpP,maskImage);
    pattern_class->reconstruct3D_by_DlpCameraSystem(cameraP,dlpP,pCloud);
    qDebug()<<"Points num : "<<pCloud.size();

    //save
    QString pointCloud_path= QFileDialog::getSaveFileName(this,"Save PointCloud Path","../ScanData/pCloud.ply","*.ply");

    pattern_class->savePointCloud(pCloud,pointCloud_path);

    show_cloudFileInfoList(cloudList);

    show_reconstructionResult(pCloud, cloud, pointCloud_path.toStdString());

    //meshlab
//    QString open_file = pointCloud_path;

//    QProcess *process = new QProcess;
//    QStringList str;
//    str << open_file;
//    process->start("F:/meshlab2016/MeshLab/meshlab.exe",str);

//    projectSignalPattern(&projector_4500,1);

}

void MainWindow::show_ply_cloud_file(QListWidgetItem* item)
{
    //QListWidgetItem* item = ui.listWidget->item ( i);
    QWidget* widget = ui->reconstruction_listWidget->itemWidget(item);
    QCheckBox* box = ( QCheckBox*) widget ;

    if(box->isChecked())
    {
        ui->showTab->setCurrentWidget(ui->cloudView);
        //QMessageBox::information(this,"Double Clicked",item->text());

        pcl::PLYReader reader;

        std::string cloudName = item->text().toStdString();

        std::string name_;
        //去除空格
        std::string mark = " ";
        size_t nSize = mark.size();
        while(1)
        {
            size_t pos = cloudName.find(mark);    //  尤其是这里
            if(pos == std::string::npos)
            {
                break;
            }
            cloudName.erase(pos, nSize);          
        }
        //去除符号
        std::string::size_type index = 0;
        for (index;index != cloudName.size();++index)
        {
            //std::ispunct(),判断是否为标点符号或者特殊符号
            if (!ispunct(cloudName[index]))
            {
                name_ += cloudName[index];
            }
        }

        std::string path = cloud_path.toStdString() + cloudName ;
        std::cout<< path <<std::endl;

        reader.read<pcl::PointXYZ>(path,*cloud);

        add_ply_cloud(cloud, name_);
    }
    else{

        delete_ply_cloud(item);
    }
}

void MainWindow::delete_ply_cloud(QListWidgetItem *item)
{
    std::string name = item->text().toStdString();
    name = removeSymbol(name," ");
    name = removeSymbol(name,".");
    name = removeSymbol(name,"_");
    std::cout<<name<<std::endl;
    viewer->removePointCloud(name);
    viewer->resetCamera();
    ui->qvtkWidget->update ();
}

void MainWindow::show_reconstructionResult(std::vector<cv::Point3f> &pCloud, PointCloud::Ptr pclPCloud,std::string filename)
{
    if(pCloud.empty())
        return;

    std::vector<cv::Point3f>::iterator iter = pCloud.begin();
    for(;iter != pCloud.end(); ++iter)
    {
        PointT point;
        point.x = (*iter).x;
        point.y = (*iter).y;
        point.z = (*iter).z;
        pclPCloud->points.push_back(point);
    }

    std::size_t found = filename.find_last_of("/\\");
    std::string plyName = filename.substr(found+1,filename.size());
    std::string name = removeSymbol(plyName,".");
    name = removeSymbol(name," ");
    name = removeSymbol(name,"_");

    add_ply_cloud(pclPCloud,name);
}
