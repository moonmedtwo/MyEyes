#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

//#define DATABASE_ROOT "~/Documents/pcl/user_database/"
//#define SCENE_ROOT	  "~/Documents/pcl/user_scene/"
#define DATABASE_ROOT "../../../pcl/user_database/"
#define SCENE_ROOT	  "../../../pcl/user_scene/"

#define GRABBER_TIMEOUT	     10000

/*
 * @brief: Tell user to calm down
 */
inline
void
pleaseWait_MsgBox(const std::string &s)
{
    QMessageBox msgBox;
    std::stringstream ss;
    ss << s << " is executing, please wait!";
    msgBox.setText(QString::fromStdString(ss.str()));
    msgBox.exec();
}

/*
 * @brief: update member variable and UI for grabber
 */
inline
void
MainWindow::updateGrabberState(const grabberState &gState)
{
   gState_ = gState;
   if(gState_ == grabberIsRunning)
   {
        static unsigned count = 0;
        static double last = pcl::getTime ();
        double now = pcl::getTime ();
        ++count;
        if (now - last >= 1.0)
        {
          cloud_fps_ = double(count)/double(now - last);
          count = 0;
          last = now;
          std::string s = std::to_string((int)cloud_fps_);
          ui->label_cloudRate->setText(QString::fromStdString(s));
        }
   }
   switch(gState_)
   {
         case grabberStopped:
         {
            grabberWDT->stop();
            ui->pushButton_restartGrabber->setText("Grabber stopped");
         }
            break;
         case grabberIsStarting:
         {
            grabberWDT->start(GRABBER_TIMEOUT);
            ui->pushButton_restartGrabber->setText("Grabber starting");
         }
             break;
         case grabberIsRunning:
         {
            grabberWDT->start(GRABBER_TIMEOUT);
            ui->pushButton_restartGrabber->setText("Grabber running");
         }
             break;

         case grabberHang:
         {
            grabberWDT->stop();
            delete grabber_;
            grabber_ = NULL;
            ui->pushButton_restartGrabber->setText("Grabber hang !!!");
         }
            break;
        default:
            break;
   }
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    grabber_(NULL),
    grabberWDT(NULL),
    pRecognitionThread_(NULL)
{
    ui->setupUi(this);
    initViewer();
}

MainWindow::~MainWindow()
{
    if(grabber_->isRunning())
        grabber_->stop();

    delete ui;
}

/*
 * callback from OpenNIGrabber when a new cloud is acquired
 */
void
MainWindow::cloudCallback(const CloudConstPtr &cloud)
{
    if(cloud_mtx_.tryLock(0))
    {
//        FPS_CALC(__FUNCTION__);
        cloud_pass_.reset(new Cloud);
        filterPassThrough(cloud,*cloud_pass_);
        new_cloud_ = true;
        cloud_mtx_.unlock();
        emit cloudChanged();
    }
}

void
MainWindow::kinect_init()
{
   if(grabber_)
   {
       delete grabber_;
       grabber_ = NULL;
   }
   if(grabberWDT)
   {
       delete grabberWDT;
       grabberWDT = NULL;
   }
   // Kinect image grabber
   grabberWDT = new QTimer;
   connect(grabberWDT,SIGNAL(timeout()),this,SLOT(grabberDie()));

   try
   {
        grabber_ = new pcl::OpenNIGrabber("#1");
   }
   catch(const std::exception &e)
   {
       std::cerr << e.what() << std::endl;
   }

   if(!grabber_)
   {
       std::cerr << "OpenNIGrabber failed" << std::endl;
       return;
   }

   boost::function<void (const CloudConstPtr&)> f\
           = boost::bind(&MainWindow::cloudCallback,this,_1);
   grabber_->registerCallback(f);
   w_grabberStart(grabber_);

   cloud_pass_.reset(new Cloud);

   connect(this,SIGNAL(cloudChanged()),this,SLOT(updateViewer()),Qt::QueuedConnection);

   ui->qvtkWidget_cloudViewer->update();
}

/*
 * @brief: part of MainWindow constructor
 */
void
MainWindow::initViewer()
{
   // Create new pointer with no interacter as false in the 2nd arg
   viewer_.reset(new pcl::visualization::PCLVisualizer("PCLViewer",false));
   ui->qvtkWidget_cloudViewer->SetRenderWindow(viewer_->getRenderWindow());
   viewer_->setupInteractor(ui->qvtkWidget_cloudViewer->GetInteractor(),\
                              ui->qvtkWidget_cloudViewer->GetRenderWindow());
}

/*
 * SLOT
 * @brief: slot to update QVtk
 */
void
MainWindow::updateViewer()
{
    //    FPS_CALC("	updateViewer");
    if(!cloud_pass_)
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        return;
    }
    if(new_cloud_)
    {
        if(gState_ != grabberStopped)
            updateGrabberState(grabberIsRunning);
        // Protect pointcloud from being changed in cloudCallback
        cloud_mtx_.lock();
        if(!viewer_->updatePointCloud(cloud_pass_,"cloudpass"))
        {
            viewer_->addPointCloud(cloud_pass_,"cloudpass");
            viewer_->resetCameraViewpoint("cloudpass");
            viewer_->addCoordinateSystem(1,"XYZcoordinates",0);
        }
        new_cloud_ = false;
        cloud_mtx_.unlock();
        ui->qvtkWidget_cloudViewer->update();
    }

}

/*
 * SLOT
 * @brief: check grabber hang
 */
void
MainWindow::grabberDie()
{
    updateGrabberState(grabberHang);
}

/*
 * @brief: start/stop acquiring frame from kinect
 */
void
MainWindow::on_pushButton_restartGrabber_clicked()
{
   if(!grabber_) kinect_init();
   else
   {
        if(grabber_->isRunning())
        {
            w_grabberStop(grabber_);
        }
        else
        {
            w_grabberStart(grabber_);
        }
   }
}

/*
 * @brief: wrapper for OpenNI grabber
 */
void
MainWindow::w_grabberStart(pcl::Grabber *grabber)
{
    if(!grabber)
    {
        std::cout << "grabber is null" << std::endl;
        return;
    }
    try {
        grabber->start();
        updateGrabberState(grabberIsStarting);
    } catch(const std::exception &e){
        std::cerr << e.what() << std::endl;
    }
}

/*
 * @brief: wrapper for OpenNI grabber
 */
void
MainWindow::w_grabberStop(pcl::Grabber *grabber)
{
    if(!grabber)
    {
        std::cout << "grabber is null" << std::endl;
        return;
    }
    try {
        grabber->stop();
        updateGrabberState(grabberStopped);
    } catch(const std::exception &e){
        std::cerr << e.what() << std::endl;
    }
}

/*
 * @brief: create a thread to perform object recognition
 */
void MainWindow::on_pushButton_recognize_clicked()
{
    // the last thread has finished and pointer is reset
    if(NULL == pRecognitionThread_)
    {
        CloudPtr model(new Cloud);
        CloudPtr scene(new Cloud);
        {
            pcl::PCDReader reader;
            std::string model_r(DATABASE_ROOT);
            std::string scene_r(SCENE_ROOT);
            std::string path = model_r + "sapporo_blue/sapporo_blue_db_0.pcd";
            reader.read(path,*model);
            path = scene_r + "home/home_0_2.pcd"; /*"sapporo_blue_scene/sapporo_blue_s_0_0.pcd"; */
            reader.read(path,*scene);
        }

        cloud_mtx_.lock();
        pRecognitionThread_ = new UserRecognizer_Thread(scene,model);
        connect(pRecognitionThread_,&UserRecognizer_Thread::finished, pRecognitionThread_, &QObject::deleteLater);
        connect(pRecognitionThread_,&UserRecognizer_Thread::destroyed, this, &MainWindow::RecognitionThreadDestroyed);
        pRecognitionThread_->start();
        cloud_mtx_.unlock();
    }
    else pleaseWait_MsgBox(__FUNCTION__);
}

