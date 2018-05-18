#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QSerialPortInfo>
#include <string>

#define REG_OBJECT_ID "regObj_"

#define GRABBER_TIMEOUT	     10000
typedef typename pcl::visualization::PointCloudColorHandlerCustom<PointType> ColorHandler;

/*****************************************
 * UTIL FUNTIONS *************************
 *****************************************/

/*
 * @brief: filter cloud field ("x" or "y" or "z")
 */
void
MainWindow::filterPassThrough(const CloudConstPtr &cloud, Cloud &result)
{
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, filter_z);
    //pass.setFilterLimits (0.0, 1.5);
    //pass.setFilterLimits (0.0, 0.6);
    pass.setKeepOrganized (false);
    pass.setInputCloud (cloud);
    pass.filter (result);
}
/*
 * @brief: down sample the cloud, using voxel grid
 */
void
MainWindow::gridSample(const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  pcl::VoxelGrid<PointType> grid;
  //pcl::ApproximateVoxelGrid<PointType> grid;
  grid.setLeafSize (float (leaf_size), float (leaf_size), float (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
  //result = *cloud;
}
/*
 * @brief: init a table of different colors
 */
void
initColorHash(std::vector<std::vector<double> > &colorHash)
{
    std::cout << __FUNCTION__ << std::endl;
    int step = 255;
    {
        int i = 0;
        while(i * step <= 255)
        {
            int j = 0;
            while(j * step <= 255)
            {
                int k = 0;
                while(k * step <= 255)
                {
//                    std::cout << i << " " << j << " " << k << std::endl;
                    std::vector<double> a;
                    a.push_back(i*step); a.push_back(j*step); a.push_back(k*step);
//                    std::cout << "------ "
//                              << a[0] << " "
//                              << a[1] << " "
//                              << a[2]
//                              << std::endl;
                    colorHash.push_back(a);
                    k++;
                }
                j++;
            }
            i++;
        }
    }
    std::cout << "Generated " << colorHash.size()
              << " colors with step = " << step << std::endl;
}
/*****************************************
 * UTIL FUNTIONS *************************
 *****************************************/
void
MainWindow::errorHandler(const QString &error, int type)
{
    switch (type) {
    case QMessageBox::Critical:
        QMessageBox::critical(this, tr("Critical Error"),error);
        break;
    case QMessageBox::Warning:
        QMessageBox::warning(this, tr("Warning"),error);
        break;
    case QMessageBox::Information:
        QMessageBox::information(this, tr("FYI"),error);
        break;
    default:
        QMessageBox::question(this, tr("Not sure what you meant"),error);
        break;
    }

}
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
    cloud_pass_(new Cloud),
    model_(new Cloud),
    model_dir_(DEFAULT_MODEL_DIR),
    pRecognitionThread_(NULL),
    filter_z(10)
{
    PRINT_CURRENT_TIME(__FUNCTION__);
    ui->setupUi(this);

    initViewer();
    initSerialPortSettings();
}

MainWindow::~MainWindow()
{
    if(grabber_->isRunning())
        grabber_->stop();

    delete ui;
}
/*
 * @brief: callback from OpenNIGrabber when a new cloud is acquired
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
/*
 * @brief: manually reset recognition thread pointer
 * @brief: since Qt does not do this
 */
void
MainWindow::recognitionThreadDestroyed()
{
   std::cout << __FUNCTION__ << std::endl;
   pRecognitionThread_ = NULL;
}
/*
 * @brief: initialize grabber interface for kinect
 */
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
void
MainWindow::initSerialPortSettings()
{

    // Add availalbe port
    const auto infos = QSerialPortInfo::availablePorts();
    for(const QSerialPortInfo &info : infos)
    {
        ui->Comm_comboBox->addItem(info.portName());
    }
    // Vital connections
    connect(&comm,&UserComm::dataReceived,
            this,&MainWindow::serialPort_packetReceived);
    connect(&comm,&UserComm::connected,
            this, &MainWindow::serialPort_connected);
    connect(&comm,&UserComm::disconnected,
            this, &MainWindow::serialPort_disconnected);
    connect(&comm,&UserComm::errorSig,
            this, &MainWindow::serialPort_handleError);

    connect(this,&MainWindow::openSerialPort,
            &comm,&UserComm::openSerialPort);
    connect(this,&MainWindow::closeSerialPort,
            &comm,&UserComm::closeSerialPort);
    connect(this,&MainWindow::serialPort_write,
            &comm, &UserComm::writeData);
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
//            viewer_->addCoordinateSystem(1,"XYZcoordinates",0);
        }
        new_cloud_ = false;
        cloud_mtx_.unlock();
        ui->qvtkWidget_cloudViewer->update();
    }

}
/*
 * @brief: remove list of cloud from visualizer
 * @in[1]: vector of cloud pointers
 * @in[2]: id of the cloud set
 */
void
MainWindow::removeCloudFromViz(std::vector<CloudPtr> &clouds,
                                         const std::string &id)
{
   if(clouds.size())
   {
        for(unsigned i = 0; i < clouds.size();i++)
        {
            std::stringstream ss;
            ss << id << i;
            viewer_->removePointCloud(ss.str());
        }
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
 * SLOT
 * @brief: collect result from recognition thread
 */
void
MainWindow::recognitionResult(std::vector<CloudPtr> results)
{
   if(results.size())
   {
       recognizedObjects_.clear();
       for(unsigned i = 0; i < results.size(); i++)
       {
           recognizedObjects_.push_back(results[i]);
       }
       std::cout << "Collected: " << recognizedObjects_.size() << std::endl;

       if(recognizedObjects_.size())
       {
           std::vector<std::vector<double> > colorHash;
           initColorHash(colorHash);
           for(unsigned i = 0; i < recognizedObjects_.size();i++)
           {
               std::stringstream ss;
               std::string id(REG_OBJECT_ID);
               ss << id << i;
               int colorIdx = colorHash.size() - i % colorHash.size() - 1;
               ColorHandler cHandler(recognizedObjects_[i],
                                     colorHash[colorIdx][0],
                                     colorHash[colorIdx][1],
                                     colorHash[colorIdx][2]);

               // Combobox for user interaction
               ui->comboBox_recognizedObjects->addItem(QString::fromStdString(ss.str()));
               ui->comboBox_recognizedObjects->setItemData(i,
                                                           QColor(colorHash[colorIdx][0],
                                                                  colorHash[colorIdx][1],
                                                                  colorHash[colorIdx][2]),
                                                           Qt::BackgroundRole);

               if(!viewer_->updatePointCloud(recognizedObjects_[i],cHandler,ss.str()))
               {
                   viewer_->addPointCloud(recognizedObjects_[i],cHandler,ss.str(),0);
               }
           }
           ui->qvtkWidget_cloudViewer->update();
       }
   }
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
void
MainWindow::on_pushButton_recognize_clicked()
{
    // the last thread has finished and pointer is reset
    if(NULL == pRecognitionThread_)
    {
        // Clear last results
        ui->comboBox_recognizedObjects->clear();
        removeCloudFromViz(recognizedObjects_,REG_OBJECT_ID);
        recognizedObjects_.clear();
        UserRecognizer_Window myWindow(this, model_dir_);
        myWindow.exec();
        if(myWindow.result() != QDialog::Accepted)
        {
            return;
        }
        // Update model_dir_
        model_dir_ = myWindow.get_ModelDir();
        std::vector<std::string> cloud_lists = myWindow.get_cloud_list();
        if(cloud_lists.size() == 0)
        {
            MainWindow::errorHandler("No models found",QMessageBox::Information);
            return;
        }

        CloudPtr scene(new Cloud() );
#if 1
        cloud_mtx_.lock();
        scene.swap(cloud_pass_);
        cloud_pass_.reset(new Cloud);
        cloud_mtx_.unlock();
#else
        pcl::PCDReader reader;
        std::string scene_r(SCENE_ROOT);
        scene_r = scene_r +  "sapporo_blue_scene/sapporo_blue_s_1_1.pcd";/*"home/home_0_2.pcd";*/;
        reader.read(scene_r,*scene);
#endif
        // Check input
        if(scene->points.size() <= 0)
        {
            std::stringstream ss;
            ss << "invalid input" << std::endl;
            ss << "--- scene: " << scene->points.size() << std::endl;
            MainWindow::errorHandler(QString::fromStdString(ss.str()),
                                     QMessageBox::Critical);
            return;
        }

        pRecognitionThread_ = new UserRecognizer_Thread(scene,cloud_lists);

        // Make it auto delete
        connect(pRecognitionThread_,&UserRecognizer_Thread::finished,
                pRecognitionThread_, &QObject::deleteLater);

        // Notify user
        connect(pRecognitionThread_,&UserRecognizer_Thread::destroyed,
                this, &MainWindow::recognitionThreadDestroyed);

        // Notify user
        connect(pRecognitionThread_,&UserRecognizer_Thread::errorHandler,
                this, &MainWindow::errorHandler);

        // Result transmission
        qRegisterMetaType<std::vector<CloudPtr> >("std::vector<CloudPtr");
        connect(pRecognitionThread_,SIGNAL(resultReady(std::vector<CloudPtr>)),
                this,SLOT(recognitionResult(std::vector<CloudPtr>)));

        pRecognitionThread_->start();
    }
    else pleaseWait_MsgBox("pRecognitionThread_");
}

void
MainWindow::on_pushButton_visData_clicked()
{
    if(NULL != pRecognitionThread_)
    {
        pleaseWait_MsgBox("pRecognitionThread_");
    }
    if(recognizedObjects_.size())
    {
        std::vector<std::vector<double> > colorHash;
        initColorHash(colorHash);
        for(unsigned i = 0; i < recognizedObjects_.size();i++)
        {
            std::stringstream ss;
            std::string id(REG_OBJECT_ID);
            ss << id << i;
            int colorIdx = colorHash.size() - i % colorHash.size() - 1;
            ColorHandler cHandler(recognizedObjects_[i],
                                  colorHash[colorIdx][0],
                                  colorHash[colorIdx][1],
                                  colorHash[colorIdx][2]);

            // Combobox for user interaction
            ui->comboBox_recognizedObjects->addItem(QString::fromStdString(ss.str()));
            ui->comboBox_recognizedObjects->setItemData(i,
                                                        QColor(colorHash[colorIdx][0],
                                                               colorHash[colorIdx][1],
                                                               colorHash[colorIdx][2]),
                                                        Qt::BackgroundRole);

            if(!viewer_->updatePointCloud(recognizedObjects_[i],cHandler,ss.str()))
            {
                viewer_->addPointCloud(recognizedObjects_[i],cHandler,ss.str(),0);
            }
        }
        ui->qvtkWidget_cloudViewer->update();
    }
}

void
MainWindow::on_Comm_comboBox_currentTextChanged(const QString &arg1)
{
   comm.setPortname(arg1);
   std::cout << "portname: " << comm.getPortname().toStdString() << std::endl;
}

void
MainWindow::on_pushButton_OpenComm_clicked()
{
   if(!comm.isOpen())
       emit(openSerialPort());
   else
       emit(closeSerialPort());

   // Disable till port is opened or closed
//   ui->pushButton_OpenComm->setEnabled(false);
}
/*
 * SLOT
 * @brief: notify connection
 * @brief: this must be connected with UserComm signal
 */
void
MainWindow::serialPort_connected()
{
    ui->pushButton_OpenComm->setText("Disconnect");
//    ui->pushButton_OpenComm->setEnabled(true);
    ui->label_CommStatus->setText(tr("Connected @ %1, %2")
                                  .arg(comm.getPortname())
                                  .arg(comm.getBaudrate()));
}
/*
 * SLOT
 * @brief: notify connection
 * @brief: this must be connected with UserComm signal
 */
void
MainWindow::serialPort_disconnected()
{
    ui->pushButton_OpenComm->setText("Connect");
//    ui->pushButton_OpenComm->setEnabled(true);
    ui->label_CommStatus->setText("No connection");
}
/*
 * SLOT
 * @brief: get the good packet
 * @brief: this must be connected with UserComm signal
 */
void
MainWindow::serialPort_packetReceived(const QByteArray &data)
{
   std::cout << __FUNCTION__ << std::endl;
   // TODO process data here
}
/*
 * SLOT
 * @brief: notify error
 * @brief: this must be connected with UserComm signal
 */
void
MainWindow::serialPort_handleError(const QString &error)
{
   MainWindow::errorHandler(error,QMessageBox::Warning);
}

void MainWindow::on_pushButton_cmdGrab_clicked()
{
    QByteArray cmdGrab = QByteArray("\xbb\x03\x00\x00\x00\x00",6);
    emit(serialPort_write(cmdGrab));
}

void MainWindow::on_horizontalSlider_Filter_z_valueChanged(int value)
{
    filter_z = double(value/100.0);
    QString str_z = QString::number(filter_z,'f',3);
    ui->label_Filter_z->setText(str_z);
}
