#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QSerialPortInfo>
#include <string>

#include "util.h"

#define REG_OBJECT_ID "regObj_"

#define GRABBER_TIMEOUT	    				10000
#define SERIALPORT_ACK_TIMEOUT	 			5000
#define SERIALPORT_ACK_RETRIES 				3
#define SERIALPORT_INTERVAL_BETWEEN_RETRY   (SERIALPORT_TIMEOUT/SERIALPORT_RETRIES) // 50 = 2.5 * STM_UART_CHECK_INTERVAL
#define SERIALPORT_ROUTINE_TIMEOUT			5000
#define SERIALPORT_ROUTINE_RETRIES			(SERIALPORT_ROUTINE_TIMEOUT/UART_ROUTINE_INTERVAL_MS)
typedef typename pcl::visualization::PointCloudColorHandlerCustom<PointType> ColorHandler;

/*****************************************
 * UTIL FUNTIONS *************************
 *****************************************/

/*
 * @brief: round to n decimal places
 */
double
user_rounding(double var, unsigned long decimal)
{
   int multiplier = 1;
   while (decimal--)
   {
       multiplier *= 10;
   }
   double value = (int)(var * multiplier + 0.5);
   return (double)value/multiplier;
}

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

    commWDT(NULL),
    filter_z(10),
    trackObject_selectedIdx_(-1)
{
    PRINT_CURRENT_TIME(__FUNCTION__);
    ui->setupUi(this);

    viewer_init();
    serialPort_init();

    userSysticks.start();
}

MainWindow::~MainWindow()
{
    if(grabber_->isRunning())
        grabber_->stop();

    delete grabber_;
    delete grabberWDT;
    delete commWDT;

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
        if(userTracker.hasTarget())
        {
            userTracker.trackingRoutine(cloud_pass_);
        }
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

//   // Kinect image grabber
//   if(!grabberWDT)
//    grabberWDT = new QTimer;
//   connect(grabberWDT,SIGNAL(timeout()),this,SLOT(grabberDie()));

   // First time init
   if(!grabberWDT)
   {
       grabberWDT = new QTimer;
       connect(grabberWDT,SIGNAL(timeout()),this,SLOT(grabberDie()));

       connect(ui->text_roto00,SIGNAL(textChanged()),this,SLOT(textEditChanged()));
       connect(ui->text_roto01,SIGNAL(textChanged()),this,SLOT(textEditChanged()));
       connect(ui->text_roto02,SIGNAL(textChanged()),this,SLOT(textEditChanged()));
       connect(ui->text_roto03,SIGNAL(textChanged()),this,SLOT(textEditChanged()));

       connect(ui->text_roto10,SIGNAL(textChanged()),this,SLOT(textEditChanged()));
       connect(ui->text_roto11,SIGNAL(textChanged()),this,SLOT(textEditChanged()));
       connect(ui->text_roto12,SIGNAL(textChanged()),this,SLOT(textEditChanged()));
       connect(ui->text_roto13,SIGNAL(textChanged()),this,SLOT(textEditChanged()));

       connect(ui->text_roto20,SIGNAL(textChanged()),this,SLOT(textEditChanged()));
       connect(ui->text_roto21,SIGNAL(textChanged()),this,SLOT(textEditChanged()));
       connect(ui->text_roto22,SIGNAL(textChanged()),this,SLOT(textEditChanged()));
       connect(ui->text_roto23,SIGNAL(textChanged()),this,SLOT(textEditChanged()));

       connect(ui->text_roto30,SIGNAL(textChanged()),this,SLOT(textEditChanged()));
       connect(ui->text_roto31,SIGNAL(textChanged()),this,SLOT(textEditChanged()));
       connect(ui->text_roto32,SIGNAL(textChanged()),this,SLOT(textEditChanged()));
       connect(ui->text_roto33,SIGNAL(textChanged()),this,SLOT(textEditChanged()));

       ui->text_roto00->setText("0");
       ui->text_roto01->setText("0");
       ui->text_roto02->setText("-1");
       ui->text_roto03->setText("0");

       ui->text_roto10->setText("1");
       ui->text_roto11->setText("0");
       ui->text_roto12->setText("0");
       ui->text_roto13->setText("0");

       ui->text_roto20->setText("0");
       ui->text_roto21->setText("-1");
       ui->text_roto22->setText("0");
       ui->text_roto23->setText("0.055");

       ui->text_roto30->setText("0");
       ui->text_roto31->setText("0");
       ui->text_roto32->setText("0");
       ui->text_roto33->setText("1");
   }

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
MainWindow::viewer_init()
{
   // Create new pointer with no interacter as false in the 2nd arg
   viewer_.reset(new pcl::visualization::PCLVisualizer("PCLViewer",false));
   ui->qvtkWidget_cloudViewer->SetRenderWindow(viewer_->getRenderWindow());
   viewer_->setupInteractor(ui->qvtkWidget_cloudViewer->GetInteractor(),\
                              ui->qvtkWidget_cloudViewer->GetRenderWindow());
}
void
MainWindow::serialPort_init()
{

    // Add availalbe port
    const auto infos = QSerialPortInfo::availablePorts();
    for(const QSerialPortInfo &info : infos)
    {
        ui->Comm_comboBox->addItem(info.portName());
    }
    // Initilize packet status
    for(int i = 0; i < UserComm::NUMB_OF_CMD; i++)
        packetStatus_[i] = UserComm::idle;

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

    if(!commWDT)
        commWDT = new QTimer;

    connect(commWDT,SIGNAL(timeout()),this,SLOT(serialPort_routine()));

    // Manage connection live
    packetRetries_[static_cast<int>(UserComm::CMD_ENDPOINT)] = SERIALPORT_ROUTINE_RETRIES;
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
            viewer_->resetCamera();
//            viewer_->addCoordinateSystem(1,"XYZcoordinates",0);
        }
        new_cloud_ = false;
        cloud_mtx_.unlock();

        if(userTracker.hasTarget())
        {
            drawResult();
        }
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
 * @brief: update progress of recognition thread
 */
void
MainWindow::recognitionProgress(int percent, int model, int totalmodel)
{
    ui->progressBar_recognitionThread->setValue(percent);
    std::stringstream ss;
    if(percent != 100)
    {
        ss << "Processing model " << model + 1 << " out of " << totalmodel;
    }
    else
    {
        ss << "Found object with model " << model + 1;
    }
    ui->label_progressRecognition->setText(QString::fromStdString(ss.str()));
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
        // Stop tracking
        userTracker.stopTracking();
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

        // Notify thread progress
        connect(pRecognitionThread_,&UserRecognizer_Thread::destroyed,
                this, &MainWindow::recognitionThreadDestroyed);
        connect(pRecognitionThread_,&UserRecognizer_Thread::progress,
                this, &MainWindow::recognitionProgress);

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
MainWindow::on_Comm_comboBox_currentTextChanged(const QString &arg1)
{
   comm.setPortname(arg1);
   std::cout << "portname: " << comm.getPortname().toStdString() << std::endl;
}

void
MainWindow::on_pushButton_OpenComm_clicked()
{
   if(!QString::compare(ui->pushButton_OpenComm->text(),"Connect"))
       emit(openSerialPort());
   else
       emit(closeSerialPort());

   // Disable till port is opened or closed
//   ui->pushButton_OpenComm->setEnabled(false);
}
/*
 * SLOT
 * @brief: serial port routine check
 * @brief: this must be connected with commWDT timeout
 */
void
MainWindow::serialPort_routine()
{
    for(int i = 0; i < UserComm::NUMB_OF_CMD;i++)
    {
        if(packetStatus_[i] == UserComm::waitingACK)
        {
            if(packetRetries_[i] &&
               (packetTimeout_[i] < SERIALPORT_ACK_TIMEOUT))
            {
                uint8_t len = 0;
                UserComm::PROTOCOL_CMD packetType =
                        static_cast<UserComm::PROTOCOL_CMD>(i);
                uint8_t *pBuf = serialPort_wrapPacket(packetType, len);
                if(!pBuf) return;
                emit(serialPort_write(pBuf,len));

                packetRetries_[i]--;
                packetTimeout_[i] += UART_ROUTINE_INTERVAL_MS;
            }
        }
    }

    uint8_t &connRetries =  packetRetries_[static_cast<int>(UserComm::CMD_ENDPOINT)];
    connRetries--;
    if(!connRetries)
    {
        serialPort_disconnected();
    }
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
    commWDT->start(UART_ROUTINE_INTERVAL_MS);
    QPalette pal = this->palette();
    pal.setColor(QPalette::Window,Qt::cyan);
    this->setPalette(pal);
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
    commWDT->stop();
    QPalette pal = this->palette();
    pal.setColor(QPalette::Window,Qt::white);
    this->setPalette(pal);
}
/*
 * SLOT
 * @brief: get the good packet
 * @brief: this must be connected with UserComm signal
 */
void
MainWindow::serialPort_packetReceived(const QByteArray &data)
{
    std::cout << __FUNCTION__ << ": ";
    uint8_t dataLen = data[1];
    uint8_t buf[dataLen];
    for(int i = 0; i < dataLen; i++)
    {
        buf[i] = data[DATASTART_OFFSET + i] & 0xFF;
        printf("[%02x]",buf[i]);
    }
    std::cout << std::endl;
    UserComm::PROTOCOL_CMD cmd =
            static_cast<UserComm::PROTOCOL_CMD>(buf[0]);
    switch (cmd)
    {
    case UserComm::CMD_GRAB:
    case UserComm::CMD_DROP:
    case UserComm::CMD_MOVETOXYZ:
    case UserComm::CMD_STOP:
        if(buf[1] == ACK)
           serialPort_update_PacketStatus(UserComm::idle,cmd);
        else
            while(1);
        break;
    case UserComm::CMD_ENDPOINT:
        packetRetries_[static_cast<int>(UserComm::CMD_ENDPOINT)] = SERIALPORT_ACK_RETRIES;
        if(dataLen != CMD_ENDPOINT_LENGTH)
            return;

        updateEndpoint(Util_parseFloat(&buf[1]),
                       Util_parseFloat(&buf[5]),
                       Util_parseFloat(&buf[9]));

        // Manage connection live
        packetRetries_[static_cast<int>(UserComm::CMD_ENDPOINT)] = SERIALPORT_ROUTINE_RETRIES;
        serialPort_connected();

        break;
    default:
        break;
    }
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

/*
 * @brief: wrap packet
 */
uint8_t *
MainWindow::serialPort_wrapPacket(UserComm::PROTOCOL_CMD packetType, uint8_t &len)
{
    switch (packetType)
    {
    case UserComm::CMD_GRAB:
    case UserComm::CMD_DROP:
    case UserComm::CMD_STOP:
        len = 1;
        break;
    case UserComm::CMD_MOVETOXYZ:
        len = 13;
        break;
    default:
        len = 0;
        break;
    }

    if(len == 0)
        return NULL;

    uint8_t *pBuf = (uint8_t*)(malloc(256)); // Will be free in UserComm::writeData
    if(!pBuf)
        return NULL;

    uint8_t cmd = static_cast<uint8_t>(packetType);
    pBuf[0] = cmd;

    if(packetType == UserComm::CMD_MOVETOXYZ)
    {
       Util_bufferFloat(pBuf+1,setpoint_x_);
       Util_bufferFloat(pBuf+5,setpoint_y_);
       Util_bufferFloat(pBuf+9,setpoint_z_);
    }

    return pBuf;
}
 /* * @brief: send packet according to command
 */
void
MainWindow:: serialPort_sendPacket(UserComm::PROTOCOL_CMD packetType)
{
    uint8_t len = 0;
    uint8_t *pBuf = serialPort_wrapPacket(packetType, len);
    if(!pBuf) return;
    emit(serialPort_write(pBuf,len));
    serialPort_update_PacketStatus(UserComm::waitingACK,packetType);
}
/*
 * @brief: switch packet status to manage communication process
 */
void
MainWindow::serialPort_update_PacketStatus(UserComm::packetStatus toStatus,
                                          UserComm::PROTOCOL_CMD packetType)
{
    if(packetType >= UserComm::NUMB_OF_CMD)
        return;

    uint8_t idx = static_cast<uint32_t>(packetType);

    packetStatus_[idx] = toStatus;
    if(toStatus == UserComm::waitingACK)
    {
        // Reset timeout for retry mechanism
        packetTimeout_[idx] = 0;
        packetRetries_[idx] = SERIALPORT_ACK_RETRIES;
    }
    printf("status changed [%02x]\r\n", idx);
    for(int i = 0; i < UserComm::NUMB_OF_CMD; i++)
        printf("[%02x] ",static_cast<uint8_t>(packetStatus_[i]));
    std::cout << std::endl;
}
/*
 * @brief: get current packet status to manage communication process
 */
UserComm::packetStatus
MainWindow::serialPort_get_PacketStatus(UserComm::PROTOCOL_CMD packetType)
{
    if(packetType >= UserComm::NUMB_OF_CMD)
        return UserComm::outOfRange;

    uint32_t idx = static_cast<uint32_t>(packetType);
    return packetStatus_[idx];
}

void
MainWindow::on_horizontalSlider_Filter_z_valueChanged(int value)
{
    filter_z = double(value/100.0);
    QString str_z = QString::number(filter_z,'f',3);
    ui->label_Filter_z->setText(str_z);
}

void
MainWindow::on_pushButton_trackObject_clicked()
{
    if(trackObject_selectedIdx_ < 0)
    {
        emit(errorHandler("No recognized object selected",QMSGBOX_Critical));
        return;
    }

    if(userTracker.hasTarget())
    {
        userTracker.stopTracking();
    }
    userTracker.setReferenceCloud(recognizedObjects_[trackObject_selectedIdx_]);

    // Clear visualizer
    ui->comboBox_recognizedObjects->clear();
    removeCloudFromViz(recognizedObjects_,REG_OBJECT_ID);
    recognizedObjects_.clear();
}

void
MainWindow::on_comboBox_recognizedObjects_currentIndexChanged(int index)
{
   trackObject_selectedIdx_ = index;
}

void
MainWindow::on_pushButton_resetView_clicked()
{
   viewer_->resetCameraViewpoint("cloudpass");
   viewer_->resetCamera();
}

void
MainWindow::on_pushButton_toggleCoord_clicked()
{
   if(!viewer_->removeCoordinateSystem("coordinateSystem",0))
   {
       viewer_->addCoordinateSystem(1.0,"coordinateSystem",0);
   }
   ui->qvtkWidget_cloudViewer->update();
}

void
MainWindow::on_pushButton_stopTracking_clicked()
{
    userTracker.stopTracking();
    viewer_->removePointCloud("particleCentroid",0);

    viewer_->removeAllShapes();
}

void
MainWindow::drawResult()
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr particles = userTracker.getParticleCloud();

    if(particles == nullptr)
    {
        // Wait for next frame
        return;
    }

    // Get the average centroid after 2 seconds
    static double x = 0;
    static double y = 0;
    static double z = 0;
    static unsigned count = 0;
    static double last = pcl::getTime();
    double now = pcl::getTime();
    ++count;

    Eigen::Vector4f c;
    pcl::compute3DCentroid<pcl::PointXYZ> (*particles, c);

    c[0] = user_rounding(c[0],3);
    c[1] = user_rounding(c[1],3);
    c[2] = user_rounding(c[2],3);

    x += c[0];
    y += c[1];
    z += c[2];
    if(now - last < 2)
        return;

    pcl::PointXYZ centroid(x/count,y/count,z/count);

    x = y = z = 0;
    count = 0;
    last = now;
    // Get the average centroid after 2 seconds

    pcl::PointCloud<pcl::PointXYZ>::Ptr particleCentroid(new pcl::PointCloud<pcl::PointXYZ>);
    particleCentroid->points.push_back(centroid);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color (particleCentroid, 0, 0, 255);
    if (!viewer_->updatePointCloud (particleCentroid, blue_color, "particleCentroid"))
    {
        viewer_->addPointCloud(particleCentroid,"particleCentroid",0);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  20.0, "particleCentroid");
    }

    viewer_->removeShape("R");
    viewer_->addText((boost::format ("number of Reference PointClouds: %d") % userTracker.getReferenceCloudSize()).str(),
                     10, // X pos
                     20, // Y pos
                     20, // Font size
                     1.0,1.0,1.0,
                     "R");

    viewer_->removeShape("P");
    viewer_->addText((boost::format ("number of Particles: %d") % userTracker.getParticleSize()).str(),
                     10,40,20,
                     1.0,1.0,1.0,
                     "P");

    viewer_->removeShape("F");
    viewer_->addText((boost::format ("Fit ratio: %f") % userTracker.getFitRatio()).str(),
                     10,60,20,
                     1.0,1.0,1.0,
                     "F");

    viewer_->removeShape("trackingFPS");
    viewer_->addText((boost::format ("Tracking FPS : %f") % userTracker.getTrackingFPS()).str(),
                     10,76,20,
                     1.0,1.0,1.0,
                     "trackingFPS");

    viewer_->removeShape("C");
    viewer_->addText((boost::format ("Centroid: [%d][%d][%d]") % c[0] % c[1] % c[2]).str(),
                     10,100,20,
                     1.0,1.0,1.0,
                     "C");

    Eigen::Vector4f cam_coords = {c[0],c[1],c[2],1};
    Eigen::Vector4f target_robot_coords = Cam_vs_Robot_Rototranslation(cam_coords,
                                                                       cam_rototranslation_mat_);
    updateSetpoint(target_robot_coords[0],
                   target_robot_coords[1],
                   target_robot_coords[2]);
#if 0
            {
                std::vector<double> min_points(3,0);
                std::vector<double> max_points(3,0);
                userTracker.getBoundingBox(min_points[0], max_points[0],
                                           min_points[1], max_points[1],
                                           min_points[2], max_points[2]);
                if(! viewer_->addCube(min_points[0], max_points[0],
                                      min_points[1], max_points[1],
                                      min_points[2], max_points[2]))
                {
                     viewer_->removeShape("cube");
                     viewer_->addCube(min_points[0], max_points[0],
                                      min_points[1], max_points[1],
                                      min_points[2], max_points[2]);
                }
            }
#endif
}

void
MainWindow::updateEndpoint(float x, float y, float z)
{
    endpoint_x_ = x;
    endpoint_y_ = y;
    endpoint_z_ = z;

    ui->lcdNumber_endpoint_x->display((int)(x*1000));
    ui->lcdNumber_endpoint_y->display((int)(y*1000));
    ui->lcdNumber_endpoint_z->display((int)(z*1000));
}

void
MainWindow::updateSetpoint(float x, float y, float z)
{
    setpoint_x_ = x;
    setpoint_y_ = y;
    setpoint_z_ = z;

    ui->lcdNumber_setpoint_x->display((int)(x*1000));
    ui->lcdNumber_setpoint_y->display((int)(y*1000));
    ui->lcdNumber_setpoint_z->display((int)(z*1000));
}

void
MainWindow::on_pushButton_cmdGrab_clicked()
{
    serialPort_sendPacket(UserComm::CMD_GRAB);
}
void MainWindow::on_pushButton_cmdMoveToXYZ_clicked()
{
    setPosition_window myWindow(this);
    myWindow.exec();

    if(myWindow.result() != QDialog::Accepted)
    {
        return;
    }
    updateSetpoint((float)(myWindow.setX_mm/1000.f),
                   (float)(myWindow.setY_mm/1000.f),
                   (float)(myWindow.setZ_mm/1000.f));
    serialPort_sendPacket(UserComm::CMD_MOVETOXYZ);
}
void MainWindow::on_pushButton_cmdStop_clicked()
{
    serialPort_sendPacket(UserComm::CMD_STOP);
}
void MainWindow::on_pushButton_cmdDrop_clicked()
{
    serialPort_sendPacket(UserComm::CMD_DROP);
}

Eigen::Vector4f
MainWindow::Cam_vs_Robot_Rototranslation(Eigen::Vector4f &cam_coordinates,
                                         Eigen::Matrix4f &rototranslation)
{
    return rototranslation*cam_coordinates;
}
void
MainWindow::updateCamRototranslationMat(float val,int rows, int cols)
{
    if(rows < 0 && rows > 3) return;
    if(cols < 0 && cols > 3) return;

    cam_rototranslation_mat_(rows,cols) = val;

//    std::cout << cam_rototranslation_mat_ << std::endl;
}
/*
 * SLOT
 */
void
MainWindow::textEditChanged()
{
    QTextEdit *_text = qobject_cast<QTextEdit*>(sender());
    if( _text == ui->text_roto00 || _text == ui->text_roto01 ||
        _text == ui->text_roto02 || _text == ui->text_roto03 ||

        _text == ui->text_roto10 || _text == ui->text_roto11 ||
        _text == ui->text_roto12 || _text == ui->text_roto13 ||

        _text == ui->text_roto20 || _text == ui->text_roto21 ||
        _text == ui->text_roto22 || _text == ui->text_roto23 ||

        _text == ui->text_roto30 || _text == ui->text_roto31 ||
        _text == ui->text_roto32 || _text == ui->text_roto33)
    {
        int rows = _text->objectName().at(9).unicode() - 48;
        int cols = _text->objectName().at(10).unicode() - 48;
        float val = _text->toPlainText().toFloat();
        updateCamRototranslationMat(val,rows,cols);
    }
}
