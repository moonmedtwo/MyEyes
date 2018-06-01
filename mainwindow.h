#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include <commons.h>

#include <QMutex>
#include <QTimer>
#include <QElapsedTimer>
#include <vtkRenderWindow.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include "userrecognizer_thread.h"
#include "userrecognizer_window.h"
#include "usercomm.h"
#include "usertracking.h"


enum grabberState
{
    grabberStopped = 0,
    grabberIsStarting = 1,
    grabberIsRunning = 2,
    grabberHang = 3,
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void
    viewer_init();

    void
    serialPort_init();
    void
    serialPort_update_PacketStatus(UserComm::packetStatus toStatus,
                                  UserComm::PROTOCOL_CMD packetType);
    UserComm::packetStatus
    serialPort_get_PacketStatus(UserComm::PROTOCOL_CMD packetType);
    void
    serialPort_sendPacket(UserComm::PROTOCOL_CMD packetType);
    uint8_t *
    serialPort_wrapPacket(UserComm::PROTOCOL_CMD packetType, uint8_t &len);

    void
    cloudCallback(const CloudConstPtr &cloud);

    void
    kinect_init();

    void
    w_grabberStart(pcl::Grabber *grabber);
    void
    w_grabberStop(pcl::Grabber *grabber);

    inline
    void
    updateGrabberState(const grabberState &gState);

    void
    filterPassThrough(const CloudConstPtr &cloud, Cloud &result);

    void
    gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01);

    void
    removeCloudFromViz(std::vector<CloudPtr> &clouds, const std::string &prename);

    void
    drawResult();

    void
    updateEndpoint(float x,float y, float z);

public slots:
    void
    updateViewer();

    void
    grabberDie();

    void
    recognitionThreadDestroyed();
    void
    recognitionResult(std::vector<CloudPtr> results);
    void
    recognitionProgress(int percent, int model, int totalmodel);

    void
    serialPort_connected();
    void
    serialPort_disconnected();
    void
    serialPort_packetReceived(const QByteArray &data);
    void
    serialPort_handleError(const QString &error);
    void
    serialPort_routine();

    void
    errorHandler(const QString &error,int type);

signals:
    void
    cloudChanged();

    void
    openSerialPort();
    void
    closeSerialPort();
//    void
//    serialPort_write(const QByteArray &data);
    void
    serialPort_write(uint8_t *pData, uint8_t len);

private:
    Ui::MainWindow *ui;

protected:
    QElapsedTimer userSysticks;

    pcl::Grabber * grabber_;
    QTimer * grabberWDT;      	// Timer to check grabber hang
    enum grabberState gState_;

    QMutex cloud_mtx_;
    QMutex viewer_mtx_;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    bool new_cloud_;
    CloudPtr cloud_pass_;
    CloudPtr model_;
    std::string model_dir_;

    std::vector<CloudPtr> recognizedObjects_;
    UserRecognizer_Thread *pRecognitionThread_;

    float cloud_fps_;

    UserComm comm;
    QTimer * commWDT; // Timer to check serial port hang
    UserComm::packetStatus packetStatus_[UserComm::NUMB_OF_CMD];
    uint8_t packetRetries_[UserComm::NUMB_OF_CMD];
    uint32_t packetTimeout_[UserComm::NUMB_OF_CMD];

    double filter_z;

    UserTracking userTracker;
    int	trackObject_selectedIdx_;

    float endpoint_x_;
    float endpoint_y_;
    float endpoint_z_;

private slots:
    void on_pushButton_restartGrabber_clicked();
    void on_pushButton_recognize_clicked();
    void on_Comm_comboBox_currentTextChanged(const QString &arg1);
    void on_pushButton_OpenComm_clicked();
    void on_pushButton_cmdGrab_clicked();
    void on_horizontalSlider_Filter_z_valueChanged(int value);
    void on_pushButton_trackObject_clicked();
    void on_comboBox_recognizedObjects_currentIndexChanged(int index);
    void on_pushButton_resetView_clicked();
    void on_pushButton_toggleCoord_clicked();
    void on_pushButton_stopTracking_clicked();
    void on_pushButton_cmdMoveToXYZ_clicked();
    void on_pushButton_cmdStop_clicked();
    void on_pushButton_cmdDrop_clicked();
};

#endif // MAINWINDOW_H
