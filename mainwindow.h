#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <QMainWindow>
#include <commons.h>

#include <QMutex>
#include <QTimer>
#include <vtkRenderWindow.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include "userrecognizer_thread.h"
#include "userrecognizer_window.h"
#include "usercomm.h"



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
    initViewer();

    void
    initSerialPortSettings();

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
    serialPort_connected();
    void
    serialPort_disconnected();
    void
    serialPort_packetReceived(const QByteArray &data);
    void
    serialPort_handleError(const QString &error);

    void
    errorHandler(const QString &error,int type);

signals:
    void
    cloudChanged();

    void
    openSerialPort();
    void
    closeSerialPort();
    void
    serialPort_write(const QByteArray &data);

private:
    Ui::MainWindow *ui;

protected:
    pcl::Grabber * grabber_;
    QTimer * grabberWDT;

    QMutex cloud_mtx_;
    QMutex viewer_mtx_;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    bool new_cloud_;
    CloudPtr cloud_pass_;
    CloudPtr model_;
    std::string model_dir_;

    std::vector<CloudPtr> recognizedObjects_;

    enum grabberState gState_;
    UserRecognizer_Thread *pRecognitionThread_;

    float cloud_fps_;

    UserComm comm;

    double filter_z;

private slots:
    void on_pushButton_restartGrabber_clicked();
    void on_pushButton_recognize_clicked();
    void on_pushButton_visData_clicked();
    void on_Comm_comboBox_currentTextChanged(const QString &arg1);
    void on_pushButton_OpenComm_clicked();
    void on_pushButton_cmdGrab_clicked();
    void on_horizontalSlider_Filter_z_valueChanged(int value);
};

#endif // MAINWINDOW_H
