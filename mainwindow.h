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

#include "userRecognizer_thread.h"

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
    filterPassThrough(const CloudConstPtr &cloud, Cloud &result)
    {
        pcl::PassThrough<PointType> pass;
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 10.0);
        //pass.setFilterLimits (0.0, 1.5);
        //pass.setFilterLimits (0.0, 0.6);
        pass.setKeepOrganized (false);
        pass.setInputCloud (cloud);
        pass.filter (result);
    }

    void
    gridSample (const CloudConstPtr &cloud, Cloud &result, double leaf_size = 0.01)
    {
      pcl::VoxelGrid<PointType> grid;
      //pcl::ApproximateVoxelGrid<PointType> grid;
      grid.setLeafSize (float (leaf_size), float (leaf_size), float (leaf_size));
      grid.setInputCloud (cloud);
      grid.filter (result);
      //result = *cloud;
    }

public slots:
    void
    updateViewer();

    void
    grabberDie();

    void
    RecognitionThreadDestroyed()
    {
       std::cout << __FUNCTION__ << std::endl;
       pRecognitionThread_ = NULL;
    }

signals:
    void cloudChanged();

private:
    Ui::MainWindow *ui;

protected:
    pcl::Grabber * grabber_;
    QTimer * grabberWDT;

    QMutex cloud_mtx_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    bool new_cloud_;
    CloudPtr cloud_pass_;
    CloudPtr model_;

    enum grabberState gState_;
    UserRecognizer_Thread *pRecognitionThread_;

    float cloud_fps_;

private slots:
    void on_pushButton_restartGrabber_clicked();
    void on_pushButton_recognize_clicked();
};

#endif // MAINWINDOW_H
