#include "userrecognizer_window.h"
#include "ui_userRecognizer_window.h"

/*
 * @brief: parse cloud list from QStringList
 */
void
UserRecognizer_Window::update_cloud_list(const QStringList &list, const QString &root)
{
    if(list.size() > 0)
    {
        cloud_list_.clear();
        for(int i = 0; i < list.size(); ++i)
        {
           std::string s_root = root.toStdString();
           QString path = list.at(i);
           std::string s_path = path.toStdString();
           if(s_root.back() != '/')
           {
               s_root += '/';
           }
           cloud_list_.push_back(s_root + s_path);
        }
    }

}

/*
 * @brief: render point cloud to UI
 */
void
UserRecognizer_Window::renderCloud(const std::string &path)
{
    if(!viewer_started_)
    {
        viewer_.reset(new pcl::visualization::PCLVisualizer("model_vis",false));
        ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
//        viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(),\
//                                 ui->qvtkWidget->GetRenderWindow());
        viewer_->addCoordinateSystem(0.1,"coordinateAxis",0);
        viewer_started_ = true;
    }

    pcl::PCDReader reader;
    CloudPtr cloud(new Cloud);
    reader.read(path,*cloud);

    if(cloud->points.size() <= 0)
        return;

    viewer_->removeAllPointClouds();
    viewer_->addPointCloud(cloud,"cloud",0);
    viewer_->resetCameraViewpoint("cloud");
    viewer_->resetCamera();
    ui->qvtkWidget->update();
}

/*
 * @brief: filter all files related to pointcloud in dir
 * @brief: add files to cloud_list_ with update_cloud_list()
 */
void
UserRecognizer_Window::filterCloudinDir(QDir &dir)
{
    dir.setNameFilters(QStringList("*.pcd"));
    dir.setFilter(QDir::Files);
    dir.setSorting(QDir::Name);
    update_cloud_list(dir.entryList(),dir.path());
}

/*
 * @brief: update private member variables
 * @brief: and UI when model_dir_ changes
 */
void
UserRecognizer_Window::updateModelDir(const std::string &dir)
{
    // Update private variable
    std::cout << __FUNCTION__ << " " << model_dir_ << std::endl;
    model_dir_ = dir;
    // Convert to QString
    QString dir_path(QString::fromStdString(model_dir_));
    ui->label_modelPath->setText(dir_path);

    {
        QDir curDir(dir_path);
        filterCloudinDir(curDir);
    }

    if(cloud_list_.size() > 0)
    {
        renderCloud(cloud_list_[0]);
    }
}

UserRecognizer_Window::UserRecognizer_Window(QWidget *parent,const std::string &model_dir) :
    QDialog(parent),
    ui(new Ui::UserRecognizer_Window),
    model_dir_(model_dir)
{
    ui->setupUi(this);
    updateModelDir(model_dir_);
}

UserRecognizer_Window::~UserRecognizer_Window()
{
    delete ui;
}

void UserRecognizer_Window::on_pushButton_chooseModel_clicked()
{
    QString dir =
            QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                              DATABASE_ROOT,
                                              QFileDialog::ShowDirsOnly |
                                              QFileDialog::DontResolveSymlinks);
    updateModelDir(dir.toStdString());
}
