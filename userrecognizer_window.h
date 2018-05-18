#ifndef USERRECOGNIZER_WINDOW_H
#define USERRECOGNIZER_WINDOW_H

#include <QDialog>
#include <mainwindow.h>
#include "commons.h"

#include <QDir>
#include <QFileDialog>

namespace Ui {
class UserRecognizer_Window;
}

class UserRecognizer_Window : public QDialog
{
    Q_OBJECT

public:
    explicit UserRecognizer_Window(QWidget *parent = 0,
                                   const std::string &model_dir = DEFAULT_MODEL_DIR);
    ~UserRecognizer_Window();

    void
    update_cloud_list(const QStringList &list, const QString &root);

    void
    renderCloud(const std::string &path);

    void
    filterCloudinDir(QDir &dir);

    void
    updateModelDir(const std::string &dir);

    void
    renderViewer();

    std::string
    get_ModelDir()
    {
        return model_dir_;
    }

    std::vector<std::string>
    get_cloud_list()
    {
       return cloud_list_;
    }

private slots:
    void
    on_pushButton_chooseModel_clicked();

signals:
    void
    errorHandler(const QString &error, int type);


private:
    Ui::UserRecognizer_Window *ui;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    bool viewer_started_;

    std::string model_dir_;
    std::vector<std::string> cloud_list_;
};

#endif // USERRECOGNIZER_WINDOW_H
