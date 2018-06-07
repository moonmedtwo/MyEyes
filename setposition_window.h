#ifndef SETPOSITION_WINDOW_H
#define SETPOSITION_WINDOW_H

#include <QDialog>

namespace Ui {
class setPosition_window;
}

class setPosition_window : public QDialog
{
    Q_OBJECT

public:
    explicit setPosition_window(QWidget *parent = 0);
    ~setPosition_window();

    int setX_mm;
    int setY_mm;
    int setZ_mm;

private slots:
    void on_buttonBox_accepted();

private:
    Ui::setPosition_window *ui;
};

#endif // SETPOSITION_WINDOW_H
