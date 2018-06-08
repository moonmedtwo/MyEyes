#ifndef SETPOSITION_WINDOW_H
#define SETPOSITION_WINDOW_H

#include <QDialog>
#include "user_debug.h"

namespace Ui {
class setPosition_window;
}

class setPosition_window : public QDialog
{
    Q_OBJECT

public:
    explicit setPosition_window(float lastX = 0, float lastY = 0, float lastZ = 0, QWidget *parent = 0);
    ~setPosition_window();

    float
    get_X_setPosition();
    float
    get_Y_setPosition();
    float
    get_Z_setPosition();


private slots:
    void on_buttonBox_accepted();
    void keyPressEvent(QKeyEvent *event);

private:
    Ui::setPosition_window *ui;

    int setX_mm;
    int setY_mm;
    int setZ_mm;
};

#endif // SETPOSITION_WINDOW_H
