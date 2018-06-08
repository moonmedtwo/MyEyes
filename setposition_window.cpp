#include "setposition_window.h"
#include "ui_setposition_window.h"
#include <QKeyEvent>

setPosition_window::setPosition_window(float lastX, float lastY, float lastZ, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::setPosition_window),
    setX_mm(lastX*1000),
    setY_mm(lastY*1000),
    setZ_mm(lastZ*1000)
{
    ui->setupUi(this);
    ui->lineEdit_X->setText(QString::number(setX_mm));
    ui->lineEdit_Y->setText(QString::number(setY_mm));
    ui->lineEdit_Z->setText(QString::number(setZ_mm));

    connect(ui->lineEdit_X,SIGNAL(returnPressed()),ui->buttonBox,SLOT(setFocus()));
    connect(ui->lineEdit_Y,SIGNAL(returnPressed()),ui->buttonBox,SLOT(setFocus()));
    connect(ui->lineEdit_Z,SIGNAL(returnPressed()),ui->buttonBox,SLOT(setFocus()));

    // TODO fix ENTER
}

float
setPosition_window::get_X_setPosition()
{
    return (float)(setX_mm/1000.0f);
}

float
setPosition_window::get_Y_setPosition()
{
    return (float)(setY_mm/1000.0f);
}

float
setPosition_window::get_Z_setPosition()
{
    return (float)(setZ_mm/1000.0f);
}

setPosition_window::~setPosition_window()
{
    delete ui;
}

void
setPosition_window::on_buttonBox_accepted()
{
    setX_mm = ui->lineEdit_X->text().toInt();
    setY_mm = ui->lineEdit_Y->text().toInt();
    setZ_mm = ui->lineEdit_Z->text().toInt();
}

void
setPosition_window::keyPressEvent(QKeyEvent *event)
{
//    switch(event->key())
//    {
//        case Qt::Key_Enter:
//            this->accept();
//        break;
//    default:
//        break;
//    }
}

