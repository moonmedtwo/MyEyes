#include "setposition_window.h"
#include "ui_setposition_window.h"

setPosition_window::setPosition_window(QWidget *parent) :
    QDialog(parent),
    setX_mm(0),
    setY_mm(0),
    setZ_mm(0),
    ui(new Ui::setPosition_window)
{
    ui->setupUi(this);
}

setPosition_window::~setPosition_window()
{
    delete ui;
}

void setPosition_window::on_buttonBox_accepted()
{
    setX_mm = ui->text_X->toPlainText().toInt();
    setY_mm = ui->text_Y->toPlainText().toInt();
    setZ_mm = ui->text_Z->toPlainText().toInt();
}
