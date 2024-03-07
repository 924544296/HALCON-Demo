#include "widget.h"
#include "./ui_widget.h"
#include "mainwindow.h"


Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    // --
    ui->comboBox->addItem("摄像头");
}

Widget::~Widget()
{
    delete ui;
}


void Widget::on_pushButton_clicked()
{
    switch (ui->comboBox->currentIndex())
    {
    case 0:
    {
        MainWindow *w = new MainWindow;
        w->show();
        break;
    }
    }
}

