#include "mainwindow_2.h"
#include "ui_mainwindow_2.h"

MainWindow_2::MainWindow_2(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow_2)
{
    ui->setupUi(this);
}

MainWindow_2::~MainWindow_2()
{
    delete ui;
}
