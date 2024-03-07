#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // --
    ui->label->setScaledContents(true);
    ui->label_2->setScaledContents(true);
    widid1 = static_cast<Hlong>(ui->label->winId());
    OpenWindow(0, 0, ui->label->width(), ui->label->height(), widid1, "visible", "", &window_handle1);
    widid2 = static_cast<Hlong>(ui->label_2->winId());
    OpenWindow(0, 0, ui->label_2->width(), ui->label_2->height(), widid2, "visible", "", &window_handle2);
//    OpenFramegrabber("DirectShow", 1, 1, 0, 0, 0, 0, "default", 8, "rgb", -1, "false",
//              "default", HTuple("[0] BisonCam, NB Pro"), 0, -1, &window_handle1);
    //
//    ptimer = new QTimer;
//    ptimer->start(10);
    pworker->set_window_handle(window_handle1, window_handle2);
    pworker->moveToThread(pthread);
//    pthread->start();
    //
//    connect(ptimer, &QTimer::timeout, pworker, &ThreadWorker::read_image);
    connect(this, &MainWindow::start_work, pworker, &WorkThread::read_image);
}

MainWindow::~MainWindow()
{
    // --
    on_pushButton_2_clicked();
    delete pworker;
    delete pthread;
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    pworker->set_flag_start(true);
    pthread->start();
    emit start_work();
}


void MainWindow::on_pushButton_2_clicked()
{
    pworker->set_flag_start(false);
    pthread->quit();
    pthread->wait();
}


void MainWindow::on_pushButton_3_clicked()
{
    if (pworker->flag_start == false)
    {
        QMessageBox::information(this,
            tr("老哥"),
            tr("你要打开摄像头才能截图。"),
            QMessageBox::Ok | QMessageBox::Cancel,
            QMessageBox::Ok);
        return;
    }
    pworker->set_flag_read(true);
}

