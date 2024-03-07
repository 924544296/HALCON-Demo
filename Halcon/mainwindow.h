#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QTimer>
#include <QMessageBox>
#include "workthread.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

private:
    Ui::MainWindow *ui;
    // --
    HObject image;
    HTuple w_image, h_image, window_handle1, window_handle2;
    Hlong widid1, widid2;
    QThread *pthread = new QThread;
    WorkThread *pworker = new WorkThread;
    //    QTimer *ptimer;

signals:
    void start_work();
};

#endif // MAINWINDOW_H
