#ifndef WORKTHREAD_H
#define WORKTHREAD_H

#include <QObject>
#include <HalconCpp.h>
using namespace HalconCpp;


class WorkThread : public QObject
{
    Q_OBJECT
public:
    explicit WorkThread(QObject *parent = nullptr);
    // --
    void set_window_handle(HTuple window_handle1_, HTuple window_handle2_);
    void set_flag_start(bool flag_start_);
    void set_flag_read(bool flag_read_);
    void read_image();
    bool flag_start=false;
    bool flag_read=false;

private:
    HObject image;
    HTuple window_handle, window_handle1, window_handle2, w_image, h_image;

signals:

};

#endif // WORKTHREAD_H
