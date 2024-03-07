#include "workthread.h"

WorkThread::WorkThread(QObject *parent)
    : QObject{parent}
{
    OpenFramegrabber("DirectShow", 1, 1, 0, 0, 0, 0, "default", 8, "rgb", -1, "false",
              "default", HTuple("[0] BisonCam, NB Pro"), 0, -1, &window_handle);
    GrabImage(&image, window_handle);
    GetImageSize(image, &w_image, &h_image);
}


void WorkThread::set_window_handle(HTuple window_handle1_, HTuple window_handle2_)
{
    window_handle1 = window_handle1_;
//    OpenFramegrabber("DirectShow", 1, 1, 0, 0, 0, 0, "default", 8, "rgb", -1, "false",
//              "default", HTuple("[0] BisonCam, NB Pro"), 0, -1, &window_handle1);
//    GrabImage(&image, window_handle1);
//    GetImageSize(image, &w_image, &h_image);
    window_handle2 = window_handle2_;
    HDevWindowStack::Push(window_handle1);
    HDevWindowStack::Push(window_handle2);
    SetPart(window_handle1, 0, 0, h_image - 1, w_image - 1);
    SetPart(window_handle2, 0, 0, h_image - 1, w_image - 1);
}


void WorkThread::set_flag_start(bool flag_start_)
{
    flag_start = flag_start_;
}


void WorkThread::set_flag_read(bool flag_read_)
{
    flag_read = flag_read_;
}


void WorkThread::read_image()
{
    while (flag_start)
    {
        GrabImage(&image, window_handle);
        DispObj(image, window_handle1);
        if (flag_read)
        {
            DispObj(image, window_handle2);
            flag_read = false;
        }
    }
}
