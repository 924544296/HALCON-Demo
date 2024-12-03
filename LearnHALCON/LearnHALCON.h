#pragma once

#include <iostream>
//using namespace std;
#include "halconcpp/HalconCpp.h"
using namespace HalconCpp;
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
//using namespace cv;
#include <direct.h> 


cv::Mat HImageToMat(HalconCpp::HObject& H_img);
HalconCpp::HObject MatToHImage(cv::Mat& cv_img);
bool get_whole_object(cv::Mat& image_cv, int threshold);


struct ControlParam {
	float gamma = 5.0;//图像增强gamma值
	int package_threshold_low = 0;//包装袋二值化low参数
	int package_threshold_high = 235;//包装袋二值化high参数
	int circleKernel = 1;//开运算圆心核半径
	float rectangularity = 0.5; //矩形度
	int area_low = 40000;//面积筛选low参数
	int area_high = 100000;//面积筛选low参数
	int rect2_len1_min;//最小外接矩形长边最小值
	int rect2_len1_max;//最小外接矩形长边最大值
	int rect2_len2_min;//最小外接矩形短边最小值
	int rect2_len2_max;//最小外接矩形短边最大值
	int food_threshold_low = 0;//食品二值化low参数
	int food_threshold_high = 118;//食品二值化high参数
	int nearFoodwidth = 20;//靠近食品一侧的检测宽度
	int farawayFoodwidth = 20;//远离食品一侧的检测宽度
};


bool GetClampingImg(cv::Mat inputImg, ControlParam inputParam, cv::Mat& clampingImg_gamma, cv::Mat& clampingImg_original);



cv::Mat HalconHDR(cv::Mat inputImg);

cv::Mat Pow(cv::Mat inputImg, float alpha);

cv::Mat PowEmp(cv::Mat inputImg, float alpha, float beta, int mask);

//cv::Mat PowEmp(cv::Mat inputImg);

cv::Mat calcuGammaTrans(cv::Mat srcImage, double gamma);

void addContrast(cv::Mat& srcImg);

cv::Mat putMatToBackground(cv::Mat img);

cv::Mat ImageEnhance(cv::Mat& src);

cv::Mat emphasize(cv::Mat& input, int MaskWidth, float Factor);

cv::Mat AGCIE(cv::Mat& src);

cv::Mat AGCWD(cv::Mat& src, double alpha);

std::vector<cv::Rect> calcuObjectLocation(cv::Mat edgeImg);

class ObjsRect {
public:
	cv::Rect objectRect;
	float class_id;
};

void hardDetect(cv::Mat& image, int threshVal, int boundingRectMinVal, std::vector<ObjsRect>& foreignRects);











