#include "LearnHALCON.h"


//int main() {
//
//	//cv::Mat image = cv::imread("D:\\data\\can_he\\data\\抠图\\09-16-08-14.jpg", cv::IMREAD_COLOR);
//	cv::Mat image = cv::imread("E:\\D\\data\\can_he\\data\\抠图\\15-40-19-3.jpg", cv::IMREAD_COLOR);
//	cv::namedWindow("image", cv::WINDOW_NORMAL);
//	cv::imshow("image", image);
//	cv::waitKey(0);
//	cv::destroyAllWindows();
//	//std::cout << "--------------------------------before get_whole_object" << std::endl;
//	bool has_object = get_whole_object(image, 220);
//	cv::namedWindow(std::to_string(has_object), cv::WINDOW_NORMAL);
//	cv::imshow(std::to_string(has_object), image);
//	cv::waitKey(0);
//	cv::destroyAllWindows();
//
//	//ControlParam controlParam;
//	//cv::Mat clampImg_gamma, clampImg_original;
//	//cv::Mat image = cv::imread("D:\\data\\dou_gan\\data\\新线阵\\11-07-04-3.jpg", cv::IMREAD_COLOR);
//	//cv::imshow("image", image);
//	//cv::waitKey(0);
//	//bool isGet = GetClampingImg(image, controlParam, clampImg_gamma, clampImg_original);
//	//cv::imshow("clampImg_gamma", clampImg_gamma);
//	//cv::waitKey(0);
//
//
//	std::cout << std::endl << "Done." << std::endl;
//	return 0;
//}


int main() {


	//加载文件夹图片	
	std::string path_name;
	path_name = "F:\\D\\data\\jin_fu_hou_ping0\\data\\date\\2024-11-28\\origin";
	std::string path = path_name + "\\*.jpg";
	std::string path_out = path_name + "_enhanced\\";
	_mkdir(path_out.c_str());


	std::vector<cv::Mat> images;
	// 必须cv的String
	std::vector<cv::String> fn;
	cv::glob(path, fn, false);
	for (int i = 0; i < fn.size(); i++) {
		images.push_back(cv::imread(fn[i], cv::IMREAD_GRAYSCALE));
	}


	double minVal, maxVal;
	cv::Point minLoc, maxLoc;
	size_t name_start = path.rfind("\\") + 1;
	//ControlParam inputParam;
	cv::Mat image, clampingImg_gamma, clampingImg_original;
	bool has_edge;
	for (int img_idx = 0; img_idx < images.size(); img_idx++) {
		std::string name = fn[img_idx];
		std::string imagename = name.substr(name_start, name.size() + 1 - name_start);
		std::cout << "image name:" << imagename << std::endl;
		image = images[img_idx];



		//image.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 0);
		//image.at<cv::Vec3b>(0, 1) = cv::Vec3b(255, 255, 255);
		//
		//image = PowEmp(image);
		//image = ImageEnhance(image);
		//image = calcuGammaTrans(image, 0.6);
		//image = emphasize(image, 5, 2);
		//image = HalconHDR(image);
		//image = PowEmp(image, 0.1, 3.0, 17);
		image = PowEmp(image, 0.3, 3.0, 50);
		//image = Pow(image, 0.1);
		//image = calcuGammaTrans(image, 0.4);
		//image = AGCIE(image);
		//image = AGCWD(image, 0.1);
		//has_edge = GetClampingImg(image, inputParam, clampingImg_gamma, clampingImg_original);
		//if (has_edge)
		//{
			//cv::imwrite(path_out + imagename, clampingImg_gamma);
		//}

		//cv::imwrite(path_out + imagename, clampingImg_original);
		// 
		//cv::minMaxLoc(image, &minVal, &maxVal, &minLoc, &maxLoc);
		//std::cout << "minVal, maxVal: " << minVal << ", " << maxVal << std::endl;


		cv::imwrite(path_out + imagename, image);
		//
		// 		
		//cv::namedWindow(imagename, cv::WINDOW_NORMAL);
		//cv::imshow(imagename, image);
		//cv::waitKey(0);
	}


	return 0;
}