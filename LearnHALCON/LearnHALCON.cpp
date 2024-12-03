#include "LearnHALCON.h"


cv::Mat HImageToMat(HalconCpp::HObject& H_img)
{
	cv::Mat cv_img;
	HalconCpp::HTuple channels, w, h;

	HalconCpp::ConvertImageType(H_img, &H_img, "byte");
	HalconCpp::CountChannels(H_img, &channels);

	if (channels.I() == 1)
	{
		HalconCpp::HTuple pointer;
		GetImagePointer1(H_img, &pointer, nullptr, &w, &h);
		int width = w.I(), height = h.I();
		int size = width * height;
		cv_img = cv::Mat::zeros(height, width, CV_8UC1);
		memcpy(cv_img.data, (void*)(pointer.L()), size);
	}

	else if (channels.I() == 3)
	{
		HalconCpp::HTuple pointerR, pointerG, pointerB;
		HalconCpp::GetImagePointer3(H_img, &pointerR, &pointerG, &pointerB, nullptr, &w, &h);
		int width = w.I(), height = h.I();
		int size = width * height;
		cv_img = cv::Mat::zeros(height, width, CV_8UC3);
		uchar* R = (uchar*)(pointerR.L());
		uchar* G = (uchar*)(pointerG.L());
		uchar* B = (uchar*)(pointerB.L());
		for (int i = 0; i < height; ++i)
		{
			uchar* p = cv_img.ptr<uchar>(i);
			for (int j = 0; j < width; ++j)
			{
				p[3 * j] = B[i * width + j];
				p[3 * j + 1] = G[i * width + j];
				p[3 * j + 2] = R[i * width + j];
			}
		}
	}
	return cv_img;
}


HalconCpp::HObject MatToHImage(cv::Mat& cv_img)
{
	HalconCpp::HObject H_img;

	if (cv_img.channels() == 1)
	{
		int height = cv_img.rows, width = cv_img.cols;
		int size = height * width;
		uchar* temp = new uchar[size];

		memcpy(temp, cv_img.data, size);
		HalconCpp::GenImage1(&H_img, "byte", width, height, (Hlong)(temp));

		delete[] temp;
	}
	else if (cv_img.channels() == 3)
	{
		int height = cv_img.rows, width = cv_img.cols;
		int size = height * width;
		uchar* B = new uchar[size];
		uchar* G = new uchar[size];
		uchar* R = new uchar[size];

		for (int i = 0; i < height; i++)
		{
			uchar* p = cv_img.ptr<uchar>(i);
			for (int j = 0; j < width; j++)
			{
				B[i * width + j] = p[3 * j];
				G[i * width + j] = p[3 * j + 1];
				R[i * width + j] = p[3 * j + 2];
			}
		}
		HalconCpp::GenImage3(&H_img, "byte", width, height, (Hlong)(R), (Hlong)(G), (Hlong)(B));

		delete[] R;
		delete[] G;
		delete[] B;
	}
	return H_img;
}


bool get_whole_object(cv::Mat& image_cv, int threshold)
{
	//
	HObject image_ho, image_gamma, image_result, region_image_ho, region_image_ho_connected, region_image_ho_selected, region_image_dilation, region_image_erosion, region_image_ho_trans, region_image_ho_selected_i;
	HTuple w, h, row1s, row2s, col1s, col2s, num_obj, cols, rows, pixels;
	bool has_object = false;

	//
	image_ho = MatToHImage(image_cv);
	Rgb1ToGray(image_ho, &image_ho);
	GammaImage(image_ho, &image_gamma, 8, 0, 0, 255, "true");
	GetImageSize(image_ho, &w, &h);

	Threshold(image_gamma, &region_image_ho, 0, threshold);
	Connection(region_image_ho, &region_image_ho_connected);
	SelectShape(region_image_ho_connected, &region_image_ho_selected, "area", "and", 100, w*h);
	SelectShape(region_image_ho_selected, &region_image_ho_selected, "height", "and", 50, h);
	DilationRectangle1(region_image_ho_selected, &region_image_dilation, 91, 91);
	ErosionRectangle1(region_image_dilation, &region_image_erosion, 91, 91);
	ShapeTrans(region_image_erosion, &region_image_ho_trans, "convex");

	CountObj(region_image_ho_trans, &num_obj);
	if (num_obj.I() < 1)
	{
		return false;
	}

	SmallestRectangle1(region_image_ho_trans, &row1s, &col1s, &row2s, &col2s);

	GenImageConst(&image_result, "byte", w, h);
	GenImageProto(image_result, &image_result, 255);
	for (int i = 0; i < num_obj.I(); i++)
	{
		//if ((row2s[i] - row1s[i]).I() < 50)
		//{
		//	continue;
		//}
		if (col1s[i].I() > 0 && col2s[i].I() < (w - 1))
		{
			SelectObj(region_image_ho_trans, &region_image_ho_selected_i, i + 1);
			GetRegionPoints(region_image_ho_selected_i, &rows, &cols);
			GetGrayval(image_ho, rows, cols, &pixels);
			SetGrayval(image_result, rows, cols, pixels);
			has_object = true;
		}
	}

	Compose3(image_result, image_result, image_result, &image_result);
	image_cv = HImageToMat(image_result);

	return has_object;
}


bool GetClampingImg(cv::Mat inputImg, ControlParam inputParam, cv::Mat& clampingImg_gamma, cv::Mat& clampingImg_original) {
	//非空判断
	//if (outputTarget.clampRoi.size())std::vector<cv::Rect>().swap(outputTarget.clampRoi);

	try
	{
		// Local iconic variables
		HObject  ho_inputImage, ho_Gray_inputImage, ho_GammaImage, ho_SelectedRegions_rect;
		HObject	 ho_SelectedRegions_rectLen1, ho_SelectedRegions_rectLen2;
		HObject  ho_Regions_wholeInputImage, ho_RegionOpening, ho_ConnectedRegions;
		HObject  ho_SelectedRegions, ho_RegionTrans, ho_Rectangle;
		HObject  ho_ImageBackground_gamma_clamp, ho_ImageSingle_gamma_clamp;
		HObject  ho_ImageBackground_original_clamp, ho_ImageSingle_original_clamp;
		HObject  ho_RegionFillUp_todo, ho_ImageReduced, ho_Region_food;
		HObject  ho_FoodRegionTrans, ho_Rectangle_polygons, ho_Polygons;
		HObject  ho_Cross1, ho_Cross_foodcenter, ho_Cross_packcenter;
		HObject  ho_ImageAffineTrans, ho_Region_scale, ho_RegionFillUp_scale;
		HObject  ho_Rectangle_todo, ho_SelectedRegions_todo, ho_RegionDifference;
		HObject  ho_ConnectedRegions_food, ho_SelectedRegions_food;
		// Local control variables
		HTuple  hv_ImageFiles, hv_Index, hv_Width_inputImage;
		HTuple  hv_Height_inputImage, hv_Second_start, hv_Row1_rect1;
		HTuple  hv_Column1_rect1, hv_Row2_rect1, hv_Column2_rect1;
		HTuple  hv_Number_select, hv_Row_pack_rect2, hv_Column_pack_rect2;
		HTuple  hv_Phi_pack_rect2, hv_Length1_pack_rect2, hv_Length2_pack_rect2;
		HTuple  hv_Region_Index, hv_LeftPoint, hv_RightPoint, hv_Row_food_rect2;
		HTuple  hv_Column1_food_rect2, hv_Phi1_food_rect2, hv_Length1_food_rect2;
		HTuple  hv_Length2_food_rect2, hv_Row_polygons, hv_Col_polygons;
		HTuple  hv_Length_polygons, hv_Phi1_polygons, hv_pack_center_x;
		HTuple  hv_pack_center_y, hv_food_center_x, hv_food_center_y;
		HTuple  hv_tuple_center_Rows, hv_tuple_center_Columns, hv_HomMat2D;
		HTuple  hv_HomMat2DRotate_image, hv_HomMat2DRotate, hv_Qx;
		HTuple  hv_Qy, hv_Qx_center, hv_Qy_center, hv_Indices_x;
		HTuple  hv_Indices_y, hv_cornerAxy, hv_cornerBxy, hv_cornerCxy;
		HTuple  hv_cornerDxy, hv_temp, hv_isNearLeft, hv_cornerAxy_mv;
		HTuple  hv_cornerBxy_mv, hv_cornerCxy_mv, hv_cornerDxy_mv;
		HTuple  hv_tuple_cornerX, hv_tuple_cornerY, hv_HomMat2DInvert;
		HTuple  hv_Px, hv_Py, hv_Px_Index, hv_Py_Index, hv_Rows_clamp;
		HTuple  hv_Cols_clamp, hv_Grayvals_clamp_gamma, hv_Grayvals_clamp_original, hv_Width_clamp;
		HTuple  hv_Height_clamp, hv_WindowImageSingle_clamp, hv_Second_end;
		HTuple  hv_Number_select1, hv_Number_select2, hv_Number_select3, hv_Number_select4;
		HTuple  hv_Area, hv_Area1, hv_Row1, hv_Column1;

		ho_inputImage = MatToHImage(inputImg);
		GetImageSize(ho_inputImage, &hv_Width_inputImage, &hv_Height_inputImage);
		Rgb1ToGray(ho_inputImage, &ho_Gray_inputImage);
		GammaImage(ho_Gray_inputImage, &ho_GammaImage, inputParam.gamma, 0, 0, 255, "true");

		Threshold(ho_GammaImage, &ho_Regions_wholeInputImage, inputParam.package_threshold_low, inputParam.package_threshold_high);
		OpeningCircle(ho_Regions_wholeInputImage, &ho_RegionOpening, inputParam.circleKernel);
		Connection(ho_RegionOpening, &ho_ConnectedRegions);

		SelectShape(ho_ConnectedRegions, &ho_SelectedRegions, "area", "and", inputParam.area_low, inputParam.area_high);
		AreaCenter(ho_SelectedRegions, &hv_Area, &hv_Row1, &hv_Column1);
		std::vector<int>area;
		for (int idx = 0; idx < hv_Area.Length(); idx++) {
			area.push_back(hv_Area[idx].D());
		}

		ShapeTrans(ho_SelectedRegions, &ho_RegionTrans, "convex");
		SelectShape(ho_RegionTrans, &ho_SelectedRegions_rect, "rectangularity", "and", inputParam.rectangularity, 1);
		AreaCenter(ho_SelectedRegions, &hv_Area1, &hv_Row1, &hv_Column1);
		std::vector<int>area1;
		for (int idx = 0; idx < hv_Area.Length(); idx++) {
			area1.push_back(hv_Area[idx].D());
		}
		//SelectShape(ho_SelectedRegions_rect, &ho_SelectedRegions_rectLen1, "rect2_len2","and", inputParam.rect2_len2_min, inputParam.rect2_len2_max);
		//SelectShape(ho_SelectedRegions_rectLen1, &ho_SelectedRegions_rectLen2, "rect2_len1","and", inputParam.rect2_len1_min, inputParam.rect2_len1_max);

		SelectShape(ho_SelectedRegions_rect, &ho_SelectedRegions, "area", "and", inputParam.area_low, inputParam.area_high);
		SmallestRectangle1(ho_SelectedRegions, &hv_Row1_rect1, &hv_Column1_rect1, &hv_Row2_rect1,
			&hv_Column2_rect1);
		CountObj(ho_SelectedRegions, &hv_Number_select);

		//生成一张白图，贴漏油夹料图
		GenImageConst(&ho_ImageBackground_gamma_clamp, "byte", hv_Width_inputImage, hv_Height_inputImage);
		GenImageProto(ho_ImageBackground_gamma_clamp, &ho_ImageSingle_gamma_clamp, 255);
		GenImageConst(&ho_ImageBackground_original_clamp, "byte", hv_Width_inputImage, hv_Height_inputImage);
		GenImageProto(ho_ImageBackground_original_clamp, &ho_ImageSingle_original_clamp, 255);
		clampingImg_gamma = HImageToMat(ho_ImageSingle_gamma_clamp);
		clampingImg_original = HImageToMat(ho_ImageSingle_original_clamp);
		//  ShowMat(ho_GammaImage);
		if ((hv_Number_select < 1)) {
			return 0;//没检测目标
		}

		SmallestRectangle2(ho_SelectedRegions, &hv_Row_pack_rect2, &hv_Column_pack_rect2,
			&hv_Phi_pack_rect2, &hv_Length1_pack_rect2, &hv_Length2_pack_rect2);

		//遍历最终筛选出的region
		{
			HTuple end_val29 = hv_Number_select;
			HTuple step_val29 = 1;
			for (hv_Region_Index = 1; hv_Region_Index.Continue(end_val29, step_val29); hv_Region_Index += step_val29)
			{
				hv_LeftPoint.Clear();
				hv_LeftPoint.Append(HTuple(hv_Row1_rect1[hv_Region_Index - 1]));
				hv_LeftPoint.Append(HTuple(hv_Column1_rect1[hv_Region_Index - 1]));
				hv_RightPoint.Clear();
				hv_RightPoint.Append(HTuple(hv_Row2_rect1[hv_Region_Index - 1]));
				hv_RightPoint.Append(HTuple(hv_Column2_rect1[hv_Region_Index - 1]));

				//对切边图进行过滤
				if (0 != (HTuple(HTuple(hv_LeftPoint[0]) == 0).TupleOr(HTuple(hv_LeftPoint[1]) == 0)))
				{
					continue;
				}
				if (0 != (HTuple(HTuple(hv_RightPoint[0]) == (hv_Height_inputImage - 1)).TupleOr(HTuple(hv_RightPoint[1]) == (hv_Width_inputImage - 1))))
				{
					continue;
				}

				//select_obj的区域索引号从1开始，非0
				SelectObj(ho_SelectedRegions, &ho_RegionFillUp_todo, hv_Region_Index);

				//求辣条位置的中心点
				ReduceDomain(ho_GammaImage, ho_RegionFillUp_todo, &ho_ImageReduced);
				Threshold(ho_ImageReduced, &ho_Region_food, inputParam.food_threshold_low, inputParam.food_threshold_high);
				Connection(ho_Region_food, &ho_ConnectedRegions_food);
				SelectShape(ho_ConnectedRegions_food, &ho_SelectedRegions_food, "area", "and", 10000, 99999);
				ShapeTrans(ho_SelectedRegions_food, &ho_FoodRegionTrans, "convex");
				SmallestRectangle2(ho_FoodRegionTrans, &hv_Row_food_rect2, &hv_Column1_food_rect2,
					&hv_Phi1_food_rect2, &hv_Length1_food_rect2, &hv_Length2_food_rect2);

				//包装袋中心点+辣条中心点
				hv_pack_center_x = HTuple(hv_Column_pack_rect2[hv_Region_Index - 1]);
				hv_pack_center_y = HTuple(hv_Row_pack_rect2[hv_Region_Index - 1]);
				hv_food_center_x = hv_Column1_food_rect2;
				hv_food_center_y = hv_Row_food_rect2;

				hv_tuple_center_Rows.Clear();
				hv_tuple_center_Rows.Append(hv_food_center_y);
				hv_tuple_center_Rows.Append(hv_pack_center_y);
				hv_tuple_center_Columns.Clear();
				hv_tuple_center_Columns.Append(hv_food_center_x);
				hv_tuple_center_Columns.Append(hv_pack_center_x);

				//********************获取包装袋四个角点坐标*******************************
				GenRectangle2ContourXld(&ho_Rectangle_polygons, HTuple(hv_Row_pack_rect2[hv_Region_Index - 1]),
					HTuple(hv_Column_pack_rect2[hv_Region_Index - 1]), HTuple(hv_Phi_pack_rect2[hv_Region_Index - 1]),
					HTuple(hv_Length1_pack_rect2[hv_Region_Index - 1]), HTuple(hv_Length2_pack_rect2[hv_Region_Index - 1]));
				GenPolygonsXld(ho_Rectangle_polygons, &ho_Polygons, "ramer", 2);
				GetPolygonXld(ho_Polygons, &hv_Row_polygons, &hv_Col_polygons, &hv_Length_polygons,
					&hv_Phi1_polygons);

				if (0 != ((hv_Row_polygons.TupleLength()) > 4))
				{
					hv_Row_polygons = hv_Row_polygons.TupleSelectRange(0, 3);
					hv_Col_polygons = hv_Col_polygons.TupleSelectRange(0, 3);
				}
				else
				{
					hv_Row_polygons = hv_Row_polygons;
					hv_Col_polygons = hv_Col_polygons;
				}


				//********************包装袋四个角点和中心点、辣条中心点、图像旋转*******************************

				//图像的旋转中心XY与点旋转的XY相反，且旋转方向相反（AffineTransPoint2d、AffineTransImage）
				HomMat2dIdentity(&hv_HomMat2D);
				HomMat2dRotate(hv_HomMat2D, -HTuple(hv_Phi_pack_rect2[hv_Region_Index - 1]),
					hv_pack_center_y, hv_pack_center_x, &hv_HomMat2DRotate_image);
				HomMat2dRotate(hv_HomMat2D, HTuple(hv_Phi_pack_rect2[hv_Region_Index - 1]), hv_pack_center_x,
					hv_pack_center_y, &hv_HomMat2DRotate);
				AffineTransPoint2d(hv_HomMat2DRotate, hv_Col_polygons, hv_Row_polygons, &hv_Qx,
					&hv_Qy);
				AffineTransPoint2d(hv_HomMat2DRotate, hv_tuple_center_Columns, hv_tuple_center_Rows,
					&hv_Qx_center, &hv_Qy_center);
				AffineTransImage(ho_ImageReduced, &ho_ImageAffineTrans, hv_HomMat2DRotate_image,
					"constant", "false");
				//***********************************************************************************************
				TupleSortIndex(hv_Qx, &hv_Indices_x);
				TupleSortIndex(hv_Qy, &hv_Indices_y);

				//确定ABCD点的顺序，默认按照从上到下从左到右找点，且水平矩形的左上角点为A点
				hv_cornerAxy.Clear();
				hv_cornerAxy.Append(HTuple(hv_Qx[HTuple(hv_Indices_x[0])]));
				hv_cornerAxy.Append(HTuple(hv_Qy[HTuple(hv_Indices_x[0])]));
				hv_cornerBxy.Clear();
				hv_cornerBxy.Append(HTuple(hv_Qx[HTuple(hv_Indices_x[1])]));
				hv_cornerBxy.Append(HTuple(hv_Qy[HTuple(hv_Indices_x[1])]));
				hv_cornerCxy.Clear();
				hv_cornerCxy.Append(HTuple(hv_Qx[HTuple(hv_Indices_x[2])]));
				hv_cornerCxy.Append(HTuple(hv_Qy[HTuple(hv_Indices_x[2])]));
				hv_cornerDxy.Clear();
				hv_cornerDxy.Append(HTuple(hv_Qx[HTuple(hv_Indices_x[3])]));
				hv_cornerDxy.Append(HTuple(hv_Qy[HTuple(hv_Indices_x[3])]));
				if (0 != (HTuple(hv_Qy[HTuple(hv_Indices_x[0])]) > HTuple(hv_Qy[HTuple(hv_Indices_x[1])])))
				{
					hv_temp = hv_cornerAxy;
					hv_cornerAxy.Clear();
					hv_cornerAxy.Append(HTuple(hv_Qx[HTuple(hv_Indices_x[1])]));
					hv_cornerAxy.Append(HTuple(hv_Qy[HTuple(hv_Indices_x[1])]));
					hv_cornerBxy = hv_temp;
				}

				if (0 != (HTuple(hv_Qy[HTuple(hv_Indices_x[2])]) > HTuple(hv_Qy[HTuple(hv_Indices_x[3])])))
				{
					hv_temp = hv_cornerCxy;
					hv_cornerCxy.Clear();
					hv_cornerCxy.Append(HTuple(hv_Qx[HTuple(hv_Indices_x[3])]));
					hv_cornerCxy.Append(HTuple(hv_Qy[HTuple(hv_Indices_x[3])]));
					hv_cornerDxy = hv_temp;
				}

				//判断辣条位置是偏左还是偏右
				hv_isNearLeft = 0;
				if (0 != (HTuple(hv_Qx_center[0]) < HTuple(hv_Qx_center[1])))
				{
					hv_isNearLeft = 1;
				}

				if (0 != (hv_isNearLeft > 0))
				{
					hv_cornerAxy_mv.Clear();
					hv_cornerAxy_mv.Append(HTuple(hv_cornerAxy[0]) + inputParam.nearFoodwidth);
					hv_cornerAxy_mv.Append(HTuple(hv_cornerAxy[1]));
					hv_cornerBxy_mv.Clear();
					hv_cornerBxy_mv.Append(HTuple(hv_cornerAxy[0]) + inputParam.nearFoodwidth);
					hv_cornerBxy_mv.Append(HTuple(hv_cornerBxy[1]));
					hv_cornerCxy_mv.Clear();
					hv_cornerCxy_mv.Append(HTuple(hv_cornerCxy[0]) - inputParam.farawayFoodwidth);
					hv_cornerCxy_mv.Append(HTuple(hv_cornerCxy[1]));
					hv_cornerDxy_mv.Clear();
					hv_cornerDxy_mv.Append(HTuple(hv_cornerDxy[0]) - inputParam.farawayFoodwidth);
					hv_cornerDxy_mv.Append(HTuple(hv_cornerDxy[1]));
				}
				else
				{
					hv_cornerAxy_mv.Clear();
					hv_cornerAxy_mv.Append(HTuple(hv_cornerAxy[0]) + inputParam.farawayFoodwidth);
					hv_cornerAxy_mv.Append(HTuple(hv_cornerAxy[1]));
					hv_cornerBxy_mv.Clear();
					hv_cornerBxy_mv.Append(HTuple(hv_cornerAxy[0]) + inputParam.farawayFoodwidth);
					hv_cornerBxy_mv.Append(HTuple(hv_cornerBxy[1]));
					hv_cornerCxy_mv.Clear();
					hv_cornerCxy_mv.Append(HTuple(hv_cornerCxy[0]) - inputParam.nearFoodwidth);
					hv_cornerCxy_mv.Append(HTuple(hv_cornerCxy[1]));
					hv_cornerDxy_mv.Clear();
					hv_cornerDxy_mv.Append(HTuple(hv_cornerDxy[0]) - inputParam.nearFoodwidth);
					hv_cornerDxy_mv.Append(HTuple(hv_cornerDxy[1]));
				}

				if (0 != (HTuple(hv_cornerAxy_mv[0]) < 0))
				{
					hv_cornerAxy_mv[0] = 0;
				}
				if (0 != (HTuple(hv_cornerAxy_mv[1]) < 0))
				{
					hv_cornerAxy_mv[1] = 0;
				}
				if (0 != (HTuple(hv_cornerBxy_mv[0]) < 0))
				{
					hv_cornerBxy_mv[0] = 0;
				}
				if (0 != (HTuple(hv_cornerBxy_mv[1]) < 0))
				{
					hv_cornerBxy_mv[1] = 0;
				}
				if (0 != (HTuple(hv_cornerCxy_mv[0]) < 0))
				{
					hv_cornerCxy_mv[0] = 0;
				}
				if (0 != (HTuple(hv_cornerCxy_mv[1]) < 0))
				{
					hv_cornerCxy_mv[1] = 0;
				}
				if (0 != (HTuple(hv_cornerDxy_mv[0]) < 0))
				{
					hv_cornerDxy_mv[0] = 0;
				}
				if (0 != (HTuple(hv_cornerDxy_mv[1]) < 0))
				{
					hv_cornerDxy_mv[1] = 0;
				}

				hv_tuple_cornerX.Clear();
				hv_tuple_cornerX.Append(HTuple(hv_cornerAxy_mv[0]));
				hv_tuple_cornerX.Append(HTuple(hv_cornerCxy_mv[0]));
				hv_tuple_cornerX.Append(HTuple(hv_cornerDxy_mv[0]));
				hv_tuple_cornerX.Append(HTuple(hv_cornerBxy_mv[0]));
				hv_tuple_cornerX.Append(HTuple(hv_cornerAxy_mv[0]));
				hv_tuple_cornerY.Clear();
				hv_tuple_cornerY.Append(HTuple(hv_cornerAxy_mv[1]));
				hv_tuple_cornerY.Append(HTuple(hv_cornerCxy_mv[1]));
				hv_tuple_cornerY.Append(HTuple(hv_cornerDxy_mv[1]));
				hv_tuple_cornerY.Append(HTuple(hv_cornerBxy_mv[1]));
				hv_tuple_cornerY.Append(HTuple(hv_cornerAxy_mv[1]));
				HomMat2dInvert(hv_HomMat2DRotate, &hv_HomMat2DInvert);
				AffineTransPoint2d(hv_HomMat2DInvert, hv_tuple_cornerX, hv_tuple_cornerY, &hv_Px, &hv_Py);


				//**********************处理超出图像边界的点***************************************
				{
					HTuple end_val166 = (hv_Px.TupleLength()) - 1;
					HTuple step_val166 = 1;
					for (hv_Px_Index = 0; hv_Px_Index.Continue(end_val166, step_val166); hv_Px_Index += step_val166)
					{
						if (0 != (HTuple(hv_Px[hv_Px_Index]) < 0))
						{
							hv_Px[hv_Px_Index] = 0;
						}
					}
				}

				{
					HTuple end_val171 = (hv_Py.TupleLength()) - 1;
					HTuple step_val171 = 1;
					for (hv_Py_Index = 0; hv_Py_Index.Continue(end_val171, step_val171); hv_Py_Index += step_val171)
					{
						if (0 != (HTuple(hv_Py[hv_Py_Index]) < 0))
						{
							hv_Py[hv_Py_Index] = 0;
						}
					}
				}

				{
					HTuple end_val177 = (hv_Px.TupleLength()) - 1;
					HTuple step_val177 = 1;
					for (hv_Px_Index = 0; hv_Px_Index.Continue(end_val177, step_val177); hv_Px_Index += step_val177)
					{
						if (0 != (HTuple(hv_Px[hv_Px_Index]) > hv_Width_inputImage))
						{
							hv_Px[hv_Px_Index] = hv_Width_inputImage;
						}
					}
				}

				{
					HTuple end_val182 = (hv_Py.TupleLength()) - 1;
					HTuple step_val182 = 1;
					for (hv_Py_Index = 0; hv_Py_Index.Continue(end_val182, step_val182); hv_Py_Index += step_val182)
					{
						if (0 != (HTuple(hv_Py[hv_Py_Index]) > hv_Height_inputImage))
						{
							hv_Py[hv_Py_Index] = hv_Height_inputImage;
						}
					}
				}
				//*********************************************************************************
				GenRegionPolygon(&ho_Region_scale, hv_Py, hv_Px);
				FillUp(ho_Region_scale, &ho_RegionFillUp_scale);
				//RegionToMat(ho_RegionFillUp_scale, hv_Width_inputImage, hv_Height_inputImage);
				SelectObj(ho_SelectedRegions, &ho_SelectedRegions_todo, hv_Region_Index);
				// RegionToMat(ho_SelectedRegions_todo, hv_Width_inputImage, hv_Height_inputImage);
				Difference(ho_SelectedRegions_todo, ho_RegionFillUp_scale, &ho_RegionDifference);
				GetRegionPoints(ho_RegionDifference, &hv_Rows_clamp, &hv_Cols_clamp);
				GetGrayval(ho_GammaImage, hv_Rows_clamp, hv_Cols_clamp, &hv_Grayvals_clamp_gamma);//从gamma图扣
				GetGrayval(ho_Gray_inputImage, hv_Rows_clamp, hv_Cols_clamp, &hv_Grayvals_clamp_original);//从原图扣
				SetGrayval(ho_ImageSingle_gamma_clamp, hv_Rows_clamp, hv_Cols_clamp, hv_Grayvals_clamp_gamma);
				SetGrayval(ho_ImageSingle_original_clamp, hv_Rows_clamp, hv_Cols_clamp, hv_Grayvals_clamp_original);
				//ShowMat(ho_ImageSingle_clamp);
			}
		}

		clampingImg_gamma = HImageToMat(ho_ImageSingle_gamma_clamp);
		clampingImg_original = HImageToMat(ho_ImageSingle_original_clamp);
		//ShowMat(ho_ImageSingle_clamp);
		return 1;

	}
	// catch (Exception) 
	catch (HException& HDevExpDefaultException)
	{
		return 0;
	}



}




cv::Mat HalconHDR(cv::Mat inputImg) {


	// Local iconic variables
	HObject  ho_Image, ho_Image1, ho_Z1, ho_Image1_16;
	HObject  ho_Z_IS, ho_Is, ho_Ii, ho_IsS, ho_Is_Reel, ho_Ws;
	HObject  ho_W_Sum, ho_IX, ho_I_Laplace, ho_C, ho_S0, ho_S1;
	HObject  ho_IX_Gray, ho_IX_5, ho_IX_Sqrt, ho_IX_ET, ho_E;
	HObject  ho_C_W, ho_S_W, ho_E_W, ho_Wi, ho_Ws_Nor, ho_Is_LPyrs;
	HObject  ho_Ws_GPyrs, ho_IS_j, ho_IS_j255, ho_J_GPyr, ho_J_LPyr;
	HObject  ho_G_GPn, ho_J_GPp, ho_J_GPq, ho_J_Sub, ho_IJ_LPyr;
	HObject  ho_WS_j, ho_WJ_GPyr, ho_IW_LPyrs, ho_IW_LPk, ho_I_LPm;
	HObject  ho_W_GPm, ho_IW_LPm, ho_I_Fusion, ho_IF_Zoom, ho_I_Res;
	HObject  ho_ImageConverted, ho_ImageScaled, ho_ImageGauss;
	HObject  ho_ImageEmphasize, ho_ImageScaled2;

	// Local control variables
	HTuple  hv_Z_Num, hv_Width, hv_Height, hv_Type;
	HTuple  hv_N, hv_A, hv_i, hv_I_W, hv_I_H, hv_Is_Num, hv_Laplace_Mask;
	HTuple  hv_I_Width, hv_I_Height, hv_Sigma, hv_C_Pow, hv_S_Pow;
	HTuple  hv_E_Pow, hv_Pyr_Size, hv_n, hv_j, hv_G_Layer, hv_n_Index;
	HTuple  hv_p, hv_q_W, hv_q_H, hv_k, hv_m, hv_F_W, hv_F_H;
	HTuple  hv_Min, hv_Max, hv_Range, hv_S, hv_W, hv_H;

	//read_image (Image, ImageFiles[Index])
	//parse_filename (ImageFiles[Index], BaseName, Extension, Directory)

	ho_Image = MatToHImage(inputImg);

	Rgb1ToGray(ho_Image, &ho_Image1);


	/// 多帧叠加
	CountObj(ho_Image1, &hv_Z_Num);
	SelectObj(ho_Image1, &ho_Z1, 1);
	GetImageSize(ho_Z1, &hv_Width, &hv_Height);
	GetImageType(ho_Z1, &hv_Type);

	//for z := 1 to Z_Num by 1
	  //select_obj (Image1, Zi, z)
	  //convert_image_type (Zi, Zi_Real, 'int8')
	  //add_image (Z_Sum, Zi_Real, Z_Sum, 1.0, 0)
	//endfor
	//scale_image (Z_Sum, Z_Sum, 1.0/Z_Num, 0)
	//convert_image_type (Z_Sum, Z_I, Type)
	ConvertImageType(ho_Image1, &ho_Image1_16, hv_Type);
	ScaleImage(ho_Image1_16, &ho_Z_IS, 1, 0);
	//illuminate (Z_IS, Z_IS, 50, 50, 0.1)

	//emphasize (Z_IS, Z_IS, 7, 7, 1)





	/// 多曝光图像
	GaussFilter(ho_Z_IS, &ho_Z_IS, 3);
	Rgb1ToGray(ho_Z_IS, &ho_Image);
	hv_N = 5;
	hv_A = 1.5;
	GenEmptyObj(&ho_Is);
	{
		HTuple end_val35 = hv_N;
		HTuple step_val35 = 1;
		for (hv_i = 1; hv_i.Continue(end_val35, step_val35); hv_i += step_val35)
		{
			ScaleImage(ho_Image, &ho_Ii, 1 + ((hv_i - 1) * hv_A), 0);
			ConcatObj(ho_Is, ho_Ii, &ho_Is);
		}
	}
	CopyObj(ho_Is, &ho_IsS, 1, -1);





	/// 权重计算
	GetImageType(ho_Image, &hv_Type);
	GetImageSize(ho_Image, &hv_I_W, &hv_I_H);
	ConvertImageType(ho_IsS, &ho_Is_Reel, "real");
	if (0 != (hv_Type == HTuple("byte")))
	{
		ScaleImage(ho_Is_Reel, &ho_Is_Reel, 1.0 / 255, 0);
	}
	else if (0 != (hv_Type == HTuple("uint2")))
	{
		ScaleImage(ho_Is_Reel, &ho_Is_Reel, 1.0 / 65535, 0);
	}
	GenEmptyObj(&ho_Ws);
	GenImageConst(&ho_W_Sum, "real", hv_I_W, hv_I_H);
	//compose3 (W_Sum, W_Sum, W_Sum, W_Sum)
	CountObj(ho_IsS, &hv_Is_Num);
	{
		HTuple end_val58 = hv_Is_Num;
		HTuple step_val58 = 1;
		for (hv_i = 1; hv_i.Continue(end_val58, step_val58); hv_i += step_val58)
		{
			SelectObj(ho_Is_Reel, &ho_IX, hv_i);

			//// 对比度
			hv_Laplace_Mask.Clear();
			hv_Laplace_Mask[0] = 3;
			hv_Laplace_Mask[1] = 3;
			hv_Laplace_Mask[2] = 1;
			hv_Laplace_Mask[3] = 0;
			hv_Laplace_Mask[4] = 1;
			hv_Laplace_Mask[5] = 0;
			hv_Laplace_Mask[6] = 1;
			hv_Laplace_Mask[7] = -4;
			hv_Laplace_Mask[8] = 1;
			hv_Laplace_Mask[9] = 0;
			hv_Laplace_Mask[10] = 1;
			hv_Laplace_Mask[11] = 1;
			ConvolImage(ho_IX, &ho_I_Laplace, hv_Laplace_Mask, "mirrored");
			AbsImage(ho_I_Laplace, &ho_C);


			//// 饱和度
			//decompose3 (IX, IX_R, IX_G, IX_B)
			//scale_image (IX_R, IX_R3, 1/3.0, 0)
			//scale_image (IX_G, IX_G3, 1/3.0, 0)
			//scale_image (IX_B, IX_B3, 1/3.0, 0)
			GetImageSize(ho_IX, &hv_I_Width, &hv_I_Height);
			GenImageConst(&ho_S0, "real", hv_I_Width, hv_I_Height);
			ScaleImage(ho_S0, &ho_S1, 1, 1);


			//// 曝光度
			hv_Sigma = 0.2;
			Rgb1ToGray(ho_IX, &ho_IX_Gray);
			ScaleImage(ho_IX_Gray, &ho_IX_5, 1, -0.3);
			PowImage(ho_IX_5, &ho_IX_Sqrt, 2);
			ScaleImage(ho_IX_Sqrt, &ho_IX_ET, -0.3 / (hv_Sigma * hv_Sigma), 0);
			ExpImage(ho_IX_ET, &ho_E, "e");
			//compose3 (E, E, E, E)


			//// 权重
			hv_C_Pow = 1.0;
			hv_S_Pow = 1.0;
			hv_E_Pow = 1.0;
			PowImage(ho_C, &ho_C_W, hv_C_Pow);
			PowImage(ho_S1, &ho_S_W, hv_S_Pow);
			PowImage(ho_E, &ho_E_W, hv_E_Pow);
			//mult_image (C_W, E_W, Wi, 1, 0)
			CopyImage(ho_E_W, &ho_Wi);
			ConcatObj(ho_Ws, ho_Wi, &ho_Ws);


			//// 权重归一化
			AddImage(ho_W_Sum, ho_Wi, &ho_W_Sum, 1.0, 0);

		}
	}
	DivImage(ho_Ws, ho_W_Sum, &ho_Ws_Nor, 1.0, 0);


	/// 拉普拉斯金字塔
	//// 金字塔层数

	hv_Pyr_Size = 0.5;
	hv_n = 8;

	GenEmptyObj(&ho_Is_LPyrs);
	GenEmptyObj(&ho_Ws_GPyrs);
	//// 逐张金字塔
	{
		HTuple end_val123 = hv_N;
		HTuple step_val123 = 1;
		for (hv_j = 1; hv_j.Continue(end_val123, step_val123); hv_j += step_val123)
		{
			///// 图像序列金字塔
			SelectObj(ho_Is_Reel, &ho_IS_j, hv_j);

			//////高斯金字塔
			ScaleImage(ho_IS_j, &ho_IS_j255, 255.0, 0);
			GenGaussPyramid(ho_IS_j255, &ho_J_GPyr, "weighted", hv_Pyr_Size);
			CountObj(ho_J_GPyr, &hv_G_Layer);
			if (0 != (hv_n > hv_G_Layer))
			{
				hv_n = hv_G_Layer;
			}
			hv_n = hv_G_Layer;
			TupleGenSequence(1, hv_n, 1, &hv_n_Index);
			SelectObj(ho_J_GPyr, &ho_J_GPyr, hv_n_Index);

			//////拉普拉斯金字塔
			GenEmptyObj(&ho_J_LPyr);
			SelectObj(ho_J_GPyr, &ho_G_GPn, hv_n);
			ConcatObj(ho_J_LPyr, ho_G_GPn, &ho_J_LPyr);
			for (hv_p = hv_n; hv_p >= 2; hv_p += -1)
			{
				SelectObj(ho_J_GPyr, &ho_J_GPp, hv_p);
				SelectObj(ho_J_GPyr, &ho_J_GPq, hv_p - 1);
				GetImageSize(ho_J_GPq, &hv_q_W, &hv_q_H);
				ZoomImageSize(ho_J_GPp, &ho_J_GPp, hv_q_W, hv_q_H, "weighted");
				SubImage(ho_J_GPq, ho_J_GPp, &ho_J_Sub, 1, 0);
				ConcatObj(ho_J_LPyr, ho_J_Sub, &ho_J_LPyr);
			}
			CopyObj(ho_J_LPyr, &ho_IJ_LPyr, 1, -1);
			ConcatObj(ho_Is_LPyrs, ho_J_LPyr, &ho_Is_LPyrs);



			///// 权重金字塔
			SelectObj(ho_Ws_Nor, &ho_WS_j, hv_j);

			//////高斯金字塔
			GenGaussPyramid(ho_WS_j, &ho_J_GPyr, "weighted", hv_Pyr_Size);
			CountObj(ho_J_GPyr, &hv_G_Layer);
			if (0 != (hv_n > hv_G_Layer))
			{
				hv_n = hv_G_Layer;
			}
			TupleGenSequence(1, hv_n, 1, &hv_n_Index);
			TupleInverse(hv_n_Index, &hv_n_Index);
			SelectObj(ho_J_GPyr, &ho_J_GPyr, hv_n_Index);


			CopyObj(ho_J_GPyr, &ho_WJ_GPyr, 1, -1);
			ConcatObj(ho_Ws_GPyrs, ho_WJ_GPyr, &ho_Ws_GPyrs);

		}
	}



	//// 金字塔逐层融合
	GenEmptyObj(&ho_IW_LPyrs);
	{
		HTuple end_val178 = hv_n;
		HTuple step_val178 = 1;
		for (hv_k = 1; hv_k.Continue(end_val178, step_val178); hv_k += step_val178)
		{
			GenEmptyObj(&ho_IW_LPk);

			{
				HTuple end_val181 = hv_N;
				HTuple step_val181 = 1;
				for (hv_m = 1; hv_m.Continue(end_val181, step_val181); hv_m += step_val181)
				{
					SelectObj(ho_Is_LPyrs, &ho_I_LPm, hv_k + ((hv_m - 1) * hv_n));
					SelectObj(ho_Ws_GPyrs, &ho_W_GPm, hv_k + ((hv_m - 1) * hv_n));
					MultImage(ho_I_LPm, ho_W_GPm, &ho_IW_LPm, 1, 0);
					if (0 != (hv_m == 1))
					{
						CopyImage(ho_IW_LPm, &ho_IW_LPk);
					}
					else
					{
						AddImage(ho_IW_LPk, ho_IW_LPm, &ho_IW_LPk, 1, 0);
					}
				}
			}

			ConcatObj(ho_IW_LPyrs, ho_IW_LPk, &ho_IW_LPyrs);

		}
	}




	//// 金字塔拉普拉斯融合
	GenEmptyObj(&ho_I_Fusion);
	{
		HTuple end_val201 = hv_n;
		HTuple step_val201 = 1;
		for (hv_k = 1; hv_k.Continue(end_val201, step_val201); hv_k += step_val201)
		{
			SelectObj(ho_IW_LPyrs, &ho_IW_LPk, hv_k);
			if (0 != (hv_k == 1))
			{
				CopyImage(ho_IW_LPk, &ho_I_Fusion);
			}
			else
			{
				GetImageSize(ho_IW_LPk, &hv_F_W, &hv_F_H);
				ZoomImageSize(ho_I_Fusion, &ho_IF_Zoom, hv_F_W, hv_F_H, "weighted");
				AddImage(ho_IF_Zoom, ho_IW_LPk, &ho_I_Fusion, 1, 0);
			}
		}
	}


	/// 转8位图
	MinMaxGray(ho_I_Fusion, ho_I_Fusion, 0, &hv_Min, &hv_Max, &hv_Range);
	hv_S = 255 / (hv_Max - hv_Min);
	hv_A = (-hv_S) * hv_Min;
	ScaleImage(ho_I_Fusion, &ho_I_Res, hv_S, hv_A);
	ConvertImageType(ho_I_Res, &ho_ImageConverted, "byte");
	ScaleImage(ho_ImageConverted, &ho_ImageScaled, 1.0, 0);
	GetImageSize(ho_ImageScaled, &hv_W, &hv_H);

	GaussFilter(ho_ImageScaled, &ho_ImageGauss, 3);
	Emphasize(ho_ImageGauss, &ho_ImageEmphasize, 20, 20, 1);
	GaussFilter(ho_ImageEmphasize, &ho_ImageGauss, 3);
	ScaleImage(ho_ImageGauss, &ho_ImageScaled2, 1.3, 10);

	cv::Mat retMat = HImageToMat(ho_ImageScaled2);
	return  retMat;
}

cv::Mat Pow(cv::Mat inputImg, float alpha) {
	//
	inputImg.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 0);
	inputImg.at<cv::Vec3b>(0, 1) = cv::Vec3b(255, 255, 255);

	// Local iconic variables
	HObject  ho_Image, ho_PowImage, ho_ImageScaleMax;
	HObject  ho_ImageEmphasize;

	ho_Image = MatToHImage(inputImg);

	PowImage(ho_Image, &ho_PowImage, alpha);
	ScaleImageMax(ho_PowImage, &ho_ImageScaleMax);

	cv::Mat ret = HImageToMat(ho_ImageScaleMax);
	return ret;
}

cv::Mat PowEmp(cv::Mat inputImg, float alpha, float beta, int mask) {
	//
	inputImg.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 0);
	inputImg.at<cv::Vec3b>(0, 1) = cv::Vec3b(255, 255, 255);

	// Local iconic variables
	HObject  ho_Image, ho_PowImage, ho_ImageScaleMax;
	HObject  ho_ImageEmphasize;

	ho_Image = MatToHImage(inputImg);

	PowImage(ho_Image, &ho_PowImage, alpha);
	ScaleImageMax(ho_PowImage, &ho_ImageScaleMax);
	Emphasize(ho_ImageScaleMax, &ho_ImageEmphasize, mask, mask, beta);

	cv::Mat ret = HImageToMat(ho_ImageEmphasize);
	return ret;
}

//cv::Mat PowEmp(cv::Mat inputImg) {
//
//
//	// Local iconic variables
//	HObject  ho_Image, ho_GammaImage, ho_ImageScaled;
//	HObject  ho_ImageEmphasize;
//
//	ho_Image = MatToHImage(inputImg);
//
//	GammaImage(ho_Image, &ho_GammaImage, 0.316667, 0.055, 0.0031308, 255, "true");
//	ScaleImage(ho_GammaImage, &ho_ImageScaled, 1, 20);
//	Emphasize(ho_ImageScaled, &ho_ImageEmphasize, 50, 50, 3);
//
//	cv::Mat ret = HImageToMat(ho_ImageEmphasize);
//	return ret;
//}

cv::Mat calcuGammaTrans(cv::Mat srcImage, double gamma) {
	cv::Mat dstImage;
	unsigned char lut[256];
	for (int i = 0; i < 256; i++)
	{
		lut[i] = cv::saturate_cast<uchar>(pow((float)i / 255.0, gamma) * 255.0f);
	}
	dstImage = srcImage.clone();
	int channels = srcImage.channels();
	switch (channels)
	{
	case 1:
	{
		cv::MatIterator_<uchar> it = dstImage.begin<uchar>();
		cv::MatIterator_<uchar> end = dstImage.end<uchar>();
		while (it != end)
		{
			*it = lut[(*it)];
			it++;
		}
		break;
	}
	case 3:
	{
		cv::MatIterator_<cv::Vec3b> it = dstImage.begin<cv::Vec3b>();
		cv::MatIterator_<cv::Vec3b> end = dstImage.end<cv::Vec3b>();
		while (it != end)
		{
			(*it)[0] = lut[(*it)[0]];
			(*it)[1] = lut[(*it)[1]];
			(*it)[2] = lut[(*it)[2]];
			it++;
		}
		break;
	}
	case 4:
	{
		cv::MatIterator_<cv::Vec4b> it = dstImage.begin<cv::Vec4b>();
		cv::MatIterator_<cv::Vec4b> end = dstImage.end<cv::Vec4b>();
		while (it != end)
		{
			(*it)[0] = lut[(*it)[0]];
			(*it)[1] = lut[(*it)[1]];
			(*it)[2] = lut[(*it)[2]];
			it++;
		}
		break;
	}
	default:
		break;
	}
	return dstImage;
}

void addContrast(cv::Mat& srcImg) {
	cv::Mat lookUpTable(1, 256, CV_8U);
	double temp = pow(1.1, 2);
	uchar* p = lookUpTable.data;
	for (int i = 0; i < 256; ++i)
		p[i] = cv::saturate_cast<uchar>(i * temp);
	LUT(srcImg, lookUpTable, srcImg);
}

cv::Mat putMatToBackground(cv::Mat img) {
	cv::Mat saveObjMat(1024, 1024, img.type(), cv::Scalar(255, 255, 255));
	cv::Rect locationRect;
	locationRect.x = saveObjMat.cols / 2 - img.cols / 2;
	locationRect.y = saveObjMat.rows / 2 - img.rows / 2;
	locationRect.x = locationRect.x < 0 ? 0 : locationRect.x;
	locationRect.y = locationRect.y < 0 ? 0 : locationRect.y;

	int endx, endy;
	endx = locationRect.x + img.cols;
	endy = locationRect.y + img.rows;
	endx = endx > 1024 ? 1024 : endx;
	endy = endy > 1024 ? 1024 : endy;

	locationRect.width = endx - locationRect.x;
	locationRect.height = endy - locationRect.y;

	cv::Mat scaleObjMat;
	cv::resize(img, scaleObjMat, cv::Size(locationRect.width, locationRect.height));

	scaleObjMat.copyTo(saveObjMat(locationRect));
	return saveObjMat;
}

cv::Mat ImageEnhance(cv::Mat& src) {
	HObject  ho_Image = MatToHImage(src);
	HObject  ho_ImageScaleMax, ho_ImageScaled, ho_ImageEmphasize, ho_ImageEmphasize2;
	ScaleImage(ho_Image, &ho_ImageScaled, 1.05, 10);
	Emphasize(ho_ImageScaled, &ho_ImageEmphasize, 50, 50, 0.8);
	Emphasize(ho_ImageEmphasize, &ho_ImageEmphasize2, 50, 50, 1);
	cv::Mat dst = HImageToMat(ho_ImageEmphasize2);
	return dst;
}

cv::Mat emphasize(cv::Mat& input, int MaskWidth, float Factor)
{
	//公式res := round((orig - mean) * Factor) + orig
	//等价于在MaskHeight、MaskWidth的空间内中心化后增加方差
	cv::Mat mean, temp;

	//等价于求指定范围窗口内的均值
	cv::blur(input, mean, cv::Size(MaskWidth, MaskWidth));
	temp.create(input.size(), input.type());

	if (input.type() == CV_8UC1)
	{
		for (int i = 0; i < input.rows; i++)
		{
			const uchar* rptr = input.ptr<uchar>(i);
			uchar* mptr = mean.ptr<uchar>(i);
			uchar* optr = temp.ptr<uchar>(i);
			for (int j = 0; j < input.cols; j++)
			{
				//float temp = round((rptr[j] - mptr[j]) * Factor) + rptr[j] * 1.0f;
				//std::cout << temp << "	";
				optr[j] = cv::saturate_cast<uchar>(round((rptr[j] - mptr[j]) * Factor) + rptr[j] * 1.0f);
			}
			//std::cout<<std::endl;
		}
	}
	else if (input.type() == CV_8UC3)
	{
		for (int i = 0; i < input.rows; i++)
		{
			const uchar* rptr = input.ptr<uchar>(i);
			uchar* mptr = mean.ptr<uchar>(i);
			uchar* optr = temp.ptr<uchar>(i);
			for (int j = 0; j < input.cols; j++)
			{
				//饱和转换 小于0的值会被置为0 大于255的值会被置为255
				optr[j * 3] = cv::saturate_cast<uchar>(round((rptr[j * 3] - mptr[j * 3]) * Factor) + rptr[j * 3] * 1.0f);
				optr[j * 3 + 1] = cv::saturate_cast<uchar>(round((rptr[j * 3 + 1] - mptr[j * 3 + 1]) * Factor) + rptr[j * 3 + 1] * 1.0f);
				optr[j * 3 + 2] = cv::saturate_cast<uchar>(round((rptr[j * 3 + 2] - mptr[j * 3 + 2]) * Factor) + rptr[j * 3 + 2] * 1.0f);
			}
		}
	}
	return temp;
}


cv::Mat AGCIE(cv::Mat& src) {
	cv::Mat dst;
	int rows = src.rows;
	int cols = src.cols;
	int channels = src.channels();
	int total_pixels = rows * cols;

	cv::Mat L;
	cv::Mat HSV;
	std::vector<cv::Mat> HSV_channels;
	if (channels == 1) {
		L = src.clone();
	}
	else {
		cv::cvtColor(src, HSV, CV_BGR2HSV_FULL);
		cv::split(HSV, HSV_channels);
		L = HSV_channels[2];
	}

	cv::Mat L_norm;
	L.convertTo(L_norm, CV_64F, 1.0 / 255.0);

	cv::Mat mean, stddev;
	cv::meanStdDev(L_norm, mean, stddev);
	double mu = mean.at<double>(0, 0);
	double sigma = stddev.at<double>(0, 0);

	double tau = 3.0;

	double gamma;
	if (4 * sigma <= 1.0 / tau) { // low-contrast
		gamma = -std::log2(sigma);
	}
	else { // high-contrast
		gamma = std::exp((1.0 - mu - sigma) / 2.0);
	}

	std::vector<double> table_double(256, 0);
	for (int i = 1; i < 256; i++) {
		table_double[i] = i / 255.0;
	}

	if (mu >= 0.5) { // bright image
		for (int i = 1; i < 256; i++) {
			table_double[i] = std::pow(table_double[i], gamma);
		}
	}
	else { // dark image
		double mu_gamma = std::pow(mu, gamma);
		for (int i = 1; i < 256; i++) {
			double in_gamma = std::pow(table_double[i], gamma);;
			table_double[i] = in_gamma / (in_gamma + (1.0 - in_gamma) * mu_gamma);
		}
	}

	std::vector<uchar> table_uchar(256, 0);
	for (int i = 1; i < 256; i++) {
		table_uchar[i] = cv::saturate_cast<uchar>(255.0 * table_double[i]);
	}

	cv::LUT(L, table_uchar, L);

	if (channels == 1) {
		dst = L.clone();
	}
	else {
		cv::merge(HSV_channels, dst);
		cv::cvtColor(dst, dst, CV_HSV2BGR_FULL);
	}

	return dst;
}


cv::Mat AGCWD(cv::Mat& src, double alpha) {
	cv::Mat dst;
	int rows = src.rows;
	int cols = src.cols;
	int channels = src.channels();
	int total_pixels = rows * cols;

	cv::Mat L;
	cv::Mat HSV;
	std::vector<cv::Mat> HSV_channels;
	if (channels == 1) {
		L = src.clone();
	}
	else {
		cv::cvtColor(src, HSV, CV_BGR2HSV_FULL);
		cv::split(HSV, HSV_channels);
		L = HSV_channels[2];
	}

	int histsize = 256;
	float range[] = { 0,256 };
	const float* histRanges = { range };
	int bins = 256;
	cv::Mat hist;
	calcHist(&L, 1, 0, cv::Mat(), hist, 1, &histsize, &histRanges, true, false);

	double total_pixels_inv = 1.0 / total_pixels;
	cv::Mat PDF = cv::Mat::zeros(256, 1, CV_64F);
	for (int i = 0; i < 256; i++) {
		PDF.at<double>(i) = hist.at<float>(i) * total_pixels_inv;
	}

	double pdf_min, pdf_max;
	cv::minMaxLoc(PDF, &pdf_min, &pdf_max);
	cv::Mat PDF_w = PDF.clone();
	for (int i = 0; i < 256; i++) {
		PDF_w.at<double>(i) = pdf_max * std::pow((PDF_w.at<double>(i) - pdf_min) / (pdf_max - pdf_min), alpha);
	}

	cv::Mat CDF_w = PDF_w.clone();
	double culsum = 0;
	for (int i = 0; i < 256; i++) {
		culsum += PDF_w.at<double>(i);
		CDF_w.at<double>(i) = culsum;
	}
	CDF_w /= culsum;

	std::vector<uchar> table(256, 0);
	for (int i = 1; i < 256; i++) {
		table[i] = cv::saturate_cast<uchar>(255.0 * std::pow(i / 255.0, 1 - CDF_w.at<double>(i)));
	}

	cv::LUT(L, table, L);

	if (channels == 1) {
		dst = L.clone();
	}
	else {
		cv::merge(HSV_channels, dst);
		cv::cvtColor(dst, dst, CV_HSV2BGR_FULL);
	}

	return dst;

}



std::vector<cv::Rect> calcuObjectLocation(cv::Mat edgeImg) {
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	findContours(edgeImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point());

	std::vector<cv::Rect> rects;
	for (int i = 0; i < contours.size(); i++) {
		cv::Point2f vertex[4];
		cv::Rect boundingRect = cv::boundingRect(cv::Mat(contours[i]));
		rects.push_back(boundingRect);
	}

	return rects;
}


void hardDetect(cv::Mat& image, int threshVal, int boundingRectMinVal, std::vector<ObjsRect>& foreignRects)
{
	if (image.channels() == 3)
	{
		cvtColor(image, image, cv::COLOR_BGR2GRAY);
	}

	cv::Mat thresh;
	cv::threshold(image, thresh, threshVal, 255, cv::THRESH_BINARY_INV);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(thresh, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	//std::vector<cv::Rect> rects;
	for (int i = 0; i < contours.size(); i++) {
		ObjsRect foreignRect;
		cv::Rect boudingRect = cv::boundingRect(cv::Mat(contours[i]));
		if (boudingRect.width >= boundingRectMinVal || boudingRect.height >= boundingRectMinVal) {
			foreignRect.objectRect = boudingRect;
			foreignRect.class_id = 0;
			foreignRects.push_back(foreignRect);
		}

	}
}



