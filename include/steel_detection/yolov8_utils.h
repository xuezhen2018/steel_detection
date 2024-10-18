#pragma once
#include<iostream>
#include <numeric>
#include <ros/ros.h>
#include<opencv2/opencv.hpp>

#define YOLO_P6 false //
#define ORT_OLD_VISON 12  //
struct OutputSeg {
	int id;             //
	float confidence;   //
	cv::Rect box;       //
//	cv::Mat boxMask;       
};

void DrawPred(cv::Mat& img, std::vector<OutputSeg> result, std::vector<std::string> classNames, std::vector<cv::Scalar> color, ros::Time img_time, std::string out_path);
void LetterBox(const cv::Mat& image, cv::Mat& outImage,
	cv::Vec4d& params, //[ratio_x,ratio_y,dw,dh]
	const cv::Size& newShape = cv::Size(640, 640),
	bool autoShape = false,
	bool scaleFill = false,
	bool scaleUp = true,
	int stride = 32,
	const cv::Scalar& color = cv::Scalar(114, 114, 114));




