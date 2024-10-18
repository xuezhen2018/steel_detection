#pragma once
#include "steel_detection/yolov8_utils.h"
using namespace cv;
using namespace std;

void LetterBox(const cv::Mat& image, cv::Mat& outImage, cv::Vec4d& params, const cv::Size& newShape,
	bool autoShape, bool scaleFill, bool scaleUp, int stride, const cv::Scalar& color)
{
	if (false) {
		int maxLen = MAX(image.rows, image.cols);
		outImage = Mat::zeros(Size(maxLen, maxLen), CV_8UC3);
		image.copyTo(outImage(Rect(0, 0, image.cols, image.rows)));
		params[0] = 1;
		params[1] = 1;
		params[3] = 0;
		params[2] = 0;
	}

	cv::Size shape = image.size();
	float r = std::min((float)newShape.height / (float)shape.height,
		(float)newShape.width / (float)shape.width);
	if (!scaleUp)
		r = std::min(r, 1.0f);

	float ratio[2]{ r, r };
	int new_un_pad[2] = { (int)std::round((float)shape.width * r),(int)std::round((float)shape.height * r) };

	auto dw = (float)(newShape.width - new_un_pad[0]);
	auto dh = (float)(newShape.height - new_un_pad[1]);

	if (autoShape)
	{
		dw = (float)((int)dw % stride);
		dh = (float)((int)dh % stride);
	}
	else if (scaleFill)
	{
		dw = 0.0f;
		dh = 0.0f;
		new_un_pad[0] = newShape.width;
		new_un_pad[1] = newShape.height;
		ratio[0] = (float)newShape.width / (float)shape.width;
		ratio[1] = (float)newShape.height / (float)shape.height;
	}

	dw /= 2.0f;
	dh /= 2.0f;

	if (shape.width != new_un_pad[0] && shape.height != new_un_pad[1])
	{
		cv::resize(image, outImage, cv::Size(new_un_pad[0], new_un_pad[1]));
	}
	else {
		outImage = image.clone();
	}

	int top = int(std::round(dh - 0.1f));
	int bottom = int(std::round(dh + 0.1f));
	int left = int(std::round(dw - 0.1f));
	int right = int(std::round(dw + 0.1f));
	params[0] = ratio[0];
	params[1] = ratio[1];
	params[2] = left;
	params[3] = top;
	cv::copyMakeBorder(outImage, outImage, top, bottom, left, right, cv::BORDER_CONSTANT, color);
}

bool compareValue_x(const cv::Point& pt1, const cv::Point& pt2)
{
    return pt1.x < pt2.x;
}

bool compareValue_y(const cv::Point& pt1, const cv::Point& pt2)
{
    return pt1.y < pt2.y;
}

void DrawPred(Mat& img, vector<OutputSeg> result, std::vector<std::string> classNames, vector<Scalar> color, ros::Time img_time, std::string out_path) {
//	Mat mask = img.clone();
    if(!result.empty()){
        std::vector<cv::Point> xxyy;
        for (int i = 0; i < result.size(); i++) {

            if (result[i].box.width > 25 ){
             rectangle(img, result[i].box, color[result[i].id], 2, 8);   
            }
            
            // result[i].box.width = 45;
            // result[i].box.height = 45;
            // rectangle(img, result[i].box, color[result[i].id], 2, 8);
            int central_x = result[i].box.x + result[i].box.width/2;
            int central_y = result[i].box.y + result[i].box.height/2;
            xxyy.push_back(cv::Point(central_x,central_y));
            int font_face = cv::FONT_HERSHEY_SIMPLEX;
            double font_scale = 0.5;
            int thickness = 1;
            // classNames[result[i].id] + ":" + 
            string label = to_string(result[i].confidence);
            // putText(img, label, cv::Point(central_x,central_y), font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);
            
        }

        xxyy.push_back(cv::Point(1280, 1280));

        std::sort(xxyy.begin(), xxyy.end(), compareValue_x);
        std::vector<cv::Point> line_arr_x;
        for (int index = 0; index < xxyy.size(); index++){
            if(xxyy[index + 1].x - xxyy[index].x < 40){
                line_arr_x.push_back(xxyy[index]);
            }else{
                if(line_arr_x.size() >= 1){
                    line_arr_x.push_back(xxyy[index]);
                    cv::Vec4f line_para;
                    cv::fitLine(line_arr_x, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
                    cv::Point point0;
                    point0.x = line_para[2];
                    point0.y = line_para[3];

                    double k = line_para[1] / line_para[0];

                    //计算直线的端点(y = k(x - x0) + y0)
                    cv::Point point1, point2;
                    point1.x = 0;
                    point1.y = k * (0 - point0.x) + point0.y;
                    point2.x = 1280;
                    point2.y = k * (1280 - point0.x) + point0.y;

                    cv::line(img, point1, point2, cv::Scalar(255, 0, 0), 1, 8, 0);

                }
                line_arr_x.clear();
            }
        }


        std::sort(xxyy.begin(), xxyy.end(), compareValue_y);
        std::vector<cv::Point> line_arr_y;
        for (int index = 0; index < xxyy.size(); index++){
            if(xxyy[index + 1].y - xxyy[index].y < 30){
                line_arr_y.push_back(xxyy[index]);
            }else{
                if(line_arr_y.size() >= 1){
                    line_arr_y.push_back(xxyy[index]);
                    cv::Vec4f line_para;
                    cv::fitLine(line_arr_y, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
                    cv::Point point0;
                    point0.x = line_para[2];
                    point0.y = line_para[3];

                    double k = line_para[1] / line_para[0];

                    //计算直线的端点(y = k(x - x0) + y0)
                    cv::Point point1, point2;
                    point1.x = 0;
                    point1.y = k * (0 - point0.x) + point0.y;
                    point2.x = 1280;
                    point2.y = k * (1280 - point0.x) + point0.y;

                    cv::line(img, point1, point2, cv::Scalar(255, 0, 0), 1, 8, 0);

                }
                line_arr_y.clear();
            }
        }

    }

	// imshow("1", img);
    // std::string file_name = out_path + std::to_string(img_time.sec)+".png";
	// imwrite(file_name, img);
	// waitKey();
	//destroyAllWindows();

}
