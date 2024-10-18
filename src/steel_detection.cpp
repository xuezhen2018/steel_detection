#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "steel_detection/steeldetect.h"
#include "steel_detection/yolov8_onnx.h"


#include <vector>
#include "steel_detection/gnuplot_i.hpp"


using namespace std;
using namespace cv;
using namespace dnn;

ros::Time img_time, depth_time;
bool img_received = false;
cv_bridge::CvImagePtr cv_ptr;
Mat img;
Mat depth;
// std_msgs::Header img_header;


int findMajority(const std::vector<uint16_t>& nums) {
    std::unordered_map<uint16_t, int> count; // 哈希表用于记录元素出现次数

    uint16_t majority = 1; // 存储众数
    int maxCount = 0; // 记录众数出现的最大次数

    for (int num : nums) {
        count[num]++;
        if (count[num] > maxCount) {
            maxCount = count[num];
            majority = num;
        }
    }

    return majority;
}

void real_img_callback(const sensor_msgs::Image &real_img) {
    try {
        cv_ptr = cv_bridge::toCvCopy(real_img, sensor_msgs::image_encodings::BGR8);
        img_time = real_img.header.stamp;
        // img_header = real_img.header;
        img_received = true;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    img = cv_ptr->image;
}

void real_depth_callback(const sensor_msgs::Image &real_depth) {
    try {
        cv_ptr = cv_bridge::toCvCopy(real_depth, sensor_msgs::image_encodings::TYPE_16UC1);
        depth_time = real_depth.header.stamp;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    depth = cv_ptr->image;
}

cv::Mat get_img_depth(Mat& img, Mat& depth){

    cv::Mat new_depth = depth.clone();

    vector<uint16_t> flat_depth;

    for (int i = 0; i < new_depth.rows; i++) {
        for (int j = 0; j < new_depth.cols; j++) {
            if (new_depth.at<uint16_t>(i, j) > 1000) {
                new_depth.at<uint16_t>(i, j) = 0;
            }
            if (new_depth.at<uint16_t>(i, j) > 100){
                flat_depth.push_back(new_depth.at<uint16_t>(i, j));
            }
        }
    }

    // cout << "flat_depth.size: " << flat_depth.size() << endl;

    // cv::Mat depth_img_100 = depth.clone();
    // depth_img_100.setTo(0, depth <= 100);
    // cv::Mat flat_depth = depth_img_100.reshape(1, depth_img_100.total());
    int th = findMajority(flat_depth);
    // cout << "th: " << th << endl;
    for (int i = 0; i < new_depth.rows; i++) {
        for (int j = 0; j < new_depth.cols; j++) {
            if (new_depth.at<uint16_t>(i, j) < th - 60) {
                new_depth.at<uint16_t>(i, j) = 1;
            } else {
                new_depth.at<uint16_t>(i, j) = 0;
            }
        }
    }

    // cv::Mat depth_img_repeated = cv::repeat(depth, 3, 3);

    // 使用cv::normalize函数将16位深度的图像转换为8位深度的图像
    cv::convertScaleAbs(new_depth, new_depth);

    cv::cvtColor(new_depth, new_depth, cv::COLOR_GRAY2BGR);
    cv::Mat res_img = img.mul(new_depth);
    return res_img;

}

void img_detection(Yolov8Onnx& model, Mat& img, Mat& res_img, ros::Publisher& obj_publisher, string& out_path) {
    // generate color
    vector<Scalar> color;
    color.push_back(Scalar(0, 255, 0));
    color.push_back(Scalar(0, 0, 255));

    vector<OutputSeg> result;

    // define mask & draw on the image
    int rect_x1 = 100;
    int rect_x2 = 848-100;
    int rect_y1 = 50;
    int rect_y2 = 480-50;

    Rect rect;
    rect.x = rect_x1;
    rect.y = rect_y1;
    rect.width = rect_x2 - rect_x1;
    rect.height = rect_y2 - rect_y1;
    rectangle(img, rect, (0,0,0), 2, 8);

    // detect and filter the results
    if (model.OnnxDetect(res_img, result)) {

        // define publisher
        steel_detection::steeldetect detect_msg;
        detect_msg.header.stamp = img_time;
        detect_msg.header.frame_id = "real_sense_link";

        // camera parameters
        double fx = 600.0;
        double fy = 600.0;

        // filter the result according to the mask
        vector<OutputSeg> result_res;
        for (int i=0; i<result.size(); i++) {
            if (result[i].box.x<rect_x1) {
                continue;
            } else if ((result[i].box.x+result[i].box.width)>rect_x2) {
                continue;
            } else if (result[i].box.y<rect_y1) {
                continue;
            } else if ((result[i].box.y+result[i].box.height)>rect_y2) {
                continue;
            } else {
                result_res.push_back(result[i]);

                detect_msg.is_bundled.push_back(result[i].confidence);
                
                

                // obtain the depth
                Mat rect = depth(result[i].box);
                uint16_t dp_min = 10000;
                for (int n=0; n<rect.rows; n++) {
                    for (int j=0; j<rect.cols; j++) {
                        if (rect.at<uint16_t>(n,j) < 100) {
                            continue;
                        } else {
                            dp_min = min(dp_min, rect.at<uint16_t>(n,j));
 
                        }
                    }
                }


                // 生成文件名
                // std::ostringstream filenameStream;
                // filenameStream << "./src/steel_detection/depth_output/" << img_time << result[i].box.x << "+" << result[i].box.y << ".txt";
                // std::string filename = filenameStream.str();
                
                // 打开文件以追加的方式写入数据
                // std::ofstream outfile(filename, std::ios_base::app);

                // for (int n = 0; n < rect.rows; n++) {
                //     for (int j = 0; j < rect.cols; j++) {
                //         if (rect.at<uint16_t>(n, j) >= 100) {
                //             dp_min = std::min(dp_min, rect.at<uint16_t>(n, j));

                //            // 将 n, j 和深度值写入文件
                //             outfile << "n: " << n << ", j: " << j << ", Depth: " << rect.at<uint16_t>(n, j) << "\n";
                //         }
                //     }
                // }

                // // 关闭文件
                // outfile.close();
  
                // calculate the position
                float Z = float(dp_min) / 1000;
                float X = ((result[i].box.x + result[i].box.width/2) - img.cols/2) * Z / fx;
                // std::cout << result[i].box.x << ", " << result[i].box.width/2 << ", " << img.cols/2 << ", " << Z << ", " << fx << std::endl;
                float Y = ((result[i].box.y + result[i].box.height/2) - img.rows/2) * Z / fy;

                geometry_msgs::Pose pose;
                pose.position.x = X;
                pose.position.y = Y;
                pose.position.z = Z;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;
                pose.orientation.w = 1.0;
                detect_msg.poses.push_back(pose);

                // TODO
                detect_msg.is_qualified.push_back(0.5);

                // TODO
                detect_msg.direction_angle = 45.0;
            }
        }

        detect_msg.poinsize = result_res.size();
        cout << "Image processed! Number of rectangles: " << result_res.size() << endl;

        // draw the results to the images
        DrawPred(img, result_res, model._className, color, img_time, out_path);

        // save the results into output directory
        std::string file_name = out_path + std::to_string(img_time.sec)+".png";
        imwrite(file_name, img);

        // publish the message
        obj_publisher.publish(detect_msg);
    } else {
        cout << "Detect Failed!" << endl;
    }
}


int main(int argc, char** argv) {

    // load the detection model
    string model_path = "./src/steel_detection/models/best.onnx";
    Yolov8Onnx task_detect_model;
    bool model_state = task_detect_model.ReadModel(model_path, false);
    if (!model_state) {
        cout << "model not loaded! " << endl;
        return -1;
    }

    // make output dir
    string out_path = "./src/steel_detection/output/";
    fstream _file;
    string commond;
    _file.open(out_path, ios::in);
    if (! _file) {
        cout << "[" << out_path << "]" << " not created. Now create it." << endl;
        commond = "mkdir -p "+out_path;
        system(commond.c_str());
    } else {
        cout << "[" << out_path << "]"  << " already exists. Now empty it." << endl;
        commond = "cd "+out_path+ " && rm *.png && cd ..";
        system(commond.c_str());
    }

    ros::init(argc, argv, "steel_detect");
    ros::NodeHandle nh("~");

    ros::Subscriber real_img_sub = nh.subscribe("/camera/color/image_raw", 10, &real_img_callback);
    ros::Subscriber real_depth_sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 10, &real_depth_callback);

    ros::Publisher img_obj_pub = nh.advertise<steel_detection::steeldetect>("/steel_detection", 10);

    ros::Rate loop_rate(10);

    bool robot_running_state = true;
    
    while (robot_running_state) {
        img_received = false;
        ros::spinOnce();

        

        if (img_received && abs(img_time.toSec()-depth_time.toSec()) < 1e-6 ) {
        // if (img_received) {
            cv::Mat res_img = get_img_depth(img, depth);
            img_detection(task_detect_model, img, res_img, img_obj_pub, out_path);
        }

        loop_rate.sleep();
    }


    return 0;
}