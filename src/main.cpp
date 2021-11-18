#include<iostream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/Time.h>
#include<std_msgs/Header.h>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/imgcodecs/imgcodecs.hpp"
#include <opencv2/highgui/highgui_c.h>
#include "neural_network_detector/NeuralNetworkFeedback.h"

using namespace std;

// function to read a csv file
void read_csv(const string &filename, vector <cv::Mat> &images_c1, vector <cv::Mat> &images_c2,
              vector<vector<int>> &bb_c1, vector<vector<int>> &bb_c2, char separator = ',') {

    int bb_data_length = 4;
    int intr_data_length = 4;
    int extr_data_length = 4;

    // temp
    vector<int> bb_temp;
    vector<float> intr_temp;
    vector<float> extr_temp;

    std::ifstream file(filename.c_str(), ifstream::in);
    if (!file) {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }

    string line, path, bb, intr, extr;
    while (getline(file, line)) {
        stringstream liness(line);

        getline(liness, path, separator);
        images_c1.push_back(cv::imread(path, cv::IMREAD_COLOR));

        getline(liness, path, separator);
        images_c2.push_back(cv::imread(path, cv::IMREAD_COLOR));

        // bounding_box data [max_y, min_y, max_x, max_x] // todo check if this is correct
        for (auto i = 0; i < bb_data_length; i++) {
            getline(liness, bb, separator);
            bb_temp.push_back(atoi(bb.c_str()));
        }
        bb_c1.push_back(bb_temp);
        bb_temp.clear();
        for (auto i = 0; i < bb_data_length; i++) {
            getline(liness, bb, separator);
            bb_temp.push_back(atoi(bb.c_str()));
        }
        bb_c2.push_back(bb_temp);
        bb_temp.clear();

        // intrinsic data
        for (auto i = 0; i < intr_data_length; i++) {
            getline(liness, intr, separator);
            intr_temp.push_back(atoi(intr.c_str()));
        }
        intr_temp.clear();
        for (auto i = 0; i < intr_data_length; i++) {
            getline(liness, intr, separator);
            intr_temp.push_back(atoi(intr.c_str()));
        }
        intr_temp.clear();
        // todo convert to matrix if necessary

        // extrinsic data
        for (auto i = 0; i < extr_data_length; i++) {
            getline(liness, extr, separator);
            extr_temp.push_back(atoi(extr.c_str()));
        }
        extr_temp.clear();
        for (auto i = 0; i < extr_data_length; i++) {
            getline(liness, extr, separator);
            extr_temp.push_back(atoi(extr.c_str()));
        }
        extr_temp.clear();
        // todo convert to matrix if necessary

    }
}

neural_network_detector::NeuralNetworkFeedback toInfoMsg(vector<int> bb, ros::Time time, int id = 1, int seq = 0) {
    // todo create header
    neural_network_detector::NeuralNetworkFeedback info;
    info.header.stamp = time;
    info.header.frame_id = "drone_"+to_string(id);
    info.header.seq = seq;

    // todo adapt to msg type
    int s;
    info.xcenter = bb[0];
    info.ycenter = bb[1];
    info.ymax = bb[2];
    info.ymin = bb[3];

    return info;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "BagFromImages");

    if (argc != 5) {
        cerr
                << "Usage: rosrun BagFromImages BagFromImages <path to csv> <frequency> <path to output bag>"
                << endl;
        return 0;
    }

    ros::start();

    // Vector of paths to image
    string filename = argv[1];

    // Frequency
    double freq = atof(argv[2]);

    // Output bag
    string output_bag_path = argv[3];
    rosbag::Bag bag_out_d1(output_bag_path + "_d1.bag", rosbag::bagmode::Write);
    rosbag::Bag bag_out_d2(output_bag_path + "_d2.bag", rosbag::bagmode::Write);

    ros::Time t = ros::Time::now();

    const float T = 1.0f / freq;
    ros::Duration d(T);

    vector <cv::Mat> images_c1;
    vector <cv::Mat> images_c2;
    vector<vector<int>> bb_c1;
    vector<vector<int>> bb_c2;
    read_csv(filename, images_c1, images_c2, bb_c1, bb_c2);

    for (size_t i = 0; i < images_c1.size(); i++) {
        if (!ros::ok())
            break;

        cv_bridge::CvImage cvImage;
        // write info to bag 1
        cvImage.image = images_c1[i];
        cvImage.encoding = sensor_msgs::image_encodings::RGB8;
        cvImage.header.stamp = t;
        bag_out_d1.write("/drone_1/camera/image_raw", ros::Time(t), cvImage.toImageMsg());
        bag_out_d1.write("/drone_1/info", ros::Time(t), toInfoMsg(bb_c1[i], ros::Time(t), 1));

        // write info to bag 2
        bag_out_d2.write("/drone_2/camera/image_raw", ros::Time(t), cvImage.toImageMsg());
        bag_out_d2.write("/drone_2/info", ros::Time(t), toInfoMsg(bb_c1[i], ros::Time(t), 2));

        t += d;
        cout << i << " th frame " << endl;
    }

    bag_out_d1.close();
    bag_out_d2.close();

    ros::shutdown();

    return 0;
}
