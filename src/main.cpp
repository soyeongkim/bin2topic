#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dirent.h>
#include <algorithm>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

// Function to get sorted list of bin files in the directory
std::vector<std::string> getSortedBinFiles(const std::string& directory) {
    std::vector<std::string> files;
    struct dirent* entry;
    DIR* dp = opendir(directory.c_str());
    
    if (dp == nullptr) {
        perror("opendir");
        return files;
    }

    while ((entry = readdir(dp))) {
        std::string filename = entry->d_name;
        if (filename.find(".bin") != std::string::npos) {
            files.push_back(filename);
        }
    }

    closedir(dp);

    std::sort(files.begin(), files.end(), [](const std::string& a, const std::string& b) {
        int num_a = std::stoi(a.substr(0, a.find(".bin")));
        int num_b = std::stoi(b.substr(0, b.find(".bin")));
        return num_a < num_b;
    });

    return files;
}

// Function to load a bin file into a PCL point cloud
pcl::PointCloud<pcl::PointXYZI>::Ptr loadBinFile(const std::string& file_path) {
    std::ifstream input(file_path.c_str(), std::ios::binary);
    if (!input) {
        std::cerr << "Cannot open file: " << file_path << std::endl;
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI point;

    while (input.read(reinterpret_cast<char*>(&point), sizeof(pcl::PointXYZI))) {
        cloud->push_back(point);
    }

    input.close();
    return cloud;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bin2topic");
    
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("bin2topic/pointcloud", 1);

    std::string directory;
    double delta_time_sec;
    std::string frame_id;

    nh.param<std::string>("bin2topic/directory", directory, ".");
    nh.param<std::string>("bin2topic/frame_id", frame_id, "base_link");
    nh.param<double>("bin2topic/delta_time_sec", delta_time_sec, 1.0);

    // Print parameters
    ROS_INFO_STREAM("Directory: " << directory);
    ROS_INFO_STREAM("Frame ID: " << frame_id);
    ROS_INFO_STREAM("Delta Time (sec): " << delta_time_sec);

    std::vector<std::string> bin_files = getSortedBinFiles(directory);

    ros::Rate rate(1.0 / delta_time_sec);

    for (const auto& bin_file : bin_files) {
        if (!ros::ok()) break;

        std::string file_path = directory + "/" + bin_file;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = loadBinFile(file_path);

        if (cloud) {
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*cloud, output);
            output.header.frame_id = frame_id;
            output.header.stamp = ros::Time::now();
            pub.publish(output);
            ROS_INFO_STREAM("Published " << bin_file);
        }

        rate.sleep();
    }

    return 0;
}
