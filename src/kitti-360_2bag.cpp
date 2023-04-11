#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}

void load_ground_truth(std::string path, std::vector<int> &vIndex, std::vector<Eigen::Matrix4d> &vPose)
{
    std::ifstream infileGt;
    infileGt.open(path.c_str());

    while(!infileGt.eof())
    {
        std::string s;
        getline(infileGt,s);

        int index;

        Eigen::Matrix3d R_temp;
        Eigen::Vector3d t_temp;
        Eigen::Matrix4d T_temp = Eigen::MatrixXd::Identity(4, 4);

        std::stringstream ss;
        ss << s;
        ss >> std::setprecision(6) >> index;
        ss >> std::setprecision(6) >> T_temp(0, 0); ss >> std::setprecision(6) >> T_temp(0, 1); ss >> std::setprecision(6) >> T_temp(0, 2); ss >> std::setprecision(6) >> T_temp(0, 3);
        ss >> std::setprecision(6) >> T_temp(1, 0); ss >> std::setprecision(6) >> T_temp(1, 1); ss >> std::setprecision(6) >> T_temp(1, 2); ss >> std::setprecision(6) >> T_temp(1, 3);
        ss >> std::setprecision(6) >> T_temp(2, 0); ss >> std::setprecision(6) >> T_temp(2, 1); ss >> std::setprecision(6) >> T_temp(2, 2); ss >> std::setprecision(6) >> T_temp(2, 3);

        vIndex.push_back(index);
        vPose.push_back(T_temp);
    }

    infileGt.close();

    std::cout << "load Pose successfully!" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti-360_2bag");
    ros::NodeHandle n("~");
    std::string dataset_folder, sequence_number, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);
    n.getParam("sequence_number", sequence_number);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);
    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud", 2);

    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);

    std::string timestamp_path = "data_2d_raw/" + sequence_number + "/image_00/timestamps.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);

    std::string ground_truth_path = "data_poses/" + sequence_number + "/cam0_to_world.txt";
    std::vector<int> vIndex;
    std::vector<Eigen::Matrix4d> vPose;
    load_ground_truth(dataset_folder + ground_truth_path, vIndex, vPose);

    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);

    std::string line;
    std::size_t line_num = 0;

    bool first_data = true;
    double first_timestamp;

    std::cout << "vIndex[0] = " << vIndex[0] << std::endl;
    std::cout << "vIndex.back() = " << vIndex.back() << std::endl;

    ros::Rate r(10.0 / publish_delay);
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        if(line_num < vIndex[0])
        {
            line_num++;
            continue;
        }

        if(line_num > vIndex.back())
        {
            line_num++;
            break;
        }

        std::stringstream ss;
        ss << line;
        std::string data_str, time_str;
        ss >> data_str; ss >> time_str;

        for (int i = 0; i < time_str.size(); i++){
            if (time_str[i] == ':'){
                time_str[i] = ' ';
            }
        }

        std::stringstream ss_slip;
        ss_slip << time_str;
        std::string hour_str, minute_str, seconds_str;
        ss_slip >> hour_str; ss_slip >> minute_str; ss_slip >> seconds_str;

        float hour = stof(hour_str);
        float minute = stof(minute_str);
        float seconds = stof(seconds_str);

        float timestamp = hour * 3600 + minute * 60 + seconds;

        if(first_data)
        {
            first_data = false;
            first_timestamp = timestamp;
        }

        timestamp = timestamp - first_timestamp;

        std::stringstream left_image_path, right_image_path;
        left_image_path << dataset_folder << "data_2d_raw/" + sequence_number + "/image_00/data_rect/" << std::setfill('0') << std::setw(10) << line_num << ".png";
        cv::Mat left_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);

        assert(!left_image.empty());

        // read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << "data_3d_raw/" + sequence_number + "/velodyne_points/data/" 
                        << std::setfill('0') << std::setw(10) << line_num << ".bin";
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());

        std::vector<Eigen::Vector3d> lidar_points;
        std::vector<float> lidar_intensities;
        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
            lidar_intensities.push_back(lidar_data[i+3]);

            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point);
        }

        assert(laser_cloud.points.size() > 0);

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        laser_cloud_msg.header.frame_id = "/camera_init";
        pub_laser_cloud.publish(laser_cloud_msg);

        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();
        pub_image_left.publish(image_left_msg);

        if (to_bag)
        {
            std::cout << std::fixed << "timestamp = " << timestamp << " index = " << line_num << std::endl;
            bag_out.write("/image_left", ros::Time::now(), image_left_msg);
            bag_out.write("/kitti/velo/pointcloud", ros::Time::now(), laser_cloud_msg);
        }

        line_num ++;
        r.sleep();
    }
    bag_out.close();
    std::cout << "Done \n";


    return 0;
}
