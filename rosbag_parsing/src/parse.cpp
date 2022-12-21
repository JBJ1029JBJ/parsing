#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <stdio.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

pcl::PointCloud<pcl::PointXYZI> cloudmsg2cloud(sensor_msgs::PointCloud2ConstPtr cloudmsg);

int main(int argc, char ** argv)
{
    rosbag::Bag bag;
    // bag file open
    //bag.open("/home/a/bag/first_scene.bag", rosbag::bagmode::Read);
    //bag.open("/home/a/bag/second_scene.bag",rosbag::bagmode::Read);
    bag.open("/home/a/bag/third_scene.bag",rosbag::bagmode::Read);

    int count_img = 0;
    int count_point = 0;
    int count_save_img = 0;
    int second_count_save_img = 1034;
    int third_count_save_img = 1928;
    int count_save_point = 0;
    int second_count_save_point = 1034;
    int third_count_save_point = 1928;
    int index_img = 0;
    int index_point = 0;
    char buffer[256];
    char buffer_point[256];

    bool img_done = false;
    bool point_done =false;

    // Set topics to read (topic name)
    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/image_color/compressed"));
    topics.push_back(std::string("/pandar"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // Read topic data  
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    { 
        // First topic : /camera/image_color/compressed     Type : sensor_msgs/CompressedImage
        sensor_msgs::CompressedImageConstPtr msg_img = m.instantiate<sensor_msgs::CompressedImage>();
        if(msg_img != NULL)
        {
            count_img++;
            index_img = count_img%3;   
            std::cout << "count of /camera/image_color : " << count_img << std::endl;
            cv::Mat image_raw = cv_bridge::toCvCopy(msg_img,"bgr8")->image;
            img_done = true;

            if(index_img == 0)
            {
                sprintf(buffer,"/home/a/bag/image/%03d.png",third_count_save_img);
                std::cout << buffer << std::endl;
                cv::imwrite(buffer,image_raw);
                third_count_save_img++;
            }
        }

        // Second topic : /pandar   Type : sensor_msg/PointCloud2
        sensor_msgs::PointCloud2ConstPtr msg_point = m.instantiate<sensor_msgs::PointCloud2>();
        if(msg_point != NULL)
        {
            count_point++;
            //index_point = count_point%0;
            std::cout << "count of /pandar : " << count_point << std::endl;

            sprintf(buffer_point,"/home/a/bag/point/%03d.pcd",third_count_save_point);
            std::cout << buffer_point << std::endl;

            pcl::io::savePCDFileBinary(buffer_point,cloudmsg2cloud(msg_point));
            third_count_save_point++;
        }
    }

    bag.close();

    return 0;
}

pcl::PointCloud<pcl::PointXYZI> cloudmsg2cloud(sensor_msgs::PointCloud2ConstPtr cloudmsg)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud_dst;
        pcl::fromROSMsg(*cloudmsg,cloud_dst);

        return cloud_dst;
    }