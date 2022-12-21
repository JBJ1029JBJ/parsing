#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <algorithm>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <Eigen/Dense>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

pcl::PointCloud<pcl::PointXYZI> Passthrough_ob(pcl::PointCloud<pcl::PointXYZI> point, double minx, double maxx, double miny, double maxy);
cv::Mat undistortion_image(cv::Mat img);
cv::Mat CameraLidar_Fusion(cv::Mat img, pcl::PointCloud<pcl::PointXYZI> point);
std::vector<float> rpyxyz;

int main()
{
    char buffer[256];
    int num = 0;
    std::vector<std::string> campath;
    std::vector<std::string> scanpath;


    cv::Mat undist; cv::Mat Fusion_point;
    pcl::PointCloud<pcl::PointXYZI> roi_point;

    rpyxyz.resize(6);

    std::string cam_path("/home/a/bag/image3/*.png");
    cv::glob(cam_path,campath,false);
    std::string ld_path("/home/a/bag/point3/*.pcd");
    cv::glob(ld_path,scanpath,false);

    std::sort(campath.begin(),campath.end());
    std::sort(scanpath.begin(),scanpath.end());

    while(num <= campath.size())
    {
    if(num >= campath.size())
        {
        printf("Finish\n");
        }
        else if(campath.size() == scanpath.size() && campath.size() != 0 && scanpath.size() != 0)
        {
            cv::Mat img = cv::imread(campath[num]);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
            if(pcl::io::loadPCDFile<pcl::PointXYZI> (scanpath[num],*cloud) == -1)
            {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            }
            roi_point = Passthrough_ob(*cloud,0,50,-10,10);
            undist = undistortion_image(img);
            Fusion_point = CameraLidar_Fusion(undist,roi_point);

            cv::imshow("Fusion result", undist);
            sprintf(buffer,"/home/a/bag/result/%04d.png",num+1928);
            std::cout << buffer << std::endl;
            cv::imwrite(buffer,undist);
            cv::waitKey(3);
            num++;
        }
    }
}

pcl::PointCloud<pcl::PointXYZI> Passthrough_ob(pcl::PointCloud<pcl::PointXYZI> point, double minx, double maxx, double miny, double maxy)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> filter;
    pcl::PassThrough <pcl::PointXYZI> pass;

    *cloud = point;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-3,3);

    pass.filter(*cloud_filter);
    pass.setInputCloud(cloud_filter);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(minx, maxx);

    pass.filter(*cloud_filter);
    pass.setInputCloud(cloud_filter);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(miny, maxy);

    pass.filter(*cloud_filter);

    filter = *cloud_filter;

    return filter;
}

cv::Mat undistortion_image(cv::Mat img)
{
    cv::Mat output;
    cv::Mat Camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat DistCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);

    Camera_matrix=(cv::Mat1d(3,3) << 1806.820309, 0.000000, 1033.711216,
                                     0.000000, 1805.927347, 816.626313,
                                     0.000000, 0.000000, 1.000000);
    DistCoeffs=(cv::Mat1d(1,5) << -0.137354, 0.057845, 0.000487, 0.002065, 0.000000);

    cv::undistort(img,output,Camera_matrix,DistCoeffs);

    return output;
}

cv::Mat CameraLidar_Fusion(cv::Mat img, pcl::PointCloud<pcl::PointXYZI> point)
{
        Eigen::MatrixX3f Camera_matrix = Eigen::Matrix3f::Identity();
    Camera_matrix << 1806.820309, 0.000000, 1033.711216,
                     0.000000, 1805.927347, 816.626313,
                     0.000000, 0.000000, 1.000000;
 
    Eigen::MatrixXf H(3, 4);
    H << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0;

    float rot_x = 3.164041+rpyxyz.at(0); float rot_y = -1.562969+rpyxyz.at(1); float rot_z = -1.583024+rpyxyz.at(2);
    float x = 1.033056+rpyxyz.at(3); float y = 0.058157+rpyxyz.at(4); float z = 3.255586+rpyxyz.at(5);

    Eigen::Affine3f transf = pcl::getTransformation(x,y,z,rot_x,rot_y,rot_z);
    
    Eigen::Matrix4f Rotation;
    Rotation = transf.matrix();

    int maxx = 0; int maxy = 0;
    int minx = 2048; int miny = 1024;

    Eigen::MatrixXf OutPoint(3, 1);

    for(int i = 0; i < point.size(); i++){
        int color_scale = 0;
        color_scale = point[i].x * 11;
        if(color_scale >= 255) color_scale = 255;

        Eigen::MatrixXf Point(4, 1);

            Point(0) = point[i].x;
            Point(1) = point[i].y;
            Point(2) = point[i].z;
            Point(3) = 1;

            OutPoint = Camera_matrix * H * Rotation * Point;

            float x = OutPoint(0)/OutPoint(2);
            float y = OutPoint(1)/OutPoint(2);

            if((x < 2048 && x >= 0) && (y < 1024 && y >= 0)){
                //img.at<cv::Vec3b>(y, x)[0] = 0; img.at<cv::Vec3b>(y, x)[1] = color_scale; img.at<cv::Vec3b>(y, x)[2] = 255-color_scale;
                cv::circle(img,cv::Point(x,y),1.5,cv::Scalar(0,color_scale,255-color_scale),1,-1,0);
                if(maxx < x){maxx = x;}
                if(minx > x){minx = x;}
                if(maxy < y){maxy = y;}
                if(miny > y){miny = y;}
            }
    
    }
    
    return img;
}