//
// Created by indemind on 12/8/22.
//

// C++ 标准库
#include <iostream>
#include <string>
using namespace std;

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include "../get_camera/get_camera_parameter.h"

#include <Eigen/Core>


// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
 double camera_factor = 1000;
 double camera_cx;// = 325.5;
 double camera_cy;// = 253.5;
 double camera_fx;// = 518.0;
 double camera_fy;// = 519.0;

// 主函数
int main(int argc, char ** argv)
{
    // 图像矩阵
    cv::Mat rgb, depth;
    rgb = cv::imread("/data/Depth/data/20221111_depth_distance_sparse3/20221111_902_distance_0/20221111_0818/cam0/28_1668154708419269.jpg");
    depth = cv::imread("/data/Depth/data/20221111_depth_distance_sparse3/depth/20221111_902_distance_0/20221111_0818/cam0/28_1668154708419269.png", -1);  //在cv::imread参数中加入-1，表示不改变读取图像的类型直接读取

    psl::CameraParam camera;
    std::string file = "/data/Depth/data/20221111_depth_distance_sparse3/20221111_902_distance_0/config.yaml";
    GetCameraParameter(file, camera);

    camera_fx = camera._P[0];
    camera_fy= camera._P[5];
    camera_cx = camera._P[2];
    camera_cy = camera._P[6];  //
    // 间距
    double b = 0.05; // 通过Pr[3]/Pl[0] 获取，得到这个数据。
    //焦距  0.00027 * fx    ; 0.00027 * fy 可能是K的，也可能是P的，目前使用P的   0.00027 = dx/image.size
    double f = 0.0766;//    0.00027

    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud::Ptr cloud(new PointCloud);
//    std::cout << depth << std::endl;
//    std::cout << depth.size()<< " , "  << depth.channels() <<std::endl;
    // 遍历深度图
    for (int m = 0; m < depth.rows; m++)
        for (int n = 0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
//            ushort d = depth.ptr<ushort>(m)[n];
            unsigned int d = depth.at<uchar>(m,n*3);
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            // 把p加入到点云中
            cloud->points.push_back(p);
        }


    //绕Z轴旋转180度
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    float theta = M_PI ;
    transform_1(0, 0) = cos(theta);
    transform_1(0, 1) = -sin(theta);
    transform_1(1, 0) = sin(theta);
    transform_1(1, 1) = cos(theta);

    pcl::transformPointCloud(*cloud, *cloud, transform_1);

    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << "point cloud size = " << cloud->points.size() << endl;
    cloud->is_dense = false;
    pcl::io::savePLYFile("/data/Depth/data/20221111_depth_distance_sparse3/ply.ply", *cloud);   //将点云数据保存为ply文件
    pcl::io::savePCDFile("/data/Depth/data/20221111_depth_distance_sparse3/pcd.pcd", *cloud);   //将点云数据保存为pcd文件
    // 清除数据并退出

    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()){ };

    cloud->points.clear();
    cout << "Point cloud saved." << endl;
    return 0;
}
