//
// Created by indemind on 12/8/22.
//

#include "utils.h"
#include <opencv2/core.hpp>
#include <fstream>

const std::string PCL_DEPTH = "depth";
const std::string PCL_DISPARITY = "pcl_by_disparity";

bool GetCameraParameter(const std::string& configFile, psl::CameraParam &camera)
{
    cv::FileStorage fileStream = cv::FileStorage(configFile, cv::FileStorage::READ);

    if (not fileStream.isOpened())
    {
        std:: cout <<"file not exist <" + configFile + ">";
        return false;
    }

    // TODO : the exception for lack option
    cv::Mat_<double> kl, dl, pl, rl;
    fileStream["Kl"] >> kl;
    fileStream["Dl"] >> dl;
    fileStream["Pl"] >> pl;
    fileStream["Rl"] >> rl;

    memcpy(camera._K, kl.data, sizeof(camera._K));
    memcpy(camera._R, rl.data, sizeof(camera._R));
    memcpy(camera._P, pl.data, sizeof(camera._P));
    memcpy(camera._D, dl.data, sizeof(camera._D));

    fileStream.release();
    return true;
}


void SaveCloudPoint(const std::string &imageL, const std::string &image)
{
    cv::Mat rgb, depth;
    rgb = cv::imread(imageL);
    depth = cv::imread(image, -1);  //在cv::imread参数中加入-1，表示不改变读取图像的类型直接读取
    std::string pointCloudSavePLY = image.substr(0, image.find_last_of('.')) + PCL_DISPARITY + ".ply";
    std::string pointCloudSavePCD = image.substr(0, image.find_last_of('.')) + PCL_DISPARITY + ".pcd";

    if (depth.channels() == 3)
    {
        pointCloudSavePLY = pointCloudSavePLY.substr(0, pointCloudSavePLY.find_last_of('_')) + "_" + PCL_DEPTH + ".ply";
        pointCloudSavePCD = pointCloudSavePCD.substr(0, pointCloudSavePCD.find_last_of('.')) + "_" + PCL_DEPTH + ".pcd";
    }

    PointCloud::Ptr cloud(new PointCloud);
    // 遍历深度图
    for (int m = 0; m < depth.rows; m++)
        for (int n = 0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            //            ushort d = depth.ptr<ushort>(m)[n];
            //            unsigned int d = depth.at<uchar>(m,n*3);

            //使用视差图
            unsigned int d=depth.at<uchar>(m,n);

            //使用深度图
            if (depth.channels() == 3)
            {
                d = depth.at<uchar>(m,n*3) + depth.at<uchar>(m,n*3 + 1) + depth.at<uchar>(m,n*3+2);
            }

            //使用视差图
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            // 使用深度图像
            //            p.z = double(d) / camera_factor;

//            // 使用视差图
//            p.z = double(3423.0 / d) / camera_factor;
//
//            //使用深度图
//            if (depth.channels() == 3)
//            {
                p.z = double(d) / camera_factor;
//            }

            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            //            // 从rgb图像中获取它的颜色
            //            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
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
    //    pcl::transformPointCloud(*cloud, *cloud, transform_1);

    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << "point cloud size = " << cloud->points.size() << endl;
    cloud->is_dense = false;
    pcl::io::savePLYFile(pointCloudSavePLY, *cloud);   //将点云数据保存为ply文件
    pcl::io::savePCDFile(pointCloudSavePCD, *cloud);   //将点云数据保存为pcd文件

    //显示点云图像
    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()){ };

    cloud->points.clear();
    cout << "Point cloud saved." << endl;
}

void ReadPointCloud(const std::string &pointCloudSavePLY)
{
    PointCloud::Ptr cloudLoad(new PointCloud);
    pcl::io::loadPLYFile(pointCloudSavePLY, *cloudLoad);

    //显示点云图像
    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(cloudLoad);
    while (!viewer.wasStopped()){ };

    cloudLoad->points.clear();
}