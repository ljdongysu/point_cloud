//
// Created by donglijian on 2023/11/29.
//
#include <iostream>
#include <Eigen/Core>
// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
namespace orb2indemind
{
    double camera_factor = 10;
    double camera_cx = 314.9;// = 325.5;
    double camera_cy = 222.4;// = 253.5;
    double camera_fx = 287.2;// = 518.0;
    double camera_fy = 287.2;// = 519.0;
    double orb_camera_cx = 239.3;// = 325.5;
    double orb_camera_cy = 178.91;// = 253.5;
    double orb_camera_fx = 359.3;// = 518.0;
    double orb_camera_fy = 359.3;// = 519.0;

    void Depth2PointCloud(const cv::Mat &depth, std::vector <Eigen::Vector3d> &cloud)
    {
        for (int m = 0; m < depth.rows; m++)
            for (int n = 0; n < depth.cols; n++)
            {
                unsigned short d = depth.ptr<uint16_t>(m)[n];
//            d = d / 100;

                // d 可能没有值，若如此，跳过此点
                if (d == 0)
                    continue;

                // d 存在值，则向点云增加一个点
                Eigen::Vector3d p;
                // 计算这个点的空间坐标
                p.z() = d / camera_factor;       // 正方向朝前
                p.x() = (n - orb_camera_cx) * p.z() / orb_camera_fx; // 正方向朝右
                p.y() = (m - orb_camera_cy) * p.z() / orb_camera_fy;   // 正方向朝下

                // 把p加入到点云中
                cloud.push_back(p);

            }
    }

    void PointCloud2Depth(const std::vector<Eigen::Vector3d>& pointCloud, cv::Mat& depthImage, int width, int height)
    {
        // 创建深度图像
        depthImage = cv::Mat::zeros(height, width, CV_16U);

        // 遍历点云数据
        for (const auto& point : pointCloud)
        {
            // 计算深度值
            double x = point.x();
            double y = point.y();
            double z = point.z();

            int u = static_cast<int>(std::round(x * camera_fx / z) + camera_cx);
            int v = static_cast<int>(std::round(y * camera_fy / z) + camera_cy);

            // 更新深度图像
            if (u >= 0 && u < width && v >= 0 && v < height)
            {
                unsigned short depthValue = static_cast<unsigned short>(z * camera_factor); // 假设深度单位为米，转换为厘米
                depthImage.at<uint16_t>(v, u) = depthValue;
            }
        }
    }

    void PointStructConvert(std::vector<Eigen::Vector3d> &cloud, const Eigen::Vector3d &point)
    {
        for (int i = 0; i < cloud.size(); ++i)
        {
            cloud[i].x() = cloud[i].x() - point.x();
            cloud[i].y() = cloud[i].y() - point.y();
        }
    }
}

int main(int argc, char ** argv)
{
    cv::Mat orbDepth = cv::imread("/data/ABBY/BASE/TRAIN/data_Capture_Img_Tof/data_2023_11_27/new/1.5m/Depth/016255_0008_1701081237549_1701081237549_Depth_640x400.png",-1);
    std::vector<Eigen::Vector3d> cloud;
    orb2indemind::Depth2PointCloud(orbDepth, cloud);
    cv::Mat imageDepth;
    Eigen::Vector3d point;
    point.x() = 5;
    point.y() = 23.5;

//    orb2indemind::PointStructConvert(cloud, point);
    orb2indemind::PointCloud2Depth(cloud, imageDepth, 640, 400);

    PointCloud::Ptr cloudShow(new PointCloud);

    for (int i = 0; i < cloud.size(); ++i)
    {
        PointT p;
        p.x = cloud[i].x();
        p.y = cloud[i].y();
        p.z = cloud[i].z();
        if (p.y  > 0)
        {
            continue;
        }
        cloudShow->push_back(p);
    }

    cv::imwrite("image_depth.png", imageDepth * 10);

    //显示点云图像
    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(cloudShow);
    while (!viewer.wasStopped()){ };

}