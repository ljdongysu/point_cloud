//
// Created by indemind on 12/8/22.
//

#ifndef DEPTH_CLOUD_POINT_UTILS_H
#define DEPTH_CLOUD_POINT_UTILS_H

#include <iostream>
#include <string>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

double camera_factor = 100;
double camera_cx;// = 325.5;
double camera_cy;// = 253.5;
double camera_fx;// = 518.0;
double camera_fy;// = 519.0;

namespace psl
{
    struct CameraParam
    {
        double _TSC[16];  // 4X4 camera to imu
        int _width;
        int _height;

        float b;
        float bf;

        // distortion_type:equidistant
        double _focal_length[2];     // fx,fy
        double _principal_point[2];  // cx,cy
        // Rectification matrix (stereo cameras only)
        // A rotation matrix aligning the camera coordinate system to the ideal
        // stereo image plane so that epipolar lines in both stereo images are
        // parallel.
        double _R[9];
        // Projection/camera matrix
        //     [fx'  0  cx' Tx]
        // P = [ 0  fy' cy' Ty]
        //     [ 0   0   1   0]
        double _P[12];
        // Intrinsic camera matrix for the raw (distorted) images.
        //     [fx  0 cx]
        // K = [ 0 fy cy]
        //     [ 0  0  1]
        double _K[9];
        // The distortion parameters, size depending on the distortion model.
        // For us, the 4 parameters are: (k1, k2, t1, t2).
        double _D[4];

        CameraParam()
        {
        }
    };
}  // namespace psl


bool GetCameraParameter(const std::string& configFile, psl::CameraParam &camera);

void ReadPointCloud(const std::string &pointCloudSavePLY);

void SaveCloudPoint(const std::string &imageL, const std::string &image
                    , const std::string &tofImage = "", const std::string &slamDepthFile = "");

void Depth2PointCloud(const cv::Mat &depth, const cv::Mat &rgb, PointCloud::Ptr cloud, bool usedTof = false);

bool GetSlamDepth(const std::string& yamlFile, cv::Mat &slamDepthImage);

#endif //DEPTH_CLOUD_POINT_UTILS_H
