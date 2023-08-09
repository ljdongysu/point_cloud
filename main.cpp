
// test PSL lib demo from :https://blog.csdn.net/m0_48919875/article/details/123863892
//#include <iostream>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//int main (int argc, char** argv)
//{
//  pcl::PointCloud<pcl::PointXYZ> cloud;
//
//  // Fill in the cloud data
//  cloud.width    = 5;
//  cloud.height   = 1;
//  cloud.is_dense = false;
//  cloud.points.resize (cloud.width * cloud.height);
//
//  for (size_t i = 0; i < cloud.points.size (); ++i)
//  {
//    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//  }
//
//  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
//  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;
//
//  for (size_t i = 0; i < cloud.points.size (); ++i)
//    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
//
//  return (0);
//}

//
//#include <iostream>
//#include <pcl/common/common_headers.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/console/parse.h>
//
//int main(int argc, char **argv) {
//    std::cout << "Test PCL !!!" << std::endl;
//
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
//    uint8_t r(255), g(15), b(15);
//    for (float z(-1.0); z <= 1.0; z += 0.05)
//    {
//        for (float angle(0.0); angle <= 360.0; angle += 5.0)
//        {
//            pcl::PointXYZRGB point;
//            point.x = 0.5 * cosf (pcl::deg2rad(angle));
//            point.y = sinf (pcl::deg2rad(angle));
//            point.z = z;
//            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
//                            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
//            point.rgb = *reinterpret_cast<float*>(&rgb);
//            point_cloud_ptr->points.push_back (point);
//        }
//        if (z < 0.0)
//        {
//            r -= 12;
//            g += 12;
//        }
//        else
//        {
//            g -= 12;
//            b += 12;
//        }
//    }
//    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
//    point_cloud_ptr->height = 1;
//
//    pcl::visualization::CloudViewer viewer ("test");
//    viewer.showCloud(point_cloud_ptr);
//    while (!viewer.wasStopped()){ };
//    return 0;
//}

#include <iostream>
#include <string>
using namespace std;

#include "utils/utils.h"

int main(int argc, char ** argv)
{
    if (argc < 4)
    {
        std::cout << "please input: " << std::endl;
        std::cout << "\trgb image file path: [string]" << std::endl;
        std::cout << "\tdepth or disparity image file path: [string]" << std::endl;
        std::cout << "\tcamera config file path: [string]" << std::endl;
        std::cout <<"error!"<< std::endl;
        return 0;
    }
    // 图像矩阵
    std::string rgbImageFile = argv[1];//"/data/Depth/data/20221111_depth_distance_sparse3/20221111_902_distance_0/20221111_0818/cam0/28_1668154708419269.jpg";
    std::string depthDisparityImageFile = argv[2];// "/data/Depth/data/20221111_depth_distance_sparse3/depth/20221111_902_distance_0/20221111_0818/cam0/28_1668154708419269.png";
    std::string file = argv[3];//"/data/Depth/data/20221111_depth_distance_sparse3/20221111_902_distance_0/config.yaml";
    std::string tofImageFile = "";
    std::string slamDepthFile = "";

    if (argc >4)
    {
        tofImageFile = argv[4];
        if (argc > 5)
        {
            slamDepthFile = argv[5];
        }
    }

    psl::CameraParam camera;

    GetCameraParameter(file, camera);

    camera_fx = camera._P[0];
    camera_fy= camera._P[5];
    camera_cx = camera._P[2];
    camera_cy = camera._P[6];

    SaveCloudPoint(rgbImageFile,depthDisparityImageFile, tofImageFile, slamDepthFile);

    return 0;
}