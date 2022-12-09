//
// Created by indemind on 12/8/22.
//

// C++ 标准库
#include <iostream>
#include <string>
using namespace std;

#include "../utils/utils.h"

// 主函数
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

    std::string rgbImageFile =  argv[1];//"/data/Depth/data/20221111_depth_distance_sparse3/20221111_902_distance_0/20221111_0818/cam0/28_1668154708419269.jpg";
    std::string depthDisparityImageFile = argv[2];// "/data/Depth/data/20221111_depth_distance_sparse3/depth/20221111_902_distance_0/20221111_0818/cam0/28_1668154708419269.png";
    std::string file = argv[3];//"/data/Depth/data/20221111_depth_distance_sparse3/20221111_902_distance_0/config.yaml";
    psl::CameraParam camera;

    GetCameraParameter(file, camera);

    camera_fx = camera._P[0];
    camera_fy= camera._P[5];
    camera_cx = camera._P[2];
    camera_cy = camera._P[6];

    SaveCloudPoint(rgbImageFile,depthDisparityImageFile);

    return 0;
}
