//
// Created by indemind on 12/8/22.
//

#include "get_camera_parameter.h"
#include <opencv2/core.hpp>
#include <fstream>

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
}
