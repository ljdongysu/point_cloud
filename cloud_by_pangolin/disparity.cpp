//
// Created by indemind on 12/8/22.
//

#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include "../get_camera/get_camera_parameter.h"

using namespace std;
using namespace Eigen;

// 文件路径，如果不对，请调整

string left_file = "/data/Depth/data/20221111_depth_distance_sparse3/20221111_902_distance_0/20221111_0818/cam0/28_1668154708419269.jpg";
string right_file = "/data/Depth/data/20221111_depth_distance_sparse3/20221111_902_distance_0/20221111_0818/cam1/28_1668154708419269.jpg";
string disparity_file = "/data/Depth/data/20221111_depth_distance_sparse3/gray/20221111_902_distance_0/20221111_0818/cam0/28_1668154708419269.png";
std::string configFile = "/data/Depth/data/20221111_depth_distance_sparse3/20221111_902_distance_0/config.yaml";
//笔记：
//拿到left.png  right.png disparity.png 三张图，但是right.png实际上是没有用到的。
//以left中像素点进行遍历，与left每个像素点对应，在disparity中有每个点的视差信息，用0-255表示。
//因此对于每个像素点（v,u）,其空间点的深度 z = fb/d
//在使用pangolin画图时，每个点具有4个值，x y z  w ，z已经算出，x y 利用像素到空间的缩放平移公式  u = fx*x+cx/z 来计算，w就用left对应点上点颜色即可
// 在pangolin中画图，已写好，无需调整
void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud);

int main(int argc, char **argv) {
    psl::CameraParam camera;
    GetCameraParameter(configFile, camera);

//    for (int i = 0; i < 12; ++i)
//    {
//        std::cout << "camera._P["<<i<<"]: " << camera._P[i]<<", ";
//    }
//    std::cout << std::endl;
//
//    for (int i = 0; i < 9; ++i)
//    {
//        std::cout << "camera._K["<<i<<"]: " << camera._K[i]<<", ";
//    }
//    std::cout << std::endl;
//
//    for (int i = 0; i < 9; ++i)
//    {
//        std::cout << "camera._R["<<i<<"]: " << camera._R[i]<<", ";
//    }
//    std::cout << std::endl;
    // 内参
    //是左相机点缩放系数fx fy,和平移系数cx cy
//    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    double fx = camera._P[0],  fy= camera._P[5], cx = camera._P[2], cy = camera._P[6];  // 改为K
    // 间距
    double b = 0.05; // 通过Pr[3]/Pl[0] 获取，得到这个数据。
    //焦距  0.00027 * fx    ; 0.00027 * fy 可能是K的，也可能是P的，目前使用P的   0.00027 = dx/image.size
    double f = 0.0766;//    0.00027

    // 读取图像
    cv::Mat left = cv::imread(left_file, 0);
//    cv::Mat right = cv::imread(right_file, 0);
    cv::Mat disparity = cv::imread(disparity_file, 0); // disparty 为CV_8U,单位为像素

    // 生成点云
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;

    // TODO 根据双目模型计算点云
    // 如果你的机器慢，请把后面的v++和u++改成v+=2, u+=2
    for (int v = 0; v < left.rows; v++)
        for (int u = 0; u < left.cols; u++) {

            Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色

            // start your code here (~6 lines)
            // start your code here (~6 lines)
            unsigned int d=disparity.at<uchar>(v,u);
            //视差图disparty中(v,u)点，带有视差信息d，其深度d用像素值来表达0-255之间
            if(d==0)
                continue;  //视差d=0表示该点没有深度不用估计
            //基线长度b=0.573  Z=fb/d  f是内参--焦距，b是基线长度，d是视差
            double z=(f*b*1000)/d;

            point[2]=z;
            point[0]=(u-cx)*z/fx;
            point[1]=(v-cy)*z/fy;
            pointcloud.push_back(point);

            // 根据双目模型计算 point 的位置
            // end your code her
            // end your code here
        }

    // 画出点云
    showPointCloud(pointcloud);
    return 0;
}

void showPointCloud(const vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3f(p[3], p[3], p[3]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}

