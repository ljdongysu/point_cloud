## 深度图/视差图转点云图
## 依赖库
###  工程cloud_by_pangolin
1. 使用pangolin库将视差图转为点云图
2. sudo apt install libglew-dev cmake libboost-dev libboost-thread-dev libboost-filesystem-dev libeigen3-dev -y
3. git clone https://github.com/stevenlovegrove/Pangolin.git
4. cmake buidl && cd build
   cmake -DCMAK_INSTALL_PREFIX=./install ..
   make -j6
   make install 
### 工程cloud_by_pcl
1. 使用 PCL将深度图转为点云图
2. https://vtk.org/files/release/7.1/VTK-7.1.1.zip 和https://github.com/PointCloudLibrary/pcl/tree/pcl-1.12.1 下载VTK和PCL库
3. 安装流程参考 https://www.notion.so/indemind/d8044f8f347e40f095629658ac7c99a1


### 运行
1. 可能报错找不到指定库，需要再运行时候添加对应库路径
2. cloud_by_pcl 目前看效果更好一些
3. cloud_by_pangolin效果稍微差一些