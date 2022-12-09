//
// Created by indemind on 12/9/22.
//

// C++ 标准库
#include <iostream>
#include <string>
using namespace std;

#include "utils/utils.h"
#include <iostream>
#include <string>
using namespace std;

int main(int argc, char ** argv)
{
    if (argc < 2)
    {
        std::cout << "please input: " << std::endl;
        std::cout << "\tpoint cloud ply file path: [string]" << std::endl;
        std::cout <<"error!"<< std::endl;
        return 0;
    }
    ReadPointCloud(argv[1]);
}