#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argv, char **argc)
{   //等价于pcl::PointCloud<pcl::PointXYZ> mycloud;  一个是用指针一个是直接用变量
    pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    mycloud->width = 10;
    mycloud->height = 1;
    mycloud->is_dense = false;
    mycloud->points.resize(mycloud->width * mycloud->height);
    for (auto &point : *mycloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    pcl::io::savePCDFileASCII("../pcd_read.pcd",*mycloud);
    return (0);
}
