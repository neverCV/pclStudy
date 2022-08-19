#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argv, char **argc)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../pcd_read.pcd", *mycloud) == -1)
    { //等价于pcl::console::print (pcl::console::L_ERROR, __VA_ARGS__)
        pcl::console::print(pcl::console::L_ERROR, "its just a test!\n");
        PCL_ERROR("无法打开文件！");
        return (-1);
    }
    else
    {
        std::cout << "Loaded " << mycloud->width * mycloud->height << " data points from file" << std::endl;
        for (const auto &point : *mycloud)
            std::cout << "    " << point.x
                      << " " << point.y
                      << " " << point.z << std::endl;

        return (0);
    }
}
