#include <pcl/point_cloud.h> //PointCloud的模板
#include <pcl/octree/octree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char **argv)
{
    pcl::visualization::PCLVisualizer viewer("viewer");
    srand((unsigned int)time(NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->width = 10000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        (*cloud)[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud)[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        (*cloud)[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }
    float resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    pcl::PointXYZ searchPoint;

    searchPoint.x = 900.f;
    searchPoint.y = 900.f;
    searchPoint.z = 900.f;

    std::vector<int> pointIdxvec;
    // Search for neighbors within a voxel at given point referenced by a point index.
    // 查找统一体素内的元素有哪些
    if (octree.voxelSearch(searchPoint, pointIdxvec))
    {
        std::cout << "Neighbors within voxel search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z << ")"
                  << std::endl;
        for (std::size_t i = 0; i < pointIdxvec.size(); ++i)
            std::cout << "    " << (*cloud)[pointIdxvec[i]].x
                      << " " << (*cloud)[pointIdxvec[i]].y
                      << " " << (*cloud)[pointIdxvec[i]].z << std::endl;
    }

    viewer.addPointCloud(cloud);
    viewer.spin();
}