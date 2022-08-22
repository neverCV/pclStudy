#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char **argv)
{
    srand((unsigned int)time(NULL));
    float resloution = 16.0f;
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resloution);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clouda(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb(new pcl::PointCloud<pcl::PointXYZ>);
    clouda->width = 128;
    clouda->height = 1;
    clouda->points.resize(clouda->width * clouda->height);
    for (std::size_t i = 0; i < clouda->size(); ++i)
    {
        (*clouda)[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
        (*clouda)[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
        (*clouda)[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    cloudb->width = 128;
    cloudb->height = 1;
    cloudb->points.resize(cloudb->width * cloudb->height);
    for (std::size_t i = 0; i < cloudb->size(); ++i)
    {
        (*cloudb)[i].x = 64.0f * rand() / (RAND_MAX + 1.0f);
        (*cloudb)[i].y = 64.0f * rand() / (RAND_MAX + 1.0f);
        (*cloudb)[i].z = 64.0f * rand() / (RAND_MAX + 1.0f);
    }

    octree.setInputCloud(clouda);
    octree.addPointsFromInputCloud();
    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree.switchBuffers();

    // Add points from cloudB to octree
    octree.setInputCloud(cloudb);
    octree.addPointsFromInputCloud();

    std::vector<int> newPointIdxVector;

    // Get vector of point indices from octree voxels which did not exist in previous buffer
    // 后面那个应该是判断该体素内有多少个点才将其认为是变化的（以前没有现在有）
    octree.getPointIndicesFromNewVoxels(newPointIdxVector, 4);

    // Output points
    std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
    for (std::size_t i = 0; i < newPointIdxVector.size(); ++i)
        std::cout << i << "# Index:" << newPointIdxVector[i]
                  << "  Point:" << (*cloudb)[newPointIdxVector[i]].x << " "
                  << (*cloudb)[newPointIdxVector[i]].y << " "
                  << (*cloudb)[newPointIdxVector[i]].z << std::endl;
}