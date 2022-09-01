#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
int main(int argc, char **argv)
{
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_flitered(new pcl::PCLPointCloud2());

    pcl::PCDReader reader;
    reader.read("/home/lixu/Downloads/table_scene_lms400.pcd", *cloud);
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.02f, 0.02f, 0.02f);
    sor.filter(*cloud_flitered);
    std::cerr << "PointCloud after filtering: " << cloud_flitered->width * cloud_flitered->height
              << " data points (" << pcl::getFieldsList(*cloud_flitered) << ")." << std::endl;

    pcl::PCDWriter writer;
    writer.write("table_scene_lms400_downsampled.pcd", *cloud_flitered,
                 Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

    return (0);
}
