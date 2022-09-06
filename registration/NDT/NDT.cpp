/* 对于本代码来说，滤波和不滤波相比，结果差距几乎可以忽略不计（可以降采样配准后再把旋转矩阵应用到源点云上）*/

#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    // Loading first scan of room.
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("room_scan1.pcd", *target_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;

    // Loading second scan of room from new perspective.
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("room_scan2.pcd", *input_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // approximate_voxel_filter.setLeafSize(0.15, 0.15, 0.15);
    // approximate_voxel_filter.setInputCloud(input_cloud);
    // approximate_voxel_filter.filter(*filtered_cloud);
    // std::cout << "Filtered cloud contains " << filtered_cloud->size()
    //           << " data points from room_scan2.pcd" << std::endl;
    std::cout << "input cloud contains " << input_cloud->size()
              << " data points from room_scan2.pcd" << std::endl;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);
    ndt.setMaximumIterations(35);
    // ndt.setInputSource(filtered_cloud);
    ndt.setInputSource(input_cloud);
    ndt.setInputTarget(target_cloud);

    // 初始对齐矩阵？
    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);

    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
    //  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

    pcl::visualization::PCLVisualizer::Ptr
        viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                   1, "output cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return (0);
}
