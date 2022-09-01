// 有问题！！！！ 跑不了Eigen有问题
//./cmake-build-debug/normalIntegralImages
// normalIntegralImages: /usr/include/eigen3/Eigen/src/Core/util/XprHelper.h:133: Eigen::internal::variable_if_dynamic<T, Value>::variable_if_dynamic(T) [with T = long int; int Value = 3]: Assertion `v == T(Value)' failed.
已放弃(核心已转储)
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

int main()
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/lixu/Downloads/table_scene_mug_stereo_textured.pcd", *cloud);
    // std::cout << cloud->height << std::endl;
    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}