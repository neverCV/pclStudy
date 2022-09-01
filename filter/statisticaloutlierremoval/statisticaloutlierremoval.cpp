#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("/home/lixu/Downloads/table_scene_lms400.pcd", *cloud);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    //设置看周围多少个点
    sor.setMeanK(50);
    //相当于3sigma原则一样的系数，落在mean+- var*1.0范围内的认为是正常点，落在之外的是离群点
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    // 翻转条件，这样可以看那些被筛除掉了 那些被留下了
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd", *cloud_filtered, false);

    return (0);
}