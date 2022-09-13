/*ndt测试，由于它要算概率分布，当点过少的时候是不能用的（点都那么少了 就遍历了）*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
using std::cin;
using std::cout;
using std::endl;
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>(100, 1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto &point : *cloud_in)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    cout << "Saved " << cloud_in->size() << " data points to input:" << endl;
    for (auto &point : *cloud_in)
        cout << point << endl;
    *cloud_out = *cloud_in;
    cout << "\nsize " << cloud_out->size() << endl;
    for (auto &point : *cloud_out)
        point.x += 0.7f;
    // point.x += 0.1 * rand() / (RAND_MAX + 1.0f);
    cout << "Transformed " << cloud_in->size() << " data points:" << endl;
    for (auto point : *cloud_out)
        cout << point << endl;
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // ndt.setTransformationEpsilon(0.001);
    // // Setting maximum step size for More-Thuente line search.
    // ndt.setStepSize(0.1);
    // // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    // ndt.setResolution(1.0);
    ndt.setInputSource(cloud_in);
    ndt.setInputTarget(cloud_out);
    ndt.setMaximumIterations(1);
    pcl::PointCloud<pcl::PointXYZ> Final;
    // Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitZ());
    // Eigen::Translation3f init_translation(0, 0, 0);
    // Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    // ndt.align(Final, init_guess);
    ndt.align(Final);
    cout << "Final: " << endl;
    for (auto point : Final)
        cout << point << endl;

    cout << "has converged:" << ndt.hasConverged() << " source: " << ndt.getFitnessScore() << endl;

    cout << ndt.getFinalTransformation() << endl;
    return 0;
}