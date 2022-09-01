/*icp学习，align（cloud），返回的是input点云经过配准后的结果，也就是说配
准是动input，使其与target重叠，getFitnessScore得到的是距离和*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
using std::cin;
using std::cout;
using std::endl;
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>(5, 1));
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
        // point.x += 0.7f;
        point.x += 0.1 * rand() / (RAND_MAX + 1.0f);
    cout << "Transformed " << cloud_in->size() << " data points:" << endl;
    for (auto point : *cloud_out)
        cout << point << endl;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    pcl::PointCloud<pcl::PointXYZ> Final;

    icp.align(Final);
    cout << "Final: " << endl;
    for (auto point : Final)
        cout << point << endl;

    cout << "has converged:" << icp.hasConverged() << " source: " << icp.getFitnessScore() << endl;

    cout << icp.getFinalTransformation() << endl;
    return 0;
}