#include <iostream>
#include <pcl/io/pcd_io.h>
// #include<pcl/common/io.h>
#include <thread>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
    pcl::PointCloud<pcl::Normal> n_cloudb;
    pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

    cloud_a.width = 5;
    cloud_a.height = cloud_b.height = n_cloudb.height = 1;
    cloud_a.points.resize(cloud_a.width * cloud_a.height);
    if (strcmp(argv[1], "-p") == 0)
    {
        cloud_b.width = 3;
        cloud_b.points.resize(cloud_b.width * cloud_b.height);
    }
    else
    {
        n_cloudb.width = 5;
        n_cloudb.points.resize(n_cloudb.width * n_cloudb.height);
    }
    // 也可以用这个，可能不知道cl0uda如何迭代吧
    for (auto &&Point : cloud_a)
    {
        Point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        Point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        Point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    // for (std::size_t i = 0; i < cloud_a.size(); ++i)
    // {
    //     cloud_a[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    //     cloud_a[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    //     cloud_a[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    // }
    std::cerr << "Cloud A: " << std::endl;
    for (std::size_t i = 0; i < cloud_a.size(); ++i)
        std::cerr << "    " << cloud_a[i].x << " " << cloud_a[i].y << " " << cloud_a[i].z << std::endl;

    if (strcmp(argv[1], "-p") == 0)
    {
        for (std::size_t i = 0; i < cloud_b.size(); ++i)
        {
            cloud_b[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud_b[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud_b[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
        }
    }

    else
    { //也可以用这个 可能判定没有范围吧
        for (auto &&Point : n_cloudb)
        {
            Point.normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
            Point.normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
            Point.normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
        }
        // for (std::size_t i = 0; i < n_cloudb.size(); ++i)
        // {
        //     n_cloudb[i].normal[0] = 1024 * rand() / (RAND_MAX + 1.0f);
        //     n_cloudb[i].normal[1] = 1024 * rand() / (RAND_MAX + 1.0f);
        //     n_cloudb[i].normal[2] = 1024 * rand() / (RAND_MAX + 1.0f);
        // }
    }

    std::cerr << "Cloud B: " << std::endl;
    if (strcmp(argv[1], "-p") == 0)
        for (std::size_t i = 0; i < cloud_b.size(); ++i)
            std::cerr << "    " << cloud_b[i].x << " " << cloud_b[i].y << " " << cloud_b[i].z << std::endl;
    else
        for (std::size_t i = 0; i < n_cloudb.size(); ++i)
            std::cerr << "    " << n_cloudb[i].normal[0] << " " << n_cloudb[i].normal[1] << " " << n_cloudb[i].normal[2] << std::endl;
    if (strcmp(argv[1], "-p") == 0)
    {
        cloud_c = cloud_a;
        cloud_c += cloud_b;
        std::cerr << "Cloud C: " << std::endl;
        for (std::size_t i = 0; i < cloud_c.size(); ++i)
            std::cerr << "    " << cloud_c[i].x << " " << cloud_c[i].y << " " << cloud_c[i].z << " " << std::endl;
    }
    else
    {
        pcl::concatenateFields(cloud_a, n_cloudb, p_n_cloud_c);
        std::cerr << "Cloud C: " << std::endl;
        for (std::size_t i = 0; i < p_n_cloud_c.size(); ++i)
            std::cerr << "    " << p_n_cloud_c[i].x << " " << p_n_cloud_c[i].y << " " << p_n_cloud_c[i].z << " " << p_n_cloud_c[i].normal[0] << " " << p_n_cloud_c[i].normal[1] << " " << p_n_cloud_c[i].normal[2] << std::endl;
    }
    // shared_ptr一般初始化的时候就要给一个指针，我猜测不然的话他会默认没有引用，直接释放。
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    // std::cout<<cloud.use_count()<<std::endl;
    // *cloud = cloud_a;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(&cloud_a);

    cloud->resize(cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->size(); ++i)
    {

        std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud_a");
    viewer->initCameraParameters();
    //spin就是刷新页面
    viewer->spin();
    //下面这个实现的是只要界面没关就一直100ms刷新一次页面
    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce(100);
    // }
    return (0);
}
