#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char **argv)
{   //time(NULL)为返回当前系统时间，srand可以为rand设置随机种子
    srand(time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 1000;
    cloud->height = 1;
    //需要resize来从新分配容器？否则size是不变的
    cloud->points.resize(cloud->width * cloud->height);
    //随机生成数据
    for (auto &Point : *cloud)
    {
        Point.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        Point.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        Point.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }
    //构建一个kdtree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    //设置kdtree的点云，
    kdtree.setInputCloud(cloud);

    // 想要查找的点
    pcl::PointXYZ searchpoint;  
    searchpoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchpoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchpoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    //想要查找的个数
    int K = 10;
    //用来存储最近的K个点的下标
    std::vector<int> pointIdxNKNSearch(K);
    //用来存储最近的K个点与查询点间距离
    std::vector<float> pointNKNSqyaredDistance(K);

    std::cout << "K nearest neighbor search at (" << searchpoint.x
              << " " << searchpoint.y
              << " " << searchpoint.z
              << ") with K=" << K << std::endl;
    

    // 最近邻搜索，还有一个圆周搜索kdtree.radiusSearch（看看有那些点落在园内）
    if (kdtree.nearestKSearch(searchpoint, K, pointIdxNKNSearch, pointNKNSqyaredDistance))
    {

        // for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
        //     std::cout << "    " << (*cloud)[pointIdxNKNSearch[i]].x
        //               << " " << (*cloud)[pointIdxNKNSearch[i]].y
        //               << " " << (*cloud)[pointIdxNKNSearch[i]].z
        //               << " (squared distance: " << pointNKNSqyaredDistance[i] << ")" << std::endl;

        // for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
        //     std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
        //               << " " << cloud->points[pointIdxNKNSearch[i]].y
        //               << " " << cloud->points[pointIdxNKNSearch[i]].z
        //               << " (squared distance: " << pointNKNSqyaredDistance[i] << ")" << std::endl;

        // for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
        //     std::cout << "    " << cloud->data()[pointIdxNKNSearch[i]].x
        //               << " " << cloud->data()[pointIdxNKNSearch[i]].y
        //               << " " << cloud->data()[pointIdxNKNSearch[i]].z
        //               << " (squared distance: " << pointNKNSqyaredDistance[i] << ")" << std::endl;
        id_t();
        std::cout << "cloud->data():" << cloud->data() << std::endl;
        std::cout << "cloud:" << cloud << std::endl;
        std::cout << "&(cloud->points):" << &(cloud->points) << std::endl;

        std::cout << "(*cloud)[0]" << (*cloud)[0] << std::endl;
        std::cout << "cloud->data()[0]:" << cloud->data()[0] << std::endl;
        std::cout << "cloud->points[0]:" << cloud->points[0] << std::endl;
        //三者为等价访问
        std::cout << "&(*cloud)[0]" << &(*cloud)[0] << std::endl;
        std::cout << "&(cloud->data()[0]):" << &(cloud->data()[0]) << std::endl;
        std::cout << "&(cloud->points[0]):" << &(cloud->points[0]) << std::endl;
        
        }
}