#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>

// 不支持XYZI类型！！！！！！！！！！！
// 似乎只支持XYZ 和XYZRGB系列？
int main(int argc, char **argv)
{

    //构建一个存储待压缩点云数据的共享指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWaitForCompress(new pcl::PointCloud<pcl::PointXYZ>());
    //构建一个存储压缩后点云数据的共享指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCompressed(new pcl::PointCloud<pcl::PointXYZ>());
    //点云数据可视化
    pcl::visualization::PCLVisualizer beforeCompress("beforeCompress");
    pcl::visualization::PCLVisualizer afterCompress("afterCompress");
    //数据压缩的编码器与解码器
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *PointCloudEncoder;
    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *PointCloudDecoder;

    std::stringstream compressedData;

    // afterCompress.close();

    // homework.pcd 是xyzi类型的
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("homework.pcd", *cloudWaitForCompress) == -1)
    {
        PCL_ERROR("打不开homework.pcd! 请输入争取的文件名！");
        return -1;
    }
    std::cout << "压缩前点的个数为： "
              << cloudWaitForCompress->width * cloudWaitForCompress->height
              << std::endl;

    // 对压缩前的点云进行显示
    beforeCompress.addPointCloud<pcl::PointXYZ>(cloudWaitForCompress, "beforeCompress");
    beforeCompress.initCameraParameters();

    // 开始对点云数据进行压缩
    bool showStatistics = true;

    pcl::io::compression_Profiles_e compressionConfig = pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR;
    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(compressionConfig, showStatistics);
    PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();

    PointCloudEncoder->encodePointCloud(cloudWaitForCompress, compressedData);
    // PointCloudEncoder->decodePointCloud(compressedData,cloudCompressed);
    PointCloudDecoder->decodePointCloud(compressedData, cloudCompressed);

    std::cout << "压缩后点的个数为： "
              << cloudCompressed->width * cloudCompressed->height
              << std::endl;

    afterCompress.addPointCloud<pcl::PointXYZ>(cloudCompressed, "afterCompress");
    afterCompress.initCameraParameters();
    while (!beforeCompress.wasStopped() && !afterCompress.wasStopped())
    {
        beforeCompress.spinOnce(100);
        afterCompress.spinOnce(100);
    }
}