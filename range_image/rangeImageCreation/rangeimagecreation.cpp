#include <pcl/range_image/range_image.h>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/common/common.h>
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (float y = -500.0f; y < 500.0f; y += 1.0f)
    {
        for (float z = -500.0f; z < 500.0f; z += 1.0f)
        {
            pcl::PointXYZ point;
            point.x = 2000.0f - y + z;un
            point.y = y;
            point.z = z;
            cloud.points.push_back(point);
        }
    }
    cloud.width = cloud.size();
    cloud.height = 1;

    float angularResolution = (float)(1.0f * (M_PI / 180.0f)); //   1.0 degree in radians
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));   // 360.0 degree in radians
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel = 0.00;
    float minRange = 0.0f;
    int borderSize = 1;
    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    pcl::visualization::RangeImageVisualizer rangeimageviwer("range image");
    pcl::visualization::PCLVisualizer cloudviwer("cloud");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr(&cloud);
    cloudviwer.addPointCloud<pcl::PointXYZ>(cloudptr, "test");
    // rangeimageviwer.spin();
    std::cout << rangeImage << "\n";

    // std::vector<pcl::visualization::Camera> mycamera;
    while (1)
    {
        rangeimageviwer.showRangeImage(rangeImage);
        cloudviwer.spin();
        // cloudviwer.getCameras(mycamera);
    }
}