#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
// test git push

void showHelp(char *program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}

// This is the main function
int main(int argc, char **argv)
{

  if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
  {
    showHelp(argv[0]);
    return 0;
  }

  // Fetch point cloud filename in arguments | Works with PCD and PLY files
  std::vector<int> filenames;
  bool file_is_pcd = false;
  //他默认只输入一个文件，所以要么为1要么为0，为1就是检测到了，为0就是没检测到，实际上当文件数比较多的时候
  // filenames是一个vector，存储有所有满足要求的下标
  filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

  if (filenames.size() != 1)
  {
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

    if (filenames.size() != 1)
    {
      showHelp(argv[0]);
      return -1;
    }
    else
    {
      file_is_pcd = true;
    }
  }

  // Load file | Works with PCD and PLY files
  // pcl::PointCloud<pcl::PointXYZ>::Ptr 其中的ptr是创建一个智能指针，类似于python中的变量，当没有人再使用它的时候就会自动销毁
  // source_cloud (new pcl::PointCloud<pcl::PointXYZ> ())其中source_cloud是类名，括号里面的是初始化方式的一种，叫拷贝构造函数初始化，
  //他会调用括号内对象的拷贝构造函数，对待初始化变量初始化，new是开辟一个空间，并返回指针。
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  if (file_is_pcd)
  {
    // loadPCDFile是个内联函数，本质是调用了PCDReader.read（），read函数返回值为0（成功）或-1（失败），
    //这部分运行结束后，pcdfile里的点云数据被存储在source_cloud这个指针内（都是引用传递）
    if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud) < 0)
    {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                << std::endl;
      showHelp(argv[0]);
      return -1;
    }
  }
  else
  {
    if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud) < 0)
    {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl
                << std::endl;
      showHelp(argv[0]);
      return -1;
    }
  }

  /* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
  */
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  float theta = M_PI / 4; // The angle of rotation in radians
  transform_1(0, 0) = std::cos(theta);
  transform_1(0, 1) = -sin(theta);
  transform_1(1, 0) = sin(theta);
  transform_1(1, 1) = std::cos(theta);
  //    (row, column)

  // Define a translation of 2.5 meters on the x axis.
  transform_1(0, 3) = 2.5;

  // Print the transformation
  printf("Method #1: using a Matrix4f\n");
  std::cout << transform_1 << std::endl;

  /*  METHOD #2: Using a Affine3f
    This method is easier and less error prone
  */
  // Affine3f等价于Transform<float,3,Affine> 其中affine代表维度+1，3x3矩阵变为4x4,多出的用0001替代
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << 2.5, 0.0, 0.0;

  // The same rotation matrix as before; theta radians around Z axis
  transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

  // Print the transformation
  printf("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

  // Visualization
  printf("\nPoint cloud colors :  white  = original point cloud\n"
         "                        red  = transformed point cloud\n");
  pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

  // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
  viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

  viewer.addCoordinateSystem(1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  // viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped())
  { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce();
  }

  return 0;
}