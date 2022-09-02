#include <pcl/memory.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
//#include <direct.h>

using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerGenericField;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//全局变量，作为可视化界面存在（项目小，搞几个全局变量没事）
pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2;
// pcl::visualization::PCLVisualizer test;

//构建一个结构体，这个似乎和class没啥区别，只是这个struct构成的都是public
struct PCD
{ // 点云共享指针
    PointCloud::Ptr cloud;
    // why can't ?!!!!!!!!!!!!!
    // PointCloud::Ptr cloud(new PointCloud);
    //名字
    std::string f_name;

    //应该是写了个构造函数，: 到括号间的是初始化参数，应该是调用了cloud 对象的构造函数进行初始化？，PCD()内为空，说明没有变量传入，{}为空说明不执行任何语句，这个构造函数存在的价值仅仅是为了让你在创造结构体时对cloud进行初始化（那为什么不这么写 PointCloud::Ptr cloud(new PointCloud)   等下试试 看看会不会报错   (破案了 不行 但不知道为什么不行)）
    PCD() : cloud(new PointCloud){};
};

// 一个用来对比名字的结构体？看谁的名字小
struct PCDCompartor
{
    bool operator()(const PCD &p1, const PCD &p2)
    {
        return (p1.f_name < p2.f_name);
    }
};
// 派生一个类，父类pcl::PointRepresentation含有纯虚函数，所以是抽象类，官网也说了要派生的话必须要重写copyToFloatArray这个纯虚函数，并且要指定nr_dimensions_维度，这个类本质上好像是把点改成向量的类。PointRepresentation provides a set of methods for converting a point structs/object into an n-dimensional vector.

class MyPointRepresentation : public ::pcl::PointRepresentation<PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
    MyPointRepresentation()
    {
        // Define the number of dimensions
        nr_dimensions_ = 4;
    }

    /* 加const表明只读，不会修改原数据，如果修改了const对象数据成员或者调用了非const函数会报错  主要是提高代码健壮性 详见
    https://blog.csdn.net/weixin_42562387/article/details/114017352?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-114017352-blog-7816977.pc_relevant_multi_platform_featuressortv2dupreplace&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-114017352-blog-7816977.pc_relevant_multi_platform_featuressortv2dupreplace&utm_relevant_index=1*/
    virtual void copyToFloatArray(const PointNormalT &p, float *out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
    p->removePointCloud("vp1_target");
    p->removePointCloud("vp1_source");

    PointCloudColorHandlerCustom<PointT> tgt_h(cloud_target, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> src_h(cloud_source, 255, 0, 0);

    p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
    p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

    PCL_INFO("Press q to begin the registration.\n");
    // 可能是调用了PCL_viewer的应用，其中q是退出界面,在这里反映的估计是按q退出等待，毕竟也是单线程 阻断了进行显示就没法处理数据了
    p->spin();
}

void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
    p->removePointCloud("source");
    p->removePointCloud("target");

    PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
    if (!tgt_color_handler.isCapable())
        PCL_WARN("Cannot create curvature color handler!");

    PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");
    if (!src_color_handler.isCapable())
        PCL_WARN("Cannot create curvature color handler!");

    p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
    p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

    p->spinOnce();
}

// std::vector<PCD,Eigen::aligned_allocator<PCD>> 首先是定义了一个vector 是PCD类型，由于PCD本身用到的结构的底层是Eigen，所以需要传入Eigen的内存管理方式，Eigen::aligned_allocator<PCD>，它本身又是个模板函数，所以传入PCD这个结构体
void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD>> &models)
{
    std::string extension(".pcd");
    for (int i = 1; i < argc; i++)
    {
        std::string fname = std::string(argv[i]);
        if (fname.size() < extension.size())
            continue;
        /*批量转换代码，对于一维来说有四个参数，要变换元素的起始、结束位置，结果存储的起始位置
        要用用的一元算符函数，也就是函数
        tolower本身是一个C++函数，可以把字符串中的大写全换为小写。
        (int (*)(int))是一个函数指针,函数返回值类型 (* 指针变量名) (函数参数列表);
        本身这外面又有个括号，可能是做强制类型转换，此时估计是不用命名变量名的就像（int ）强转不用给你强转的对象起名一样，把tolower这个函数改成了函数指针，指向tolower?
        详解见 http://c.biancheng.net/view/228.html*/
        // 直接写tolower也行
        // std::cout << "before:" << fname << std::endl;
        // std::transform(fname.begin(), fname.end(), fname.begin(), (int (*)(int))tolower);
        // std::transform(fname.begin(), fname.end(), fname.begin(), tolower);
        // std::cout << "after:" << fname << std::endl;
        // std::cout << "#######################" << std::endl;
        if (fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
        {
            PCD m;
            m.f_name = argv[i];
            pcl::io::loadPCDFile(argv[i], *m.cloud);
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);
            models.push_back(m);
        }
    }
}

//点云配准
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output,
               Eigen::Matrix4f &final_transform, bool downsample = false)
{
    PointCloud::Ptr src(new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);
    pcl::VoxelGrid<PointT> grid;
    if (downsample)
    {
        grid.setLeafSize(0.05, 0.05, 0.05);
        grid.setInputCloud(cloud_src);
        grid.filter(*src);

        grid.setInputCloud(cloud_tgt);
        grid.filter(*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }
    PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30);

    norm_est.setInputCloud(src);
    // Copy fields and point cloud data from cloud_in to cloud_out. 推测此时的src只有点数据没有法向量，points_with_normals_src只有法向量没有点数据，所以要把src拷贝到points_with_normals_src从而构造一个有点和法向量的点云
    //等下可视化一下看看            可视化失败！！！！          自己没给指针初始化，是个野指针，没实例化！！！！
    norm_est.compute(*points_with_normals_src);

    /* 根据Debug单步执行可知，这个代码的本质是吧src里所有的数据copy到points_with_normals_src
    而points_with_normals_src创建之初是没有XYZ坐标的 全都是0，0，0，1，其中1是为了内存对齐的*/
    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    // test.addPointCloud(src,"src",0);
    // 不知为何 cloud_src显示不出来
    // PointCloudColorHandlerCustom<PointT> cloud_src_hh(cloud_src, 0, 255, 0);
    // test.addPointCloud(cloud_src, cloud_src_hh, "cloud_src", 1);
    // test.spin();

    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues(alpha);

    //为什么是PointNormalT 不是PointCloudWithNormals类型
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon(1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance(0.1);
    // Set the point representation
    reg.setPointRepresentation(pcl::make_shared<const MyPointRepresentation>(point_representation));

    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(2);
    for (int i = 0; i < 30; ++i)
    {
        PCL_INFO("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);

        // accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation() * Ti;

        // if the difference between this transformation and the previous one
        // is smaller than the threshold, refine the process by reducing
        // the maximal correspondence distance
        // 获取源 <-> 目标中两个对应点之间的最大距离阈值。
        //如果距离大于此阈值，则在对齐过程中将忽略这些点。
        if (std::abs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

        prev = reg.getLastIncrementalTransformation();

        // visualize current state
        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
    }

    //
    // Get the transformation from target to source
    targetToSource = Ti.inverse();

    //
    // Transform target back in source frame
    pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

    p->removePointCloud("source");
    p->removePointCloud("target");

    PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
    p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
    p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);

    PCL_INFO("Press q to continue the registration.\n");
    p->spin();

    p->removePointCloud("source");
    p->removePointCloud("target");

    // add the source to the transformed target
    *output += *cloud_src;

    final_transform = targetToSource;
}

int main(int argc, char **argv)
{
    // Load data
    std::vector<PCD, Eigen::aligned_allocator<PCD>> data;
    loadData(argc, argv, data);

    // Check user input
    if (data.empty())
    {
        PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
        PCL_ERROR(
            "[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
        return (-1);
    }
    PCL_INFO("Loaded %d datasets.", (int)data.size());

    // Create a PCLVisualizer object
    p = new pcl::visualization::PCLVisualizer(argc, argv, "Pairwise Incremental Registration example");
    // 这里才是界面一分为二的部分！
    p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);
    p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

    PointCloud::Ptr result(new PointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

    for (std::size_t i = 1; i < data.size(); ++i)
    {
        source = data[i - 1].cloud;
        target = data[i].cloud;

        // Add visualization data
        showCloudsLeft(source, target);

        PointCloud::Ptr temp(new PointCloud);
        PCL_INFO("Aligning %s (%zu) with %s (%zu).\n", data[i - 1].f_name.c_str(),
                 static_cast<std::size_t>(source->size()), data[i].f_name.c_str(),
                 static_cast<std::size_t>(target->size()));
        pairAlign(source, target, temp, pairTransform, true);

        // transform current pair into the global transform
        pcl::transformPointCloud(*temp, *result, GlobalTransform);

        // update the global transform
        GlobalTransform *= pairTransform;

        // save aligned pair, transformed into the first cloud's frame
        std::stringstream ss;
        ss << i << ".pcd";
        pcl::io::savePCDFile(ss.str(), *result, true);
    }
    return 0;
}