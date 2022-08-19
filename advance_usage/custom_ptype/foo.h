// foo.h
#ifndef PCL_FOO_
#define PCL_FOO_
#include <pcl/common/common.h>
template <typename PointT>
class Foo
{
public:
  void
  compute(const pcl::PointCloud<PointT> &input,
          pcl::PointCloud<PointT> &output);
};
#endif // PCL_FOO_