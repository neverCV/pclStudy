// impl/foo.hpp

#ifndef PCL_IMPL_FOO_
#define PCL_IMPL_FOO_
// #include <pcl/common/common.h>
#include "foo.h"

template <typename PointT> 
void Foo<PointT>::compute (const pcl::PointCloud<PointT> &input,
              pcl::PointCloud<PointT> &output)
{
  output = input;
}

#endif // PCL_IMPL_FOO_
