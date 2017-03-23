#pragma once

#include <boost/regex.hpp>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

class BaseTransformer {
public:
    static boost::regex three_tuple_match;

    virtual unsigned int getNumTransforms() = 0;
    virtual std::string getFilenameSuffix(unsigned int transform_idx) = 0;
    virtual void doTransform(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                             pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                             unsigned int transform_idx) = 0;
    virtual ~BaseTransformer(){};
};
