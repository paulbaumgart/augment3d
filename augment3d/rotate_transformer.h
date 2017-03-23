#pragma once

#include <boost/tuple/tuple.hpp>
#include "base_transformer.h"

class RotateTransformer : public BaseTransformer {
    std::vector<boost::tuple<Eigen::Vector3d, double> > rotations;
public:
    RotateTransformer(const std::string& axis_str, const double& from_degrees,
                      const double& to_degrees, unsigned int steps);
    virtual unsigned int getNumTransforms();
    virtual std::string getFilenameSuffix(unsigned int transform_idx);
    virtual void doTransform(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                             pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                             unsigned int transform_idx);
};
