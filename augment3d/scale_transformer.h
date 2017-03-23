#pragma once

#include <boost/tuple/tuple.hpp>
#include "base_transformer.h"

class ScaleTransformer : public BaseTransformer {
    std::vector<boost::tuple<double, double, double> > scalings;
public:
    ScaleTransformer(const std::string& from_str, const std::string& to_str, unsigned int steps);
    virtual unsigned int getNumTransforms();
    virtual std::string getFilenameSuffix(unsigned int transform_idx);
    virtual void doTransform(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                             pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                             unsigned int transform_idx);
};
