#pragma once

#include "base_transformer.h"

class ReflectTransformer : public BaseTransformer {
    Eigen::Vector3d plane_normal;
public:
    ReflectTransformer(const std::string& plane_normal_str);
    virtual unsigned int getNumTransforms();
    virtual std::string getFilenameSuffix(unsigned int ignored);
    virtual void doTransform(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                             pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                             unsigned int transform_idx);
};
