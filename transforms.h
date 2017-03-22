#pragma once

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

void rotatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& axis,
                      float radians);

void scalePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float scale_x, float scale_y,
                     float scale_z);

void reflectPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       const Eigen::Vector3f& plane_normal);
