#include "transforms.h"

void rotatePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3f& axis,
                      float radians) {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(radians, axis.normalized()));
    pcl::transformPointCloud(*cloud, *cloud, transform);
}

void scalePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float scale_x, float scale_y,
                     float scale_z) {
    Eigen::Affine3f transform = Eigen::Affine3f(Eigen::Scaling(scale_x, scale_y, scale_z));
    pcl::transformPointCloud(*cloud, *cloud, transform);
}

void reflectPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       const Eigen::Vector3f& plane_normal) {
    Eigen::Matrix3f subtrahend = 2 * plane_normal.normalized() *
                                 plane_normal.normalized().transpose();
    // Householder matrix equation
    Eigen::Affine3f transform = Eigen::Affine3f(Eigen::Matrix3f::Identity() - subtrahend);
    pcl::transformPointCloud(*cloud, *cloud, transform);
}
