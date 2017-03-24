#include <boost/lexical_cast.hpp>
#include "reflect_transformer.h"


ReflectTransformer::ReflectTransformer(const std::string& plane_normal_str) {
    boost::sregex_iterator iter_plane_normal(plane_normal_str.begin(), plane_normal_str.end(),
                                             BaseTransformer::three_tuple_match);
    Eigen::Vector3d plane_normal(boost::lexical_cast<double>(iter_plane_normal->str(1)),
                                 boost::lexical_cast<double>(iter_plane_normal->str(2)),
                                 boost::lexical_cast<double>(iter_plane_normal->str(3)));
    this->plane_normal = plane_normal.normalized();
}

unsigned int ReflectTransformer::getNumTransforms() {
    return 2;
}

std::string ReflectTransformer::getFilenameSuffix(unsigned int transform_idx) {
    if (transform_idx > 0) {
        std::ostringstream suffix;
        suffix.precision(5);
        suffix << "reflect";
        suffix << this->plane_normal.x() << ",";
        suffix << this->plane_normal.y() << ",";
        suffix << this->plane_normal.z();
        return suffix.str();
    } else {
        return std::string();
    }
}

void ReflectTransformer::doTransform(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                                     pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                                     unsigned int transform_idx) {
    if (transform_idx > 0) {
        // Householder matrix equation
        Eigen::Matrix3d transform = Eigen::Matrix3d::Identity() - (2 * this->plane_normal *
                                                                   this->plane_normal.transpose());
        pcl::transformPointCloud(cloud_in, cloud_out, Eigen::Affine3d(transform));
    } else {
        cloud_out = cloud_in;
    }
}
