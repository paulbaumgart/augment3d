#include <boost/lexical_cast.hpp>
#include <pcl/common/angles.h>
#include "rotate_transformer.h"


RotateTransformer::RotateTransformer(const std::string& axis_str, const double& from_degrees,
                                     const double& to_degrees, unsigned int steps) {
    boost::sregex_iterator iter_axis(axis_str.begin(), axis_str.end(),
                                     BaseTransformer::three_tuple_match);
    Eigen::Vector3d axis(boost::lexical_cast<double>(iter_axis->str(1)),
                         boost::lexical_cast<double>(iter_axis->str(2)),
                         boost::lexical_cast<double>(iter_axis->str(3)));
    for (int i = 0; i < steps; i++) {
        double degrees = (double)i / (steps - 1) * (to_degrees - from_degrees) + from_degrees;
        this->rotations.push_back(boost::tuple<Eigen::Vector3d, double>(axis.normalized(),
                                                                        degrees));
    }
}

unsigned int RotateTransformer::getNumTransforms() {
    return this->rotations.size();
}

std::string RotateTransformer::getFilenameSuffix(unsigned int transform_idx) {
    boost::tuple<Eigen::Vector3d, double> rotation_data = this->rotations.at(transform_idx);
    Eigen::Vector3d axis = rotation_data.get<0>();
    double degrees = rotation_data.get<1>();
    if (degrees == 0) { // a rotation angle of zero means this transform has no effect
        return std::string();
    } else {
        std::ostringstream suffix;
        suffix.precision(5);
        suffix << "rotate";
        suffix << axis.x() << "," << axis.y() << "," << axis.z() << "@" << degrees;
        return suffix.str();
    }
}

void RotateTransformer::doTransform(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                                    pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                                    unsigned int transform_idx) {
    boost::tuple<Eigen::Vector3d, double> rotation_data = this->rotations.at(transform_idx);
    Eigen::Vector3d axis = rotation_data.get<0>();
    double degrees = rotation_data.get<1>();
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.rotate(Eigen::AngleAxisd(pcl::deg2rad(degrees), axis));
    pcl::transformPointCloud(cloud_in, cloud_out, transform);
}
