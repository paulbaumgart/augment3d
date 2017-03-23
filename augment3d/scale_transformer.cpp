#include <boost/lexical_cast.hpp>
#include "scale_transformer.h"


ScaleTransformer::ScaleTransformer(const std::string& from_str, const std::string& to_str,
                                   unsigned int steps) {
    boost::sregex_iterator iter_from(from_str.begin(), from_str.end(),
                                     BaseTransformer::three_tuple_match);
    double from_x = boost::lexical_cast<double>(iter_from->str(1)),
           from_y = boost::lexical_cast<double>(iter_from->str(2)),
           from_z = boost::lexical_cast<double>(iter_from->str(3));

    boost::sregex_iterator iter_to(to_str.begin(), to_str.end(),
                                   BaseTransformer::three_tuple_match);
    double to_x = boost::lexical_cast<double>(iter_to->str(1)),
           to_y = boost::lexical_cast<double>(iter_to->str(2)),
           to_z = boost::lexical_cast<double>(iter_to->str(3));

    for (int i = 0; i < steps; i++) {
        double fraction = (double)i / (steps - 1);
        double scale_x = fraction * (to_x - from_x) + from_x,
               scale_y = fraction * (to_y - from_y) + from_y,
               scale_z = fraction * (to_z - from_z) + from_z;
        this->scalings.push_back(boost::tuple<double, double, double>(scale_x, scale_y, scale_z));
    }
}

unsigned int ScaleTransformer::getNumTransforms() {
    return this->scalings.size();
}

std::string ScaleTransformer::getFilenameSuffix(unsigned int transform_idx) {
    boost::tuple<double, double, double> xyz = this->scalings.at(transform_idx);
    double scale_x = xyz.get<0>(),
           scale_y = xyz.get<1>(),
           scale_z = xyz.get<2>();
    if (scale_x == scale_y == scale_z == 1) { // scale of 1,1,1 means the transform has no effect
        return std::string();
    } else {
        std::ostringstream suffix;
        suffix.precision(5);
        suffix << "scale" << scale_x << "," << scale_y << "," << scale_z;
        return suffix.str();
    }
}

void ScaleTransformer::doTransform(const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
                                   pcl::PointCloud<pcl::PointXYZ>& cloud_out,
                                   unsigned int transform_idx) {
    boost::tuple<double, double, double> xyz = this->scalings.at(transform_idx);
    Eigen::Affine3d transform(Eigen::Scaling(xyz.get<0>(), xyz.get<1>(), xyz.get<2>()));
    pcl::transformPointCloud(cloud_in, cloud_out, transform);
}
