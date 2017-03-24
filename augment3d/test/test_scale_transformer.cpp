#include "gtest/gtest.h"
#include "../scale_transformer.h"


class ScaleTransformerTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        cloud.push_back(pcl::PointXYZ(1, 1, 1));
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
};

TEST_F(ScaleTransformerTest, ThreeScales) {
    ScaleTransformer transformer("1,1,1", "1,2,3", 3);

    EXPECT_EQ(transformer.getNumTransforms(), 3);

    EXPECT_EQ(transformer.getFilenameSuffix(0), "");
    EXPECT_EQ(transformer.getFilenameSuffix(1), "scale1,1.5,2");
    EXPECT_EQ(transformer.getFilenameSuffix(2), "scale1,2,3");

    pcl::PointCloud<pcl::PointXYZ> transformed1;
    transformer.doTransform(cloud, transformed1, 1);
    EXPECT_FLOAT_EQ(transformed1.at(0).x, 1);
    EXPECT_FLOAT_EQ(transformed1.at(0).y, 1.5);
    EXPECT_FLOAT_EQ(transformed1.at(0).z, 2);

    pcl::PointCloud<pcl::PointXYZ> transformed2;
    transformer.doTransform(cloud, transformed2, 2);

    EXPECT_FLOAT_EQ(transformed2.at(0).x, 1);
    EXPECT_FLOAT_EQ(transformed2.at(0).y, 2);
    EXPECT_FLOAT_EQ(transformed2.at(0).z, 3);
}
