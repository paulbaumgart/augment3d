#include "gtest/gtest.h"
#include "../rotate_transformer.h"


class RotateTransformerTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        cloud.push_back(pcl::PointXYZ(1, 1, 1));
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
};

TEST_F(RotateTransformerTest, ThreeRotates) {
    RotateTransformer transformer("0,1,0", 0, 180, 3);

    EXPECT_EQ(transformer.getNumTransforms(), 3);

    EXPECT_EQ(transformer.getFilenameSuffix(0), "");
    EXPECT_EQ(transformer.getFilenameSuffix(1), "rotate0,1,0@90");
    EXPECT_EQ(transformer.getFilenameSuffix(2), "rotate0,1,0@180");

    pcl::PointCloud<pcl::PointXYZ> transformed1;
    transformer.doTransform(cloud, transformed1, 1);
    EXPECT_FLOAT_EQ(transformed1.at(0).x, 1);
    EXPECT_FLOAT_EQ(transformed1.at(0).y, 1);
    EXPECT_FLOAT_EQ(transformed1.at(0).z, -1);

    pcl::PointCloud<pcl::PointXYZ> transformed2;
    transformer.doTransform(cloud, transformed2, 2);

    EXPECT_FLOAT_EQ(transformed2.at(0).x, -1);
    EXPECT_FLOAT_EQ(transformed2.at(0).y, 1);
    EXPECT_FLOAT_EQ(transformed2.at(0).z, -1);
}

TEST_F(RotateTransformerTest, RotateAroundNonNormalizedAxis) {
    RotateTransformer transformer("0,0,2", 0, 180, 2);
    transformer.doTransform(cloud, cloud, 1);
    EXPECT_FLOAT_EQ(cloud.at(0).x, -1);
    EXPECT_FLOAT_EQ(cloud.at(0).y, -1);
    EXPECT_FLOAT_EQ(cloud.at(0).z, 1);
}
