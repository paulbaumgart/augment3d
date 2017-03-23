#include "transforms.h"
#include "gtest/gtest.h"

class TransformTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        cloud->push_back(pcl::PointXYZ(1, 1, 1));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};


TEST_F(TransformTest, RotateAroundUnitBasisAxis) {
    rotatePointCloud(cloud, Eigen::Vector3f::UnitX(), M_PI);
    EXPECT_FLOAT_EQ(cloud->at(0).x, 1);
    EXPECT_FLOAT_EQ(cloud->at(0).y, -1);
    EXPECT_FLOAT_EQ(cloud->at(0).z, -1);
}

TEST_F(TransformTest, RotateAroundNonNormalizedAxis) {
    rotatePointCloud(cloud, Eigen::Vector3f(0, 0, 2), M_PI);
    EXPECT_FLOAT_EQ(cloud->at(0).x, -1);
    EXPECT_FLOAT_EQ(cloud->at(0).y, -1);
    EXPECT_FLOAT_EQ(cloud->at(0).z, 1);
}


TEST_F(TransformTest, Scale) {
    scalePointCloud(cloud, 1, 2, 3);
    EXPECT_FLOAT_EQ(cloud->at(0).x, 1);
    EXPECT_FLOAT_EQ(cloud->at(0).y, 2);
    EXPECT_FLOAT_EQ(cloud->at(0).z, 3);
}


TEST_F(TransformTest, ReflectOverUnitBasisPlaneNormal) {
    reflectPointCloud(cloud, Eigen::Vector3f::UnitX());
    EXPECT_FLOAT_EQ(cloud->at(0).x, -1);
    EXPECT_FLOAT_EQ(cloud->at(0).y, 1);
    EXPECT_FLOAT_EQ(cloud->at(0).z, 1);
}

TEST_F(TransformTest, ReflectOverNonNormalizedPlaneNormal) {
    reflectPointCloud(cloud, Eigen::Vector3f(1, 1, 1));
    EXPECT_FLOAT_EQ(cloud->at(0).x, -1);
    EXPECT_FLOAT_EQ(cloud->at(0).y, -1);
    EXPECT_FLOAT_EQ(cloud->at(0).z, -1);
}
