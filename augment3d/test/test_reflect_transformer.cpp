#include "gtest/gtest.h"
#include "../reflect_transformer.h"


class ReflectTransformerTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        cloud.push_back(pcl::PointXYZ(1, 1, 1));
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
};

TEST_F(ReflectTransformerTest, ReflectOverUnitBasisPlaneNormal) {
    ReflectTransformer transformer("1,0,0");

    EXPECT_EQ(transformer.getNumTransforms(), 2);

    EXPECT_EQ(transformer.getFilenameSuffix(0), "");
    EXPECT_EQ(transformer.getFilenameSuffix(1), "reflect1,0,0");

    transformer.doTransform(cloud, cloud, 0);
    EXPECT_FLOAT_EQ(cloud.at(0).x, 1);
    EXPECT_FLOAT_EQ(cloud.at(0).y, 1);
    EXPECT_FLOAT_EQ(cloud.at(0).z, 1);

    transformer.doTransform(cloud, cloud, 1);
    EXPECT_FLOAT_EQ(cloud.at(0).x, -1);
    EXPECT_FLOAT_EQ(cloud.at(0).y, 1);
    EXPECT_FLOAT_EQ(cloud.at(0).z, 1);
}

TEST_F(ReflectTransformerTest, ReflectOverNonNormalizedPlaneNormal) {
    ReflectTransformer transformer("1,1,1");
    transformer.doTransform(cloud, cloud, 1);
    EXPECT_FLOAT_EQ(cloud.at(0).x, -1);
    EXPECT_FLOAT_EQ(cloud.at(0).y, -1);
    EXPECT_FLOAT_EQ(cloud.at(0).z, -1);
}
