#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "transforms.h"


// This function displays the help
void showHelp(char* program_name) {
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
}

std::vector<int> getFileArgvIndexes(int argc, char** argv, bool* file_is_pcd=NULL) {
    if (file_is_pcd) {
        *file_is_pcd = false;
    }

    std::vector<int> file_idxs;
    file_idxs = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
    if (file_idxs.size() < 1) {
        file_idxs = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
        if (file_is_pcd) {
            *file_is_pcd = true;
        }
    }
    return file_idxs;
}

// Load file: works with PCD and PLY files
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(char* filename, bool file_is_pcd) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    int result;
    if (file_is_pcd) {
        result = pcl::io::loadPCDFile(filename, *source_cloud);
    } else {
        result = pcl::io::loadPLYFile(filename, *source_cloud);
    }
    if (result < 0) {
        return pcl::PointCloud<pcl::PointXYZ>::Ptr();
    } else {
        return source_cloud;
    }
}


void displayPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int red=255, int green=0,
                       int blue=0) {
    pcl::visualization::PCLVisualizer viewer("Point Cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, red, green,
                                                                                  blue);
    viewer.addPointCloud(cloud, color_handler, "cloud");
    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                            "cloud");
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}


int main (int argc, char** argv) {
    // Show help
    if (pcl::console::find_switch(argc, argv, "-h") ||
        pcl::console::find_switch(argc, argv, "--help")) {
        showHelp(argv[0]);
        return 0;
    }

    bool file_is_pcd;
    std::vector<int> file_idxs = getFileArgvIndexes(argc, argv, &file_is_pcd);

    if (file_idxs.size() != 1)  {
        showHelp(argv[0]);
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loadPointCloud(argv[file_idxs[0]], file_is_pcd);
    if (!cloud) {
        std::cout << "Error loading point cloud " << argv[file_idxs[0]] << std::endl;
        showHelp(argv[0]);
        return -1;
    }

    rotatePointCloud(cloud, Eigen::Vector3f::UnitZ(), M_PI);
    reflectPointCloud(cloud, Eigen::Vector3f::UnitY());
    scalePointCloud(cloud, 2, 1, 1);

    displayPointCloud(cloud);

    return 0;
}
