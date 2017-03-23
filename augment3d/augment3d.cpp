#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include "transformers.h"
#include "flags.h"

// Load point cloud from file: supports PCD and PLY formats
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const boost::filesystem::path& input_path) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    int load_file_result = -1;
    if (input_path.extension() == ".pcd") {
        load_file_result = pcl::io::loadPCDFile(input_path.string(), *cloud);
    } else {
        load_file_result = pcl::io::loadPLYFile(input_path.string(), *cloud);
    }
    if (load_file_result < 0) {
        return pcl::PointCloud<pcl::PointXYZ>::Ptr();
    } else {
        return cloud;
    }
}

// Write point cloud to file: supports PCD and PLY formats
int writePointCloud(const boost::filesystem::path& output_path,
                    const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    if (boost::filesystem::exists(output_path)) {
        std::cerr << "Skipping write because file exists: " << output_path << std::endl;
        return 0;
    }

    int write_file_result = -1;
    if (output_path.extension() == ".pcd") {
        write_file_result = pcl::io::savePCDFile(output_path.string(), cloud);
    } else {
        write_file_result = pcl::io::savePLYFile(output_path.string(), cloud);
    }

    if (write_file_result < 0) {
        std::cerr << "Error writing file: " << output_path << std::endl;
    } else {
        std::cerr << "Wrote file: " << output_path << std::endl;
    }
    return write_file_result;
}

std::string constructFileName(const boost::filesystem::path& input_path,
                              const std::string& suffix) {
    if (suffix.length() > 0) {
        return input_path.stem().string() + "_" + suffix + input_path.extension().string();
    } else {
        return input_path.string();
    }
}

int main (int argc, char** argv) {
    std::string program_name = argv[0];
    gflags::SetUsageMessage(program_name + " -reflect|-rotate|-scale input_file output_dir");
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    if (argc != 3)  { // gflags removes all command line flags from argc/argv
        gflags::ShowUsageWithFlags(argv[0]);
        return -1;
    }

    boost::filesystem::path input_file = boost::filesystem::path(argv[1]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = loadPointCloud(input_file);
    if (!input_cloud) {
        std::cerr << "Error loading point cloud " << input_file << std::endl;
        gflags::ShowUsageWithFlags(argv[0]);
        return -1;
    }

    boost::filesystem::path output_dir = boost::filesystem::path(argv[2]);
    if (!boost::filesystem::is_directory(output_dir)) {
        std::cerr << "Invalid output directory: " << output_dir << std::endl;
        gflags::ShowUsageWithFlags(argv[0]);
        return -1;
    }

    BaseTransformer* transformer;
    if (FLAGS_reflect) {
        transformer = new ReflectTransformer(FLAGS_reflect_normal);
    } else if (FLAGS_rotate) {
        transformer = new RotateTransformer(FLAGS_rotate_axis, FLAGS_rotate_from, FLAGS_rotate_to,
                                            FLAGS_rotate_steps);
    } else if (FLAGS_scale) {
        transformer = new ScaleTransformer(FLAGS_scale_from, FLAGS_scale_to, FLAGS_scale_steps);
    } else {
        std::cerr << "One of -reflect|-rotate|-scale must be specified." << std::endl;
        gflags::ShowUsageWithFlags(argv[0]);
        return -1;
    }

    for (int i = 0; i < transformer->getNumTransforms(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        transformer->doTransform(*input_cloud, *transformed_cloud, i);
        std::string output_file = constructFileName(input_file, transformer->getFilenameSuffix(i));
        writePointCloud(output_dir / output_file, *transformed_cloud);
    }

    delete transformer;

    return 0;
}
