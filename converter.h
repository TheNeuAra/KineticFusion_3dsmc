
#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>  
#include <fstream>
#include <iostream>
#include <sstream>
#include <memory>
#include <vector>


namespace converter {

pcl::PointCloud<pcl::PointXYZ>::Ptr VirtualSensor::ConvertDepthToPointCloud(float* depthMap, unsigned int width, unsigned int height, 
                                                                            const Eigen::Matrix3f& intrinsics) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    float fx = intrinsics(0, 0), fy = intrinsics(1, 1);
    float cx = intrinsics(0, 2), cy = intrinsics(1, 2);

    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            float depth = depthMap[y * width + x];
            if (depth <= 0 || depth == MINF) continue; // jump the invalid point
            pcl::PointXYZ point;
            point.x = (x - cx) * depth / fx;
            point.y = (y - cy) * depth / fy;
            point.z = depth;
            cloud->points.push_back(point);
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1; // 单行点云
    cloud->is_dense = false;
    return cloud;
}

 // 对点云进行滤波处理
    static pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size = 0.01f) {
        // 1. 体素滤波
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*filtered_cloud);

        return filtered_cloud;
    }
}// end of name space converter
