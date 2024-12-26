#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/io/ply_io.h>
#include <Eigen/Core>
#include <vector> //these headers will be handeled later by pch and i will also give a cmakelist for it
#include <cmath>

// these versionof tsdf still have soem problems about dealing with illigal value and initiallizing vars and adjusting weights 
//but i will handel it latter
namespace TSDFNamespace {

// Voxel structure to store TSDF and color
struct TSDFVoxel {
    float sdf;          // Signed Distance Fvalue
    float weight;       // Weight for fusion but we can add soem api to stress the order or FIFO logic and assign it to the weights here
    Eigen::Vector3f color; // to store RGB color
};

// TSDF volume parameters for outside adjustments in main 
struct TSDFParameters {
    float voxel_size;         // Voxel size to scale the points in cloud ptr
    float truncation_distance; // Truncation distance to limit range
    Eigen::Vector3i resolution; // size or resolution  of the volume 
};

// Initialize TSDF volume
std::vector<TSDFVoxel> InitializeTSDF(const TSDFParameters& params) {
    int total_voxels = params.resolution.x() * params.resolution.y() * params.resolution.z();
    return std::vector<TSDFVoxel>(total_voxels, {0.0f, 0.0f, Eigen::Vector3f(0, 0, 0)});
}
// i will use auto tsdfvolume = InitializeTSDF( params ) in main
// Update TSDF volume with a point cloud and pose
void IntegrateTSDF(std::vector<TSDFVoxel>& tsdf_volume,
                   const TSDFParameters& params,
                   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                   const Eigen::Matrix4f& pose) {
    Eigen::Matrix4f pose_inv = pose.inverse();

    for (const auto& point : cloud->points) {
        // Transform point to TSDF volume space
        Eigen::Vector4f p(point.x, point.y, point.z, 1.0f);
        p = pose_inv * p;

        // Compute voxel index
// may be we can add some speciall terms to exclude the MINF or NaN before geting idx
        int x_idx = static_cast<int>(p.x() / params.voxel_size);
        int y_idx = static_cast<int>(p.y() / params.voxel_size);
        int z_idx = static_cast<int>(p.z() / params.voxel_size);
//special cases for big cloud point but we should also give it a index or other MINF to handel over index issue rather than just avoid it
        if (x_idx < 0 || x_idx >= params.resolution.x() ||
            y_idx < 0 || y_idx >= params.resolution.y() ||
            z_idx < 0 || z_idx >= params.resolution.z())
            continue;

        int idx = x_idx + y_idx * params.resolution.x() +
                  z_idx * params.resolution.x() * params.resolution.y();

        // Compute SDF value
        float sdf = p.norm() - params.voxel_size;
        sdf = std::max(-params.truncation_distance, std::min(params.truncation_distance, sdf));

        // Weighted average update
        tsdf_volume[idx].sdf = (tsdf_volume[idx].sdf * tsdf_volume[idx].weight + sdf) / 
                               (tsdf_volume[idx].weight + 1.0f);
        tsdf_volume[idx].weight += 1.0f;

        // Update color
        tsdf_volume[idx].color = Eigen::Vector3f(point.r, point.g, point.b);
    }
}

// Extract mesh from TSDF using Marching Cubes
pcl::PolygonMesh ExtractMeshFromTSDF(const std::vector<TSDFVoxel>& tsdf_volume,
                                     const TSDFParameters& params) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    // Convert TSDF volume to point cloud
    for (int z = 0; z < params.resolution.z(); ++z) {
        for (int y = 0; y < params.resolution.y(); ++y) {
            for (int x = 0; x < params.resolution.x(); ++x) {
                int idx = x + y * params.resolution.x() + z * params.resolution.x() * params.resolution.y();
                if (tsdf_volume[idx].sdf < 0.0f) {
                    pcl::PointXYZRGB point;
                    point.x = x * params.voxel_size;
                    point.y = y * params.voxel_size;
                    point.z = z * params.voxel_size;
                    point.r = static_cast<uint8_t>(tsdf_volume[idx].color.x());
                    point.g = static_cast<uint8_t>(tsdf_volume[idx].color.y());
                    point.b = static_cast<uint8_t>(tsdf_volume[idx].color.z());
                    cloud.push_back(point);
                }
            }
        }
    }

    // Perform Marching Cubes
    pcl::MarchingCubes<pcl::PointXYZRGB> mc;
    mc.setInputCloud(cloud.makeShared());

    pcl::PolygonMesh mesh;
    mc.reconstruct(mesh);
    return mesh;
}

// Save mesh to file
void SaveMesh(const pcl::PolygonMesh& mesh, const std::string& filename) {
    pcl::io::savePLYFile(filename, mesh);
}

} // namespace TSDFNamespace

