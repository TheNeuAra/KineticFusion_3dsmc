#include "DepthProcessor.h"
#include "VirtualSensor.h"
#include <open3d/Open3D.h>


int main() {
  // Initialize VirtualSensor to load RGB and depth data
  VirtualSensor sensor;
  if (!sensor.Init("../../Data/rgbd_dataset_freiburg1_xyz/")) {
    std::cerr << "Failed to initialize the sensor!" << std::endl;
    return -1;
  }

  // Initialize the TSDF volume for 3D reconstruction
  auto tsdf_volume =
      std::make_shared<open3d::pipelines::integration::ScalableTSDFVolume>(
          0.005, // Voxel size (5mm resolution)
          0.04,  // Truncation distance (4cm)
          open3d::pipelines::integration::TSDFVolumeColorType::RGB8);

  // Process each frame from the dataset
  while (sensor.ProcessNextFrame()) {
    // Get raw depth and RGB data from the current frame
    float *rawDepth = sensor.GetDepth();
    BYTE *rawColor = sensor.GetColorRGBX();
    int width = sensor.GetDepthImageWidth();
    int height = sensor.GetDepthImageHeight();

    // Convert raw depth data to OpenCV Mat
    cv::Mat depthMap(height, width, CV_32F, rawDepth);

    // Apply depth processing: filter and fill invalid values
    cv::Mat filteredDepth =
        DepthProcessor::BilateralFilter(depthMap, 5, 75.0, 75.0);
    cv::Mat filledDepth = DepthProcessor::FillInvalidDepth(filteredDepth);

    // Convert the processed depth map to an Open3D Image
    auto depth_image = std::make_shared<open3d::geometry::Image>();
    depth_image->Prepare(width, height, 1, sizeof(float));
    memcpy(depth_image->data_.data(), filledDepth.data,
           width * height * sizeof(float));

    // Convert the raw RGB data to an Open3D Image
    auto color_image = std::make_shared<open3d::geometry::Image>();
    color_image->Prepare(width, height, 3, 1);
    memcpy(color_image->data_.data(), rawColor, width * height * 3);

    // Create an RGBD image from the color and depth images
    auto rgbd_image = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
        *color_image, *depth_image, 1000.0, 3.0, false);

    // Get camera intrinsics and the current pose (extrinsics)
    open3d::camera::PinholeCameraIntrinsic intrinsic(
        width, height, sensor.GetDepthIntrinsics()(0, 0),
        sensor.GetDepthIntrinsics()(1, 1), sensor.GetDepthIntrinsics()(0, 2),
        sensor.GetDepthIntrinsics()(1, 2));
    Eigen::Matrix4f extrinsic = sensor.GetTrajectory();

    // Integrate the RGBD image into the TSDF volume
    tsdf_volume->Integrate(*rgbd_image, intrinsic, extrinsic);

    // Display processing progress
    std::cout << "Processed frame " << sensor.GetCurrentFrameCnt() << std::endl;
  }

  // Extract the triangular mesh from the TSDF volume
  auto mesh = tsdf_volume->ExtractTriangleMesh();
  mesh->ComputeVertexNormals();

  // Save and visualize the mesh
  open3d::io::WriteTriangleMesh("output_mesh.ply", *mesh);
  open3d::visualization::DrawGeometries({mesh}, "Extracted Mesh");

  std::cout << "Mesh saved to output_mesh.ply" << std::endl;

  return 0;
}
