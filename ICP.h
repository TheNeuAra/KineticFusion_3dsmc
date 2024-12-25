#pragma once
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

#include <pcl/point_types.h>
#include <Eigen/Core>

//I have to define a self namespace to preserve my own methodes that is used inside virtualsensor.h, this ICP is use to replace the readTrajectory function inside virtualsensor and 
//I would like to use pcl to calculate pose estimation and pass it to virtualsensor
namespace ICP {

// I need a previous point cloud prevCloud and a currentCloud and then I can give back a result defined as a 4f mateix "transformation"

bool CalculatePose(pcl::PointCloud<pcl::PointXYZ>::Ptr prevCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud, Eigen::Matrix4f& transformation) {
  //some special cases
    if (!prevCloud || !currentCloud) {
        std::cerr << "Point clouds cannot be null!" << std::endl;
        return false;
    }

    // craete icp handel from IterativeClosestPoint and it takes io of which only only in type pcl::PointXYZ possible, thats why I generate a pointcloud first and then transform it into meshgrid
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  
    icp.setInputSource(currentCloud);
    icp.setInputTarget(prevCloud);
    //create a "result" handel for forthur store
    pcl::PointCloud<pcl::PointXYZ> alignedCloud;
    icp.align(alignedCloud);

  // this is very insteresting thing that some times if the cloud is too dense or sparse this happens?
    if (!icp.hasConverged()) {
        std::cerr << "ICP did not converge!" << std::endl;
        return false;
    }

    transformation = icp.getFinalTransformation(); 
    return true;
}

} // namespace ICP
