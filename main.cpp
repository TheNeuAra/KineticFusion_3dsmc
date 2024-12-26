
#include "VirtualSensor.h"


int main() {
  // Initialize VirtualSensor to load RGB and depth data
  VirtualSensor sensor;
  if (!sensor.Init("../data/rgbd_dataset_freiburg1_xyz/")) {
    std::cerr << "Failed to initialize the sensor!" << std::endl;
    return -1;
  }

  // Initialize the TSDF volume for 3D reconstruction

  while (sensor.ProcessNextFrame()) {

  }
  return 0;
}
