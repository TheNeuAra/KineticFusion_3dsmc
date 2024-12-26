
#include "VirtualSensor.h"
#include "TSDF.h"

int main() {
  // Initialize VirtualSensor to load RGB and depth data
  VirtualSensor sensor;
  if (!sensor.Init("../data/rgbd_dataset_freiburg1_xyz/")) {
    std::cerr << "Failed to initialize the sensor!" << std::endl;
    return -1;
  }

  /*
    for now,  {image acuisition: -load , -read,  -filter, -convert,  -icp,  -get poes esti} are done. and all 
    of them are encapsulated inside VirtualSensor.h. In THIS MAIN, I plan to do the tsdf, but I plan to do it also in a same fassion, which is 
    done by giving more name space  like "TSDF::" and it will be insatancialized in main to mesh and integrate but first I probably have to have a getter
    func in side virtualsensor.h eg. "getMyCurrCloud()" and then at main do some further job
  */

  while (sensor.ProcessNextFrame()) {

  }
  return 0;
}
