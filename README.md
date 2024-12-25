In branch main the code is basiclly implemented by Open3D and OpenCV 

In this branch I will try to do it in another fassion and different libs (like pcl and pnglib etc) but the core ideal is the same: 

1 update the incoming data：use the depth and rgb data from cirtain path

2 (optional)generate point cloud & generate mesh ：do back projection from depth data a sparse point-cloud and therefore generate sparse TSDF, whcih is a standard format for voxel-grid integration.

3 pose estimation in real time(or read from file): it is treaky that in some cases the pose transition can be read from sensors like IMU but it-
  can also be calculated in realtime by doing ICP precess everytime between two consecutive depth pic. 
  
4 TSDF integration: this  can be done by weighted sum of voxel value and diveded by sum of weights (between every two consecutive voxel-grids world we created)

5 surface extraction: we will try to only extract the mesh surface of object not the whole grids world.

6 (optional)use QT or other methods to visualizre the result surface.

7 (optional) use CUDA to boost real-time capability.

