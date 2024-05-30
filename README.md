# pcl_fusion
Launch RGB-D multiple inputs (tested with two realsense cameras):
- roslaunch multiple_scan_realsense.launch
  
Conversion to Pointclouds, downsample and fusion:
- perception_extra.sh

It must be considered that each point cloud must be associated with a frame of the robot with its respective orientation (robot.xacro)

Point clouds integration from different sources
![depth_1y2](https://github.com/Robcib-GIT/pcl_fusion/assets/57187750/bcb4dc0f-f6a8-4b4e-829f-96b70f8fe016)


Application to elevation maps reconstruction.
![Github](https://github.com/Robcib-GIT/pcl_fusion/assets/57187750/a24d090f-fc61-4491-96a6-74fcba9a0e2a)
