# < Real Time Pseudo Lidar >
    source : https://github.com/mileyan/Pseudo_Lidar
    maker : Jiho LEE (R.O.KOREA) / jiho264@inu.ac.kr
------------------------------------------------------------------------------------
- Real Time Vision Based LiDAR
- This Code Can Create 16 FPS Point Cloud Topics On GTX 1080Ti
- Original source is <Pseudo_Lidar>. i edited it.
------------------------------------------------------------------------------------

## Process
### 1. Input.
1. Left color image.
    >>> save to 'dev/input.png'
2. Right color image.
3. calibration + camera info.
### 2. Create Disparity Map from Stereo Image.
>>> save to 'dev/input_to_disparity.png'
### 3. Create PointCloudXYZ from Disparity Map.
### 4. Convert PointCloudXYZRGB to PointCloud2 msgs.
>>> rostopic '/pseudo_lidar'
>>> frame_id : world

------------------------------------------------------------------------------------

## Requirements(ubuntu 20.04) :
### 1. cudnn
### 2. tensorflow, numpy, opencv(pip install ver.) ...
### 3. ROS noetic
### 4. OpenCV 3.4.18 (do opencvinstall.ipynb)
### 5. oCamS ROS pkg

------------------------------------------------------------------------------------

## How to run (coex_plv1.py)
1. roscore
2. rosbag play temp.bag -l
3. !python3 coex_plv1.py
4. run rviz
> /pseudo_lidar (pointcloud2); frame_id = 'world'
------------------------------------------------------------------------------------

## Reference
1. Pseudo LiDAR 
    - Create Pseudo LiDAR : https://github.com/mileyan/Pseudo_Lidar
    - Sparsify Pseudo LiDAR : https://github.com/mileyan/Pseudo_Lidar_V2
3. Ros node : http://wiki.ros.org/pcl/Overview
4. Pcl Helper (PointCloudXYZRGB to PointCloud2 msgs) - https://github.com/udacity/RoboND-Perception-Exercises#documentation-for-pcl_helperpy
5. make bag file from KITTI dataset - https://github.com/tomas789/kitti2bag
6. calibration.
    - P, R, velo_to_cam(R, T) : http://www.cvlibs.net/publications/Geiger2013IJRR.pdf
    - velo_to_cam(R, T)(line 201~203) - https://github.com/kuixu/kitti_object_vis/blob/master/kitti_util.py
7. CoEx : https://github.com/antabangun/coex

------------------------------------------------------------------------------------
The ones below are used only in the old version

------------------------------------------------------------------------------------
#### Function
##### 1. you can choose disparity model. (from Hitnet)
        #hitnet_depth = HitNet('/home/jiho/plv1realtime/dev/eth3d.pb', ModelType.eth3d, CameraConfig(0.1, 320))
        hitnet_depth = HitNet('/home/jiho/plv1realtime/dev/middlebury_d400.pb', ModelType.middlebury, CameraConfig(0.1, 320))

        runtime : eth3d >> middlebury
        resolution : eth3d << middlebury

##### 2. calibration source.
        dev/left.yami (from oCamS ROS pkg)
        in P, R

        dev/calib_velo_to_cam.txt (from kitti raw dataset)
        in R, T

#### How to run (run_realtime_ocam.py)
    1.
       cd catkin_ws
       catkin_make
       source devel/setup.bash
       roslaunch ocams_1cgn ocams_ros.launch
    2. rviz

    *** oCamS ros output ***
        left_image_raw : BGR
        right_image_raw : GRAY(not BGR)

#### How to run (run_realtime_ocam_python3.py) 
    1. roscore
    2. !python3 run_realtime_ocam_python3.py
    3. rviz

- Hitnet : https://github.com/ibaiGorordo/HITNET-Stereo-Depth-estimation    
- oCamS-1CGN-U ros pkg (Not in this directory) : http://withrobot.com/camera/ocams-1cgn-u/
- oCamS python3 code (Not in their github) : "WITHROBOT"<withrobot@withrobot.com>

#### oCam Setting
- If you want change camera resolution or frame_rate
>>> edit '../catkin_ws/src/ocams1_cgn/launch/ocams_ros.launch
>>> edit Parameter

- If you want change camera calibration 
>>> roslaunch ocams_1cgn calibration.launch
>>> you must have '8x6 3cm checker board' 
>>> print this paper https://markhedleyjones.com/projects/calibration-checkerboard-collection
>>> follow https://www.youtube.com/watch?v=DPENw80cVmI&ab_channel=WITHROBOTInc.