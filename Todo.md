middlebury model에서
    oCamS-1CGN-U-F 는 right_image가 grayscale임
    >> reshape시 H,W,3이 아닌 H,W,1로 해야함.
    >> 이후 hitnet에서 convert할 때에, GRAY2BGR거친 다음에 BGR2RGB 해야함.
    kitti_util.py 에 if문으로 조건 추가 완료
    

input image is 1280*720. we want 1280*360
image2 = image2[179:539,:,:]
image3 = image3[179:539,:,:]
이미지를 임의로 반 잘랐을 때의 문제점.
== calibration 적용이 이미지를 되돌리는 의미인데, 반 잘라낸 이미지에선 이 값의 의미없어짐.
그럼 결국 lidar생성을 반 만 하는 방법이있는데 연산이 아까움




cd catkin_ws
catkin_make
source devel/setup.bash
roslaunch ocams_1cgn ocams_ros.launch


cd catkin_ws
source devel/setup.bash
roslaunch ocams_1cgn ocams_ros.launch

cd catkin_ws
source devel/setup.bash
roslaunch ocams_1cgn calibration.launch
