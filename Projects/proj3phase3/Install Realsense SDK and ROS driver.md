## Install Realsense SDK and ROS driver

1. cd into ~/workspace and pull github repo

   `cd ~/workspace`

   `git clone https://github.com/HKUST-Aerial-Robotics/HKUST-ELEC5660-Introduction-to-Aerial-Robotics.git `

2. build install realsense sdk

   1. cd into librealsense-2.50.0

      `cd /home/dji/workspace/HKUST-ELEC5660-Introduction-to-Aerial-Robotics/lab/realsense_sdk_driver/librealsense-2.50.0 `

   2. create build dir under librealsense-2.50.0

      `mkdir ./build`

   3. cd into build and compile
      `sudo cmake ../`
      ` sudo make -j4`
      `sudo make install`

3. cd into catkin workspace 

    `cd /home/dji/workspace/ELEC5660_lab_code/src`

   copy realsense ros package into workspace

   ​	`cp -r /home/dji/workspace/HKUST-ELEC5660-Introduction-to-Aerial-Robotics/lab/realsense_sdk_driver/realsense-ros-ros1-legacy ./ `

4. build realsense ros package

   `cd ../`

   `catkin make`

5. test realseense ros package

   `source ./devel/setup.bash`

   `roslaunch realsense2_camera rs_camera.launch`

   lunch rqt_image_view to see image is published or not

   `rqt_image_view`

   

