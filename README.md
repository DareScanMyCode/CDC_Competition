# Team XMU MAC
- This is our initial submission.
- We are continuing to improve the performance and fix some bugs, and will submit a more stable version before the deadline.
- If there are any issues running the code, or if the performance is unsatisfactory, please contact us. See the emails in the end of this file.
- Xiamen Uninversity

## Something IMPORTANT
- In the Python file "caric_competition_xmu/src/inspector/get_points_in_faces.py", you may need to verify the Python path. We require Python 3, so please use "#!/usr/bin/env python3" here. This is the same as in "ppcom_router.py" and "odom2tf.py" within the caric_mission package.

- Running 'chmod +x get_points_in_faces.py' in step 2 below is essential!

- The initialization step may take up to 60 seconds (depending on the bounding boxes), after which the inspectors will take off. Thank you for your patience.

- After initialization, the console may pause for 10 seconds or longer, then the inspectors will take off. The phototakers will take off after receiving map information from the inspectors.

- Please launch the ppcom_router.py node.

## How to run:
  1. Transfer the project folder to your ws_caric/src directory and unzip it.
  2. run in terminal:
  ```bash
  cd ~/ws_caric
  catkin clean caric_competition_xmu # if you have build it before
  catkin build caric_competition_xmu
  catkin build caric_competition_xmu # maybe failed, please try again
  source devel/setup.bash
  cd src/caric_competition_xmu/src/inspector
  chmod +x get_points_in_faces.py  # essential
  roslaunch caric_competition_xmu xmu_launch.launch scenario:="mbs"
  roslaunch caric_competition_xmu xmu_launch.launch scenario:="hangar"
  roslaunch caric_competition_xmu xmu_launch.launch scenario:="crane"
  ```
  3. you can also modify the `scenario` in the launch files in "caric_competition_xmu/launch/xmu_all/xmu_launch.launch"
  4. `catkin clean` since we have modified the user-defined ros msgs.

## Testing
- Please, if the perfomace is significantly lower than the typical performance below, tell us.
### Settings
- System: Ubuntu 20.04 with ROS noedic
- Hardware: Laptop/PC with
  - AMD Ryzen 7 8845H, 32G RAM
  - Intel Core i7 12650H, 32G RAM
- Bounding boxes: Only one witch contains the main target in each scene.
### Typical Performance
- mbs
  - 1000 points, 600 score in 350s
- crane
  - 1000 points, 500 score in 300s
- hangar
  - 500 points, 230 score in 250s

## Some Issues
- We have observe that, somtimes, the program does not launch well. One method to solve this problem is that we can reload a terminate, and wait for some time for ros to kill the previous processes.
- We also observe that somtimes Lidar does not work well on our computers.
- "Error: The length of line segment AB is zero" is common, just ignore it.

