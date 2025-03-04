vcs import src/franka_ros2 < src/franka_ros2/franka.repos --recursive --skip-existing
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select libfranka
source install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select franka_bringup franka_description franka_example_controllers franka_fr3_moveit_config franka_gazebo_bringup franka_ign_ros2_control franka_gripper franka_hardware franka_msgs
source install/setup.bash
colcon build --symlink-install

