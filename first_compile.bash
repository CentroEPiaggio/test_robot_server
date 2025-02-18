vcs import src/franka_ros2 < src/franka_ros2/franka.repos --recursive --skip-existing
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release franka_ros2
colcon build