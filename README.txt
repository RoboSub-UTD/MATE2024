##assuming that ros2 is installed, run these commands:

sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src

##for the depth sensor:
pip3 install smbus2

#build the workspace
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)
