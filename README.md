# gazebo_turtlebot_controller

## Vikram Setty (119696897)
Deliverables for the ROS 2 Programming Assignment 'Working with Gazebo' assignment as a part of the course ENPM808X at the University of Maryland.

### Overview
This project/package allows you to use ROS2 to simulate a robot (Turtlebot 3) that avoids hitting obstacles in its spawned world (for this package, the `turtlebot3_gazebo` world). On sensing an obstacle in front of it in close vicinity, the Turtlebot would turn. In all other scenarios, it would continue to go straight with a constant velocity.

### Dependencies
Assuming you are already on an Ubuntu 22.04 system, some of the dependencies for this project to run the package include the following:
- ROS 2 Humble
- Turtlebot 3 ROS 2 Package
- Gazebo Simulator

To, install these dependencies, run the following commands below.
```sh
sudo apt -y install ros-humble-gazebo-ros-pkgs
sudo apt -y install ros-humble-turtlebot3*
sudo apt -y install ros-humble-turtlebot4-desktop
```

### Building and Running
To build and run the simulation, run the following commands.
Firstly, navigate to the source code folder in your workspace ([ros2_ws]/src) and clone the repository.
```sh
# Navigate to the src folder of the workspace:
  cd [ros2_ws]/src
# Clone the repository
  git clone https://github.com/vikrams169/gazebo_turtlebot_controller.git
```
After that, go back to the highest level directory in the workspace and build the package and then install the package.
```sh
# Go back to the ros2 workspace:
  cd ../..
# Make sure there are no dangling dependencies using rosdep
  rosdep install -i --from-path src --rosdistro humble -y
# Build the package:
  colcon build --packages-select gazebo_turtlebot_controller
# Install the package:
  source install/setup.bash
```
To run the launch file that executes the simulation, run the following command.
```sh
# Run the Simulation
  ros2 launch gazebo_turtlebot_controller gazebo_sim.launch.py record_bag:=1
```
In case it is desired to run the simulation without recording to the bag file, change the `record_bag` argument in the above command to `0`. Also, if running the simulation again, make sure to delete the newly created bag directory `my_bag`.

To get more information about the bag file created on running the simulation (if the appropriate flag was used), run the commands below.
```sh
# Get more information about the bag file
  ros2 bag info my_bag
# Play the bag file
  ros2 bag play my_bag
```
An example of a bag file that may be obtained is shown in `results/my_bag`.

### Google Coding Style Verification
To check how the written code conforms to the Google C++ style guide, look at the `results/cpplint_output.txt` (path relative to the package directory in the `src` folder of your workspace) file to see the output on using the *Cpplint* tool on this project. You should not be able to see any issues or problems, with all the files processed successfully.

This can be self-verified as well by running the following command in the highest-level directory of the project.
```sh
# Install Cpplint(ignore if already installed):
  sudo apt install cpplint
# Go to the package in the workspace
  cd ~/[ros2_ws]/src/gazebo_turtlebot_controller
# Self-check Google code style conformity using Cpplint:
  cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp
```

On running the above command, you should see the same output in the `results/cpplint_output.txt` file.

### Static Code Analysis
To check the static code analysis of this project, check the `results/cppcheck_output.txt` (path relative to the package directory in the `src` folder of your workspace) file to see the output on using the *Cppcheck* tool. You should not be able to see any issues or problems, with all the files checked successfully.

This can be self-verified as well by running the following command in the highest-level directory of the project.
```sh
# Install Cppcheck (ignore if already installed):
  sudo apt install cppcheck
# Go to the package in the workspace
  cd ~/[ros2_ws]/src/gazebo_turtlebot_controller
# Self-check the static code analysis using Cppcheck:
  cppcheck --enable=all --std=c++11 --std=c++17 --enable=information --check-config --suppress=missingInclude --suppress=*:*test*/ --suppress=unmatchedSuppression $( find . -name *.cpp | grep -vE -e "^./build/")
```