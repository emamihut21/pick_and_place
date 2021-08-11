# PALLETIZING TASK
This project was made for Fanuc m6ib industrial robot, configured from [ROS.org](http://wiki.ros.org/fanuc). It is a Python project which makes a pallet of boxes appeared in MoveIt! plan. Every box appears random orriented on z coordinate, at a specified-from-keyboard position, the robot grabes it and creates a pallet. 
## Grab the box
![Picture1](https://user-images.githubusercontent.com/79519915/129092186-7acc5e8d-5dcc-47c4-bd2f-f687ba00269c.png)

## Place the box
![Picture2](https://user-images.githubusercontent.com/79519915/129092281-bf08f25b-649d-4c6c-8633-b54bd3d4d57e.png)

## Final pallet
![Picture3](https://user-images.githubusercontent.com/79519915/129092442-54665aae-58de-4ea6-8d66-9b561b06b020.png)

## Building the package
The following instructions assume that a [Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) has been created and the packages for [Fanuc robot](https://github.com/ros-industrial/fanuc) were built on kinetic branch. *Please follow the previously steps before building this one.*
```
# Clone the driver 
$ cd ~/catkin_ws/src
$ git clone https://github.com/emamihut21/pick_and_place.git

# Install dependencies
$ sudo apt update
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# Build the workspace
$ catkin_make

# Activate the workspace
$ source devel/setup.bash

# Launch the project
$ roslaunch pick_and_place palletizing.launch
```


