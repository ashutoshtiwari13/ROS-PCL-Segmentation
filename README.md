## Cluster based segmentation using ROS node with PCL

This is a mini-project for creating a ```ROS node``` for image segmentation on a cluttered table with PCL(Point Cloud Library). It is run and tested in the Udacity simulator ```robo-nd``` environment.

## Details

### Setup
The Gazebo world with the same table top and random objects from ```ransac_pointcloud``` directory is used. A simple stick robot with an RGB-D camera attached to its head via a pan-tilt joint is placed in front of the table.

1. Clone the repo to the ```/src``` directory of the ROS workspace.
```sh
git clone https://github.com/ashutoshtiwari13/ROS-PCL-Segmentation.git
```
2. Install the ```Cython``` utility
```sh
$ sudo pip install cython
````
3. Build and install ```pcl-python```

```sh
$ cd python-pcl
$ python setup.py build
$ sudo python setup.py install

$ sudo apt-get install pcl-tools
```

4. Make sure you have all the dependencies resolved by using the rosdep install tool and run catkin_make:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ catkin_make
```

5. Add following to your .bashrc file.
```sh
export GAZEBO_MODEL_PATH=~/catkin_ws/src/sensor_stick/models
source ~/catkin_ws/devel/setup.bash
```
Note : source the ```~/.bashrc ``` too if error occurs

6. Start the simulation with ROS Launch
```sh
$ roslaunch roslaunch sensor_stick robot_spawn.launch
```

![PCL](https://github.com/ashutoshtiwari13/ROS-PCL-Segmentation/tree/master/sensor_stick/pcl1.png)

7. Start the image segmentation node
```sh
$ rosrun sensor_stick segmentation
```
![PCL2](https://github.com/ashutoshtiwari13/ROS-PCL-Segmentation/tree/master/sensor_stick/pcl2.png)
