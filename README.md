
## PART1 : [Image Segmentation](#Image-Segmentation)
## PART2 : [Object Recognition and Labelling](#Object-Recognition-and-Labelling)

## Image Segmentation
### Cluster based segmentation using ROS node with PCL

This is a mini-project on creating a ```ROS node``` for image segmentation for objects kept on a cluttered table with PCL(Point Cloud Library). It is run and tested on the Udacity simulator ```robo-nd``` environment.

## Details
- [Project Heads-up](#Project-Heads-up)
- [Setup](#Setup)

### Project Heads-up
- The segmentation node publishes `sensor_msgs::PCLPointCloud2` messages to the `/pcl_objects topic`. The segmentation can be visualized in RViz by selecting that topic to view.

- Initially given a point cloud in the `sensor_msgs::PointCloud2` format of the one used in the `ransac_pointcloud` directory

- This needs to be converted to a `pcl::PCLPointCloud2` data type to perform calculations using the point cloud library.  conversion functions for this are provided in the `<pcl_conversions/pcl_conversions.h>` lib.

- For Cluster based segmentation we can use voxel grid filtering (see `ransac_pointcloud` directory) to condense the data without a large loss of accuracy.

- To focus on a region of interest in the z axis range (.5-1.1) (Table top) we use the passthrough filter (see `ransac_pointcloud` directory) function of the class.


- In the simulation environment, objects are placed on a flat planar table surface. Since we wish to segment objects on the surface, we can remove the table from the could. Since the table is planar, we can use the RANSAC geometric filtration algorithm and extract the outliers to remove points corresponding to the table face.

![Edge PC](https://github.com/ashutoshtiwari13/ROS-PCL-Segmentation/blob/master/sensor_stick/pcl3.png)

- From the above point cloud, its apparent that the RANSAC algorithm has left the table edge since its not planar with the table face. Another passthrough filter removing all points below the table height should get rid of the edge.

- Finally with the edge removed, we can use Euclidean cluster segmentation to identify each unique cluster.
After computation, a different color is assigned to each cluster for visualization purposes.

### Setup
The Gazebo world with the same table top and random objects from ```ransac_pointcloud``` directory is used. A simple stick robot with an RGB-D camera attached to its head via a pan-tilt joint is placed in front of the table.

NOTE : The underlying PCL filteration methods can be found in the [RANSAC_pointcloud](https://github.com/ashutoshtiwari13/ROS-PCL-Segmentation/tree/master/ransac_pointcloud) directory.

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
$ roslaunch sensor_stick robot_spawn.launch
```

![PCL](https://github.com/ashutoshtiwari13/ROS-PCL-Segmentation/blob/master/sensor_stick/pcl1.png)

7. Start the image segmentation node
```sh
$ rosrun sensor_stick segmentation
```
![PCL2](https://github.com/ashutoshtiwari13/ROS-PCL-Segmentation/blob/master/sensor_stick/pcl2.png)

## Object Recognition and Labelling
### Object Recognition and Labelling using features
Here, a classifier is trained to detect the objects kept on the table.

## Setup
1. We need to make use of the `object_recognize` directory cloned from the previous step.

Note :  Completing the PART1 of the project is a must to run PART2

2. Make sure you have all the dependencies resolved by using the rosdep install tool and run `catkin_make`:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ catkin_make
```

5. Add following to your .bashrc file.
```sh
export GAZEBO_MODEL_PATH=~/catkin_ws/src/object_recognize/models
source ~/catkin_ws/devel/setup.bash
```
Note : source the ```~/.bashrc ``` too if error occurs

6. Launch the `training.launch` file to bring up the Gazebo environment. Only a empty scene is seen with only a sensor stick robot
```sh
$ roslaunch object_recognize training.launch
```
7. Capture and save the features of each of the object in the environment and run the below command to generate a `training_set.sav` file

```sh
$ rosrun sensor_stick capture_features.py
```

8. For Training, make sure to install the `sklearn & scipy` packages
```sh
pip install sklearn scipy
```
Run the `train_svm.py` model to train an SVM classifier on your labeled set of features.
```sh
$ rosrun sensor_stick train_svm.py
```
