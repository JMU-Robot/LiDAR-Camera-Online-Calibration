# LiDAR-Camera-Online-Calibration

This is demo code for Paper "Scene-aware Online Calibration of LiDAR and Cameras for Driving Systems"

![Local Image](https://github.com/JMU-Robot/LiDAR-Camera-Online-Calibration/tree/main/pic "Our online calibration result compare to state-of-the-art online method")

## 1. Install dependencies

Before running livox_ros_driver, ROS and Livox-SDK must be installed.

### 1.1 ROS installation

For ROS installation, please refer to the ROS installation guide :

[ROS installation guide](https://www.ros.org/install/)

&ensp;&ensp;&ensp;&ensp;***Note :***

&ensp;&ensp;&ensp;&ensp;(1) Be sure to install the full version of ROS (ros-distro-desktop-full);

&ensp;&ensp;&ensp;&ensp;(2) There are 7 to 8 steps in ROS installation, please read the installation guide in detail;

### 1.2 Livox-SDK Installation

1. Download or clone [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK) from Github to local;

2. Refer to the corresponding [README.md](https://github.com/Livox-SDK/Livox-SDK/blob/master/README.md) document to install and run Livox-SDK;

## 2. Get and build livox_ros_driver

1. Get livox_ros_driver from GitHub :

　　`git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src`

&ensp;&ensp;&ensp;&ensp;***Note :***

&ensp;&ensp;&ensp;&ensp;Be sure to use the above command to clone the code to the local, otherwise it will compile error due to the file path problem.

2. Use the following command to build livox_ros_driver :

   ```bash
   cd ws_livox
   catkin_make
   ```

3. Use the following command to update the current ROS package environment :

&ensp;&ensp;&ensp;&ensp;`source ./devel/setup.sh`

```
```

## Usage

```
```

## Contributing



## License

MIT © 
