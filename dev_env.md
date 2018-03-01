## PyCharm-Pro based Dev Environment (MACOS host and Virtualbox VM).
We are following the steps [here](https://www.jetbrains.com/help/pycharm/remote-debugging.html) but adapt them for this project.
After cloning the project and opening it up in PyCharm Pro you need to:

1. Clone the project in the host. 
2. Download Ubuntu VM and configure the Guest Additions for MACOS according to 
[these](https://gist.github.com/pantelis/f8987db8967d738f64c4bf7136ac6a84) instructions. 
3. Configure Port Forwarding in Virtualbox as shown in ![vb](/imgs/virtualbox_settings.png "Virtualbox Port Forwarding")
4. In PyCharm define a Deployment configuration as shown in 
![pycharm-dep1](/imgs/pycharm_deployment_config.png "PyCharm Deployment Config") and set the path mappings appropriately. 
![pycharm-dep2](/imgs/pycharm_deployment_dir_mappings.png "PyCharm Dir Mapping Config") 
Please note that the path mappings shown 
are for the specific package and that you may set them differently in your environment to map the 
root dir of the repo. 
5. In PyCharm define a Run Configuration as shown in 
![pycharm-run](/imgs/pycharm_run_configuration.png "PyCharm Run Config") 
6. Copy from your Host Pycharm installation directory the pycharm-debug.egg (for MACOS High Sierra this is already done)
into a dir under the root directory of this repo).  

If the above steps are executed correctly we can now deploy the project into the VM by selecting the 
project and going to Tools -> Deployment -> Upload to udacity-vm 

We are now ready to launch three terminals. In the first terminal we will be running the main 
ros process roscore
```commandline
cd CarND-Capstone-Project/ros
source devel/setup.bash
roscore
```

In the second terminal we will be launching the main package:
```commandline
cd CarND-Capstone-Project/ros
catkin_make
source devel/setup.bash
roslaunch launch/styx.launch
```

We will use the 3rd terminal for any other ros commands or to inspect messages and logs. 
```commandline
cd CarND-Capstone-Project/ros
source devel/setup.bash
```

Steps for remote debugging (using waypoint_updater.py as example)

1. In your python script for the ROS package of interest include the following code that :
```python
import sys

# The following lines are specific to Pycharm Remote Debugging configuration (pydev) that allows
# the host os to run the IDE and the Simulator and the remote Ubuntu VM to run ROS.
if sys.platform == "darwin":
    # OSX
    sys.path.append('/home/student/CarND-Capstone-Project/macos/pycharm-debug.egg')
elif sys.platform == "linux" or sys.platform == "linux2":
    # linux
    sys.path.append('')
# elif platform == "win32":
#     # Windows
#     sys.path.append('/home/student/CarND-Capstone-Project/ros/win/pycharm-debug.egg')
import pydevd
```
2. In the same script in the line of interest, include the following line. Note that this line 
is the one quoted by PyCharm in the Run Configurations config step and the IP address shown 
is the IP address of the Host OS. 
```python
pydevd.settrace('192.168.1.220', port=6700, stdoutToServer=True, stderrToServer=True)
```
3. Launch the simulator in the host. 
4. In PyCharm launch the debugger. 
5. If in the second terminal you have already launch the styx node, do a CTRL-C and then relaunch. 
6. Repeat 4 and 5 for every debug session. 

#### Installation Instructions by Udacity

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

#### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images



### Running the code with the docker

To run the code with docker, here are the steps.

1.Start the script

```
./run.sh
```

2.You're now within the container and the current directory contains the source code of your project. You can compile and run your project:

```
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

But this can be simplified by just using these 2 aliases:

Here are some of the cool aliases that are defined in the container

```
udacity_make #Compiles the project
udacity_run #Executes the project
```

To run multiple sessions, you can do following:

```sh
docker exec -it udacity_carnd bash
```

OR very simple, just run the script:

```sh
./run.sh
```
