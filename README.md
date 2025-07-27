# limo_ros
This repository contains ROS packages for limo. 

<img src="limo_bringup/img/limo.jpg" width="640" height="208" /> 

## Packages
 
 
* limo_base: ROS wrapper for limo
* limo_bringup: launch and configuration files to start ROS nodes


## Build from source code
Clone the repository and catkin_make:
```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/agilexrobotics/limo_ros.git
    $ cd ..
    $ catkin_make
```


## Usage 1 (scanning)

* Start the base node for limo

    ```
    $ roslaunch limo_bringup limo_start.launch pub_odom_tf:=false
    ```


* Start the keyboard tele-op node

    ```
    $ roslaunch limo_bringup limo_teleop_keyboard.launch
    ```


* Open the Dabai_U3 camera

    ```
    $ roslaunch astra_camera dabai_u3.launch
    ```


* Run the Rtabmap scanning 

    ```
    $ roslaunch limo_bringup limo_rtabmap_orbbec.launch
    ```


* Open Rviz GUI for monitoring 

    ```
    $ roslaunch limo_bringup rtabmap_rviz.launch
    ```


## Usage 2 (navigation)

* Start the base node for limo

    ```
    $ roslaunch limo_bringup limo_start.launch pub_odom_tf:=false
    ```


* Open the Dabai_U3 camera

    ```
    $ roslaunch astra_camera dabai_u3.launch
    ```


* Run the Rtabmap localization

    ```
    $ roslaunch limo_bringup limo_rtabmap_orbbec.launch localization:=true
    ```


* Run the Rtabmap navigation (move_base)

    ```
    $ roslaunch limo_bringup limo_navigation_rtabmap.launch
    ```

* Open Rviz GUI for monitoring 

    ```
    $ roslaunch limo_bringup rtabmap_rviz.launch
    ```
