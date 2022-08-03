# Software

We provide instructions for the software setup. Here, we provide setting files for PX4 and the ROS files used for accurate vision based flight.

For instructions on how to setup a Jetson Nano with the custom carrier board, see [carrier_board_nano](./carrier_board_nano/).

## Install dependencies

First, install ROS melodic following the [official instructions](http://wiki.ros.org/melodic/Installation).

Then, clone the repository and update the submodule with
```bash
git clone https://github.com/lis-epfl/lis-vision-flight.git
cd lis-vision-flight
git submodule update --init --recursive
```

Install the ROS dependencies for Whycon and our package with these two commands
```bash
rosdep install -y --from-path software/whycon
rosdep install -y --from-path software/lis-vision-flight-ros
```

Get the python dependencies with (however, we recommend creating a virtual environment for it first)

```bash
python2 -m pip install --user -r software/lis-vision-flight-ros/requirements.txt
```

## Prepare PX4

We use Firmware Version 1.10.1, which you can find [here](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.10.1). You can load all parameter settings from the file [lis-vision-flight-ros/config/px4_parameters.params](lis-vision-flight-ros/config/px4_parameters.params) through QGroundControl (we use version 4.1.4, which you can find [here](https://github.com/mavlink/qgroundcontrol/releases/tag/v4.1.4)) or simply inspect them and adjust the values as you see fit.

On the SD card in the PX4 Autopilot, edit (or create if it doesn't exit) the file `/etc/extras.txt` and add these lines
```
ekf2 stop
gps stop
gps start
px4flow stop
ll40ls stop
ll40ls start -b 4
sleep 8
px4flow start -a 0x45
ekf2 start

mavlink stream -d /dev/ttyS2 -r 100 -s ATTITUDE_QUATERNION
mavlink stream -d /dev/ttyS2 -r 50  -s LOCAL_POSITION_NED
mavlink stream -d /dev/ttyS2 -r 50  -s WIND_COV
mavlink stream -d /dev/ttyS2 -r 50  -s VFR_HUD
mavlink stream -d /dev/ttyS2 -r 50  -s ACTUATOR_CONTROL_TARGET0
mavlink stream -d /dev/ttyS2 -r 100 -s HIGHRES_IMU
mavlink stream -d /dev/ttyS2 -r 100 -s ATTITUDE
mavlink stream -d /dev/ttyS2 -r 50  -s NAV_CONTROLLER_OUTPUT
mavlink stream -d /dev/ttyS2 -r 50  -s GLOBAL_POSITION_INT
mavlink stream -d /dev/ttyS2 -r 50  -s GLOBAL_POSITION_SETPOINT_INT
mavlink stream -d /dev/ttyS2 -r 50  -s GPS_RAW_INT
mavlink stream -d /dev/ttyS2 -r 50  -s POSITION_TARGET_GLOBAL_INT
mavlink stream -d /dev/ttyS2 -r 50  -s SERVO_OUTPUT_RAW
mavlink stream -d /dev/ttyS2 -r 20  -s DISTANCE_SENSOR
```
This will set the right ports for the sensors and will set the required streaming rates.

## Flir camera

If you are using the Flir camera we suggest, we recommend using the [flir_camera_driver](https://github.com/ros-drivers/flir_camera_driver) for ROS. Make sure you first install the Spinnaker SDK, which you can get from the [Spinnaker SDK download page](https://www.flir.eu/support-center/iis/machine-vision/downloads/spinnaker-sdk-and-firmware-download/).

## Run

To run the controller, launch 
- roscore
- launch a camera capture node, possibly with `flir_camera_driver`
- launch an image_proc node to rectify the image

You can then launch our controller/whycon launch file to detect tags and run the guidance control with
``` bash
roslaunch lis-vision-flight vision_gnss_controller.launch
```