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

Build the whycon tag detector and source the catkin workspace

```bash
cd ../..
catkin_make
cd src/lis-vision-flight
source ../../devel/setup.bash
```

(Optional: We recommend creating a virtual environment for this step)

Get the python dependencies with

```bash
python2 -m pip install --user -r software/lis-vision-flight-ros/requirements.txt
```

## Prepare PX4

We use Firmware Version 1.10.1, which you can find [here](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.10.1). You can load all parameter settings from the file [lis-vision-flight-ros/config/px4_parameters.params](lis-vision-flight-ros/config/px4_parameters.params) through QGroundControl (we use version 4.1.4, which you can find [here](https://github.com/mavlink/qgroundcontrol/releases/tag/v4.1.4)) or simply inspect them and adjust the values as you see fit. Please refer to the [PX4 parameter list](https://docs.px4.io/v1.10/en/advanced_config/parameter_reference.html) for more information on each of the parameters.

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

If you are using the Flir camera, we recommend using the [flir_camera_driver](https://github.com/ros-drivers/flir_camera_driver) for ROS. Make sure you first install the Spinnaker SDK, which you can get from the [Spinnaker SDK download page](https://www.flir.eu/support-center/iis/machine-vision/downloads/spinnaker-sdk-and-firmware-download/).

We recommend calibrating the camera with [Kalibr](https://github.com/ethz-asl/kalibr). This will help you to get the right camera parameters, which you can put in the [calibration config file](config/19308195.yaml).

We provide a launch file that allows triggering the camera from PX4 in [flir_camera.launch](launch/flir_camera.launch).

## Run

To run the controller as we did in the publication, you can then launch the camera ([camera launch file](lis-vision-flight-ros/launch/flir_camera.launch)), the connection of the Jetson nano to the PX4 Autopilot with mavros and the vision based controller ([controller launch file](lis-vision-flight-ros/launch/vision_gnss_controller.launch)) with

``` bash
roslaunch lis-vision-flight run.launch
```

