# Software

We provide instructions for the software setup. Here, we provide setting files for PX4 and the ROS files used for accurate vision based flight.

For instructions on how to setup a Jetson Nano with the custom carrier board, see [carrier_board_nano](./carrier_board_nano/).

## Install dependencies

First install ROS melodic following the [official instructions](http://wiki.ros.org/melodic/Installation).

Then, update the submodule with
```bash
git submodule update --init --recursive
```

Install the dependencies ROS dependencies for Whycon and our package with these two commands
```bash
rosdep install -y --from-path software/whycon
rosdep install -y --from-path software/lis-vision-flight-ros
```

Get the python dependencies with (however, we recommend creating a virtual environment for it first)

```bash
python2 -m pip install --user -r software/lis-vision-flight-ros/requirements.txt
```

## Prepare PX4

We rely on Firmware Version 1.10.1, which you can find [here](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.10.1). You can load all parameter settings from the file [software/lis-vision-flight-ros/config/px4_parameters.params](software/lis-vision-flight-ros/config/px4_parameters.params) through QGroundControl

## Software setup
