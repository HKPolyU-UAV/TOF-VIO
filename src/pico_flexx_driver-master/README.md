# PMD CamBoard pico flexx Driver

![pixo_flexx_ros](https://ai.uni-bremen.de/wiki/_media/software/pico_flexx_ros.png)

## Author

- [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<wiedemeyer@cs.uni-bremen.de>>, [Institute for Artificial Intelligence](https://ai.uni-bremen.de/), University of Bremen

## Maintainers
- [Alexis Maldonado](https://ai.uni-bremen.de/team/alexis_maldonado), [Institute for Artificial Intelligence](https://ai.uni-bremen.de/), University of Bremen
- [Martin Günther](http://robotik.dfki-bremen.de/de/ueber-uns/mitarbeiter/magu02.html), [Robotics Innovation Center (RIC), DFKI](http://robotik.dfki-bremen.de/de/startseite.html).


## Table of contents
- [Description](#description)
- [Dependencies](#dependencies)
- [Install](#install)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)

## Description
This package is a ROS interface to the [CamBoard pico flexx](http://www.pmdtec.com/picoflexx/) from pmd.

Features:
- publishing point clouds, camera info, depth, ir and noise images on ROS topics
- support for dynamic reconfigure
- support for nodelets with zero copy transfers
- a launch file with nodelet manager and machine tag support
- optional static TF publisher

## Dependencies

- ROS Indigo or Kinetic (or newer should also work)
- [Royale SDK](http://www.pmdtec.com/picoflexx/) (recommended: latest version, at least 3.21)

## Status

The driver has been tested on:
 - Ubuntu 14.04 and 16.04
 - ROS Indigo and ROS Kinetic

## Install

1. Install ROS: [Instructions for Ubuntu 16.04](http://wiki.ros.org/kinetic/Installation/Ubuntu)
2. [Setup your ROS environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. Download the royale SDK from http://www.pmdtec.com/picoflexx/ and extract it

  * Ubuntu Trusty (ROS Indigo) requires libroyale version 3.11.0.42 or lower (see [#18](https://github.com/code-iai/pico_flexx_driver/issues/18)).
  * Ubuntu Xenial should work with the newest version of libroyale.

4. Extract the archive that matches your kernel architecture from the extracted SDK to `<catkin_ws>/src/pico_flexx_driver/royale`. You can find out what your kernel architecture is by running `uname -m`.

   ```
   cd <catkin_ws>/src/pico_flexx_driver/royale
   unzip <path_to_extracted_royale_sdk>/libroyale-<version_number>-LINUX-x86-64Bit.zip   # uname -m = x86_64
   # or:
   unzip <path_to_extracted_royale_sdk>/libroyale-<version_number>-LINUX-arm-32Bit.zip   # uname -m = armv7l
   # or...
   ```

5. Install the udev rules provided by the SDK

   ```
   cd <catkin_ws>/src/pico_flexx_driver/royale
   sudo cp libroyale-*/driver/udev/10-royale-ubuntu.rules /etc/udev/rules.d/
   ```

6. Run `catkin_make` or `catkin build` (if you prefer catkin_tools)
7. Plug in the CamBoard pico flexx device
8. Run `roslaunch pico_flexx_driver pico_flexx_driver.launch publish_tf:=true`
9. Start `rosrun rviz rviz`, set the `Fixed frame` to `pico_flexx_link` and add a `PointCloud2` and select `/pico_flexx/points`

*Note: The pico_flexx_driver automatically tries to use the most recent version that is extracted in `<catkin_ws>/src/pico_flexx_driver/royale`.
To use a newer release just extract it to that directory in addition to the previous one and recompile it with `catkin_make clean` and `catkin_make`.
If something does not work with a newer version, just delete the extracted directory and recompile it with `catkin_make clean` and `catkin_make`.*

## Usage

To start the pico flexx driver, please use the provided launch file:

`roslaunch pico_flexx_driver pico_flexx_driver.launch`

#### Parameters

The launch file has the following parameters:

- `base_name` (default="pico_flexx"):

  Name of the node. All topics will be advertised under this name.

- `sensor` (default=""):

  ID of the sensor that should be used. IDs of all connected devices are listed on startup.

- `use_case` (default="0"):

  ID of the use case. A list of supported use cases is listed on startup.

- `automatic_exposure` (default="true"):

  Enable or disable automatic exposure.

- `automatic_exposure_stream2` (default="true"):

  Enable or disable automatic exposure for stream 2.

- `exposure_time` (default="1000"):

  Exposure time. Only for manual exposure.

- `exposure_time_stream2` (default="1000"):

  Exposure time for stream 2. Only for manual exposure.

- `max_noise` (default="0.07"):

  Maximum allowed noise. Data with higher noise will be filtered out.

- `range_factor` (default="2.0"):

  Range of the 16-Bit mono image which should be mapped to the 0-255 range of the 8-Bit mono image. The resulting range is `range_factor` times the standard deviation around mean.

- `queue_size` (default="5"):

  Queue size for publisher.

- `publish_tf` (default="false"):

  Publish a static TF transform for the optical frame of the camera.

- `base_name_tf` (default="pico_flexx"):

  Base name of the tf frames.

- `machine` (default="localhost"):

  Machine on with the nodes should run.

- `define_machine` (default="true"):

  Whether the machine for localhost should be defined our not. Disable this if the launch file is included somewhere where machines are already defined.

- `nodelet_manager` (default="pico_flexx"):

  Name of the nodelet manager.

- `start_manager` (default="true"):

  Whether to start a nodelet manager our not. Disable this if a different nodelet manager should be used.

#### Dynamic reconfigure

Some parameters can be reconfigured during runtime, for example with `rosrun rqt_reconfigure rqt_reconfigure`. The reconfigurable parameters are:
- `use_case`:

  Choose from a list of use cases.

- `exposure_mode`, `exposure_mode_stream2`:

  `AUTOMATIC` or `MANUAL`.

- `exposure_time`, `exposure_time_stream2`:

  Exposure time. Only for manual exposure.

- `max_noise`:

  Maximum allowed noise. Data with higher noise will be filtered out.

- `range_factor`:

  Range of the 16-Bit mono image which should be mapped to the 0-255 range of the 8-Bit mono image. The resulting range is `range_factor` times the standard deviation around mean.

#### Topics

When a mixed mode use case is selected, the second stream for all topics below
is published under the `stream2` namespace (e.g.,
`/pico_flexx/stream2/points`). In mixed mode, both a low-range, high-noise,
high-frequency point cloud and a high-range, low-noise, low-frequency (5 Hz)
point cloud are published. The 5 Hz point cloud in mixed mode only allows a
maximum exposure time of 1300 microseconds, so it has slightly higher noise
than the 5 Hz point cloud in single mode at 2000 microseconds.

##### `/pico_flexx/camera_info`
Bandwidth: 0.37 KB per message (@5 Hz: ~2 KB/s, @45 Hz: ~ 17 KB/s)

This topic publishes the camera intrinsic parameters.

##### `/pico_flexx/image_depth`
Bandwidth: 153.28 KB per message (@5 Hz: ~766 KB/s, @45 Hz: ~ 6897 KB/s)

This is the distorted depth image. It is a 32-Bit float image where each pixel is a distance measured in meters along the optical axis.

##### `/pico_flexx/image_mono16`
Bandwidth: 76.67 KB per message (@5 Hz: ~383 KB/s, @45 Hz: ~ 3450 KB/s)

This is the distorted IR image. It is a 16-Bit image where each pixel is an intensity measurement.

##### `/pico_flexx/image_mono8`
Bandwidth: 38.37 KB per message (@5 Hz: ~192 KB/s, @45 Hz: ~ 1727 KB/s)

This is the distorted IR image. It is a 8-Bit image where each pixel is an intensity measurement.

##### `/pico_flexx/image_noise`
Bandwidth: 153.28 KB per message (@5 Hz: ~766 KB/s, @45 Hz: ~ 6897 KB/s)

This is the distorted noise image. It is a 32-Bit float image where each pixel is a noise value of the corresponding depth pixel (standard deviation, measured in meters).

##### `/pico_flexx/points`
Bandwidth: 720 KB per message (@5 Hz: ~3600 KB/s, @45 Hz: ~ 32400 KB/s)

This is the point cloud created by the sensor. It contains 6 fields in the following order: X, Y, Z, Noise (float), Intensity (16-Bit), Gray (8-Bit).
The 3D points themselves are undistorted, while the 2D coordinates of the points are distorted. The point cloud is organized, so that the each point belongs to the pixel with the same index in one of the other images.

## Troubleshooting

### Problems accessing two pico flexx at the same time

When you try to access two (or more) pico flexx cameras simultaneously from different processes, you get this error when starting the second driver instance:

```
[PicoFlexx::selectCamera] no cameras connected!
```

However, you only get this error when accessing the cameras *from different processes*, i.e., by starting two instances of the `pico_flexx_driver` node, or by running two instances of the `pico_flexx_nodelet` in two *different* nodelet managers. This seems to be a limitation of libroyale. Also see issue [#13](https://github.com/code-iai/pico_flexx_driver/issues/13).

**Workaround:** Start two instances of `pico_flexx_nodelet` in the *same* nodelet manager.
