# Gen2 ROS Driver Design Document

This is a living document and is subject to change based on implementation difficulties.

## Overview
Gen 2 API for DepthAI is more flexible and offers much more functionality. As such, switching from the previous API provides more power to downstream users.

## Goals and Non-Goals
This driver is created to satisfy an MVP, keeping extensibility in mind. As such, PRs will be welcome to extend the abilities of the driver, but not if they make it difficult to achieve the MVP scenario. No effort will be spent on testing the viability of the non-goals.

The non-goals of this driver are:
* safety certification
* support for non-Ubuntu LTS OS
* integration of application specific higher order functionality
* ease of porting to ROS2

The goals of the driver (as dictated by the MVP) are:
* robust and low overhead driver for DepthAI
* support for custom hardware (baseline and cameras)
* support for synchronized timestamps (trigger time) on different streams
* support for camera controls (Region of Interest, exposure/gain/white-balance, focus)
* support for decoding output tensor of MobileNetSSD

## Proposed Solution
The proposed solution will take advantage of nodelets in ROS, and will provide interfaces for data and sensor details for future extensions and custom processing and bring-up. We propose a 2 nodelet based solution:
* driver nodelet: handles the device lifecycle. Supports custom pipelines via pluginlib
* decoder nodelet: handles custom neural networks by allowing users to decode the output tensor as they wish

### Bringup
Since Gen2 allows a custom pipeline to be built, the Gen1 approach doesn't fit anymore. In order to allow them, we'll need a [pluginlib](http://wiki.ros.org/pluginlib) approach. A tututorial for writing and loading custom plugins is located [here](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin).

A custom baseclass, say `depthai_ros_driver::Pipeline` could be used for loading custom pipelines from custom classes with a name (that can be runtime modifiable using a parameter). The class will need to provide the details of `camera_streams`, `control_streams`, `input_streams` and `tensor_streams`:
* `camera_streams`: list of cameras that need to be published. The driver will use them for accessing images and converting them to ROS messages.
* `control_streams`: list of cameras that need to be controlled. The driver will use them for adjusting the RoI, exposure time, gain, white balance, etc.
* `input_streams`: list of flat-tensors that the driver will send to the camera (virtual cameras). This will allow for testing as well as using the Movidius chip for custom workloads
* `tensor_streams`: list of flat-tensors that the driver will read from the camera and publish for decoding as needed

All the streams will be synchronized to the best ability of the device (TODO: add more details here. Will this be a control stream as well?)

TODO: PointClouds are camera streams or tensor streams?

The driver will use the loaded class to retrieve the pipeline and start the main processing loop

### Send, Read, Publish Loop
The driver will be have a single thread event-loop which will allow handling ROS service/message callbacks without requiring mutexes. This will achieved by not using the `MTNodeHandle`s or the `MTCallbackQueue`.
SRPL:
* (Optional, based on pipeline) Read messages from ROS topics, send them to the device
* Read the output from device
* Publish the data
Callbacks:
* Per control callback, send the relevant data to the device

### Decode the output
Flat tensor published the driver will need to be decoded (using a nodelet to prevent multiple copies). This allows custom neural networks to be used later-on, while the MVP will focus on MobileNet only

## Interfaces
This is a speculative section, but based on the Gen 1 driver, this driver will provide different interfaces.

### ROS Service out
TODO
* `/depthai/ready`: to inform that the device is ready. Should this be a message instead? This might be required by other nodes in order for debugging/controlling the device

### ROS Topic out
* `/depthai/<stream_name>/<image_transport>` as images compatible with `CvBridge`
* `/depthai/<stream_name>/camera_info` as `CameraInfo`
* `/depthai/<stream_name>` as flat tensor (`std::vector<uint8>`)
* `/depthai/<stream_name>/points` as point cloud. The underlying stream will decide if the point cloud is colored or not
* `/depthai/<stream_name>/imu` as output from IMU
* `/tf` for the different streams

### ROS Service in
* `/depthai/<stream_name>/<control>` as input service for controlling the camera

### ROS Topic in
* `/depthai/<stream_name>/input` as flat tensor input to the stream (optionally advertised)

### ROS Parameter
These parameters are for informing the driver as well as for debugging while the driver is running.
TODO

### ROS Dynamic Parameter
These parameters will offer a dyamic interface to the device. The camera controls might be via parameter server or service
TODO

## Testability
TODO

## Milestones
TODO

## Open Questions
TODO
* ROS or ROS2? Simply replacing nodelet with component allows this document to be valid for ROS2. Should the driver be written for ROS2 to save effort in porting later?
