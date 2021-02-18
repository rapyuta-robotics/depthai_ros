# `depthai_ros_driver`

This is a work-in-progress. Rapyuta Robotics is still working on making the launch files, urdf, and other facilities available for public consumption.

Features developed:
* Publishing `sensor_msgs/Image` for image channels specified by `stream_list` argument
* Publishing NN detection results by mobilenet-SSD
* Controlling focus by publishin `depthai_ros_msgs/FocusControlCommand`  
  example:  
  Sending AF command  
  ```rostopic pub /depthai/driver/focus_ctrl depthai_ros_msgs/FocusControlCommand -1 "{focus_mode: 0, control_val: 1}"```


Features missing:
* Stereo Neural Inference

For more detailed done and undone features, please refer to https://github.com/rapyuta-robotics/depthai_ros/pull/16
