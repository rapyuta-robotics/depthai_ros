## NOTE

Please include the `depthai_common_msgs/msgpack.hpp` instead of `msgpack.hpp` in your project. This ensure the same settings are used for message pack as in the device firmware.

In case you need to be able to use other settings (for some other project), please use `depthai_common_msgs/unmsgpack.hpp` to undo the settings and then reapply them for depthai again
