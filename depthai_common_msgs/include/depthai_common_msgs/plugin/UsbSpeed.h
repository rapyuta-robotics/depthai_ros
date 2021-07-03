#include <depthai-shared/datatype/UsbSpeed.hpp>

#define DEPTHAI_COMMON_MSGS_MESSAGE_USBSPEED_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE()

MSGPACK_ADD_ENUM(dai::UsbSpeed);
