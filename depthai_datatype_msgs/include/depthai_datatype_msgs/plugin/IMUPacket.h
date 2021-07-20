#include <depthai_common_msgs/msgpack.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_IMUPACKET_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        acceleroMeter,\
        gyroscope,\
        magneticField,\
        rotationVector\
    )
