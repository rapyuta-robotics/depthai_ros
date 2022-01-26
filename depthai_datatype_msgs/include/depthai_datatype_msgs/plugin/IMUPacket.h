#include <depthai_common_msgs/libnop.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_IMUPACKET_PLUGIN_CLASS_BODY \
    NOP_STRUCTURE(\
        IMUPacket_,\
        acceleroMeter,\
        gyroscope,\
        magneticField,\
        rotationVector\
    );
