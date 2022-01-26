#include <depthai_common_msgs/libnop.hpp>

#include <depthai-shared/datatype/RawIMUData.hpp>

// MSGPACK_ADD_ENUM(dai::IMUReport::IMUReportAccuracy);

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_IMUREPORTROTATIONVECTORWACC_PLUGIN_CLASS_BODY \
    NOP_STRUCTURE(\
        IMUReportRotationVectorWAcc_,\
        i,\
        j,\
        k,\
        real,\
        accuracy,\
        sequence,\
        accuracy,\
        timestamp\
    );
