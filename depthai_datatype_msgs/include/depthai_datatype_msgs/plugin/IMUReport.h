#include <depthai_common_msgs/msgpack.hpp>

#include <depthai-shared/datatype/RawIMUData.hpp>

MSGPACK_ADD_ENUM(dai::IMUReport::IMUReportAccuracy);

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_IMUREPORT_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        sequence,\
        accuracy,\
        timestamp\
    )
