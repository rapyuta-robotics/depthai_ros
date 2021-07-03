#include <depthai-shared/datatype/RawIMUData.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_IMUREPORTACCURACY_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE()

MSGPACK_ADD_ENUM(dai::IMUReport::IMUReportAccuracy);
