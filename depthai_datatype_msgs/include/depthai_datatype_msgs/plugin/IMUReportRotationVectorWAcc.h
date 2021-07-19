#include <msgpack.hpp>

#include <depthai-shared/datatype/RawIMUData.hpp>

MSGPACK_ADD_ENUM(dai::IMUReport::IMUReportAccuracy);

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_IMUREPORTROTATIONVECTORWACC_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        i,\
        j,\
        k,\
        real,\
        accuracy,\
        sequence,\
        accuracy,\
        timestamp\
    )
