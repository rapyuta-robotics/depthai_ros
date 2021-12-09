#include <depthai_common_msgs/msgpack.hpp>

#define DEPTHAI_COMMON_MSGS_MESSAGE_MEMORYINFO_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        remaining,\
        used,\
        total\
    )
