#include <depthai_common_msgs/libnop.hpp>

#define DEPTHAI_COMMON_MSGS_MESSAGE_TIMESTAMP_PLUGIN_CLASS_BODY \
    NOP_STRUCTURE(\
        Timestamp_,\
        sec,\
        nsec\
    );
