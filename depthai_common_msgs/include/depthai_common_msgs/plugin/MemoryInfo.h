#include <depthai_common_msgs/libnop.hpp>

#define DEPTHAI_COMMON_MSGS_MESSAGE_MEMORYINFO_PLUGIN_CLASS_BODY \
    NOP_STRUCTURE(\
        MemoryInfo_,\
        remaining,\
        used,\
        total\
    );
