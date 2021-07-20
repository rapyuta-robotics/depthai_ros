#define MSGPACK_USE_DEFINE_MAP

#include <msgpack.hpp>

#define DEPTHAI_COMMON_MSGS_MESSAGE_ROTATEDRECT_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        center,\
        size,\
        angle\
    )
