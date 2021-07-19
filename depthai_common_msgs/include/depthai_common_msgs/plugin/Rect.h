#include <msgpack.hpp>

#define DEPTHAI_COMMON_MSGS_MESSAGE_RECT_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        x,\
        y,\
        width,\
        height\
    )
