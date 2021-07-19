#define MSGPACK_USE_DEFINE_MAP

#include <msgpack.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_RAWIMGFRAMESPECS_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        type,\
        width,\
        height,\
        stride,\
        bytesPP,\
        p1Offset,\
        p2Offset,\
        p3Offset\
    )
