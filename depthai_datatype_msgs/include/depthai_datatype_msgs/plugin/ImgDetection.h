#define MSGPACK_USE_DEFINE_MAP

#include <msgpack.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_IMGDETECTION_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        label,\
        confidence,\
        xmin,\
        ymin,\
        xmax,\
        ymax\
    )
