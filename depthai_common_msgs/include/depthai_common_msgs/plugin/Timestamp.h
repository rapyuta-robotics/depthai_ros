#define MSGPACK_USE_DEFINE_MAP

#include <msgpack.hpp>

#define DEPTHAI_COMMON_MSGS_MESSAGE_TIMESTAMP_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        sec,\
        nsec\
    )
