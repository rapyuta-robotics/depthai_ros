#include <depthai_common_msgs/msgpack.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_REGIONPARAMS_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        x,\
        y,\
        width,\
        height,\
        priority\
    )
