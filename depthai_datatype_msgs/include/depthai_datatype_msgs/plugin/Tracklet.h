#define MSGPACK_USE_DEFINE_MAP

#include <msgpack.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_TRACKLET_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        roi,\
        id,\
        label,\
        age,\
        status,\
        srcImgDetection,\
        spatialCoordinates\
    )
