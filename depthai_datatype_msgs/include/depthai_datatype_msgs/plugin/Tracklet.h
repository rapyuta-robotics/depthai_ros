#include <depthai_common_msgs/libnop.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_TRACKLET_PLUGIN_CLASS_BODY \
    NOP_STRUCTURE(\
        Tracklet_,\
        roi,\
        id,\
        label,\
        age,\
        status,\
        srcImgDetection,\
        spatialCoordinates\
    );
