#include <depthai_common_msgs/libnop.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_IMGDETECTION_PLUGIN_CLASS_BODY \
    NOP_STRUCTURE(\
        ImgDetection_,\
        label,\
        confidence,\
        xmin,\
        ymin,\
        xmax,\
        ymax\
    );
