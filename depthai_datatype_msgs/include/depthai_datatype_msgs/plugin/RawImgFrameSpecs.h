#include <depthai_common_msgs/libnop.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_RAWIMGFRAMESPECS_PLUGIN_CLASS_BODY \
    NOP_STRUCTURE(\
        RawImgFrameSpecs_,\
        type,\
        width,\
        height,\
        stride,\
        bytesPP,\
        p1Offset,\
        p2Offset,\
        p3Offset\
    );
