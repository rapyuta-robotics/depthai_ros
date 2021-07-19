#include <msgpack.hpp>

#include <depthai-shared/datatype/RawImgFrame.hpp>

MSGPACK_ADD_ENUM(dai::RawImgFrame::Type);

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_RAWIMGFRAME_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        fb,\
        category,\
        instanceNum,\
        sequenceNum,\
        ts\
    )
