#include <msgpack.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_MANUALEXPOSUREPARAMS_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        exposureTimeUs,\
        sensitivityIso,\
        frameDurationUs\
    )
