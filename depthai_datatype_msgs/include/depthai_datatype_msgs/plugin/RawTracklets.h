#include <msgpack.hpp>

#include <depthai-shared/datatype/RawTracklets.hpp>

MSGPACK_ADD_ENUM(dai::Tracklet::TrackingStatus);

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_RAWTRACKLETS_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        tracklets\
    )
