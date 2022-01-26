#include <depthai_common_msgs/libnop.hpp>

#include <depthai-shared/datatype/RawTracklets.hpp>

// MSGPACK_ADD_ENUM(dai::Tracklet::TrackingStatus);

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_RAWTRACKLETS_PLUGIN_CLASS_BODY \
    NOP_STRUCTURE(\
        RawTracklets_,\
        tracklets\
    );
