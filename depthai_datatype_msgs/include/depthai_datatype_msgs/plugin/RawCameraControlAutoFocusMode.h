#include <depthai-shared/datatype/RawCameraControl.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_RAWCAMERACONTROLAUTOFOCUSMODE_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE()

MSGPACK_ADD_ENUM(dai::RawCameraControl::AutoFocusMode);
