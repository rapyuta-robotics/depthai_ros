#include <depthai-shared/datatype/RawCameraControl.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_RAWCAMERACONTROLEFFECTMODE_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE()

MSGPACK_ADD_ENUM(dai::RawCameraControl::EffectMode);
