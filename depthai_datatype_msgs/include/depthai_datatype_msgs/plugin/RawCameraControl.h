#define MSGPACK_USE_DEFINE_MAP

#include <msgpack.hpp>

#include <depthai-shared/datatype/RawCameraControl.hpp>

MSGPACK_ADD_ENUM(dai::RawCameraControl::AutoFocusMode);
MSGPACK_ADD_ENUM(dai::RawCameraControl::AutoWhiteBalanceMode);
MSGPACK_ADD_ENUM(dai::RawCameraControl::SceneMode);
MSGPACK_ADD_ENUM(dai::RawCameraControl::AntiBandingMode);
MSGPACK_ADD_ENUM(dai::RawCameraControl::EffectMode);
MSGPACK_ADD_ENUM(dai::RawCameraControl::Command);
MSGPACK_ADD_ENUM(dai::RawCameraControl::ControlMode);
MSGPACK_ADD_ENUM(dai::RawCameraControl::CaptureIntent);

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_RAWCAMERACONTROL_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        cmdMask,\
        autoFocusMode,\
        lensPosition,\
        expManual,\
        aeRegion,\
        afRegion,\
        awbMode,\
        sceneMode,\
        antiBandingMode,\
        aeLockMode,\
        awbLockMode,\
        effectMode,\
        expCompensation,\
        brightness,\
        contrast,\
        saturation,\
        sharpness,\
        lumaDenoise,\
        chromaDenoise\
    )
