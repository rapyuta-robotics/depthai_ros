#include <depthai-shared/datatype/RawCameraControl.hpp>

MSGPACK_ADD_ENUM(dai::RawCameraControl::AutoFocusMode);
MSGPACK_ADD_ENUM(dai::RawCameraControl::AutoWhiteBalanceMode);
MSGPACK_ADD_ENUM(dai::RawCameraControl::SceneMode);
MSGPACK_ADD_ENUM(dai::RawCameraControl::AntiBandingMode);
MSGPACK_ADD_ENUM(dai::RawCameraControl::EffectMode);

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
