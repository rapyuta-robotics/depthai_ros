enum class CameraModel : int8_t { Perspective = 0, Fisheye = 1, Equirectangular = 2, RadialDivision = 3 };

#define DEPTHAI_COMMON_MSGS_MESSAGE_CAMERAMODEL_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE()

MSGPACK_ADD_ENUM(CameraModel);
