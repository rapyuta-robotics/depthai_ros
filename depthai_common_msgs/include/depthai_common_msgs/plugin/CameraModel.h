#include <depthai-shared/datatype/CameraModel.hpp>

#define DEPTHAI_COMMON_MSGS_MESSAGE_CAMERAMODEL_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE()

MSGPACK_ADD_ENUM(dai::CameraModel);
