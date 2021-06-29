enum class DaiCommonCameraModel : int8_t { Perspective = 0, Fisheye = 1, Equirectangular = 2, RadialDivision = 3 };

#define DEPTHAI_ROS_MSGS_MESSAGE_DAICOMMONCAMERAMODEL_PLUGIN_CLASS_BODY \
    MSGPACK_ADD_ENUM(DaiCameraModel);
