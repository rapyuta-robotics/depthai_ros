enum class DaiCameraBoardSocket : int32_t { AUTO = -1, RGB, LEFT, RIGHT };

#define DEPTHAI_ROS_MSGS_MESSAGE_DAICAMERABOARDSOCKET_PLUGIN_CLASS_BODY \
    MSGPACK_ADD_ENUM(DaiCameraBoardSocket);
