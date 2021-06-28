enum class DaiDetectionNetworkType : std::int32_t { YOLO, MOBILENET };

#define DEPTHAI_ROS_MSGS_MESSAGE_DAIDETECTIONNETWORKTYPE_PLUGIN_CLASS_BODY \
    MSGPACK_ADD_ENUM(DaiDetectionNetworkType);
