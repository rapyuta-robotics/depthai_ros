enum class DetectionNetworkType : std::int32_t { YOLO, MOBILENET };

#define DEPTHAI_COMMON_MSGS_MESSAGE_DETECTIONNETWORKTYPE_PLUGIN_CLASS_BODY \
    MSGPACK_ADD_ENUM(DaiDetectionNetworkType);
