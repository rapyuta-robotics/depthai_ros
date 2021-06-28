enum class DaiProcessorType : int32_t { LOS, LRT };

#define DEPTHAI_ROS_MSGS_MESSAGE_DAIPROCESSORTYPE_PLUGIN_CLASS_BODY \
    MSGPACK_ADD_ENUM(DaiProcessorType);
