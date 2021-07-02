enum class ProcessorType : int32_t { LOS, LRT };

#define DEPTHAI_COMMON_MSGS_MESSAGE_PROCESSORTYPE_PLUGIN_CLASS_BODY \
    MSGPACK_ADD_ENUM(DaiProcessorType);
