#include <depthai-shared/datatype/ProcessorType.hpp>

#define DEPTHAI_COMMON_MSGS_MESSAGE_PROCESSORTYPE_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE()

MSGPACK_ADD_ENUM(dai::ProcessorType);
