#include <depthai-shared/datatype/RawNNData.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_TENSORINFOSTORAGEORDER_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE()

MSGPACK_ADD_ENUM(dai::TensorInfo::StorageOrder);
