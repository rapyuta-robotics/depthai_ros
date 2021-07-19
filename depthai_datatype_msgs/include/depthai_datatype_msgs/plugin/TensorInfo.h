#include <msgpack.hpp>

#include <depthai-shared/datatype/RawNNData.hpp>

MSGPACK_ADD_ENUM(dai::TensorInfo::StorageOrder);
MSGPACK_ADD_ENUM(dai::TensorInfo::DataType);

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_TENSORINFO_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        order,\
        dataType,\
        numDimensions,\
        dims,\
        strides,\
        name,\
        offset\
    )
