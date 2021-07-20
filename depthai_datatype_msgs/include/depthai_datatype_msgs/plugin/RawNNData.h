#define MSGPACK_USE_DEFINE_MAP

#include <msgpack.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_RAWNNDATA_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE(\
        tensors,\
        batchSize\
    )
