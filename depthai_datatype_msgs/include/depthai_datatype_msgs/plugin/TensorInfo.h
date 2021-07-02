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
