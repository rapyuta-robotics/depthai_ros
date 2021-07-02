enum class TensorInfoDataType : int {
    FP16 = 0,  // Half precision floating point
    U8F = 1,   // Unsigned byte
    INT = 2,   // Signed integer (4 byte)
    FP32 = 3,  // Single precision floating point
    I8 = 4,    // Signed byte
};

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_TENSORINFODATATYPE_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE()

MSGPACK_ADD_ENUM(TensorInfoDataType);
