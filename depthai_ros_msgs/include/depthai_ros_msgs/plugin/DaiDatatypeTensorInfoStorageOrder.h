enum class DaiDatatypeTensorInfoStorageOrder : int {
    NHWC = 0x4213,
    NHCW = 0x4231,
    NCHW = 0x4321,
    HWC = 0x213,
    CHW = 0x321,
    WHC = 0x123,
    HCW = 0x231,
    WCH = 0x132,
    CWH = 0x312,
    NC = 0x43,
    CN = 0x34,
    C = 0x3,
    H = 0x2,
    W = 0x1,
};

#define DEPTHAI_ROS_MSGS_MESSAGE_DAIDATATYPETENSORINFOSTORAGEORDER_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE()

MSGPACK_ADD_ENUM(DaiDatatypeTensorInfoStorageOrder);
