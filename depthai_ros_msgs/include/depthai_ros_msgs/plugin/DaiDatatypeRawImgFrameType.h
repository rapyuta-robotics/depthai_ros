enum class DaiDatatypeRawImgFrameType {
    YUV422i,        // interleaved 8 bit
    YUV444p,        // planar 4:4:4 format
    YUV420p,        // planar 4:2:0 format
    YUV422p,        // planar 8 bit
    YUV400p,        // 8-bit greyscale
    RGBA8888,       // RGBA interleaved stored in 32 bit word
    RGB161616,      // Planar 16 bit RGB data
    RGB888p,        // Planar 8 bit RGB data
    BGR888p,        // Planar 8 bit BGR data
    RGB888i,        // Interleaved 8 bit RGB data
    BGR888i,        // Interleaved 8 bit BGR data
    RGBF16F16F16p,  // Planar FP16 RGB data
    BGRF16F16F16p,  // Planar FP16 BGR data
    RGBF16F16F16i,  // Interleaved FP16 RGB data
    BGRF16F16F16i,  // Interleaved FP16 BGR data
    GRAY8,          // 8 bit grayscale (1 plane)
    GRAYF16,        // FP16 grayscale (normalized)
    LUT2,           // 1 bit  per pixel, Lookup table
    LUT4,           // 2 bits per pixel, Lookup table
    LUT16,          // 4 bits per pixel, Lookup table
    RAW16,          // save any raw type (8, 10, 12bit) on 16 bits
    RAW14,          // 14bit value in 16bit storage
    RAW12,          // 12bit value in 16bit storage
    RAW10,          // 10bit value in 16bit storage
    RAW8,
    PACK10,  // 10bit packed format
    PACK12,  // 12bit packed format
    YUV444i,
    NV12,
    NV21,
    BITSTREAM,  // used for video encoder bitstream
    HDR,
    NONE
};

#define DEPTHAI_ROS_MSGS_MESSAGE_DAIDATATYPERAWIMGFRAMETYPE_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE()

MSGPACK_ADD_ENUM(DaiDatatypeRawImgFrameType);
