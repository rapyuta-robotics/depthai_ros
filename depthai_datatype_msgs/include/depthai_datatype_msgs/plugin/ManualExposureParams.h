#include <depthai_common_msgs/libnop.hpp>

#define DEPTHAI_DATATYPE_MSGS_MESSAGE_MANUALEXPOSUREPARAMS_PLUGIN_CLASS_BODY \
    NOP_STRUCTURE(\
        ManualExposureParams_,\
        exposureTimeUs,\
        sensitivityIso,\
        frameDurationUs\
    );
