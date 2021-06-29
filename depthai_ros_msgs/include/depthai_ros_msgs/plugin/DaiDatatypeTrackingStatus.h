enum class DaiDatatypeTrackingStatus : std::int32_t {
    NEW,     /**< The object is newly added. */
    TRACKED, /**< The object is being tracked. */
    LOST,   /**< The object gets lost now. The object can be tracked again automatically(long term tracking) or by specifying detected object manually(short
                term and zero term tracking). */
    REMOVED /**< The object is removed. */
};

#define DEPTHAI_ROS_MSGS_MESSAGE_DAIDATATYPETRACKINGSTATUS_PLUGIN_CLASS_BODY \
    MSGPACK_DEFINE()

MSGPACK_ADD_ENUM(DaiDatatypeTrackingStatus);