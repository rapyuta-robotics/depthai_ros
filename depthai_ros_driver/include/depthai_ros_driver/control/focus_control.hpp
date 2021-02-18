#pragma once

#include <depthai/pipeline/datatype/CameraControl.hpp>
#include <depthai_ros_msgs/FocusControlCommand.h>

namespace rr {
class FocusControl {
public:
    /**
     * @brief Create CameraControl for controlling depthai color camera focus
     * @return shared_ptr to a CameraControl for focus control is returned.
     *         nullptr is returned if the input command is not valid.
     */
    static std::shared_ptr<dai::CameraControl> createControl(const depthai_ros_msgs::FocusControlCommand& command);

private:
    /**
     * @brief Convert message uint8 control value to dai::CameraControl::AutoFocusMode
     * @return dai::CameraControl::AutoFocusMode corresponding to the input val is returned
     *         as the second element of the pair.
     *         false is set to the first element if input val is not valid.
     */
    static std::pair<bool, dai::CameraControl::AutoFocusMode> convertToAutoFocusMode(uint8_t val);
};
} // namespace rr