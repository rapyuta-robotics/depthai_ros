#include <depthai_ros_driver/control/focus_control.hpp>

namespace rr {
using AutoFocusMode = dai::CameraControl::AutoFocusMode;

std::shared_ptr<dai::CameraControl> FocusControl::createControl(const depthai_ros_msgs::FocusControlCommand& command) {
    auto ctrl = std::make_shared<dai::CameraControl>();
    if (command.focus_mode == depthai_ros_msgs::FocusControlCommand::FOCUS_AUTO) {
        const auto& af_mode = convertToAutoFocusMode(command.control_val);
        if (!af_mode.first) {
            return nullptr;
        }
        ctrl->setAutoFocusMode(af_mode.second);
        ctrl->setAutoFocusTrigger();
    }
    else if (command.focus_mode == depthai_ros_msgs::FocusControlCommand::FOCUS_MANUAL) {
        ctrl->setManualFocus(command.control_val);
    }
    else {
        return nullptr;
    }

    return ctrl;
}

std::pair<bool, AutoFocusMode> FocusControl::convertToAutoFocusMode(uint8_t val) {
    std::pair<bool, AutoFocusMode> mode;
    switch (val) {
    case depthai_ros_msgs::FocusControlCommand::AF_MODE_OFF:
        mode = std::make_pair(true, AutoFocusMode::OFF);
        break;
    case depthai_ros_msgs::FocusControlCommand::AF_MODE_AUTO:
        mode = std::make_pair(true, AutoFocusMode::AUTO);
        break;
    case depthai_ros_msgs::FocusControlCommand::AF_MODE_MACRO:
        mode = std::make_pair(true, AutoFocusMode::MACRO);
        break;
    case depthai_ros_msgs::FocusControlCommand::AF_MODE_CONTINUOUS_VIDEO:
        mode = std::make_pair(true, AutoFocusMode::CONTINUOUS_VIDEO);
        break;
    case depthai_ros_msgs::FocusControlCommand::AF_MODE_CONTINUOUS_PICTURE:
        mode = std::make_pair(true, AutoFocusMode::EDOF);
        break;
    case depthai_ros_msgs::FocusControlCommand::AF_MODE_EDOF:
        mode = std::make_pair(true, AutoFocusMode::CONTINUOUS_PICTURE);
        break;
    default:
        mode = std::make_pair(false, AutoFocusMode::OFF);
        break;
    }

    return mode;
}
} // namespace rr