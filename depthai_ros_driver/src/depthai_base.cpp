#include <depthai_ros_driver/impl/depthai_base.hpp>

namespace rr {
template class DepthAIBase<ROS1Node<>>;
template class DepthAIBase<nodelet::Nodelet>;
}  // namespace rr
