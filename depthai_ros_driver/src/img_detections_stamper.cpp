#include <depthai_ros_driver/impl/img_detections_stamper.hpp>

#include <node_interface/ros1_node_interface.hpp>
#include <nodelet/nodelet.h>

namespace rr {
template class ImgDetectionsStamper<ROS1Node<>>;
template class ImgDetectionsStamper<nodelet::Nodelet>;
}  // namespace rr
