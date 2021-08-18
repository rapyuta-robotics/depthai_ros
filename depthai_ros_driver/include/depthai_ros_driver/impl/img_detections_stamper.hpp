#include <depthai_ros_driver/img_detections_stamper.hpp>
/**
 * @file impl/img_detection_stamper.hpp
 * @brief Publish an image detection (NN) with the timestamp from the latest Image from sensor
 */

namespace rr {
template <class Node>
void ImgDetectionsStamper<Node>::onInit() {
    auto pnh = Node::getPrivateNodeHandle();
    int nn_queue_size = 1;
    if (pnh.getParam("nn_queue_size", nn_queue_size)) {
        pnh.setParam("nn_queue_size", nn_queue_size);
    }

    int img_queue_size = 1;
    if (pnh.getParam("img_queue_size", img_queue_size)) {
        pnh.setParam("img_queue_size", img_queue_size);
    }

    auto nh = Node::getNodeHandle();
    pub_stamped_ = nh.template advertise<depthai_datatype_msgs::ImgDetectionsStamped>("nn_stamped", 10);
    sub_nn_ = nh.subscribe("nn", 1, &ImgDetectionsStamper::nn_callback, this);
    sub_img_ = nh.subscribe("img", 1, &ImgDetectionsStamper::img_callback, this);
}

template <class Node>
void ImgDetectionsStamper<Node>::img_callback(const sensor_msgs::ImageConstPtr& img_msg) {
    last_recv_msg_ = img_msg;
}

template <class Node>
void ImgDetectionsStamper<Node>::nn_callback(const depthai_datatype_msgs::RawImgDetectionsConstPtr& nn_msg_ptr) {
    if (last_recv_msg_ != nullptr) {
        msg_stamped_.header = last_recv_msg_->header;
        msg_stamped_.detections = nn_msg_ptr->detections;
        pub_stamped_.publish(msg_stamped_);
    }
}


}  // namespace rr
