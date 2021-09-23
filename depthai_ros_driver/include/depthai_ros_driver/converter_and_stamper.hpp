#pragma once
/**
 * @file converter_and_stamper.hpp
 * @brief Copy the stamp from one topic and convert an unstamped topic into a stamped topic
 */
#include <cstdint>

#include <ros/message_traits.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <topic_tools/shape_shifter.h>

namespace rr {
/**
 * @brief Finds the length of the header without deserializing it or performing any checks
 *
 * @param data pointer to the serialized data
 * @return std::size_t length of the header
 */
std::size_t find_header_length(const std::uint8_t* const data) {
    std::size_t count = 0;

    const auto& len = [](auto t) { return ros::serialization::Serializer<decltype(t)>::serializedLength(t); };

    count += len(std::uint32_t{});  // seq
    count += len(ros::Duration{});  // stamp

    auto len_frame_id = *(reinterpret_cast<const std::uint32_t*>(data + count));

    count += len(std::uint32_t{});  // length of data in frame_id
    count += len_frame_id;          // frame_id

    return count;
}

struct FakeStream {
    FakeStream(std::vector<std::uint8_t>& buffer, std::size_t pre_count = 0)
            : data(&buffer)
            , count(pre_count) {}

protected:
    std::vector<std::uint8_t>* data;

public:
    std::size_t count;
    std::size_t length = 0;

    std::uint8_t* advance(std::size_t len) {
        auto old_count = count;
        count += len;
        if (data->size() < count) {
            data->reserve(2 * count);
            data->resize(count);
        }
        return data->data() + old_count;
    }

    std::size_t getLength() { return data->size(); }
    std::uint8_t* getData() { return data->data(); }
};

template <class MessageStamped, class Node>
struct ConverterAndStamper : Node {
public:
    ConverterAndStamper() = default;
    ~ConverterAndStamper() = default;

private:
    using ShapeShifterConstPtr = typename topic_tools::ShapeShifter::ConstPtr;
    void onInit() override {
        auto pnh = Node::getPrivateNodeHandle();
        const auto param = [&](const std::string& name, auto& val) {
            if (pnh.getParam(name, val)) {
                pnh.setParam(name, val);
            }
        };

        int stamped_input_queue_size = 1, unstamped_input_queue_size = 1, output_queue_size = 1;
        param("stamped_input_queue_size", stamped_input_queue_size);
        param("unstamped_input_queue_size", unstamped_input_queue_size);
        param("output_queue_size", output_queue_size);

        auto nh = Node::getNodeHandle();
        sub_stamped_ = nh.subscribe("stamped", stamped_input_queue_size, &ConverterAndStamper::stamped_callback, this);
        sub_unstamped_ =
                nh.subscribe("unstamped", unstamped_input_queue_size, &ConverterAndStamper::unstamped_callback, this);

        topic_tools::ShapeShifter shifter;
        namespace mt = ros::message_traits;
        shifter.morph(
                mt::md5sum<MessageStamped>(), mt::datatype<MessageStamped>(), mt::definition<MessageStamped>(), "");
        pub_stamped_ = shifter.advertise(nh, "output", output_queue_size, false);
    }

    void stamped_callback(const ShapeShifterConstPtr& stamped_message) {
        FakeStream stream(stamped_buffer_);
        stamped_message->write(stream);
    }

    void unstamped_callback(const ShapeShifterConstPtr& unstamped_message) {
        if (stamped_buffer_.empty()) {
            return;
        }
        const auto header_len = find_header_length(stamped_buffer_.data());

        const auto total_len = unstamped_message->size() + header_len;
        unstamped_buffer_.resize(total_len);
        std::copy(stamped_buffer_.cbegin(), stamped_buffer_.cbegin() + header_len, unstamped_buffer_.begin());

        FakeStream stream(unstamped_buffer_, header_len);

        /*
        prefix the header to the data by:
        * copy the header to buffer
        * copy unstamped message to buffer
        * copy both together to incoming message
        * morph the message
        */
        unstamped_message->write(stream);  // move the actual data at the end of the stream

        auto msg_stamped = boost::make_shared<topic_tools::ShapeShifter>();  // for 0 copy
        msg_stamped->read(stream);  // if only ShapeShifter allowed simple insertion into the internal msg buffer
        namespace mt = ros::message_traits;
        msg_stamped->morph(
                mt::md5sum<MessageStamped>(), mt::datatype<MessageStamped>(), mt::definition<MessageStamped>(), "");

        pub_stamped_.publish(msg_stamped);
    }

    std::vector<std::uint8_t> stamped_buffer_;
    std::vector<std::uint8_t> unstamped_buffer_;

    ros::Publisher pub_stamped_;
    ros::Subscriber sub_stamped_;
    ros::Subscriber sub_unstamped_;
};
}  // namespace rr

// #include <node_interface/ros1_node_interface.hpp>
// #include <nodelet/nodelet.h>
// namespace rr {
// using ConverterAndStamperNode = ConverterAndStamper<ROS1Node<>>;
// using ConverterAndStamperNodelet = ConverterAndStamper<nodelet::Nodelet>;
