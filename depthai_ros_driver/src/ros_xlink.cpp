#include <depthai_ros_driver/dai_utils.hpp>

#include <depthai/device/DeviceBase.hpp>
#include <depthai/pipeline/datatype/StreamPacketParser.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>
#include <depthai/pipeline/node/XLinkIn.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <range/v3/view.hpp>
#include <range/v3/algorithm/find.hpp>
#include <memory>

namespace rr {
std::vector<NodeConstPtr> filterNodesByName(const dai::Pipeline& pipeline, std::string name) {
    return filterNodesByNames(pipeline, rv::single(name));
}

std::vector<dai::Node::Connection> getConnectionsFrom(const NodeConstPtr& node) {
    const auto& connectionMap = node->getParentPipeline().getConnectionMap();
    return connectionMap | rv::values | rv::join |
           rv::filter([&](const auto& conn) { return conn.outputId == node->id; }) | ranges::to<std::vector>;
}

/**
 * @brief simple class to run a function at the destruction, unless disabled
 * @detail Not thread safe. Expected to stay on a single thread and perform cleanups
 * @tparam An invocable object with no paramters
 */
template <class T>
class Guard {
    static_assert(std::is_invocable_v<T>, "Guard needs an input that can be invoked without any parameters");
    T _func;
    bool _run = true;

public:
    Guard(T&& func)
            : _func(func) {}
    Guard(const Guard&) = delete;
    Guard(Guard&&) = delete;
    void disable() { _run = false; }
    void reenable() { _run = true; }
    bool is_enabled() const { return _run; }
    ~Guard() {
        if (_run) {
            _func();
        }
    }
};

class DepthAI : public dai::DeviceBase {
    using Base = dai::DeviceBase;

public:
    using Base::Base;

    template <class... Args>
    DepthAI(ros::NodeHandle nh, Args... args)
            : Base(args...)
            , _pub_nh(nh)
            , _sub_nh(nh) {}

    void closeImpl() override {
        _active.reset();
        Base::closeImpl();
    }

protected:
    // create stream based on name at the time of subscription
    template <class MsgType>
    auto generate_cb_lambda(std::string name) {
        auto conn = this->getConnection();
        const auto core_sub_lambda = [&, stream = dai::XLinkStream{*conn, name, dai::XLINK_USB_BUFFER_MAX_SIZE}](
                                             MsgType& msg, dai::RawBuffer& data) {
            Guard guard([] { ROS_ERROR("Communication failed: Device error or misconfiguration."); });

            // TODO:  convert msg to data here somehow
            std::vector<std::uint8_t> serialized = dai::StreamPacketParser::serializeMessage(data);
            stream.write(serialized);
            guard.disable();
        };
        return core_sub_lambda;
    }

    template <class MsgType>
    auto generate_pub_lambda(ros::NodeHandle& nh, std::string name, std::size_t q_size) {
        auto conn = this->getConnection();
        const auto pub_lambda = [this, pub = nh.advertise<MsgType>(name, q_size),
                                        // no writing happens, so 1 is sufficient
                                        stream = dai::XLinkStream{*conn, name, 1}]() {
            Guard guard([] { ROS_ERROR("Communication failed: Device error or misconfiguration."); });

            while (this->_running) {
                // block till data is read
                PacketReader reader{stream};
                auto data = reader.getData();
                // @TODO convert data to ROS message type here, somehow
                // publish data
                pub.publish(data);
            }

            guard.disable();
        };
        return pub_lambda;
    }

    bool startPipelineImpl(const dai::Pipeline& pipeline) override {
        _active = std::make_shared<std::uint8_t>(0);
        _setup_publishers(pipeline);
        _setup_subscribers(pipeline);

        return Base::startPipelineImpl(pipeline);
    }

    void _setup_publishers(const dai::Pipeline& pipeline) {
        // get all XLinkOut
        const auto& out_links = getAllInputs(pipeline);
        for (const auto& node_links : out_links) {
            const auto& node = node_links.node_to;
            // get name for publisher
            const auto& name = node->getStreamName();

            auto common_type = getCommonType(node_links.out_from);

            // create appropriate publisher using the common_type
            switch (common_type) {
                case dai::DatatypeEnum::Buffer:
                    break;
                case dai::DatatypeEnum::CameraControl:
                    break;
                case dai::DatatypeEnum::IMUData:
                    break;
                case dai::DatatypeEnum::ImageManipConfig:
                    break;
                case dai::DatatypeEnum::ImgDetections:
                    break;
                case dai::DatatypeEnum::ImgFrame:
                    break;
                case dai::DatatypeEnum::NNData:
                    break;
                case dai::DatatypeEnum::SpatialImgDetections:
                    break;
                case dai::DatatypeEnum::SpatialLocationCalculatorConfig:
                    break;
                case dai::DatatypeEnum::SpatialLocationCalculatorData:
                    break;
                case dai::DatatypeEnum::SystemInformation:
                    break;
                case dai::DatatypeEnum::Tracklets:
                    break;
                default:
                    break;
            }
        }
    }

    void _setup_subscribers(const dai::Pipeline& pipeline) {
        // get all XLinkIn
        const auto& in_links = getAllOutputs(pipeline);
        for (const auto& node_links : in_links) {
            const auto& node = node_links.node_from;
            // get name for publisher
            const auto& name = node->getStreamName();

            auto common_type = getCommonType(node_links.in_to);

            // create appropriate subscriber using the common_type
            switch (common_type) {
                case dai::DatatypeEnum::Buffer:
                    break;
                case dai::DatatypeEnum::CameraControl:
                    break;
                case dai::DatatypeEnum::IMUData:
                    break;
                case dai::DatatypeEnum::ImageManipConfig:
                    break;
                case dai::DatatypeEnum::ImgDetections:
                    break;
                case dai::DatatypeEnum::ImgFrame:
                    break;
                case dai::DatatypeEnum::NNData:
                    break;
                case dai::DatatypeEnum::SpatialImgDetections:
                    break;
                case dai::DatatypeEnum::SpatialLocationCalculatorConfig:
                    break;
                case dai::DatatypeEnum::SpatialLocationCalculatorData:
                    break;
                case dai::DatatypeEnum::SystemInformation:
                    break;
                case dai::DatatypeEnum::Tracklets:
                    break;
                default:
                    break;
            }
        }
    }

    // shared ptr for the callbacks to be called
    std::shared_ptr<std::uint8_t> _active;
    std::unordered_map<streamId_t, std::string> _stream_node_map;
    ros::CallbackQueue _pub_q, _sub_q;
    ros::NodeHandle _pub_nh, _sub_nh;
    std::unordered_map<streamId_t, ros::Publisher> _pub_map;
    std::atomic<bool> _running = true;
};
}  // namespace rr
