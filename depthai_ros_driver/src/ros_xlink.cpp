#include <depthai_ros_driver/dai_utils.hpp>

#include <depthai/device/DeviceBase.hpp>
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
           rv::filter([&](const auto& conn) { return conn.outputId == node->id; }) |
           ranges::to<std::vector>;
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
    ~Guard() {
        if (_run) {
            _func();
        }
    }
};

class DepthAI : public dai::DeviceBase {
    using Base = dai::DeviceBase;

    static auto getAllInputs(const dai::Pipeline& pipeline) {
        const auto& xlinkouts = filterNodesByType<dai::node::XLinkOut>(pipeline);
        // hack for Output
        using OutVec = decltype(xlinkouts[0]->getOutputs());
        using Output = typename OutVec::value_type;
        struct ret {
            std::shared_ptr<const dai::node::XLinkOut> node_to;
            std::vector<Output> out_from;
        };

        std::vector<ret> out;
        for (const auto& node : xlinkouts) {
            // for all input slots of the node
            ret data;
            data.node_to = node;
            const auto& inputs = node->getInputs();
            for (const auto& inp : inputs) {
                const auto& links = getInputOf(node, inp.name);
                // for all links to the slot
                for (const auto& link : links) {
                    data.out_from.push_back(link);
                }
            }
            out.emplace_back(std::move(data));
        }
        return out;
    }

    static auto getAllOutputs(const dai::Pipeline& pipeline) {
        const auto& xlinkins = filterNodesByType<dai::node::XLinkIn>(pipeline);
        // hack for Input
        using InVec = decltype(xlinkins[0]->getInputs());
        using Input = typename InVec::value_type;
        struct ret {
            std::shared_ptr<const dai::node::XLinkIn> node_from;
            std::vector<Input> in_to;
        };

        std::vector<ret> in;
        for (const auto& node : xlinkins) {
            // for all output slots of the node
            ret data;
            data.node_from = node;
            const auto& outputs = node->getOutputs();
            for (const auto& outp : outputs) {
                const auto& links = getOutputOf(node, outp.name);
                // for all links to the slot
                for (const auto& link : links) {
                    data.in_to.push_back(link);
                }
            }
            in.emplace_back(std::move(data));
        }
        return in;
    }

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

    bool startPipelineImpl(const dai::Pipeline& pipeline) override {
        _active = std::make_shared<std::uint8_t>(0);

        // copy connection by value
        const auto pub_lambda = [this](auto message, ros::NodeHandle nh, std::string name, std::size_t q_size) {
            Guard guard([&] { ROS_ERROR("Communication failed: Device error or misconfiguration."); });

            auto conn = this->getConnection();
            // writeSize = 1, since we're just reading
            dai::XLinkStream stream(*conn, name, 1);

            using MsgType = decltype(message);
            ros::Publisher pub = nh.advertise<MsgType>(name, q_size);

            while (this->_running) {
                // block till data is read
                PacketReader reader{stream};
                auto data = reader.getData();
                // publish data
                pub.publish(data);
            }

            //
            guard.disable();
        };

        const auto sub_lambda = [] {};

        // get all xlinkout
        const auto& out_links = getAllInputs(pipeline);
        for (const auto& node_links : out_links) {
            const auto& node = node_links.node_to;
            // get name for publisher
            const auto& name = node->getStreamName();

            std::optional<dai::DatatypeEnum> common_type;
            // figure out a type shared by all conenctions dumping into the XLinkOut
            for (const auto& link : node_links.out_from) {
                // get the link type
                const auto& allowed_types = link.possibleDatatypes;
                const dai::DatatypeEnum type = [&] {
                    if (allowed_types.size() > 1) {
                        return dai::DatatypeEnum::Buffer;
                    } else {
                        return allowed_types[0].datatype;
                    }
                }();
                if (!common_type) {
                    common_type = type;
                } else if (common_type.value() != type) {
                    // @TODO: find the common type, not necessarily the buffer
                    common_type = dai::DatatypeEnum::Buffer;
                }
            }
            if (!common_type) {
                // Huh, what? Anyways...
                continue;
            }

            // create appropriate publisher using the common_type
            switch (common_type.value()) {
                case dai::DatatypeEnum::Buffer : break;
                case dai::DatatypeEnum::CameraControl : break;
                case dai::DatatypeEnum::IMUData : break;
                case dai::DatatypeEnum::ImageManipConfig : break;
                case dai::DatatypeEnum::ImgDetections : break;
                case dai::DatatypeEnum::ImgFrame : break;
                case dai::DatatypeEnum::NNData : break;
                case dai::DatatypeEnum::SpatialImgDetections : break;
                case dai::DatatypeEnum::SpatialLocationCalculatorConfig : break;
                case dai::DatatypeEnum::SpatialLocationCalculatorData : break;
                case dai::DatatypeEnum::SystemInformation : break;
                case dai::DatatypeEnum::Tracklets : break;
                default:
                    break;
            }
        }

        return Base::startPipelineImpl(pipeline);
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
