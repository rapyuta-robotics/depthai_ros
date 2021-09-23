// package headers
#include <depthai_ros_driver/conversion.hpp>
#include <depthai_ros_driver/dai_utils.hpp>

// relevant 3rd party headers
#include <depthai/device/DeviceBase.hpp>
#include <depthai/pipeline/datatype/StreamMessageParser.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>
#include <depthai/pipeline/node/XLinkIn.hpp>

// ros headers
#include <camera_info_manager/camera_info_manager.h>
#include <ros/callback_queue.h>
#include <ros/publisher.h>
#include <ros/ros.h>

// lib headers
#include <range/v3/algorithm/find.hpp>
#include <range/v3/view.hpp>
#include <boost/bind/bind.hpp>

// std headers
#include <memory>
#include <thread>

// message headers
#include <depthai_datatype_msgs/datatype_msgs.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

namespace rr {
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

/**
 * @brief Reads a packet from an XLinkStream and frees it on destruction
 * @note Make sure to call `getData` or else the packet will go un-processed
 */
class PacketReader {
public:
    PacketReader(dai::XLinkStream& stream)
            : _stream(stream) {
        _packet = _stream.readRaw();  // blocking read

        _msgpack_size = static_cast<std::uint32_t>(fromLE(_packet->data + _packet->length - 4));
        if (_msgpack_size > _packet->length) {
            throw std::runtime_error("Bad packet, couldn't parse");
        }

        _buf_size = _packet->length - 8 - _msgpack_size;
    }
    ~PacketReader() { _stream.readRawRelease(); }

    auto packet() { return _packet; }

    std::uint32_t msgpackSize() { return _msgpack_size; };

    std::uint8_t* msgpackBeginPtr() { return _packet->data + _buf_size; }

    auto bufferData() {
        // copy data part
        std::vector<std::uint8_t> data(_packet->data, _packet->data + _buf_size);
        return data;
    }

    [[deprecated("Uses dai interface, to be removed soon")]] auto getData() {
        return dai::StreamMessageParser::parseMessageToADatatype(_packet);
    }

private:
    std::uint32_t fromLE(std::uint8_t* data) {
        return data[0] + data[1] * 256 + data[2] * 256 * 256 + data[3] * 256 * 256 * 256;
    };

    dai::XLinkStream& _stream;
    streamPacketDesc_t* _packet;

    std::uint32_t _msgpack_size;
    std::uint32_t _buf_size;
};

class PacketWriter {
public:
    PacketWriter(dai::XLinkStream& stream)
            : _stream(stream) {}

    static auto toLE(std::uint32_t num, std::uint8_t* le_data) {
        le_data[0] = static_cast<std::uint8_t>((num & 0x000000ff) >> 0u);
        le_data[1] = static_cast<std::uint8_t>((num & 0x0000ff00) >> 8u);
        le_data[2] = static_cast<std::uint8_t>((num & 0x00ff0000) >> 16u);
        le_data[3] = static_cast<std::uint8_t>((num & 0xff000000) >> 24u);
    };

    void write(const std::uint8_t* dat, size_t dat_size, const std::uint8_t* ser, size_t ser_size,
            std::uint32_t datatype, std::vector<std::uint8_t>& buf) {
        size_t packet_size = dat_size + ser_size + 8;

        buf.resize(packet_size);

        std::memcpy(buf.data(), dat, dat_size);
        std::memcpy(buf.data() + dat_size, ser, ser_size);
        toLE(datatype, buf.data() + packet_size - 8);
        toLE(ser_size, buf.data() + packet_size - 4);

        _stream.write(buf);
    }

    dai::XLinkStream& _stream;
};

class DeviceROS : public dai::DeviceBase {
    using Base = dai::DeviceBase;

public:
    using Base::Base;

    template <class... Args>
    DeviceROS(ros::NodeHandle nh, Args... args)
            : Base(args...)
            , _pub_nh(nh)
            , _sub_nh(nh) {
        ros::NodeHandle private_nh("~");
        private_nh.param<int>("pub_queue_size", _pub_queue_size, 10);
        private_nh.param<int>("sub_queue_size", _sub_queue_size, 10);
        // @TODO: make the service per stream. Needs refactoring to change to:
        // Map<StreamVariables> instead of Map<Variable>...
    }

    void closeImpl() override {
        _active.reset();
        Base::closeImpl();
    }

protected:
    // create stream based on name at the time of subscription
    template <class MsgType>
    auto generate_cb_lambda(const std::string& name) -> boost::function<void(const boost::shared_ptr<MsgType const>&)> {
        auto conn = this->getConnection();
        _streams[name] = std::make_unique<dai::XLinkStream>(*conn, name, dai::XLINK_USB_BUFFER_MAX_SIZE);

        const auto core_sub_lambda = [&stream = _streams[name], serial_buf = std::vector<std::uint8_t>(),
                                             writer_buf = std::vector<std::uint8_t>()](
                                             const boost::shared_ptr<MsgType const>& msg) mutable {
            Guard guard([] { ROS_ERROR("Communication failed: Device error or misconfiguration."); });

            // convert msg to data
            dai::DatatypeEnum datatype;
            auto data_ref = adapt_ros2dai<MsgType>::convert(*msg);
            data_ref.serialize(serial_buf, datatype);

            PacketWriter writer(*stream);
            writer.write(msg->data.data(), msg->data.size(), reinterpret_cast<std::uint8_t*>(serial_buf.data()),
                    serial_buf.size(), static_cast<std::uint32_t>(datatype), writer_buf);

            guard.disable();
        };

        return core_sub_lambda;
    }

    template <class MsgType>
    auto generate_pub_lambda(ros::NodeHandle& nh, std::string name, std::size_t q_size) {  // name: copied
        auto pub = create_publisher<MsgType>(nh, name, q_size);
        if constexpr (std::is_same_v<MsgType, depthai_datatype_msgs::RawImgFrame>) {
            _camera_info_manager[name] = pub.info_manager_ptr;
        }
        _enabled.try_emplace(name, std::make_unique<std::atomic<bool>>(true));

        const auto pub_lambda = [this, pub, name, enabled = _enabled[name].get()]() {
            auto conn = this->getConnection();
            auto stream = dai::XLinkStream(*conn, name, 1);  // no writing happens, so 1 is sufficient
            Guard guard([] { ROS_ERROR("Communication failed: Device error or misconfiguration."); });

            while (this->_running) {
                if (!enabled->load()) {
                    ros::Duration(0.25).sleep();
                    ROS_INFO_STREAM_THROTTLE(2, "Stream " << name << " is not enabled");
                    continue;
                }
                // block till data is read
                PacketReader reader{stream};

                // skip the rest of code to save computation if no one is listening
                if (getNumSubscribers(pub) <= 0) {
                    continue;
                }
                auto packet = reader.packet();

                msgpack::object_handle oh =
                        msgpack::unpack(reinterpret_cast<const char*>(reader.msgpackBeginPtr()), reader.msgpackSize());
                msgpack::object obj = oh.get();

                MsgType msg;
                obj.convert(msg);
                msg.data = reader.bufferData();

                // publish data, with frame_id base as the driver node name
                publish(pub, msg, ros::this_node::getName() + "/" + name);
            }

            guard.disable();
        };

        return pub_lambda;
    }

    bool startPipelineImpl(const dai::Pipeline& pipeline) override {
        _active = std::make_shared<std::uint8_t>(0);
        _setup_publishers(pipeline);
        _setup_subscribers(pipeline);

        bool status = Base::startPipelineImpl(pipeline);
        ROS_INFO_STREAM("DepthAI Driver pipeline started with status:" << status);
        return status;
    }

    void _setup_publishers(const dai::Pipeline& pipeline) {
        namespace ph = boost::placeholders;
        // get all XLinkOut
        const auto& out_links = getAllInputs(pipeline);
        ROS_INFO_STREAM("Required publishers: " << out_links.size());
        for (const auto& node_links : out_links) {
            const auto& node = node_links.node_to;
            // get name for publisher
            const auto& name = node->getStreamName();

            auto nh = ros::NodeHandle{_pub_nh, name};

            auto common_type = getCommonType(node_links.out_from);
            ROS_INFO_STREAM(name << " (pub): " << static_cast<int>(common_type));

            // create appropriate publisher using the common_type
            switch (common_type) {
                case dai::DatatypeEnum::Buffer:
                    break;
                case dai::DatatypeEnum::CameraControl:
                    // _pub_t[name] = std::thread{generate_pub_lambda<depthai_datatype_msgs::RawCameraControl>(nh,
                    // name, _pub_queue_size)};
                    break;
                case dai::DatatypeEnum::IMUData:
                    // @TODO: support IMU
                    // _pub_t[name] = std::thread{generate_pub_lambda<depthai_datatype_msgs::RawIMUData>(nh, name,
                    // _pub_queue_size)}; // Not supported
                    break;
                case dai::DatatypeEnum::ImageManipConfig:
                    break;
                case dai::DatatypeEnum::ImgDetections:
                    _pub_t[name] = std::thread{
                            generate_pub_lambda<depthai_datatype_msgs::RawImgDetections>(nh, name, _pub_queue_size)};
                    break;
                case dai::DatatypeEnum::ImgFrame: {
                    _pub_t[name] = std::thread{
                            generate_pub_lambda<depthai_datatype_msgs::RawImgFrame>(nh, name, _pub_queue_size)};
                    boost::function<bool(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)> fn =
                            boost::bind(&DeviceROS::_defaultCameraInfo, this, name, ph::_1, ph::_2);
                    _camera_info_default_srvs[name] = nh.advertiseService("reset_camera_info", fn);
                } break;
                case dai::DatatypeEnum::NNData:
                    _pub_t[name] = std::thread{
                            generate_pub_lambda<depthai_datatype_msgs::RawNNData>(nh, name, _pub_queue_size)};
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
                    _pub_t[name] = std::thread{
                            generate_pub_lambda<depthai_datatype_msgs::RawTracklets>(nh, name, _pub_queue_size)};
                    break;
                default:
                    break;
            }
            boost::function<bool(std_srvs::SetBool::Request&, std_srvs::SetBool::Response&)> fn =
                    boost::bind(&DeviceROS::_set_activation_status, this, name, ph::_1, ph::_2);
            _activation_srvs[name] = nh.advertiseService("set_state", fn);
        }
    }

    bool _set_activation_status(
            const std::string& name, std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
        auto it = _enabled.find(name);
        assert(it != _enabled.end());
        it->second->store(req.data);
        res.success = true;
        return true;
    }

    void _setup_subscribers(const dai::Pipeline& pipeline) {
        // get all XLinkIn
        const auto& in_links = getAllOutputs(pipeline);
        ROS_INFO_STREAM("Required subscribers: " << in_links.size());
        for (const auto& node_links : in_links) {
            const auto& node = node_links.node_from;
            // get name for publisher
            const auto& name = node->getStreamName();

            auto common_type = getCommonType(node_links.in_to);
            ROS_INFO_STREAM(name << " (sub): " << static_cast<int>(common_type));

            // create appropriate subscriber using the common_type
            switch (common_type) {
                case dai::DatatypeEnum::Buffer:
                    break;
                case dai::DatatypeEnum::CameraControl:
                    _sub[name] = _sub_nh.subscribe(
                            name, _sub_queue_size, generate_cb_lambda<depthai_datatype_msgs::RawCameraControl>(name));
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

    bool _defaultCameraInfo(
            const std::string& name, std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        const auto it = _camera_info_manager.find(name);
        assert(it != _camera_info_manager.end());

        const auto uri = camera_param_uri + "/default/" + name + ".yaml";
        if (_defaultManager == nullptr) {
            _defaultManager = std::make_unique<camera_info_manager::CameraInfoManager>(
                    ros::NodeHandle{_pub_nh, "_default"}, name, uri);
        } else {
            _defaultManager->setCameraName(name);
            _defaultManager->loadCameraInfo(uri);
        }
        const auto cameraInfo = _defaultManager->getCameraInfo();
        res.success = it->second->setCameraInfo(std::move(cameraInfo));

        _defaultManager->setCameraName("_default");
        _defaultManager->loadCameraInfo("");
        return true;
    }

    template <class T>
    using Map = std::unordered_map<std::string, T>;
    Map<std::thread> _pub_t;
    Map<ros::Subscriber> _sub;
    Map<std::shared_ptr<camera_info_manager::CameraInfoManager>> _camera_info_manager;
    Map<std::unique_ptr<dai::XLinkStream>> _streams;
    Map<std::unique_ptr<std::atomic<bool>>> _enabled;
    Map<ros::ServiceServer> _camera_info_default_srvs, _activation_srvs;

    std::unique_ptr<camera_info_manager::CameraInfoManager> _defaultManager;
    // ros::ServiceServer _activation_service, _camera_info_default;

    // shared ptr for the callbacks to be called
    std::shared_ptr<std::uint8_t> _active;

    ros::CallbackQueue _pub_q, _sub_q;
    ros::NodeHandle _pub_nh, _sub_nh;
    int _pub_queue_size, _sub_queue_size;

    std::atomic<bool> _running = true;
};
}  // namespace rr
