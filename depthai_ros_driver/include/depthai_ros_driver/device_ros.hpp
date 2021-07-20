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
#include <thread>

#include <depthai_datatype_msgs/datatype_msgs.h>

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

        _msgpack_size = static_cast<uint32_t>(fromLE(_packet->data + _packet->length - 4));
        if (_msgpack_size < 0) {
            throw std::runtime_error("Bad packet, couldn't parse");
        }

        _buf_size = _packet->length - 8 - _msgpack_size;
    }
    ~PacketReader() { _stream.readRawRelease(); }

    auto packet() { return _packet; }

    uint32_t msgpackSize() { return _msgpack_size; };

    uint8_t* msgpackBeginPtr() { return _packet->data + _buf_size; }

    auto bufferData() {
        // copy data part
        std::vector<uint8_t> data(_packet->data, _packet->data + _buf_size);
        return data;
    }

    [[deprecated("Uses dai interface, to be removed soon")]] auto getData() {
        return dai::StreamPacketParser::parsePacketToADatatype(_packet);
    }

private:
    uint32_t fromLE(uint8_t* data) { return data[0] + data[1] * 256 + data[2] * 256 * 256 + data[3] * 256 * 256 * 256; };

    dai::XLinkStream& _stream;
    streamPacketDesc_t* _packet;

    uint32_t _msgpack_size;
    uint32_t _buf_size;

};

class PacketWriter {
public:
    PacketWriter(dai::XLinkStream& stream)
            : _stream(stream) {}

    auto toLE(uint32_t num, uint8_t* le_data) {
        le_data[0] = static_cast<uint8_t>((num & 0x000000ff) >> 0u);
        le_data[1] = static_cast<uint8_t>((num & 0x0000ff00) >> 8u);
        le_data[2] = static_cast<uint8_t>((num & 0x00ff0000) >> 16u);
        le_data[3] = static_cast<uint8_t>((num & 0xff000000) >> 24u);
    };

    void write(const uint8_t* dat, size_t dat_size, const uint8_t* ser, size_t ser_size, uint32_t datatype,
               std::vector<uint8_t>& buf) {
        size_t packet_size = dat_size + ser_size + 8;

        buf.resize(packet_size);
        toLE(datatype, buf.data() + packet_size - 8);
        toLE(dat_size, buf.data() + packet_size - 4);

        memcpy(buf.data(), dat, dat_size);
        memcpy(buf.data() + dat_size, ser, ser_size);

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
            , _sub_nh(nh) {}

    void closeImpl() override {
        _active.reset();
        Base::closeImpl();
    }

protected:
    // create stream based on name at the time of subscription
    template <class MsgType, dai::DatatypeEnum DataType>
    auto generate_cb_lambda(std::unique_ptr<dai::XLinkStream>& stream,
                            msgpack::sbuffer& sbuf, std::vector<uint8_t>& writer_buf)
            -> boost::function<void(const boost::shared_ptr<MsgType const>&)> {
        const auto core_sub_lambda = [&stream, &sbuf, &writer_buf](const boost::shared_ptr<MsgType const>& msg) {
            Guard guard([] { ROS_ERROR("Communication failed: Device error or misconfiguration."); });

            // convert msg to data
            msgpack::pack(sbuf, *msg);
            PacketWriter writer(*stream);
            writer.write(msg->data.data(), msg->data.size(), reinterpret_cast<uint8_t*>(sbuf.data()), sbuf.size(),
                    static_cast<uint32_t>(DataType),
                    writer_buf);

            sbuf.clear();  // Prevent the sbuf data is accumeted

            guard.disable();
        };

        return core_sub_lambda;
    }

    template <class MsgType>
    auto generate_pub_lambda(ros::NodeHandle& nh, std::string name, std::size_t q_size) {  // name: copied
        ros::Publisher pub = nh.advertise<MsgType>(name, q_size);

        const auto pub_lambda = [this, pub, name]() {
            auto conn = this->getConnection();
            auto stream = dai::XLinkStream(*conn, name, 1);  // no writing happens, so 1 is sufficient
            Guard guard([] { ROS_ERROR("Communication failed: Device error or misconfiguration."); });

            while (this->_running) {
                // block till data is read
                PacketReader reader{stream};
                auto packet = reader.packet();

                msgpack::object_handle oh =
                        msgpack::unpack(reinterpret_cast<const char*>(reader.msgpackBeginPtr()), reader.msgpackSize());
                msgpack::object obj = oh.get();

                MsgType msg;
                obj.convert(msg);
                msg.data = reader.bufferData();

                // publish data
                pub.publish(msg);
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
        ROS_INFO_STREAM("Required publishers: " << out_links.size());
        for (const auto& node_links : out_links) {
            const auto& node = node_links.node_to;
            // get name for publisher
            const auto& name = node->getStreamName();

            auto common_type = getCommonType(node_links.out_from);
            ROS_INFO_STREAM(name << " (pub): " << static_cast<int>(common_type));

            // create appropriate publisher using the common_type
            switch (common_type) {
                case dai::DatatypeEnum::Buffer:
                    break;
                case dai::DatatypeEnum::CameraControl:
                    // _pub_t[name] = std::thread{generate_pub_lambda<depthai_datatype_msgs::RawCameraControl>(_pub_nh,
                    // name, 10)};
                    break;
                case dai::DatatypeEnum::IMUData:
                    // @TODO: support IMU
                    // _pub_t[name] = std::thread{generate_pub_lambda<depthai_datatype_msgs::RawIMUData>(_pub_nh, name,
                    // 10)}; // Not supported
                    break;
                case dai::DatatypeEnum::ImageManipConfig:
                    break;
                case dai::DatatypeEnum::ImgDetections:
                    _pub_t[name] = std::thread{generate_pub_lambda<depthai_datatype_msgs::RawImgDetections>(_pub_nh, name, 10)};
                    break;
                case dai::DatatypeEnum::ImgFrame:
                    _pub_t[name] =
                            std::thread{generate_pub_lambda<depthai_datatype_msgs::RawImgFrame>(_pub_nh, name, 10)};
                    break;
                case dai::DatatypeEnum::NNData:
                    _pub_t[name] = std::thread{generate_pub_lambda<depthai_datatype_msgs::RawNNData>(_pub_nh, name, 10)};
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
                    _pub_t[name] = std::thread{generate_pub_lambda<depthai_datatype_msgs::RawTracklets>(_pub_nh, name, 10)};
                    break;
                default:
                    break;
            }
        }
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
            ROS_INFO_STREAM(name << " (sub): "  << static_cast<int>(common_type));
            auto conn = this->getConnection();

            // create appropriate subscriber using the common_type
            switch (common_type) {
                case dai::DatatypeEnum::Buffer:
                    break;
                case dai::DatatypeEnum::CameraControl:
                    // @TODO: make writer to xlinkin to work
                    // _streams[name] =
                    //     std::make_unique<dai::XLinkStream>(*conn, name, dai::XLINK_USB_BUFFER_MAX_SIZE);
                    // _sub[name] = _sub_nh.subscribe(name, 1000,
                    //         generate_cb_lambda<depthai_datatype_msgs::RawCameraControl,
                    //                 dai::DatatypeEnum::CameraControl>(_streams[name], _sbuf, _writer_buf));
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
    std::unordered_map<std::string, std::thread> _pub_t;
    std::unordered_map<std::string, ros::Subscriber> _sub;
    std::unordered_map<std::string, std::unique_ptr<dai::XLinkStream>> _streams;

    msgpack::sbuffer _sbuf;  // buffer for deserializing subscribed messages
    std::vector<uint8_t> _writer_buf;  // buffer for writing to xlinkin

    std::shared_ptr<std::uint8_t> _active;
    ros::CallbackQueue _pub_q, _sub_q;
    ros::NodeHandle _pub_nh, _sub_nh;
    std::atomic<bool> _running = true;
};
}  // namespace rr
