#include <depthai_ros_driver/dai_utils.hpp>
#include <depthai_ros_driver/device_ros.hpp>

namespace rr {
std::vector<NodeConstPtr> filterNodesByName(const dai::Pipeline& pipeline, std::string name) {
    return filterNodesByNames(pipeline, rv::single(name));
}

std::vector<dai::Node::Connection> getConnectionsFrom(const NodeConstPtr& node) {
    const auto& connectionMap = node->getParentPipeline().getConnectionMap();
    return connectionMap | rv::values | rv::join |
           rv::filter([&](const auto& conn) { return conn.outputId == node->id; }) | ranges::to<std::vector>;
}

void switchEndianness(const std::uint8_t* const data, std::uint8_t* const out) {
    for (std::uint8_t i = 0; i < 4; ++i) {
        out[i] = data[3 - i];
    }
};

std::uint32_t switchEndianness(uint8_t* data) {
    std::uint32_t num;
    switchEndianness(data, reinterpret_cast<std::uint8_t*>(&num));
    return num;
};

std::uint32_t switchEndianness(std::uint32_t data) {
    std::uint32_t num;
    switchEndianness(reinterpret_cast<std::uint8_t*>(&data), reinterpret_cast<std::uint8_t*>(&num));
    return num;
};
}  // namespace rr