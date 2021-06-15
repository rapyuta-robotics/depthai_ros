#include <depthai_ros_driver/dai_utils.hpp>

#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/node/ColorCamera.hpp>
#include <depthai/pipeline/node/XLinkIn.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>

#include <gtest/gtest.h>

#include <type_traits>

namespace rr {
void printConnection(const dai::Node::Connection& conn) {
    std::cout << conn.outputName << "(of " << conn.outputId << ") --> " << conn.inputName << "(of " << conn.inputId
              << ")\n";
}

class SimplePipeline : public ::testing::Test {
protected:
    dai::Pipeline p;

    void SetUp() override {
        using namespace dai::node;
        auto cam = p.create<ColorCamera>();
        auto out = p.create<XLinkOut>();
        auto inp = p.create<XLinkIn>();

        inp->out.link(cam->inputConfig);
        cam->preview.link(out->input);
    }
};

TEST_F(SimplePipeline, FilterByName) {
    std::vector<NodeConstPtr> nodes = filterNodesByName(p, "ColorCamera");
    EXPECT_EQ(nodes.size(), 1);

    nodes = filterNodesByName(p, "ColorCamer");
    EXPECT_TRUE(nodes.empty());
}

TEST_F(SimplePipeline, Links) {
    std::vector<NodeConstPtr> nodes;

    nodes = filterNodesByName(p, "XLinkOut");
    EXPECT_EQ(nodes.size(), 1);

    for (const auto& node : nodes) {
        for (auto conn : getConnectionsTo(node)) {
            EXPECT_EQ(conn.inputId, node->id);
        }
    }

    nodes = filterNodesByName(p, "XLinkIn");
    EXPECT_EQ(nodes.size(), 1);

    for (const auto& node : nodes) {
        for (auto conn : getConnectionsFrom(node)) {
            EXPECT_EQ(conn.outputId, node->id);
        }
    }
}

TEST_F(SimplePipeline, LinkedNode) {
    std::vector<NodeConstPtr> nodes;
    NodeConstPtr node;

    nodes = filterNodesByName(p, "XLinkOut");
    EXPECT_EQ(nodes.size(), 1);
    node = nodes[0];

    const auto& inputs = getInputOf(node, "in");
    EXPECT_EQ(inputs.size(), 1);
    EXPECT_EQ(inputs[0].name, "preview");

    nodes = filterNodesByName(p, "XLinkIn");
    EXPECT_EQ(nodes.size(), 1);
    node = nodes[0];

    const auto& outputs = getOutputOf(node, "out");
    EXPECT_EQ(outputs.size(), 1);
    EXPECT_EQ(outputs[0].name, "inputConfig");
}
}  // namespace rr
