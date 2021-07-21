#pragma once

#include <memory>
#include <vector>

#include <depthai/device/Device.hpp>
#include <depthai/pipeline/Node.hpp>
#include <depthai/pipeline/Pipeline.hpp>

namespace rr {
class Pipeline {
protected:
    dai::Pipeline _pipeline;

public:
    using NodeConstPtr = std::shared_ptr<const dai::Node>;

    /**
     * @brief The first function to get called, should have the setup ready for any future calls
     * @param[in] config_data value stored in "config" private parameter of the node/nodelet
     *
     */
    void configure(const std::string& config_data = "") { onConfigure(config_data); }

    /**
     * @brief interface called by the driver to get a pipeline ready-to-go
     *
     * @return dai::Pipeline
     */
    [[nodiscard]] dai::Pipeline getPipeline() const {
        onGetPipeline();
        return _pipeline;
    }

protected:
    /**
     * @brief Default implementation for configure step, does nothing
     * @param[in] config_data value stored in "config" private parameter of the node/nodelet
     */
    virtual void onConfigure(const std::string& config_data) {}

    /**
     * @brief Returns a pipeline in constant time
     */
    virtual void onGetPipeline() const {}
};
}  // namespace rr
