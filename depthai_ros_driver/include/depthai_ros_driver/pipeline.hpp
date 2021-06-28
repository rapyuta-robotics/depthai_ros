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
     *
     */
    void configure(const std::string& config_json = "") { onConfigure(config_json); }

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
     */
    virtual void onConfigure(const std::string& config_json) {}

    /**
     * @brief Returns a pipeline in constant time
     */
    virtual void onGetPipeline() const {}
};
}  // namespace rr
