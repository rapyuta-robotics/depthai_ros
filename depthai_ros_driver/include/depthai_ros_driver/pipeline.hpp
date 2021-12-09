#pragma once

#include <memory>
#include <vector>

#include <ros/ros.h>

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
     * @param[in] nh private node handle for the driver node
     *
     */
    void configure(ros::NodeHandle& nh) { onConfigure(nh); }

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
     * @param[in] nh private node handle for the driver node
     */
    virtual void onConfigure(ros::NodeHandle& nh) {}

    /**
     * @brief Default implementation for pipeline getter preprocessor, does nothing
     */
    virtual void onGetPipeline() const {}
};
}  // namespace rr
