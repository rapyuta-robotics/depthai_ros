#pragma once

#include <depthai_ros_driver/pipeline.hpp>

namespace rr {
class PipelineEx : public Pipeline {
protected:

    void configure_preview_pipeline(const std::string& config_json);


    void configure_stereo_pipeline(const std::string& config_json);

    void configure_mobilenet_ssd_pipeline(const std::string& config_json);

};
}  // namespace rr
