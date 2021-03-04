#pragma once

#include <depthai_ros_driver/pipeline.hpp>

namespace rr {
class PipelineEx : public Pipeline {
public:

protected:
    nlohmann::json _streams;

    bool has_stream(const std::string& name) const;
    bool has_any(const std::vector<std::string>& list) const;

    bool link_to_xout(auto& source , const std::string& stream_name);

    void configure_color_pipeline(const std::string& config_json);

    void configure_stereo_pipeline(const std::string& config_json);

};
}  // namespace rr
