#pragma once

#include <depthai_ros_driver/pipeline.hpp>

namespace rr {
class PipelineEx : public Pipeline {
public:
    static bool has_stream(const nlohmann::json& streams, const std::string& name) {
        return (std::find(streams.begin(), streams.end(), name) != streams.end());
    }

    /**
     * @brief returns whether any of the list members are contained in the json array
     * @note O(N^2) algorithm
     */
    // NOTE: has_any is a O(N^2) algorithm
    static bool has_any(const nlohmann::json& streams, const std::vector<std::string>& list) {
    for (const std::string& item: list) {
        if (has_stream(streams, item)) {
            return true; // found
        }
    }

    return false;  // didn't find any
}

protected:

    void configure_color_pipeline(const std::string& config_json);

    void configure_stereo_pipeline(const std::string& config_json);

};
}  // namespace rr
