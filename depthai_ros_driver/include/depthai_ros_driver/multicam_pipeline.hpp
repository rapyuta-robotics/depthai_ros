#pragma once

#include <depthai_ros_driver/pipeline.hpp>

namespace depthai_ros_driver {

class MulticamPipeline : public rr::Pipeline {
protected:
    std::vector<std::string> color_stream_list = {
        "previewout", "jpegout", "video", "metaout"
    };
    std::vector<std::string> stereo_stream_list = {
        "left", "right", "disparity", "depth", "disparity_color", "rectified_left", "rectified_right"
    };
    std::vector<std::string> nn_stream_list = {
        "meta_d2h", "metaout", "object_tracker"
    };

protected:
    bool has_stream(const std::string& name) const;

    /**
     * @brief returns whether any of the list members are contained in the json array
     * @note O(N^2) algorithm
     */
    bool has_any(const std::vector<std::string>& list) const;

    bool link_to_xout(auto& source , const std::string& stream_name);

    void configure_color_pipeline(const std::string& config_json);

    void configure_stereo_pipeline(const std::string& config_json);

    /**
     * @brief Configure multicamera pipeline
     */
    void onConfigure(const std::string& config_json);

    /**
     * @brief Returns a pipeline in constant time
     */
    dai::Pipeline onGetPipeline() const {};

    /**
     * @brief returns the compressed streams exposed by the ROS interface
     */
    std::vector<NodeConstPtr> getCompressedVideoStreams(){};

    /**
     * @brief returns the camera control streams exposed by the ROS interface
     */
    std::vector<NodeConstPtr> getControlStreams(){};

    /**
     * @brief returns the streams required to pass information into the device, via ROS interface
     */
    std::vector<NodeConstPtr> getInputStreams(){};

    /**
     * @brief returns the streams that return a tensor, for interfacing with ROS
     */
    std::vector<NodeConstPtr> getTensorOutStreams(){};

protected:
    nlohmann::json _streams;
};

}  // namespace depthai_ros_driver
