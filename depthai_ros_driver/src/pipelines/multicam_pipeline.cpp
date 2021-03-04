#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <depthai_ros_driver/pipeline_ex.hpp>

namespace depthai_ros_driver
{
class MulticamPipeline : public rr::PipelineEx {
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
    /**
     * @brief Default implementation for configure step, does nothing
     */
    void onConfigure(const std::string& config_json) {
        ROS_INFO_STREAM("Multicam pipline config json:\n" << config_json);

        // convert json string to nlohmann::json
        const nlohmann::json json = nlohmann::json::parse(config_json);
        if (!json.contains("streams")) {
            ROS_ERROR("MulticamPipline needs \"streams\" tag for an array of streams in config_json.");
            return;
        }
        _streams = json["streams"];

        if (has_any(stereo_stream_list)) {
            configure_stereo_pipeline(config_json);
        }
        if (has_any(color_stream_list)) {
            configure_color_pipeline(config_json);
        }
    }

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
};

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::MulticamPipeline, rr::Pipeline)

} // namespace depthai_ros_driver