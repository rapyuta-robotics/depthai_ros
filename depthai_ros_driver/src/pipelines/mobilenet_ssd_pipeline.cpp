#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
#include <depthai_ros_driver/pipeline_ex.hpp>

namespace depthai_ros_driver
{
class MobilenetSSDPipeline : public rr::PipelineEx {
public:

protected:
    /**
     * @brief Configuring mobilenet-ssd configuration
     */
    void onConfigure(const std::string& config_json) {
        configure_mobilenet_ssd_pipeline(config_json);
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

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::MobilenetSSDPipeline, rr::Pipeline)

} // namespace depthai_ros_driver