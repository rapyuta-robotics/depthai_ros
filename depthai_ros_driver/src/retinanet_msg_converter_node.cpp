#include <ros/ros.h>
#include <depthai_datatype_msgs/RawNNData.h>
#include <depthai_datatype_msgs/RawImgDetections.h>
#include <depthai_datatype_msgs/ImgDetection.h>

#include "depthai-shared/datatype/RawNNData.hpp"
#include "fp16/fp16.h"  // this thing is from Hunter!

class RetinaNetMsgConverter {
public:
    RetinaNetMsgConverter() = default;
    ~RetinaNetMsgConverter() = default;
    inline void init() { onInit(); }

private:
    void onInit() {
        // nh_ = ros::NodeHandle();
        pnh_ = ros::NodeHandle("~");

        pub_ = nh_.advertise<depthai_datatype_msgs::RawImgDetections>("raw_det", 10);
        sub_ = nh_.subscribe("raw_nn", 10, &RetinaNetMsgConverter::__msg__callback__, this);

        if (!pnh_.getParam("confidence_threshold", conf_threshold_)) {
            pnh_.setParam("confidence_threshold", conf_threshold_);
        }
        if (!pnh_.getParam("tensor_name", tensor_name_)) {
            pnh_.setParam("tensor_name", tensor_name_);
        }
    }
    void __msg__callback__(const depthai_datatype_msgs::RawNNDataConstPtr& msg_in) {
        pub_msg_.detections.clear();

        auto tensor = getTensor(msg_in, tensor_name_);
        auto data = getLayerFp16(msg_in->data, tensor);
        float x1, y1, x2, y2;
        int idx_biggest_conf;
        float biggest_conf;
        // data.size() is 84 * 12276 -> 1 dimension
        // tensor.dim is [84, 12276, 1]
        // 84 -> x, y, w, h, conf_0 until conf_79
        // 12276 is the maximum bbox that can be detected
        const auto dim0 = tensor.dims[0];
        const auto dim1 = tensor.dims[1];
        const auto dim2 = tensor.dims[2];
        const auto n_cls = dim0 - 4;  // 84 - 4 = 80 classes

        // 3D idx to 1D idx -> data[h, w, c] to data[h*W*C + w*C + c]
        for (int i = 0; i < dim1; ++i) {  // loop through all possible bbox
            x1 = data[i * dim0 + 0];      // data[0 * dim1 + i]
            y1 = data[i * dim0 + 1];      // data[1 * dim1 + i]
            x2 = data[i * dim0 + 2];      // data[2 * dim1 + i]
            y2 = data[i * dim0 + 3];      // data[3 * dim1 + i]
            if (x1 > x2)
                std::swap(x1, x2);
            if (y1 > y2)
                std::swap(y1, y2);

            if (x1 < 0 || x1 > 1 || y1 < 0 || y1 > 1 || x2 < 0 || x2 > 1 || y2 < 0 || y2 > 1) {
                // skip if value is outside [0.0, 1.0]
                continue;
            }
            biggest_conf = std::numeric_limits<float>::min();
            idx_biggest_conf = -1;
            for (int cls = 0; cls < n_cls; ++cls) {
                float conf = sigmoid(data[i * dim0 + (4 + cls)]);  // data[(4 + cls) * dim1 + i]
                if (conf > biggest_conf) {
                    biggest_conf = conf;
                    idx_biggest_conf = cls;
                }
            }

            if (biggest_conf < conf_threshold_) {
                continue;
            }

            // PUSH data if all good
            // ROS_INFO_STREAM(i << ". cls: " << idx_biggest_conf << " conf: " << biggest_conf << " x1: " << x1
            //                   << " y1: " << y1 << " x2: " << x2 << " y2: " << x2);
            depthai_datatype_msgs::ImgDetection det;
            det.confidence = biggest_conf;
            det.label = idx_biggest_conf;
            det.xmin = x1;
            det.xmax = x2;
            det.ymin = y1;
            det.ymax = y2;
            pub_msg_.detections.push_back(det);
        }
        pub_.publish(pub_msg_);
    }
    inline static float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }
    /**
     * @brief Get the Tensor given a rawnndata and a the name of the tensor
     *
     * @param msg_in
     * @param tensor_name the name of the tensor
     *
     * @return depthai_datatype_msgs::TensorInfo
     */
    static depthai_datatype_msgs::TensorInfo getTensor(
            const depthai_datatype_msgs::RawNNDataConstPtr& msg_in, const std::string& tensor_name) {
        depthai_datatype_msgs::TensorInfo tensor;
        for (const auto& t : msg_in->tensors) {
            if (t.name == tensor_name) {
                assert(t.dataType == 0 && "t.dataType must be dai::TensorInfo::DataType::FP16");
                tensor = t;
            }
        }
        return tensor;
    }
    /**
     * @brief Get the Layer as vector of <Floating Point 16> which is 2 bytes
     *
     * @param data
     * @param tensor
     * @return std::vector<float>
     */
    static std::vector<float> getLayerFp16(
            const std::vector<uint8_t>& data, const depthai_datatype_msgs::TensorInfo& tensor) {
        // find its offset
        if (tensor.numDimensions > 0) {
            std::size_t size = tensor.dims[tensor.numDimensions - 1] * tensor.strides[tensor.numDimensions - 1];
            std::size_t numElements = size / 2;  // FP16

            std::vector<float> out;
            out.reserve(numElements);
            const auto* pFp16Data = reinterpret_cast<const std::uint16_t*>(&data[tensor.offset]);
            for (std::size_t i = 0; i < numElements; i++) {
                out.push_back(fp16_ieee_to_fp32_value(pFp16Data[i]));
            }
            return out;
        }
        return {};
    }

    // params
    //! @brief only include result that has confidence bigger than conf_threshold_
    float conf_threshold_ = 0.1;
    //! @brief final output tensor name
    std::string tensor_name_ = "StatefulPartitionedCall/functional_17/RetinaNet/concat_2";

    // ros variables
    ros::Subscriber sub_;
    ros::Publisher pub_;
    depthai_datatype_msgs::RawImgDetections pub_msg_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "retinanet_msg_converter_node");
    RetinaNetMsgConverter node;
    node.init();
    ros::spin();

    return 0;
}