#include <rclcpp/rclcpp.hpp>

#include <rct_ros_tools_rclcpp/data_set.h>
#include <rct_ros_tools_rclcpp/parameter_loaders.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <std_srvs/srv/empty.hpp>

#include <vector>

class DataCollection : public rclcpp::Node
{
public:
  explicit DataCollection()
  : Node("command_line_cal_node"),
    clock_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
    buffer_(clock_),
    listener_(buffer_)
  {
    this->declare_parameter("base_frame");
    this->declare_parameter("tool_frame");
    this->declare_parameter("image_topic");
    this->declare_parameter("save_dir");

    if (!this->get_parameter("base_frame", base_frame_)) {
      throw(std::runtime_error("Couldn't load parameter"));
    }
    if (!this->get_parameter("tool_frame", tool_frame_)) {
      throw(std::runtime_error("Couldn't load parameter"));
    }
    if (!this->get_parameter("image_topic", image_topic_)) {
      throw(std::runtime_error("Couldn't load parameter"));
    }
    if (!this->get_parameter("save_dir", save_dir_)) {
      throw(std::runtime_error("Couldn't load parameter"));
    }

    // Validate that we can look up required transforms
    geometry_msgs::msg::TransformStamped dummy;
    if (!captureTransform(dummy)) {
      throw std::runtime_error(
              "Transform from " + base_frame_ + " to " + tool_frame_ + " not available");
    }

    it_sub_ =
      image_transport::create_subscription(this, image_topic_,
        std::bind(&DataCollection::onNewImage, this, std::placeholders::_1), "raw");
    it_pub_ = image_transport::create_publisher(this, image_topic_ + "_out");

    trigger_server_ =
      this->create_service<std_srvs::srv::Empty>("collect",
        std::bind(&DataCollection::onTrigger, this, std::placeholders::_1, std::placeholders::_2));
    save_server_ =
      this->create_service<std_srvs::srv::Empty>("save",
        std::bind(&DataCollection::onSave, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void onNewImage(const sensor_msgs::msg::Image::ConstPtr & msg)
  {
//    RCLCPP_INFO(this->get_logger(), "New image");
    cv_bridge::CvImagePtr cv_ptr;
    try {
      if (msg->encoding == "mono16") {
        cv_bridge::CvImagePtr temp_ptr = cv_bridge::toCvCopy(msg,
            sensor_msgs::image_encodings::MONO16);

        cv::Mat img_conv;
        cv::cvtColor(temp_ptr->image, img_conv, CV_GRAY2BGR);
        img_conv.convertTo(img_conv, CV_8UC1);
        cv_ptr =
          cv_bridge::CvImagePtr(new cv_bridge::CvImage(temp_ptr->header,
            sensor_msgs::image_encodings::BGR8, img_conv));
      } else {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }

      last_frame_ = cv_ptr;
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "BAD THING HAPPEND");
      return;
    }
  }

  void onTrigger(std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr res)
  {
    RCLCPP_INFO(this->get_logger(), "Pose/Image capture triggered...");
    geometry_msgs::msg::TransformStamped pose;
    cv::Mat image;

    if (captureTransform(pose) && captureImage(image)) {
      poses_.push_back(pose);
      images_.push_back(image);
      RCLCPP_INFO(this->get_logger(), "Data collected successfully");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to capture pose/image pair");
    }
  }

  void onSave(std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr res)
  {
    rct_ros_tools::ExtrinsicDataSet data;
    for (std::size_t i = 0; i < poses_.size(); ++i) {
      cv::Mat image = images_[i];
      auto msg = poses_[i];

      Eigen::Isometry3d pose = tf2::transformToEigen(msg);
      data.images.push_back(image);
      data.tool_poses.push_back(pose);
    }
    rct_ros_tools::saveToDirectory(save_dir_, data);
  }

  bool captureImage(cv::Mat & frame)
  {
    if (last_frame_) {
      frame = last_frame_->image;
      return true;
    }
    return false;
  }

  bool captureTransform(geometry_msgs::msg::TransformStamped & out)
  {
    try {
      geometry_msgs::msg::TransformStamped t = buffer_.lookupTransform(base_frame_, tool_frame_,
          tf2::TimePointZero,
          tf2::Duration(std::chrono::seconds(
            10)));
      out = t;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Failed to compute transfrom between " << base_frame_ << " and " << tool_frame_ << ": " <<
        ex.what());
      return false;
    }
  }
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr trigger_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_server_;

  image_transport::Subscriber it_sub_;
  image_transport::Publisher it_pub_;

  std::string base_frame_;
  std::string tool_frame_;

  std::string image_topic_;

  std::vector<geometry_msgs::msg::TransformStamped> poses_;
  std::vector<cv::Mat> images_;

  std::shared_ptr<rclcpp::Clock> clock_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  cv_bridge::CvImagePtr last_frame_;

  std::string save_dir_;
};

template<typename T>
bool get(const rclcpp::SyncParametersClient::SharedPtr & pc, const std::string & key, T & value)
{
  try {
    value = pc->get_parameter<T>(key);
    return true;
  } catch (std::runtime_error & e) {
    return false;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DataCollection>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
