#include "rct_ros_tools_rclcpp/parameter_loaders.h"
//#include <xmlrpcpp/XmlRpcException.h>
#include <yaml-cpp/yaml.h>
//#include <rcl_yaml_param_parser/parser.h>

//template<typename T>
//static bool read(XmlRpc::XmlRpcValue& xml, const std::string& key, T& value)
//{
//  if (!xml.hasMember(key)) return false;
//  try {
//    value = static_cast<T>(xml[key]);
//  } catch (const XmlRpc::XmlRpcException& ex) {
////    ROS_ERROR_STREAM(ex.getMessage());
//    return false;
//  }
//  return true;
//}

//bool rct_ros_tools::loadTarget(const rclcpp::SyncParametersClient::SharedPtr& pc, const std::string& key,
//                               rct_image_tools::ModifiedCircleGridTarget& target)
//{
//  rcl_params_t * params_hdl


//  XmlRpc::XmlRpcValue xml;
//  try
//  {
//      xml = pc->get_parameter(key);
//  }
//  catch (std::runtime_error &e)
//  {
//      return false;
//  }

////  if (!nh.getParam(key, xml)) return false;

//  int rows = 0;
//  int cols = 0;
//  double spacing = 0.0;

//  if (!read(xml, "rows", rows)) return false;
//  if (!read(xml, "cols", cols)) return false;
//  if (!read(xml, "spacing", spacing)) return false;

//  target = rct_image_tools::ModifiedCircleGridTarget(rows, cols, spacing);
//  return true;
//}

//bool rct_ros_tools::loadTarget(const std::string& path, rct_image_tools::ModifiedCircleGridTarget& target)
//{
//  YAML::Node n = YAML::LoadFile(path);
//  int rows = n["target_definition"]["rows"].as<int>();
//  int cols = n["target_definition"]["cols"].as<int>();
//  double spacing = n["target_definition"]["spacing"].as<double>(); // (meters between dot centers)

//  target = rct_image_tools::ModifiedCircleGridTarget(rows, cols, spacing);
//  return true;
//}

bool rct_ros_tools::loadIntrinsics(const rclcpp::SyncParametersClient::SharedPtr& pc, const std::string& key,
                                  rct_optimizations::CameraIntrinsics& intr)
{
  rct_optimizations::CameraIntrinsics temp_intr;
  try
  {
      temp_intr.fx() = pc->get_parameter<double>(key + ".fx");
      temp_intr.fy() = pc->get_parameter<double>(key + ".fy");
      temp_intr.cx() = pc->get_parameter<double>(key + ".cx");
      temp_intr.cy() = pc->get_parameter<double>(key + ".cy");
  }
  catch (std::runtime_error &e) {
      return false;
  }

  intr = temp_intr;
  return true;
}


bool rct_ros_tools::loadPose(const rclcpp::SyncParametersClient::SharedPtr& pc, const std::string& key,
                            Eigen::Affine3d& pose)
{
  pose = Eigen::Affine3d::Identity();
  double x, y, z, qx, qy, qz, qw;

  try
  {
      x = pc->get_parameter<double>(key + ".x");
      y = pc->get_parameter<double>(key + ".y");
      z = pc->get_parameter<double>(key + ".z");
      qx = pc->get_parameter<double>(key + ".qx");
      qy = pc->get_parameter<double>(key + ".qy");
      qz = pc->get_parameter<double>(key + ".qz");
      qw = pc->get_parameter<double>(key + ".qw");
  }
  catch (std::runtime_error &e) {
      return false;
  }

  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

  return true;
}



bool rct_ros_tools::loadPose(const std::string& path, Eigen::Affine3d& pose)
{
  YAML::Node n = YAML::LoadFile(path);
  Eigen::Vector3d position;

  position(0) = n["x"].as<double>();
  position(1) = n["y"].as<double>();
  position(2) = n["z"].as<double>();

  double qw, qx, qy, qz;
  qw = n["qw"].as<double>();
  qx = n["qx"].as<double>();
  qy = n["qy"].as<double>();
  qz = n["qz"].as<double>();

  pose = Eigen::Affine3d::Identity();
  pose.translation() = position;
  pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
  return true;
}

