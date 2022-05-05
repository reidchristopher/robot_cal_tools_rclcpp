/***
 * This file contains a standalone example of performing a calibration to understand the transform
 * between a robot wrist and the camera attached to it. It assumes that we have:
 *  1. The camera intrinsics
 *  2. The target size and spacing
 *  3. A bunch of pictures of the target, and
 *  4. The position of the robot wrist (in its base frame) when each of the pictures was taken
 *
 * This is only supposed to be a sample to ease understanding. See the equivalent tool for a
 * configurable offline processor of your data
*/

// This include is used to load images and the associated wrist poses from the data/ directory
// of this package. It's my only concession to the "self-contained rule".
#include <rct_ros_tools_rclcpp/data_set.h>
// This include provide useful print functions for outputing results to terminal
#include <rct_common/print_utils.h>
// This header brings in a tool for finding the target in a given image
#include <rct_image_tools/modified_circle_grid_finder.h>
// This header brings in he calibration function for 'moving camera' on robot wrist - what we
// want to calibrate here.
#include <rct_optimizations/extrinsic_hand_eye.h>
// This header brings in the ament_index_cpp::get_package_share_directory() command so I can find the data set
// on your computer.
#include <ament_index_cpp/get_package_share_directory.hpp>
// For std::cout
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace rct_common;

/** @brief Get a value from a specified field in a yaml file and give more detailed error handling */
template <typename T>
T getYaml(const YAML::Node nd, const std::string key)
{
  T output;
  if (!nd[key])
  {
    std::string error_string = "Unable to parse YAML field %s, Error: Does not exist" + key;
    throw(std::runtime_error(error_string));
  }
  try
  {
    output = nd[key].as<T>();
  }
  catch (YAML::TypedBadConversion<T>& ex)
  {
    std::string error_string = "Unable to parse YAML field %s, Error: Does not exist" + key;
    throw(std::runtime_error(error_string));
  }
  return output;
}

template <typename T>
void loadOptionalField(const YAML::Node n, const std::string field, T& val)
{
    if(n[field])
        val = n[field].as<T>();
}

rct_optimizations::CameraIntrinsics loadIntrinsics(const YAML::Node inrinsics_yaml)
{
    rct_optimizations::CameraIntrinsics intr;
    intr.fx() = getYaml<double>(inrinsics_yaml, "fx");
    intr.fy() = getYaml<double>(inrinsics_yaml, "fy");
    intr.cx() = getYaml<double>(inrinsics_yaml, "cx");
    intr.cy() = getYaml<double>(inrinsics_yaml, "cy");
    return intr;
}

Eigen::Isometry3d loadIsometry(const YAML::Node isometry_yaml)
{
    Eigen::Isometry3d transform;
    Eigen::Vector3d translation(getYaml<double>(isometry_yaml, "x"),
                                getYaml<double>(isometry_yaml, "y"),
                                getYaml<double>(isometry_yaml, "z"));
    Eigen::Quaterniond rotation(getYaml<double>(isometry_yaml, "qw"),
                                getYaml<double>(isometry_yaml, "qx"),
                                getYaml<double>(isometry_yaml, "qy"),
                                getYaml<double>(isometry_yaml, "qz"));
    transform.translation() = translation;
    transform.linear() = rotation.matrix();
    return transform;
}

rct_image_tools::ModifiedCircleGridTarget loadTarget(const YAML::Node target_yaml)
{
    rct_image_tools::ModifiedCircleGridTarget target(getYaml<uint>(target_yaml, "rows"),
                                                     getYaml<uint>(target_yaml, "cols"),
                                                     getYaml<double>(target_yaml, "spacing"));
    return target;
}

rct_image_tools::CircleDetectorParams loadTargetFinderParams(const YAML::Node target_params_yaml)
{
    rct_image_tools::CircleDetectorParams target_finder_params;

    loadOptionalField(target_params_yaml, "minThreshold", target_finder_params.minThreshold);
    loadOptionalField(target_params_yaml, "maxThreshold", target_finder_params.maxThreshold);
    loadOptionalField(target_params_yaml, "nThresholds", target_finder_params.nThresholds);
    loadOptionalField(target_params_yaml, "minRepeatability", target_finder_params.minRepeatability);
    loadOptionalField(target_params_yaml, "circleInclusionRadius", target_finder_params.circleInclusionRadius);
    loadOptionalField(target_params_yaml, "maxRadiusDiff", target_finder_params.maxRadiusDiff);
    loadOptionalField(target_params_yaml, "maxAverageEllipseError", target_finder_params.maxAverageEllipseError);
    loadOptionalField(target_params_yaml, "filterByColor", target_finder_params.filterByColor);
    loadOptionalField(target_params_yaml, "filterByArea", target_finder_params.filterByArea);
    loadOptionalField(target_params_yaml, "minArea", target_finder_params.minArea);
    loadOptionalField(target_params_yaml, "maxArea", target_finder_params.maxArea);
    loadOptionalField(target_params_yaml, "filterByCircularity", target_finder_params.filterByCircularity);
    loadOptionalField(target_params_yaml, "minCircularity", target_finder_params.minCircularity);
    loadOptionalField(target_params_yaml, "maxCircularity", target_finder_params.maxCircularity);
    loadOptionalField(target_params_yaml, "filterByInertia", target_finder_params.filterByInertia);
    loadOptionalField(target_params_yaml, "minInertiaRatio", target_finder_params.minInertiaRatio);
    loadOptionalField(target_params_yaml, "maxInertiaRatio", target_finder_params.maxInertiaRatio);
    loadOptionalField(target_params_yaml, "filterByConvexity", target_finder_params.filterByConvexity);
    loadOptionalField(target_params_yaml, "minConvexity", target_finder_params.minConvexity);
    loadOptionalField(target_params_yaml, "maxConvexity", target_finder_params.maxConvexity);

    if (target_params_yaml["circleColor"])
    {
        if (target_params_yaml["circleColor"].as<int>() == 0)
            target_finder_params.circleColor = static_cast<uchar>(0);
        else if (target_params_yaml["circleColor"].as<int>() == 255)
            target_finder_params.circleColor = static_cast<uchar>(255);
        else
            target_finder_params.circleColor = static_cast<uchar>(0);
    }

    return target_finder_params;
}

static const std::string CONFIG_FILE_PARAMETER = "config_filepath";
static const std::string DATA_FILE_PARAMETER = "data_filepath";

class CameraOnWristServer : public rclcpp::Node
{
public:
    explicit CameraOnWristServer()
        : Node("camera_on_wrist_server")
    {
        using namespace std::placeholders;

        run_wrist_cal_srv_ = create_service<std_srvs::srv::Trigger>("run_wrist_cal", std::bind(&CameraOnWristServer::runWristCalCB, this, _1, _2));

        declare_parameter<std::string>(CONFIG_FILE_PARAMETER, "");
        declare_parameter<std::string>(DATA_FILE_PARAMETER, "");
    }
private:
    void runWristCalCB(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        std::string config_filepath, data_filepath;
        try
        {
            // get parameter named "mesh_sourcepath" and local-var name it filepath_str
            if (!get_parameter(CONFIG_FILE_PARAMETER, config_filepath))
                throw std::runtime_error("Failed to get parameter '" + CONFIG_FILE_PARAMETER + "'");
            if (!get_parameter(DATA_FILE_PARAMETER, data_filepath))
                throw std::runtime_error("Failed to get parameter '" + DATA_FILE_PARAMETER + "'");
        }
        catch (const std::exception& ex)
        {
            response->success = false;
            RCLCPP_ERROR_STREAM(get_logger(), "Run Wrist cal service failed: '" << ex.what() << "'");
            return;
        }
        YAML::Node config_yaml = YAML::LoadFile(config_filepath);
        auto check_fields = [this, config_yaml, response](const std::string field)
        {
            if(!config_yaml[field])
            {
                response->success = false;
                RCLCPP_ERROR_STREAM(get_logger(), "Run Wrist cal service failed because yaml field " << field << " was not found");
                return false;
            }
            return true;
        };

        if(!check_fields("camera_info") ||
           !check_fields("wrist_to_camera_guess") ||
           !check_fields("base_to_target_guess") ||
           !check_fields("target") ||
           !check_fields("target_finder_params"))
            return;

        // The general goal is to fill out the 'Extrinsic Camera on Wrist' problem definition
        // and then call 'rct_optimizations::optimize()' on it.

        // Step 1: Create a problem definition. It requires we set:
        // - Camera Intrinsics
        // - Guesses for the 'base to target' and 'wrist to camera'
        // - The wrist poses
        // - Correspondences (2D in image to 3D in target) for each wrist pose
        rct_optimizations::ExtrinsicHandEyeProblem2D3D problem_def;
        problem_def.intr = loadIntrinsics(config_yaml["camera_info"]);
        problem_def.camera_mount_to_camera_guess = loadIsometry(config_yaml["wrist_to_camera_guess"]);
        problem_def.target_mount_to_target_guess = loadIsometry(config_yaml["base_to_target_guess"]);
        rct_image_tools::ModifiedCircleGridTarget target = loadTarget(config_yaml["target"]);
        rct_image_tools::CircleDetectorParams target_finder_params = loadTargetFinderParams(config_yaml["target_finder_params"]);
        rct_image_tools::ModifiedCircleGridTargetFinder target_finder(target, target_finder_params);

        // Step 3.1: Load the data set (and make sure it worked)
        boost::optional<rct_ros_tools::ExtrinsicDataSet> maybe_data_set = rct_ros_tools::parseFromFile(data_filepath);
        // Attempt to load the data set via the data record yaml file:
        if (!maybe_data_set)
        {
            response->success = false;
            RCLCPP_ERROR_STREAM(get_logger(), "Run Wrist cal service failed: '" << "Failed to parse data set from path = " << data_filepath << "'");
            return;
        }
        // Now that its loaded let's create some aliases to make this nicer
        const std::vector<cv::Mat>& image_set = maybe_data_set->images;
        const std::vector<Eigen::Isometry3d>& wrist_poses = maybe_data_set->tool_poses;

        // Now we'll loop over the images and 1) find the dots, 2) pair them with the target location,
        // and 3) push them into the problem definition.
        // This process of finding the dots can fail, we we make sure they are found before adding the
        // wrist location for that image.
        problem_def.observations.reserve(image_set.size());
        for (std::size_t i = 0; i < image_set.size(); ++i)
        {
          // Try to find the circle grid in this image:
          rct_image_tools::TargetFeatures target_features;
          try
          {
            target_features = target_finder.findTargetFeatures(image_set[i]);
          }
          catch(const std::exception& ex)
          {
            std::cerr << "Unable to find the circle grid in image: " << i << "\n";
            continue;
          }

          // We found the target! Let's "zip-up" the correspondence pairs for this image - one for each
          // dot. So we create a data structure:
          rct_optimizations::Observation2D3D obs;
          obs.correspondence_set.reserve(target_features.size());

          // Create correspondences given the target and the features in the image
          obs.correspondence_set = target.createCorrespondences(target_features);

          // Let's add the wrist pose for this image as the "to_camera_mount" transform
          // Since the target is not moving relative to the camera, set the "to_target_mount" transform to identity
          obs.to_camera_mount = wrist_poses[i];
          obs.to_target_mount = Eigen::Isometry3d::Identity();

          // Finally let's add this observation to the problem!
          problem_def.observations.push_back(obs);
        }

        // Step 4: You defined a problem, let the tools solve it! Call 'optimize()'.
        rct_optimizations::ExtrinsicHandEyeResult opt_result = rct_optimizations::optimize(problem_def);

        // Step 5: Do something with your results. Here I just print the results, but you might want to
        // update a data structure, save to a file, push to a mutable joint or mutable state publisher in
        // ROS. The options are many, and it's up to you. We just try to help solve the problem.
        printOptResults(opt_result.converged, opt_result.initial_cost_per_obs, opt_result.final_cost_per_obs);
        printNewLine();

        // Note: Convergence and low cost does not mean a good calibration. See the calibration primer
        // readme on the main page of this repo.
        Eigen::Isometry3d c = opt_result.camera_mount_to_camera;
        printTransform(c, "Wrist", "Camera", "WRIST TO CAMERA");
        printNewLine();

        Eigen::Isometry3d t = opt_result.target_mount_to_target;
        printTransform(t, "Base", "Target", "BASE TO TARGET");
        printNewLine();

        response->success = opt_result.converged;
        response->message = "Finished running calibration";
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr run_wrist_cal_srv_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CameraOnWristServer>();

  rclcpp::spin(node);
  rclcpp::shutdown();
}

