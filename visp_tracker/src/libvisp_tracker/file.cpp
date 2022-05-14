#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <string>

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/format.hpp>

#include <rclcpp/rclcpp.hpp>

#include "file.hh"
#include "names.hh"


std::string
getInitFileFromModelName (const std::string& modelName,
                          const std::string& defaultPath)
{
  boost::format fmt("%1%/%2%/%2%.init");
  fmt % defaultPath % modelName;
  return fmt.str ();
}

std::string
getHelpImageFileFromModelName (const std::string& modelName,
                               const std::string& defaultPath)
{
  boost::format fmt("%1%/%2%/%2%.ppm");
  fmt % defaultPath % modelName;
  return fmt.str ();
}

std::string
getModelFileFromModelName (const std::string& modelName,
                           const std::string& defaultPath)
{
  boost::format fmt("%1%/%2%/%2%");
  fmt % defaultPath % modelName;
  return fmt.str ();
}

std::string
getConfigurationFileFromModelName (const std::string& modelName,
                                   const std::string& defaultPath)
{
  boost::format fmt("%1%/%2%/%2%.xml");
  fmt % defaultPath % modelName;
  return fmt.str ();
}

std::string
getInitialPoseFileFromModelName (const std::string& modelName,
                                 const std::string& defaultPath)
{
  boost::format fmt("%1%/%2%/%2%.0.pos");
  fmt % defaultPath % modelName;
  return fmt.str ();
}

bool
makeModelFile(boost::filesystem::ofstream& modelStream,
              std::string& fullModelPath,
              std::string& modelDescription)
{
#if 0
  std::string modelDescription;
  if (!ros::param::has(visp_tracker::model_description_param))
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to initialize: no model is provided.");
    return false;
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Trying to load the model from the parameter server.");

  ros::param::get(visp_tracker::model_description_param, modelDescription);
#endif

  char* tmpname = strdup("/tmp/tmpXXXXXX");
  if (mkdtemp(tmpname) == NULL)
  {
    RCLCPP_ERROR_STREAM
        (rclcpp::get_logger("rclcpp"), "Failed to create the temporary directory: " << strerror(errno));
    return false;
  }
  // From the content of the model description check if the model is in vrml or in cao format
  std::string vrml_header("#VRML #vrml");
  std::string cao_header("V1");
  boost::filesystem::path path(tmpname);
  if (modelDescription.compare(0, 5, vrml_header, 0, 5) == 0) {
    path /= "model.wrl";
  }
  else if (modelDescription.compare(0, 5, vrml_header, 6, 5) == 0) {
    path /= "model.wrl";
  }
  else if (modelDescription.compare(0, 2, cao_header) == 0) {
    path /= "model.cao";
  }
  else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to create the temporary model file: " << path);
    free(tmpname);
    return false;
  }
  free(tmpname);

  fullModelPath = path.native();

  modelStream.open(path);
  if (!modelStream.good())
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to create the temporary file: " << path);
    return false;
  }
  modelStream << modelDescription;
  modelStream.flush();
  return true;
}
