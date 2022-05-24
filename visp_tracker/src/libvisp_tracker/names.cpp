#include "names.hh"

namespace visp_tracker
{
  std::string default_tracker_name("tracker_mbt");
  std::string object_position_topic("object_position");
  std::string object_position_covariance_topic("object_position_covariance");
  std::string moving_edge_sites_topic("moving_edge_sites");
  std::string klt_points_topic("klt_points");
  std::string camera_velocity_topic("camera_velocity");
  std::string init_service("init_tracker");
  std::string init_service_viewer("init_tracker_viewer");
  std::string reconfigure_service_viewer("reconfigure_tracker_viewer");

  std::string default_model_path("package://visp_tracker/models");

  std::string model_description_param("model_description");
  
  std::string param_array_terminator("parameter_termintor");
  
  std::string param_angle_appear("angle_appear");
  std::string param_angle_disappear("angle_disappear");
  
  std::string param_mask_size("mask_size");
  std::string param_range("range");
  std::string param_threshold("threshold");
  std::string param_mu1("mu1");
  std::string param_mu2("mu2");
  std::string param_sample_step("sample_step");
  std::string param_strip("strip");
  std::string param_first_threshold("first_threshold");
  
  std::string param_mask_border("mask_border");
  std::string param_max_features("max_features");
  std::string param_window_size("window_size");
  std::string param_quality("quality");
  std::string param_min_distance("min_distance");
  std::string param_harris("harris");
  std::string param_size_block("size_block");
  std::string param_pyramid_lvl("pyramid_lvl");

} // end of namespace visp_tracker;
