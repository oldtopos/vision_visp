#ifndef VISP_TRACKER_NAMES_HH
# define VISP_TRACKER_NAMES_HH
# include <string>

namespace visp_tracker
{
  extern std::string default_tracker_name;
  extern std::string object_position_topic;
  extern std::string object_position_covariance_topic;
  extern std::string moving_edge_sites_topic;
  extern std::string klt_points_topic;
  extern std::string camera_velocity_topic;
  extern std::string init_service;
  extern std::string init_service_viewer;
  extern std::string reconfigure_service_viewer;

  extern std::string default_model_path;

  extern std::string model_description_param;
  
  //
  //  Parameter related
  //
  extern std::string param_array_terminator;
  
  extern std::string param_angle_appear;
  extern std::string param_angle_disappear;
  
   
  extern std::string param_mask_size;
  extern std::string param_range;
  extern std::string param_threshold;
  extern std::string param_mu1;
  extern std::string param_mu2;
  extern std::string param_sample_step;
  extern std::string param_strip;
  extern std::string param_first_threshold;
  
  extern std::string param_mask_border;
  extern std::string param_max_features;
  extern std::string param_window_size;
  extern std::string param_quality;
  extern std::string param_min_distance;
  extern std::string param_harris;
  extern std::string param_size_block;
  extern std::string param_pyramid_lvl;

} // end of namespace visp_tracker;

#endif //! VISP_TRACKER_NAMES_HH
