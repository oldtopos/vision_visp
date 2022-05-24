#ifndef __parameter_info_hpp__
#define __parameter_info_hpp__

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "std_msgs/msg/string.hpp"

#include "names.hh"

namespace visp_tracker {
  class parameter_info {
    public:
      parameter_info( std::string name, rclcpp::ParameterType type, std::string description, std::string additional_constraints, 
                        bool read_only, bool dynamic_typing,
                        double default_val, double start_val, double end_val, double step_val );
                        
      parameter_info( std::string name, rclcpp::ParameterType type, std::string description, std::string additional_constraints, 
                        bool read_only, bool dynamic_typing,
                        int default_val, int start_val, int end_val, int step_val );

      static bool create_parameters( rclcpp::Node *node, const std::vector<visp_tracker::parameter_info> &params);
      static bool create_parameter( rclcpp::Node *node, const visp_tracker::parameter_info &param );

      std::string name;
      rclcpp::ParameterType type;
      std::string description;
      std::string additional_constraints;
      bool read_only;
      bool dynamic_typing;
      
      union default_val_type
      {
          std::int32_t n;     
          double d;    
      } default_val;
      
      union start_val_type
      {
          std::int32_t n;     
          double d;     
      } start_val;
      
      union end_val_type
      {
          std::int32_t n;     
          double d;     
      } end_val;
      
      union step_val_type
      {
          std::int32_t n;     
          double d;     
      } step_val;
  };

extern std::array<parameter_info, 19> arrModelBasedSettings;
extern std::array<parameter_info, 11> arrModelBasedSettingsKlt;
extern std::array<parameter_info, 11> arrModelBasedSettingsEdge;

} //namespace visp_tracker

#endif // __parameter_info_hpp__
