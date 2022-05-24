
#include "parameter_info.hpp"

namespace visp_tracker {

parameter_info::parameter_info( std::string name, rclcpp::ParameterType type, std::string description, std::string additional_constraints, 
                        bool read_only, bool dynamic_typing,
                        double default_val, double start_val, double end_val, double step_val )
{
    this->name = name;
    this->type = type;
    this->description = description;
    this->additional_constraints = additional_constraints;
    this->read_only = read_only;
    this->dynamic_typing = dynamic_typing;
    this->default_val.d = default_val;
    this->start_val.d = start_val;
    this->end_val.d = end_val;
    this->step_val.d = step_val;
}
      
parameter_info::parameter_info( std::string name, rclcpp::ParameterType type, std::string description, std::string additional_constraints, 
                        bool read_only, bool dynamic_typing,
                        int default_val, int start_val, int end_val, int step_val )
{
    this->name = name;
    this->type = type;
    this->description = description;
    this->additional_constraints = additional_constraints;
    this->read_only = read_only;
    this->dynamic_typing = dynamic_typing;
    this->default_val.n = default_val;
    this->start_val.n = start_val;
    this->end_val.n = end_val;
    this->step_val.n = step_val;
}

bool parameter_info::create_parameters( rclcpp::Node* node, const std::vector<visp_tracker::parameter_info> &params)
{
  for( auto aParameter : params )
  {
    if( ! create_parameter( node, aParameter ) )
      return false;
  }
  
  return true;
}

bool parameter_info::create_parameter( rclcpp::Node* node, const visp_tracker::parameter_info &param )
{

    if( param.type == rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1 > dummy_range;

      rcl_interfaces::msg::IntegerRange integer_range = rcl_interfaces::build<rcl_interfaces::msg::IntegerRange>()
              .from_value( param.start_val.n )
              .to_value( param.end_val.n )
              .step( param.step_val.n );
              
      rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1 > valid_range;
      valid_range.push_back( integer_range );
      
      rcl_interfaces::msg::ParameterDescriptor parameter_desc = rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
              .name( param.name )
              .type( rclcpp::ParameterType::PARAMETER_INTEGER )
              .description(param.description)
              .additional_constraints( "" )
              .read_only( param.read_only )
              .dynamic_typing( param.dynamic_typing )
              .floating_point_range( dummy_range )
              .integer_range( valid_range );
              
      node->declare_parameter<int>(param.name, param.default_val.d, parameter_desc);
    }
    else if( param.type == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      rcl_interfaces::msg::FloatingPointRange floating_point_range = rcl_interfaces::build<rcl_interfaces::msg::FloatingPointRange>()
              .from_value( param.start_val.d )
              .to_value( param.end_val.d )
              .step( param.step_val.d );
              
      rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1 > valid_range;
      valid_range.push_back( floating_point_range );
      
      rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1 > dummy_range;

      rcl_interfaces::msg::ParameterDescriptor parameter_desc = rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
              .name( param.name )
              .type( rclcpp::ParameterType::PARAMETER_DOUBLE )
              .description(param.description)
              .additional_constraints( "" )
              .read_only( param.read_only )
              .dynamic_typing( param.dynamic_typing )
              .floating_point_range( valid_range )
              .integer_range( dummy_range );
              
      node->declare_parameter<double>(param.name, param.default_val.d, parameter_desc);
    }
    else
    {
      RCLCPP_FATAL (rclcpp::get_logger("rclcpp"), "Parameter type is not handled, currently only Integer and Double supported" );
      return false;
    }
        
    return true;
}

//
// Parameter definition replacements for ROS1 dynamic_reconfigure
//

std::array<parameter_info, 11> arrModelBasedSettingsEdge = { 
  parameter_info(
    param_angle_appear,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "Maximal angle value to consider an appearing face",
    "", false, false, 65.0, 0.0, 90.0, 0.0
  ),
  parameter_info(
    param_angle_disappear,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "Minimal angle value to consider that a face disappear",
    "", false, false, 75.0, 0.0, 90.0, 0.0
  ),
  parameter_info(
    param_mask_size,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "mask_size",
    "", false, false, 7, 3, 15, 0
  ),  
  parameter_info(
    param_range,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "range",
    "", false, false, 5, 0, 50, 0
  ),
  parameter_info(
    param_threshold,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "threshold",
    "", false, false, 2000.0, 0.0, 20000.0, 0.0
  ),
  parameter_info(
    param_mu1,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "contrast continuity (mu1)",
    "", false, false, 0.5, 0.0, 1.0, 0.0
  ),
  parameter_info(
    param_mu2,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "contrast continuity (mu2)",
    "", false, false, 0.5, 0.0, 1.0, 0.0
  ),
  parameter_info(
    param_sample_step,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "sample_step",
    "", false, false, 3.0, 1.0, 50.0, 0.0
  ),
  parameter_info(
    param_strip,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "strip",
    "", false, false, 2, 0, 10, 0
  ),
  parameter_info(
    param_first_threshold,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "good samples remaining threshold before reinitialization (edge)",
    "", false, false, 0.4, 0.0,  1.0, 0.0
  ),
  parameter_info(
    param_array_terminator,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    param_array_terminator,
    "", false, false, 0, 0, 0, 0
  ) 
};


std::array<parameter_info, 11> arrModelBasedSettingsKlt = { 
  parameter_info(
    param_angle_appear,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "Maximal angle value to consider an appearing face",
    "", false, false, 65.0, 0.0, 90.0, 0.0
  ),
  parameter_info(
    param_angle_disappear,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "Minimal angle value to consider that a face disappear",
    "", false, false, 75.0, 0.0, 90.0, 0.0
  ),
  parameter_info(
    param_mask_border,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "Face dead zone size",
    "", false, false, 5, 0, 50, 0
  ),  
  parameter_info(
    param_max_features,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "Maximum number of KLT features",
    "", false, false, 10000, 0, 30000, 0
  ),  
  parameter_info(
    param_window_size,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "Window size arround a KLT point",
    "", false, false, 5, 3, 7, 0
  ),  
  parameter_info(
    param_quality,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "Quality threshold used to consider that a point is valid",
    "", false, false, 0.05, 0.0001, 0.1, 0.0
  ),
  parameter_info(
    param_min_distance,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "Min distance between two points",
    "", false, false, 5.0, 1.0, 50.0, 0.0
  ),
  parameter_info(
    param_harris,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "Harris value",
    "", false, false, 0.01, 0.0, 0.1, 0.0
  ),
  parameter_info(
    param_size_block,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "Block size",
    "", false, false, 3, 2, 10, 0
  ),  
  parameter_info(
    param_pyramid_lvl,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "Number of levels in the pyramid",
    "", false, false, 3, 0, 5, 0
  ),
  parameter_info(
    param_array_terminator,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    param_array_terminator,
    "", false, false, 0, 0, 0, 0
  )   
};



std::array<parameter_info, 19> arrModelBasedSettings = { 
  parameter_info(
    param_angle_appear,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "Maximal angle value to consider an appearing face",
    "", false, false, 65.0, 0.0, 90.0, 0.0
  ),
  parameter_info(
    param_angle_disappear,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "Minimal angle value to consider that a face disappear",
    "", false, false, 75.0, 0.0, 90.0, 0.0
  ),
  parameter_info(
    param_mask_size,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "mask_size",
    "", false, false, 7, 3, 15, 0
  ),  
  parameter_info(
    param_range,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "range",
    "", false, false, 5, 0, 50, 0
  ),
  parameter_info(
    param_threshold,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "threshold",
    "", false, false, 2000.0, 0.0, 20000.0, 0.0
  ),
  parameter_info(
    param_mu1,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "contrast continuity (mu1)",
    "", false, false, 0.5, 0.0, 1.0, 0.0
  ),
  parameter_info(
    param_mu2,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "contrast continuity (mu2)",
    "", false, false, 0.5, 0.0, 1.0, 0.0
  ),
  parameter_info(
    param_sample_step,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "sample_step",
    "", false, false, 3.0, 1.0, 50.0, 0.0
  ),
  parameter_info(
    param_strip,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "strip",
    "", false, false, 2, 0, 10, 0
  ),
  parameter_info(
    param_first_threshold,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "good samples remaining threshold before reinitialization (edge)",
    "", false, false, 0.4, 0.0,  1.0, 0.0
  ),
  parameter_info(
    param_mask_border,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "Face dead zone size",
    "", false, false, 5, 0, 50, 0
  ),  
  parameter_info(
    param_max_features,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "Maximum number of KLT features",
    "", false, false, 10000, 0, 30000, 0
  ),  
  parameter_info(
    param_window_size,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "Window size arround a KLT point",
    "", false, false, 5, 3, 7, 0
  ),  
  parameter_info(
    param_quality,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "Quality threshold used to consider that a point is valid",
    "", false, false, 0.05, 0.0001, 0.1, 0.0
  ),
  parameter_info(
    param_min_distance,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "Min distance between two points",
    "", false, false, 5.0, 1.0, 50.0, 0.0
  ),
  parameter_info(
    param_harris,
    rclcpp::ParameterType::PARAMETER_DOUBLE,
    "Harris value",
    "", false, false, 0.01, 0.0, 0.1, 0.0
  ),
  parameter_info(
    param_min_distance,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "Min distance between two points",
    "", false, false, 3, 2, 10, 0
  ),  
  parameter_info(
    param_pyramid_lvl,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    "Number of levels in the pyramid",
    "", false, false, 3, 0, 5, 0
  ),
  parameter_info(
    param_array_terminator,
    rclcpp::ParameterType::PARAMETER_INTEGER,
    param_array_terminator,
    "", false, false, 0, 0, 0, 0
  ) 
};

} //namespace visp_tracker