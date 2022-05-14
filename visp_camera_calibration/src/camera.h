/****************************************************************************
 *
 * $Id: file.cpp 3496 2011-11-22 15:14:32Z fnovotny $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * 
 *
 * Authors:
 * Filip Novotny
 * 
 *
 *****************************************************************************/

/*!
 \file camera.h
 \brief 
 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "visp/vpVideoReader.h"
#include "sensor_msgs/srv/set_camera_info.hpp"
#include <string>

#include "visp_camera_calibration/msg/calib_point_array.hpp"
#include "visp_camera_calibration/msg/calib_point.hpp"
#include "visp_camera_calibration/srv/calibrate.hpp"

#ifndef __visp_camera_calibration_CAMERA_H__
#define __visp_camera_calibration_CAMERA_H__
namespace visp_camera_calibration
{
class Camera : public rclcpp::Node
{
private:
  //ros::AsyncSpinner spinner;
  //ros::Publisher raw_image_publisher_;
  //ros::ServiceClient calibrate_service_;

  //ros::ServiceServer set_camera_info_service_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_publisher_;
  
  rclcpp::Client<visp_camera_calibration::srv::Calibrate>::SharedPtr calibrate_service_;
  
  rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_service_;

  unsigned int queue_size_;
  unsigned int nb_points_;
  double gray_level_precision_;
  double size_precision_;

  vpVideoReader reader_;
  vpImage<unsigned char> img_;

  bool setCameraInfoCallback(const std::shared_ptr<rmw_request_id_t> request_header, 
      const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req,
      std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> res);
      
public:
  Camera(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void sendVideo();
  virtual ~Camera();
};
}
#endif /* CAMERA_H_ */
