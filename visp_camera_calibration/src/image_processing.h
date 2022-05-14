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
 \file image_processing.h
 \brief 
 */
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/srv/set_camera_info.hpp"
#include "visp/vpImage.h"
#include "visp/vpPoint.h"
#include "visp/vpCameraParameters.h"
#include <vector>
#include <string>
#include <boost/thread/thread.hpp>

#include "visp_camera_calibration/msg/calib_point_array.hpp"
#include "visp_camera_calibration/msg/calib_point.hpp"
#include "visp_camera_calibration/srv/calibrate.hpp"

#ifndef __visp_camera_calibration_IMAGE_PROCESSING_H__
#define __visp_camera_calibration_IMAGE_PROCESSING_H__
namespace visp_camera_calibration
{
class ImageProcessing : public rclcpp::Node
{
  //ros::AsyncSpinner spinner_;

  unsigned long queue_size_;
  bool pause_image_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_subscriber_;
  rclcpp::Publisher<visp_camera_calibration::msg::CalibPointArray>::SharedPtr point_correspondence_publisher_;
  
  rclcpp::Client<visp_camera_calibration::srv::Calibrate>::SharedPtr calibrate_service_;
  
  rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_bis_service_;

  vpImage<unsigned char> img_;

  std::vector<vpPoint> selected_points_;
  std::vector<vpPoint> model_points_;

  vpCameraParameters cam_;
  
  bool is_initialized;
  
  std::string calibration_path_;
  double gray_level_precision_;
  double size_precision_;
  bool pause_at_each_frame_;

  void init();
  /*!
    \brief callback corresponding to the raw_image topic.
    Computes a cMo from selected points on the image.
    Projects all points of the model to match the image of the grid.
    Add the obtained calibration object to an internal calibration list.

    \param image_and_points: image of the grid and selected keypoints to compute on
   */
  void rawImageCallback(const sensor_msgs::msg::Image::SharedPtr image);

public:

  //! service type declaration for calibrate service
  bool setCameraInfoBisCallback(const std::shared_ptr<rmw_request_id_t> request_header, 
      const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req,
      std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> res);

  ImageProcessing(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void interface();

  virtual ~ImageProcessing();
};
}
#endif /* IMAGE_PROCESSING_H_ */
