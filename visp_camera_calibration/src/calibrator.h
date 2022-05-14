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
 \file Calibrator.h
 \brief 
 */

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/srv/set_camera_info.hpp>

#include "visp_camera_calibration/msg/calib_point_array.hpp"
#include <visp/vpPoint.h>
#include <visp/vpCalibration.h>
#include "visp_camera_calibration/srv/calibrate.hpp"
#include <vector>

#include <boost/function.hpp>

#ifndef __visp_camera_calibration_CALIBRATOR_H__
#define __visp_camera_calibration_CALIBRATOR_H__
namespace visp_camera_calibration
{
  class Calibrator : public rclcpp::Node
  {
  private:
    unsigned long               queue_size_;

    rclcpp::Subscription<visp_camera_calibration::msg::CalibPointArray>::SharedPtr point_correspondence_subscriber_;
    rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_service_;
    rclcpp::Client<sensor_msgs::srv::SetCameraInfo>::SharedPtr set_camera_info_bis_service_;
    
    rclcpp::Service<visp_camera_calibration::srv::Calibrate>::SharedPtr calibrate_service_;

    std::vector<vpPoint>        selected_points_;
    std::vector<vpPoint>        model_points_;
    std::vector<vpCalibration>  calibrations_;
    /*!
      \brief callback corresponding to the point_correspondence topic.
      Adds the obtained calibration pairs objects to an internal calibration list.

      \param image_and_points: image of the grid and selected keypoints to compute on
     */
    //void pointCorrespondenceCallback(const visp_camera_calibration::msg::CalibPointArray::SharedPtr point_correspondence);
    /*!
      \brief service performing the calibration from all previously computed calibration objects.

     */
    //bool calibrateCallback(visp_camera_calibration::srv::Calibrate_Request  &req,
        //                   visp_camera_calibration::srv::Calibrate_Response &res);
  public:
    //! subscriber type declaration for raw_image topic subscriber
    void pointCorrespondenceCallback(const visp_camera_calibration::msg::CalibPointArray::SharedPtr point_correspondence) ;

    //! service type declaration for calibrate service
    void calibrate(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<visp_camera_calibration::srv::Calibrate::Request > req,
         std::shared_ptr<visp_camera_calibration::srv::Calibrate::Response> res);
         
         
    Calibrator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    void spin();
    virtual ~Calibrator();
  };
}
#endif /* CALIBRATOR_H_ */
