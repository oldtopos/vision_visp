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
 \file Calibrator.cpp
 \brief 
 */

#include "calibrator.h"
#include "names.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "visp_camera_calibration/msg/calib_point.hpp"
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

#include <vector>

namespace visp_camera_calibration
{
  Calibrator::Calibrator(const rclcpp::NodeOptions & options) : Node("calibrator", options),
    queue_size_(1000)
  {

    //define subscribers
    point_correspondence_subscriber_ = this->create_subscription<visp_camera_calibration::msg::CalibPointArray>( visp_camera_calibration::point_correspondence_topic, queue_size_, std::bind(&Calibrator::pointCorrespondenceCallback, this, std::placeholders::_1));

    //define services
    calibrate_service_ = this->create_service<visp_camera_calibration::srv::Calibrate>(visp_camera_calibration::calibrate_service, std::bind(&Calibrator::calibrate, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    //connect to services
    set_camera_info_service_ = this->create_client<sensor_msgs::srv::SetCameraInfo>(visp_camera_calibration::set_camera_info_service);
    set_camera_info_bis_service_ = this->create_client<sensor_msgs::srv::SetCameraInfo>(visp_camera_calibration::set_camera_info_bis_service);

  }

  void Calibrator::pointCorrespondenceCallback(const visp_camera_calibration::msg::CalibPointArray::SharedPtr point_correspondence) {
    vpCalibration calib_all_points;
    calib_all_points.clearPoint();

    for(std::vector<visp_camera_calibration::msg::CalibPoint>::const_iterator i = point_correspondence->points.begin();
        i != point_correspondence->points.end();
        i++
    ){
      vpImagePoint ip(i->i,i->j);
      calib_all_points.addPoint(i->x,i->y,i->z,ip);
    }
    calibrations_.push_back(calib_all_points);

  }

  void Calibrator::calibrate(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<visp_camera_calibration::srv::Calibrate::Request > req,
         std::shared_ptr<visp_camera_calibration::srv::Calibrate::Response> res) {
    std::vector<double> dev;
    std::vector<double> dev_dist;
    double lambda = .5;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "called service calibrate");
    vpCameraParameters cam;

    (void) request_header;
    
    double px = cam.get_px();
    double py = cam.get_px();
    double u0 = req->sample_width/2;
    double v0 = req->sample_height/2;
    double error = 0.;

    cam.initPersProjWithoutDistortion(px, py, u0, v0);
    vpCalibration::setLambda(lambda);

    vpCalibration::computeCalibrationMulti(vpCalibration::vpCalibrationMethodType(req->method), calibrations_, cam, error, false);

    for(std::vector<vpCalibration>::iterator i=calibrations_.begin();
        i!=calibrations_.end();
        i++
    ){
      double deviation;
      double deviation_dist;
      i->cam = cam;
      i->cam_dist = cam;
      i->computeStdDeviation(deviation,deviation_dist);
      dev.push_back(deviation);
      dev_dist.push_back(deviation_dist);
    }
    switch(req->method){
      case vpCalibration::CALIB_LAGRANGE_VIRTUAL_VS:
      case vpCalibration::CALIB_VIRTUAL_VS:
        res->std_dev_errs = dev;
        break;
      case vpCalibration::CALIB_LAGRANGE_VIRTUAL_VS_DIST:
      case vpCalibration::CALIB_VIRTUAL_VS_DIST:
        res->std_dev_errs= dev_dist;
        break;
    }


    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "" << cam);

    auto request = std::make_shared<sensor_msgs::srv::SetCameraInfo::Request>();
    request->camera_info = visp_bridge::toSensorMsgsCameraInfo(cam,req->sample_width,req->sample_height);;
    
    auto result = set_camera_info_service_->async_send_request(request);
    auto result2 = set_camera_info_bis_service_->async_send_request(request);
    
   if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result2) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "set_camera_info service called successfully");
    }else{
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_camera_info");
    }
    return;
  }
  void Calibrator::spin(){
    //rclcpp::spin();
  }

  Calibrator::~Calibrator()
  {
    // TODO Auto-generated destructor stub
  }
}
