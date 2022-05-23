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
 \file camera.cpp
 \brief 
 */


#include "camera.h"
#include "names.h"
#include <sstream>
#include <visp_bridge/image.h>
#include "visp_camera_calibration/srv/calibrate.hpp"
#include <visp/vpImage.h>
#include <visp/vpDot2.h>
#include <visp/vpCalibration.h>
#include "camera_calibration_parsers/parse.hpp"
#include "camera_calibration_parsers/parse_ini.hpp"


#include <visp/vpDisplayX.h>
#include "visp/vpTrackingException.h"
#include <boost/format.hpp>


namespace visp_camera_calibration
{
Camera::Camera(const rclcpp::NodeOptions & options) : Node("calibrator", options),
//            spinner(0),
            queue_size_(1000),
            nb_points_(4),
            img_(360,480,255)

{
  std::string images_path;

  //prepare function objects
  //set_camera_info_service_callback_t set_camera_info_callback = boost::bind(&Camera::setCameraInfoCallback, this, _1,_2);

  //define services
  // set_camera_info_service_ = n_.advertiseService(visp_camera_calibration::set_camera_info_service,set_camera_info_callback);
  
  set_camera_info_service_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(visp_camera_calibration::set_camera_info_service, std::bind(&Camera::setCameraInfoCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // raw_image_publisher_ = n_.advertise<sensor_msgs::Image>(visp_camera_calibration::raw_image_topic, queue_size_);

  raw_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(visp_camera_calibration::raw_image_topic, queue_size_ );

  // calibrate_service_ = n_.serviceClient<visp_camera_calibration::calibrate> (visp_camera_calibration::calibrate_service);
  
  calibrate_service_ =  this->create_client<visp_camera_calibration::srv::Calibrate> (visp_camera_calibration::calibrate_service);


  // ros::param::getCached(visp_camera_calibration::images_path_param,images_path);
  // ros::param::getCached(visp_camera_calibration::gray_level_precision_param,gray_level_precision);
  // ros::param::getCached(visp_camera_calibration::size_precision_param,size_precision);
  
  this->declare_parameter<std::string>(visp_camera_calibration::images_path_param);
  this->declare_parameter<double>(visp_camera_calibration::gray_level_precision_param);
  this->declare_parameter<double>(visp_camera_calibration::size_precision_param);

  rclcpp::Parameter images_path_param = this->get_parameter(visp_camera_calibration::images_path_param);
  rclcpp::Parameter gray_level_precision_param = this->get_parameter(visp_camera_calibration::gray_level_precision_param);
  rclcpp::Parameter size_precision_param = this->get_parameter(visp_camera_calibration::size_precision_param);

  images_path = images_path_param.as_string();
  this->gray_level_precision_ = gray_level_precision_param.as_double();
  this->size_precision_ = size_precision_param.as_double();

  reader_.setFileName(images_path.c_str());
  reader_.setFirstFrameIndex(1);
  reader_.open(img_);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "str=" << images_path);
  vpDisplay* disp = new vpDisplayX();
  disp->init(img_);
  disp->setTitle("camera");

  vpDisplay::display(img_);
  vpDisplay::displayCharString(img_,img_.getHeight()/2,img_.getWidth()/4,"Click to publish camera feed.",vpColor::red);
  vpDisplay::flush(img_);

  //spinner.start();
}

void Camera::sendVideo(){
  double gray_level_precision = this->gray_level_precision_;
  double size_precision = this->size_precision_;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Click to start sending image data");
  while(rclcpp::ok() && !vpDisplay::getClick(img_,false));

  for(unsigned int i=0;i<(unsigned int)reader_.getLastFrameIndex() /*&& ros::ok()*/;i++){
    reader_.acquire(img_);
    sensor_msgs::msg::Image image;

    image = visp_bridge::toSensorMsgsImage(img_);

    vpDisplay::display(img_);

    vpDisplay::displayCharString(img_,img_.getHeight()/2,img_.getWidth()/4,boost::str(boost::format("publishing frame %1% on %2%") % (i+1) % raw_image_publisher_->get_topic_name()).c_str(),vpColor::red);
    vpDisplay::flush(img_);

    raw_image_publisher_->publish(image);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending image %d/%d",i+1,(int)reader_.getLastFrameIndex());
    //vpDisplay::getClick(img_);
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "When finished selecting points, click on the camera window for calibration");
  vpDisplay::displayCharString(img_,img_.getHeight()/2+30,img_.getWidth()/4,"When finished selecting points, click here for calibration",vpColor::red);
  vpDisplay::flush(img_);
  while(rclcpp::ok() && !vpDisplay::getClick(img_,false));
  auto calibrate_comm = std::make_shared<visp_camera_calibration::srv::Calibrate::Request>();

  calibrate_comm->method = vpCalibration::CALIB_VIRTUAL_VS_DIST;
  calibrate_comm->sample_width= img_.getWidth();
  calibrate_comm->sample_height = img_.getHeight();
  
  auto calibrate_comm_result = calibrate_service_->async_send_request(calibrate_comm);
  
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), calibrate_comm_result) == rclcpp::FutureReturnCode::SUCCESS){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service called successfully");

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "standard deviation with distorsion:");
      for(std::vector<double>::iterator i = calibrate_comm_result.get()->std_dev_errs.begin();i!=calibrate_comm_result.get()->std_dev_errs.end();i++)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f",*i);
  }else{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
}

bool Camera::setCameraInfoCallback(const std::shared_ptr<rmw_request_id_t> request_header, 
      const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req,
      std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> res) {
  std::string calib_info;
  std::stringstream ss(calib_info);

  //std::ostream os;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "setting camera info");
  camera_calibration_parsers::writeCalibrationIni(ss,visp_camera_calibration::raw_image_topic,req->camera_info);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s",ss.str().c_str());
  camera_calibration_parsers::writeCalibration("calibration.ini",visp_camera_calibration::raw_image_topic,req->camera_info);


  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "end of setting camera info");
  return true;
}

Camera::~Camera()
{
  // TODO Auto-generated destructor stub
}
}
