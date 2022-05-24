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
 \file image_processing.cpp
 \brief 
 */

#include "image_processing.h"
#include "visp/vpTrackingException.h"
#include "visp/vpMouseButton.h"

#include "names.h"
#include <visp_bridge/image.h>
#include "camera_calibration_parsers/parse.hpp"

#include "visp/vpDisplayX.h"
#include "visp/vpImagePoint.h"
#include "visp/vpPixelMeterConversion.h"
#include "visp/vpMeterPixelConversion.h"
#include "visp/vpPose.h"
#include "visp/vpHomogeneousMatrix.h"
#include "visp/vpDot.h"
#include "visp/vpDot2.h"
#include "visp/vpCalibration.h"
#include <boost/format.hpp>
#include <iostream>


namespace visp_camera_calibration
{
ImageProcessing::ImageProcessing(const rclcpp::NodeOptions & options) : Node("calibrator", options),
//    spinner_(0),
    queue_size_(1000),
    pause_image_(false),
    img_(480,640,128),
    cam_(600,600,0,0),
    is_initialized(false)
{
  visp_camera_calibration::remap();
  //Setup ROS environment
  //define publishers & subscribers
  raw_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>( visp_camera_calibration::raw_image_topic, queue_size_, std::bind(&ImageProcessing::rawImageCallback, this, std::placeholders::_1));

  calibrate_service_ = this->create_client<visp_camera_calibration::srv::Calibrate> (visp_camera_calibration::calibrate_service);

  point_correspondence_publisher_ = this->create_publisher<visp_camera_calibration::msg::CalibPointArray>(visp_camera_calibration::point_correspondence_topic, queue_size_ );
  
  set_camera_info_bis_service_ = this->create_service<sensor_msgs::srv::SetCameraInfo>(visp_camera_calibration::set_camera_info_bis_service, std::bind(&ImageProcessing::setCameraInfoBisCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  this->declare_parameter<std::vector<double> >(visp_camera_calibration::model_points_param);
  this->declare_parameter<std::vector<double> >(visp_camera_calibration::selected_points_param);
  this->declare_parameter<std::string>(visp_camera_calibration::calibration_path_param);
  this->declare_parameter<double>(visp_camera_calibration::gray_level_precision_param);
  this->declare_parameter<double>(visp_camera_calibration::size_precision_param);
  this->declare_parameter<bool>(visp_camera_calibration::pause_at_each_frame_param);

  rclcpp::Parameter model_points_array_param = this->get_parameter(visp_camera_calibration::model_points_param);
  rclcpp::Parameter selected_points_array_param = this->get_parameter(visp_camera_calibration::selected_points_param);
  rclcpp::Parameter calibration_path_param = this->get_parameter(visp_camera_calibration::calibration_path_param);
  rclcpp::Parameter gray_level_precision_param = this->get_parameter(visp_camera_calibration::gray_level_precision_param);
  rclcpp::Parameter size_precision_param = this->get_parameter(visp_camera_calibration::size_precision_param);
  rclcpp::Parameter pause_at_each_frame_param = this->get_parameter(visp_camera_calibration::pause_at_each_frame_param);
  
  this->calibration_path_ = calibration_path_param.as_string();
  this->gray_level_precision_ = gray_level_precision_param.as_double();
  this->size_precision_ = size_precision_param.as_double();
  this->pause_at_each_frame_ = pause_at_each_frame_param.as_bool();
  
  std::vector<double> vec_model_points = model_points_array_param.as_double_array();
  
  for( unsigned int model_index = 0; model_index < vec_model_points.size();model_index += 3 )
  {
    double X = vec_model_points[model_index];
    double Y = vec_model_points[model_index + 1];
    double Z = vec_model_points[model_index + 2];
    vpPoint p;
    p.setWorldCoordinates(X,Y,Z);
    model_points_.push_back(p);
  }
  
  std::vector<double> vec_selected_points = selected_points_array_param.as_double_array();
   
  for( unsigned int selected_index = 0; selected_index < vec_selected_points.size(); selected_index += 3 )
  {
    double X = vec_selected_points[selected_index];
    double Y = vec_selected_points[selected_index + 1];
    double Z = vec_selected_points[selected_index + 2];
    vpPoint p;
    p.setWorldCoordinates(X,Y,Z);
    selected_points_.push_back(p);
  }

}

void ImageProcessing::init() 
{
if (! is_initialized) {
  //init graphical interface
  vpDisplay* disp = new vpDisplayX();
  disp->init(img_);
  disp->setTitle("Image processing initialisation interface");
  vpDisplay::flush(img_);
  vpDisplay::display(img_);
  vpDisplay::displayCharString(img_,img_.getHeight()/2-10,img_.getWidth()/4,"Waiting for the camera feed.",vpColor::red);
  vpDisplay::displayCharString(img_,img_.getHeight()/2+10,img_.getWidth()/4,"If you are using the example camera, you should click on it's window",vpColor::red);

  vpDisplay::flush(img_);

  //init camera
  double px = cam_.get_px();
  double py = cam_.get_px();
  double u0 = img_.getWidth()/2;
  double v0 = img_.getHeight()/2;
  cam_.initPersProjWithoutDistortion(px, py, u0, v0);

  is_initialized = true;
  }
}

bool ImageProcessing::setCameraInfoBisCallback(const std::shared_ptr<rmw_request_id_t> request_header, 
      const std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Request> req,
      std::shared_ptr<sensor_msgs::srv::SetCameraInfo::Response> res) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "saving calibration file to %s",this->calibration_path_.c_str());
  camera_calibration_parsers::writeCalibration(this->calibration_path_,visp_camera_calibration::raw_image_topic,req->camera_info);
  return true;
}


void ImageProcessing::rawImageCallback(const sensor_msgs::msg::Image::SharedPtr image){
  rclcpp::Rate loop_rate(200);
  double gray_level_precision = this->gray_level_precision_;
  double size_precision = this->size_precision_;
  bool pause_at_each_frame = this->pause_at_each_frame_ ; //Wait for user input each time a new frame is recieved.

  vpPose pose;
  vpCalibration calib;
  visp_camera_calibration::msg::CalibPointArray calib_all_points;

  img_ = visp_bridge::toVispImage(*image);
  
  init();

  vpDisplay::display(img_);
  vpDisplay::flush(img_);

  if(!pause_at_each_frame){
    vpImagePoint ip;
    vpDisplay::displayRectangle(img_,0,0,img_.getWidth(),15,vpColor::black,true);
    vpDisplay::displayCharString(img_,10,10,"Click on the window to select the current image",vpColor::red);
    vpDisplay::flush(img_);
    if(pause_image_){
      pause_image_= false;
    }
    else{
      return;
    }
  }

  pose.clearPoint();
  calib.clearPoint();
	vpImagePoint ip;

  //lets the user select keypoints
  for(unsigned int i=0;i<selected_points_.size();i++){
    try{
      vpDot2 d;
      d.setGrayLevelPrecision(gray_level_precision);
      d.setSizePrecision(size_precision);
      
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Click on point %d",i+1);
      vpDisplay::displayRectangle(img_,0,0,img_.getWidth(),15,vpColor::black,true);
      vpDisplay::displayCharString(img_,10,10,boost::str(boost::format("click on point %1%") % (i+1)).c_str(),vpColor::red);
      vpDisplay::flush(img_);  
      while(rclcpp::ok() && !vpDisplay::getClick(img_,ip,false));
      
      d.initTracking(img_, ip);

      ip.set_ij(d.getCog().get_i(),d.getCog().get_j());
      double x=0,y=0;
      vpPixelMeterConversion::convertPoint(cam_, ip, x, y);
      selected_points_[i].set_x(x);
      selected_points_[i].set_y(y);
      pose.addPoint(selected_points_[i]);
      calib.addPoint(selected_points_[i].get_oX(),selected_points_[i].get_oY(),selected_points_[i].get_oZ(), ip);
      
			vpDisplay::displayCross(img_, d.getCog(), 10, vpColor::red);
			vpDisplay::flush(img_);
    }catch(vpTrackingException e){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to init point");
    }
  }


  vpHomogeneousMatrix cMo;
  pose.computePose(vpPose::LAGRANGE, cMo) ;
  pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
  vpHomogeneousMatrix cMoTmp = cMo;

  vpCameraParameters camTmp = cam_;
  //compute local calibration to match the calibration grid with the image
  try{
    calib.computeCalibration(vpCalibration::CALIB_VIRTUAL_VS,cMoTmp,camTmp,false);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "cMo="<<std::endl <<cMoTmp<<std::endl);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "cam="<<std::endl <<camTmp<<std::endl);

    //project all points and track their corresponding image location
    for (std::vector<vpPoint>::iterator model_point_iter= model_points_.begin();
          model_point_iter!= model_points_.end() ;
          model_point_iter++){
      //project each model point into image according to current calibration
      vpColVector _cP, _p ;
      
			model_point_iter->changeFrame(cMoTmp,_cP) ;
      model_point_iter->projection(_cP,_p) ;
      vpMeterPixelConversion::convertPoint(camTmp,_p[0],_p[1], ip);
      if (10 < ip.get_u() && ip.get_u() < img_.getWidth()-10 &&
          10 < ip.get_v() && ip.get_v() < img_.getHeight()-10) {
        try {
          //track the projected point, look for match
          vpDot2 md;
          md.setGrayLevelPrecision(gray_level_precision);
          md.setSizePrecision(size_precision);

          md.initTracking(img_, ip);
          if(!rclcpp::ok())
						return;

          vpRect bbox = md.getBBox();
          vpImagePoint cog = md.getCog();
          if(bbox.getLeft()<5 || bbox.getRight()>(double)img_.getWidth()-5 ||
              bbox.getTop()<5 || bbox.getBottom()>(double)img_.getHeight()-5||
              vpMath::abs(ip.get_u() - cog.get_u()) > 10 ||
              vpMath::abs(ip.get_v() - cog.get_v()) > 10){
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "tracking failed[suspicious point location].");
          }else{
            //point matches
            double x=0, y=0;
            vpPixelMeterConversion::convertPoint(camTmp, cog, x, y)  ;
            model_point_iter->set_x(x) ;
            model_point_iter->set_y(y) ;

            md.display(img_,vpColor::yellow, 2);
            visp_camera_calibration::msg::CalibPoint cp;
            cp.i = cog.get_i();
            cp.j = cog.get_j();
            cp.x = model_point_iter->get_oX();
            cp.y = model_point_iter->get_oY();
            cp.z = model_point_iter->get_oZ();

            calib_all_points.points.push_back(cp);

            model_point_iter->display(img_,cMoTmp,camTmp) ;
            loop_rate.sleep(); //To avoid refresh problems
            vpDisplay::flush(img_);
          }
        } catch(...){
          RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "tracking failed.");
        }
      } else {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "bad projection.");
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Left click on the interface window to continue, right click to restart");
    vpDisplay::displayRectangle(img_,0,0,img_.getWidth(),15,vpColor::black,true);
    vpDisplay::displayCharString(img_,10,10,"Left click on the interface window to continue, right click to restart",vpColor::red);
    vpDisplay::flush(img_);
		
    vpMouseButton::vpMouseButtonType btn;
    while(rclcpp::ok() && !vpDisplay::getClick(img_,ip,btn, false));
		
    if(btn==vpMouseButton::button1)
      point_correspondence_publisher_->publish(calib_all_points);
    else{
      rawImageCallback(image);
      return;
    }
  }catch(...){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "calibration failed.");
  }
}

void ImageProcessing::interface()
{
  vpImagePoint ip;
  while(rclcpp::ok()){
    //rclcpp::spinOnce();
    if(vpDisplay::getClick(img_,ip,false))
      pause_image_ = true;
  }
  //rclcpp::waitForShutdown();
}

ImageProcessing::~ImageProcessing()
{
  // TODO Auto-generated destructor stub
}
}
