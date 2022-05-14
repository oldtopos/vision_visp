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
 * Calibrator node
 *
 * Authors:
 * Filip Novotny
 *
 *
 *****************************************************************************/

/*!
  \file calibrator.cpp
  \brief Calibrator node implementing a quick compute service, a compute service and 2 subscribers to world_effector_topic and camera_object_topic.
*/

#include "calibrator.h"
#include <visp_bridge/3dpose.h>
#include "names.h"
#if VISP_VERSION_INT >= (3<<16 | 2<<8 | 0)
#  include <visp3/vision/vpHandEyeCalibration.h>
#else
#  include <visp3/vision/vpCalibration.h>
#endif
#include <visp3/core/vpHomogeneousMatrix.h>


namespace visp_hand2eye_calibration
{
  void Calibrator::cameraObjectCallback(const geometry_msgs::msg::Transform::SharedPtr trans)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "new cMo: [%f,%f,%f -- %f,%f,%f,%f]", trans->translation.x,trans->translation.y,trans->translation.z,trans->rotation.x,trans->rotation.y,trans->rotation.z,trans->rotation.w);
    vpHomogeneousMatrix cMo;
    cMo = visp_bridge::toVispHomogeneousMatrix(*trans);
    cMo_vec_.push_back(cMo);
  }

  void Calibrator::worldEffectorCallback(const geometry_msgs::msg::Transform::SharedPtr trans)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "new wMe: [%f,%f,%f -- %f,%f,%f,%f]", trans->translation.x,trans->translation.y,trans->translation.z,trans->rotation.x,trans->rotation.y,trans->rotation.z,trans->rotation.w);
    vpHomogeneousMatrix wMe;
    wMe = visp_bridge::toVispHomogeneousMatrix(*trans);
    wMe_vec_.push_back(wMe);
  }

  void Calibrator::computeEffectorCameraCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCamera::Request > req,
         std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCamera::Response> res)
  {
    if (cMo_vec_.size() != wMe_vec_.size() || wMe_vec_.size() < 2)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "transformation vectors have different sizes or contain too few elements");
      return;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "computing %d values...",(int)wMe_vec_.size());
    vpHomogeneousMatrix eMc;

#if VISP_VERSION_INT >= (3<<16 | 2<<8 | 0)
    vpHandEyeCalibration::calibrate(cMo_vec_, wMe_vec_, eMc);
#elif VISP_VERSION_INT > (2<<16 | 6<<8 | 1)
    vpCalibration::calibrationTsai(cMo_vec_, wMe_vec_, eMc);
#else
    vpCalibration::calibrationTsai(cMo_vec_.size(), &(cMo_vec_[0]), &(wMe_vec_[0]), eMc);
#endif
    geometry_msgs::msg::Transform trans;
    trans = visp_bridge::toGeometryMsgsTransform(eMc);

    res->effector_camera = trans;
    return;
  }
  
  void Calibrator::computeEffectorCameraQuickCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick::Request > req,
         std::shared_ptr<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick::Response> res)
  {
    visp_hand2eye_calibration::msg::TransformArray camera_object = req->camera_object;
    visp_hand2eye_calibration::msg::TransformArray world_effector = req->world_effector;
    std::vector<vpHomogeneousMatrix> cMo_vec;
    std::vector<vpHomogeneousMatrix> wMe_vec;
    for(unsigned int i=0;i<camera_object.transforms.size();i++){
      cMo_vec.push_back(visp_bridge::toVispHomogeneousMatrix(camera_object.transforms[i]));
      wMe_vec.push_back(visp_bridge::toVispHomogeneousMatrix(world_effector.transforms[i]));
    }
    if (camera_object.transforms.size() != world_effector.transforms.size() || world_effector.transforms.size() < 2)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "transformation vectors have different sizes or contain too few elements");
      return;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "computing...");
    vpHomogeneousMatrix eMc;
#if VISP_VERSION_INT >= (3<<16 | 2<<8 | 0)
    vpHandEyeCalibration::calibrate(cMo_vec, wMe_vec, eMc);
#elif VISP_VERSION_INT > (2<<16 | 6<<8 | 1)
    vpCalibration::calibrationTsai(cMo_vec, wMe_vec, eMc);
#else
    vpCalibration::calibrationTsai(cMo_vec.size(), &(cMo_vec[0]), &(wMe_vec[0]),eMc);
#endif
    geometry_msgs::msg::Transform trans;
    trans = visp_bridge::toGeometryMsgsTransform(eMc);
    res->effector_camera = trans;
    return;
  }

  void Calibrator::resetCallback(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<visp_hand2eye_calibration::srv::Reset::Request > req,
         std::shared_ptr<visp_hand2eye_calibration::srv::Reset::Response> res)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "reseting...");
    cMo_vec_.clear();
    wMe_vec_.clear();
    return;
  }

  Calibrator::Calibrator(const rclcpp::NodeOptions & options) : Node("calibrator", options),
    //check_inputs_(ros::NodeHandle(), ros::this_node::getName()),
    queue_size_(1000)
  {
    //define subscribers
    camera_object_subscriber_ = this->create_subscription<geometry_msgs::msg::Transform>( visp_hand2eye_calibration::camera_object_topic, queue_size_, std::bind(&Calibrator::cameraObjectCallback, this, std::placeholders::_1));
    world_effector_subscriber_ = this->create_subscription<geometry_msgs::msg::Transform>( visp_hand2eye_calibration::world_effector_topic, queue_size_, std::bind(&Calibrator::worldEffectorCallback, this, std::placeholders::_1));

    //define services
    compute_effector_camera_service_ = this->create_service<visp_hand2eye_calibration::srv::ComputeEffectorCamera>(visp_hand2eye_calibration::compute_effector_camera_service, std::bind(&Calibrator::computeEffectorCameraCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    compute_effector_camera_quick_service_ = this->create_service<visp_hand2eye_calibration::srv::ComputeEffectorCameraQuick>(visp_hand2eye_calibration::compute_effector_camera_quick_service, std::bind(&Calibrator::computeEffectorCameraQuickCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    reset_service_ = this->create_service<visp_hand2eye_calibration::srv::Reset>(visp_hand2eye_calibration::reset_service, std::bind(&Calibrator::resetCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));


    //ros::V_string topics;
    //topics.push_back(visp_hand2eye_calibration::camera_object_topic);
    //topics.push_back(visp_hand2eye_calibration::world_effector_topic);


   // check_inputs_.start(topics, 60.0);
    //if (!ros::ok())
    //  return;


  }

  void Calibrator::spin()
  {
    //ros::spin();
  }
  Calibrator::~Calibrator()
  {

  }
}
