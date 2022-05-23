#include <stdexcept>
#include <boost/bind.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visp3/core/vpImage.h>

#include <visp_tracker/srv/init.hpp>

#include "names.hh"
#include "conversion.hh"
#include "callbacks.hh"

#include <visp3/mbt/vpMbGenericTracker.h>


void imageCallback(vpImage<unsigned char>& image,
                const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info )
{
  (void) info;
  try
  {
    rosImageToVisp(image, msg);
  }
  catch(std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "dropping frame: " << e.what());
  }
}

void imageCallback(vpImage<unsigned char>& image,
                   std::shared_ptr<std_msgs::msg::Header> &header,
                   std::shared_ptr<sensor_msgs::msg::CameraInfo> &info,
                   const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr &infoConst )
{
  imageCallback(image, msg, info);
  header->stamp.sec = msg->header.stamp.sec;
  header->stamp.nanosec = msg->header.stamp.nanosec;
  header->frame_id = msg->header.frame_id;
  info = std::make_shared<sensor_msgs::msg::CameraInfo>(*infoConst);
}

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image)
{
  return boost::bind(imageCallback, std::ref(image), _1, _2);
}

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image,
                  std::shared_ptr<std_msgs::msg::Header> &header,
                  std::shared_ptr<sensor_msgs::msg::CameraInfo> &info)
{
  return boost::bind
      (imageCallback,
        std::ref(image), std::ref(header), std::ref(info), _1, _2);
}


void reconfigureCallback(vpMbGenericTracker &tracker,
                         vpImage<unsigned char>& I,
                         vpMe& moving_edge,
                         vpKltOpencv& kltTracker,
                         boost::recursive_mutex& mutex,
                         std::shared_ptr<visp_tracker::ModelBasedSettingsConfig> config,
                         uint32_t level)
{
  (void) level;
  mutex.lock ();
  try
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reconfigure Model Based Hybrid Tracker request received.");

    convertModelBasedSettingsConfigToVpMbTracker<visp_tracker::ModelBasedSettingsConfig>(*config, tracker);

    convertModelBasedSettingsConfigToVpMe<visp_tracker::ModelBasedSettingsConfig>(*config, moving_edge, tracker);
    //         moving_edge.print();

    convertModelBasedSettingsConfigToVpKltOpencv<visp_tracker::ModelBasedSettingsConfig>(*config, kltTracker, tracker);

    vpHomogeneousMatrix cMo;
    tracker.getPose(cMo);

    // Check if the image is ready to use
    if (I.getHeight() != 0 && I.getWidth() != 0) {
      tracker.initFromPose(I, cMo);
    }
  }
  catch (...)
  {
    mutex.unlock ();
    throw;
  }
  mutex.unlock ();
}

void reconfigureEdgeCallback(vpMbGenericTracker &tracker,
                             vpImage<unsigned char>& I,
                             vpMe& moving_edge,
                             boost::recursive_mutex& mutex,
                             std::shared_ptr<visp_tracker::ModelBasedSettingsEdgeConfig> config,
                             uint32_t level)
{
  (void) level;

  mutex.lock ();
  try
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reconfigure Model Based Edge Tracker request received.");

    convertModelBasedSettingsConfigToVpMbTracker<visp_tracker::ModelBasedSettingsEdgeConfig>(*config, tracker);
    convertModelBasedSettingsConfigToVpMe<visp_tracker::ModelBasedSettingsEdgeConfig>(*config, moving_edge, tracker);
    // moving_edge.print();

    // Check if the image is ready to use
    if (I.getHeight() != 0 && I.getWidth() != 0) {
      vpHomogeneousMatrix cMo;
      tracker.getPose(cMo);
      // Could not use initFromPose for edge tracker
      // init() function has to be fixed in the trunk first
      // It might have to reset the meLines
      tracker.setPose(I, cMo);
    }
  }
  catch (...)
  {
    mutex.unlock ();
    throw;
  }
  mutex.unlock ();
}

void reconfigureKltCallback(vpMbGenericTracker &tracker,
                            vpImage<unsigned char>& I,
                            vpKltOpencv& kltTracker,
                            boost::recursive_mutex& mutex,
                            std::shared_ptr<visp_tracker::ModelBasedSettingsKltConfig> config,
                            uint32_t level)
{
  (void) level;
  mutex.lock ();
  try
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reconfigure Model Based KLT Tracker request received.");

    convertModelBasedSettingsConfigToVpMbTracker<visp_tracker::ModelBasedSettingsKltConfig>(*config, tracker);
    convertModelBasedSettingsConfigToVpKltOpencv<visp_tracker::ModelBasedSettingsKltConfig>(*config, kltTracker, tracker);

    // Check if the image is ready to use
    if (I.getHeight() != 0 && I.getWidth() != 0) {
      vpHomogeneousMatrix cMo;
      tracker.getPose(cMo);
      tracker.initFromPose(I, cMo);
    }
  }
  catch (...)
  {
    mutex.unlock ();
    throw;
  }
  mutex.unlock ();
}

void reInitViewerCommonParameters(rclcpp::Node *nh,
                                  vpMbGenericTracker &tracker)
{
  auto clientViewer = nh->create_client<visp_tracker::srv::Init>(visp_tracker::reconfigure_service_viewer);

  auto srv = std::make_shared<visp_tracker::srv::Init::Request>();
  convertVpMbTrackerToInitRequest(tracker, srv);
  
  auto result = clientViewer->async_send_request(srv);

  if (rclcpp::spin_until_future_complete(nh->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->initialization_succeed)
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tracker Viewer initialized with success.");
    else
      throw std::runtime_error("failed to initialize tracker viewer.");
  }
}

void reconfigureCallbackAndInitViewer(rclcpp::Node *nh,
                                      vpMbGenericTracker &tracker,
                                      vpImage<unsigned char>& I,
                                      vpMe& moving_edge,
                                      vpKltOpencv& kltTracker,
                                      boost::recursive_mutex& mutex,
                                      std::shared_ptr<visp_tracker::ModelBasedSettingsConfig> config,
                                      uint32_t level)
{
  reconfigureCallback(tracker,I,moving_edge,kltTracker,mutex,config,level);
  reInitViewerCommonParameters(nh,tracker);
}

void reconfigureEdgeCallbackAndInitViewer(rclcpp::Node *nh,
                                          vpMbGenericTracker &tracker,
                                          vpImage<unsigned char>& I,
                                          vpMe& moving_edge,
                                          boost::recursive_mutex& mutex,
                                          std::shared_ptr<visp_tracker::ModelBasedSettingsEdgeConfig> config,
                                          uint32_t level)
{
  reconfigureEdgeCallback(tracker,I,moving_edge,mutex,config,level);
  reInitViewerCommonParameters(nh,tracker);
}

void reconfigureKltCallbackAndInitViewer(rclcpp::Node *nh,
                                         vpMbGenericTracker &tracker,
                                         vpImage<unsigned char>& I,
                                         vpKltOpencv& kltTracker,
                                         boost::recursive_mutex& mutex,
                                         std::shared_ptr<visp_tracker::ModelBasedSettingsKltConfig> config,
                                         uint32_t level)
{
  reconfigureKltCallback(tracker,I,kltTracker,mutex,config,level);
  reInitViewerCommonParameters(nh,tracker);
}
