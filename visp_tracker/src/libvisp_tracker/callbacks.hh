#ifndef VISP_TRACKER_CALLBACKS_HH
# define VISP_TRACKER_CALLBACKS_HH
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

# include <boost/thread/recursive_mutex.hpp>
# include <image_transport/image_transport.hpp>
# include <sensor_msgs/msg/image.hpp>
# include <std_msgs/msg/header.hpp>

# include <string>

# include <visp3/core/vpImage.h>
# include <visp3/mbt/vpMbGenericTracker.h>
# include <visp3/me/vpMe.h>
# include <visp3/klt/vpKltOpencv.h>

# include <visp_tracker/ModelBasedSettingsConfig.h>
# include <visp_tracker/ModelBasedSettingsKltConfig.h>
# include <visp_tracker/ModelBasedSettingsEdgeConfig.h>

void
imageCallback(vpImage<unsigned char>& image,
                const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info );

void
imageCallback(vpImage<unsigned char>& image,
              std::shared_ptr<std_msgs::msg::Header> &header,
              std::shared_ptr<sensor_msgs::msg::CameraInfo> &info,
              const sensor_msgs::msg::Image::ConstSharedPtr &msg,
              const sensor_msgs::msg::CameraInfo::ConstSharedPtr &infoConst );


image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image);

image_transport::CameraSubscriber::Callback
bindImageCallback(vpImage<unsigned char>& image,
                  std::shared_ptr<std_msgs::msg::Header> &header,
                  std::shared_ptr<sensor_msgs::msg::CameraInfo> &info);

rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
        
void reconfigureCallback(vpMbGenericTracker &tracker,
                         vpImage<unsigned char>& I,
                         vpMe& moving_edge,
                         vpKltOpencv& kltTracker,
                         boost::recursive_mutex& mutex,
                         std::shared_ptr<visp_tracker::ModelBasedSettingsConfig> config,
                         uint32_t level);

void reconfigureEdgeCallback(vpMbGenericTracker &tracker,
                             vpImage<unsigned char>& I,
                             vpMe& moving_edge,
                             boost::recursive_mutex& mutex,
                             std::shared_ptr<visp_tracker::ModelBasedSettingsEdgeConfig> config,
                             uint32_t level);

void reconfigureKltCallback(vpMbGenericTracker &tracker,
                            vpImage<unsigned char>& I,
                            vpKltOpencv& kltTracker,
                            boost::recursive_mutex& mutex,
                            std::shared_ptr<visp_tracker::ModelBasedSettingsKltConfig> config,
                            uint32_t level);

void reInitViewerCommonParameters(rclcpp::Node *nh,
                                  vpMbGenericTracker &tracker);

void reconfigureCallbackAndInitViewer(rclcpp::Node *nh,
                                      vpMbGenericTracker &tracker,
                                      vpImage<unsigned char>& I,
                                      vpMe& moving_edge,
                                      vpKltOpencv& kltTracker,
                                      boost::recursive_mutex& mutex,
                                      std::shared_ptr<visp_tracker::ModelBasedSettingsConfig> config,
                                      uint32_t level);

void reconfigureEdgeCallbackAndInitViewer(rclcpp::Node *nh,
                                          vpMbGenericTracker &tracker,
                                          vpImage<unsigned char>& I,
                                          vpMe& moving_edge,
                                          boost::recursive_mutex& mutex,
                                          std::shared_ptr<visp_tracker::ModelBasedSettingsEdgeConfig> config,
                                          uint32_t level);

void reconfigureKltCallbackAndInitViewer(rclcpp::Node *nh,
                                         vpMbGenericTracker &tracker,
                                         vpImage<unsigned char>& I,
                                         vpKltOpencv& kltTracker,
                                         boost::recursive_mutex& mutex,
                                         std::shared_ptr<visp_tracker::ModelBasedSettingsKltConfig> config,
                                         uint32_t level);


#endif //! VISP_TRACKER_CALLBACKS_HH
