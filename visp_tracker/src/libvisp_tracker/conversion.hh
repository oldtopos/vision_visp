#ifndef VISP_TRACKER_CONVERSION_HH
# define VISP_TRACKER_CONVERSION_HH
# include <boost/optional.hpp>

# include <rclcpp/rclcpp.hpp>

# include <geometry_msgs/msg/transform.hpp>
# include <geometry_msgs/msg/transform_stamped.hpp>
# include <geometry_msgs/msg/pose.hpp>
# include <sensor_msgs/msg/image.hpp>
# include <sensor_msgs/msg/camera_info.hpp>
# include <tf2/transform_datatypes.h>
# include <tf2/LinearMath/Transform.h>

# include <visp_tracker/srv/init.hpp>

# include <visp3/core/vpConfig.h>
# include <visp3/mbt/vpMbGenericTracker.h>

# include <visp3/core/vpHomogeneousMatrix.h>
# include <visp3/core/vpCameraParameters.h>
# include <visp3/me/vpMe.h>
# include <visp3/klt/vpKltOpencv.h>

/// \brief Convert a ROS image into a ViSP one.
///
/// This function copy a ROS image into a ViSP image.
/// If the size are not matching, the ViSP image will be
/// resized.
///
/// \warning Some encodings only are supported.
///
/// \param dst ViSP destination image
/// \param src ROS source image
void rosImageToVisp(vpImage<unsigned char>& dst,
                    const sensor_msgs::msg::Image::ConstSharedPtr &src);

/// \brief Convert a ViSP image into a ROS one.
///
/// This function copy a ViSP image into a ROS image.
/// The whole content of the ROS image will be reset except
/// the following field which will not be set:
/// - header
/// - is_bigendian
///
/// \param dst ROS destination image
/// \param src ViSP source image
void vispImageToRos(std::shared_ptr<sensor_msgs::msg::Image> dst,
                    const vpImage<unsigned char>& src);

std::string convertVpMbTrackerToRosMessage(const vpMbGenericTracker &tracker);

std::string convertVpMeToRosMessage(const vpMbGenericTracker &tracker, const vpMe& moving_edge);

std::string convertVpKltOpencvToRosMessage(const vpMbGenericTracker &tracker, const vpKltOpencv& klt);

void vpHomogeneousMatrixToTransform(std::shared_ptr<geometry_msgs::msg::Transform> dst,
                                    const vpHomogeneousMatrix& src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
                                    const geometry_msgs::msg::Transform src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
                                    const geometry_msgs::msg::TransformStamped src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
                                    const std::shared_ptr<tf2::Transform> src);

void transformToVpHomogeneousMatrix(vpHomogeneousMatrix& dst,
                                    const std::shared_ptr<geometry_msgs::msg::Pose> src);

void convertVpMbTrackerToInitRequest(const vpMbGenericTracker &tracker,
                                     std::shared_ptr<visp_tracker::srv::Init::Request> srv);

void convertInitRequestToVpMbTracker(const std::shared_ptr<visp_tracker::srv::Init::Request> req,
                                     vpMbGenericTracker &tracker);

void convertVpMeToInitRequest(const vpMe& moving_edge,
                              const vpMbGenericTracker &tracker,
                              std::shared_ptr<visp_tracker::srv::Init::Request> srv);

void convertInitRequestToVpMe(const std::shared_ptr<visp_tracker::srv::Init::Request> req,
                              vpMbGenericTracker &tracker,
                              vpMe& moving_edge);

void convertVpKltOpencvToInitRequest(const vpKltOpencv& klt,
                                     const vpMbGenericTracker &tracker,
                                     std::shared_ptr<visp_tracker::srv::Init::Request> srv);

void convertInitRequestToVpKltOpencv(const std::shared_ptr<visp_tracker::srv::Init::Request> req,
                                     vpMbGenericTracker &tracker,
                                     vpKltOpencv& klt);

void initializeVpCameraFromCameraInfo(vpCameraParameters& cam,
                                      std::shared_ptr<sensor_msgs::msg::CameraInfo> info);

// Dynamic reconfigure template functions
template<class ConfigType>
void convertModelBasedSettingsConfigToVpMbTracker(const ConfigType& config,
                                                  vpMbGenericTracker &tracker)
{
  tracker.setAngleAppear(vpMath::rad(config.angle_appear_));
  tracker.setAngleDisappear(vpMath::rad(config.angle_disappear_));
}

template<class ConfigType>
void convertVpMbTrackerToModelBasedSettingsConfig(const vpMbGenericTracker &tracker,
                                                  ConfigType& config)
{
  config.angle_appear_ = vpMath::deg(tracker.getAngleAppear());
  config.angle_disappear_ = vpMath::deg(tracker.getAngleDisappear());
}

template<class ConfigType>
void convertModelBasedSettingsConfigToVpMe(const ConfigType& config,
                                           vpMe& moving_edge,
                                           vpMbGenericTracker &tracker)
{
  tracker.setGoodMovingEdgesRatioThreshold(config.first_threshold_);
  moving_edge.setThreshold( config.threshold_ );
  moving_edge.setMaskSize( config.mask_size_ );
  moving_edge.setRange( config.range_ );
  moving_edge.setMu1( config.mu1_ );
  moving_edge.setMu2( config.mu2_ );
  moving_edge.setSampleStep( config.sample_step_ );
  moving_edge.setStrip( config.strip_ );

  //FIXME: not sure if this is needed.
  moving_edge.initMask();
  //Reset the tracker and the node state.
  tracker.setMovingEdge(moving_edge);
}

template<class ConfigType>
void convertVpMeToModelBasedSettingsConfig(const vpMe& moving_edge,
                                           const vpMbGenericTracker &tracker,
                                           ConfigType& config)
{
  config.first_threshold_ = tracker.getGoodMovingEdgesRatioThreshold();
  config.threshold_ = moving_edge.getThreshold();
  config.mask_size_ = moving_edge.getMaskSize();
  config.range_ = moving_edge.getRange();
  config.mu1_ = moving_edge.getMu1();
  config.mu2_ = moving_edge.getMu2();
  config.sample_step_ = moving_edge.getSampleStep();
  config.strip_ = moving_edge.getStrip();
}

template<class ConfigType>
void convertModelBasedSettingsConfigToVpKltOpencv(const ConfigType& config,
                                                  vpKltOpencv& klt,
                                                  vpMbGenericTracker &tracker)
{
  klt.setMaxFeatures(config.max_features_);
  klt.setWindowSize(config.window_size_);
  klt.setQuality(config.quality_);
  klt.setMinDistance(config.min_distance_);
  klt.setHarrisFreeParameter(config.harris_);
  klt.setBlockSize(config.size_block_);
  klt.setPyramidLevels(config.pyramid_lvl_);
  tracker.setKltMaskBorder((unsigned)config.mask_border_);

  tracker.setKltOpencv(klt);
}

template<class ConfigType>
void convertVpKltOpencvToModelBasedSettingsConfig(const vpKltOpencv& klt,
                                                  const vpMbGenericTracker &tracker,
                                                  ConfigType& config)
{
  config.max_features_ = klt.getMaxFeatures();
  config.window_size_ = klt.getWindowSize();
  config.quality_ = klt.getQuality();
  config.min_distance_ = klt.getMinDistance();
  config.harris_ = klt.getHarrisFreeParameter();
  config.size_block_ = klt.getBlockSize();
  config.pyramid_lvl_ = klt.getPyramidLevels();
  config.mask_border_ = tracker.getKltMaskBorder();
}

#endif //! VISP_TRACKER_CONVERSION_HH
