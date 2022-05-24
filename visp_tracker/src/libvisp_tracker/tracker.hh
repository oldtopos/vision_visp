#ifndef VISP_TRACKER_TRACKER_HH
# define VISP_TRACKER_TRACKER_HH
# include <boost/filesystem/path.hpp>
# include <boost/thread/recursive_mutex.hpp>

//# include <dynamic_reconfigure/server.h>

//# include <image_proc/advertisement_checker.h>
# include <rcpputils/visibility_control.hpp>
# include <image_transport/image_transport.hpp>

# include <geometry_msgs/msg/twist_stamped.hpp>

# include <sensor_msgs/msg/image.hpp>
# include <sensor_msgs/msg/camera_info.hpp>

# include <tf2_ros/transform_broadcaster.h>
# include <tf2_ros/transform_listener.h>

# include <visp_tracker/srv/init.hpp>
# include <visp_tracker/ModelBasedSettingsConfig.h>
# include <visp_tracker/ModelBasedSettingsKltConfig.h>
# include <visp_tracker/ModelBasedSettingsEdgeConfig.h>
# include <visp_tracker/msg/moving_edge_sites.hpp>
# include <visp_tracker/msg/klt_points.hpp>

# include <visp3/core/vpCameraParameters.h>
# include <visp3/core/vpHomogeneousMatrix.h>
# include <visp3/core/vpImage.h>
# include <visp3/mbt/vpMbGenericTracker.h>
# include <visp3/me/vpMe.h>

# include <string>

#include "parameter_info.hpp"

namespace visp_tracker
{
  class Tracker : public rclcpp::Node
  {
  public:
    typedef vpImage<unsigned char> image_t;

    enum State
    {
      WAITING_FOR_INITIALIZATION,
      TRACKING,
      LOST
    };

//    COMPOSITION_PUBLIC.  FIXME for node composition

    Tracker (const rclcpp::NodeOptions & options );
             
    Tracker (const rclcpp::NodeOptions & options,
                 std::shared_ptr<rclcpp::Node> nh,
                 std::shared_ptr<rclcpp::Node> privateNh,
             bool exiting,
             unsigned queueSize = 5u);
    
    ~Tracker();
    
    void spin();
  protected:
    void initCallback(const std::shared_ptr<rmw_request_id_t> request_header, 
                        const std::shared_ptr<visp_tracker::srv::Init::Request> req,
                        std::shared_ptr<visp_tracker::srv::Init::Response> res);

    void updateMovingEdgeSites(std::shared_ptr<visp_tracker::msg::MovingEdgeSites> sites);
    void updateKltPoints(std::shared_ptr<visp_tracker::msg::KltPoints> klt);

    void checkInputs();
    void waitForImage();

    void objectPositionHintCallback
    (const std::shared_ptr<geometry_msgs::msg::TransformStamped> );

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);

  private:
    bool exiting ()
    {
      return exiting_ || !rclcpp::ok();
    }

    void spinOnce ()
    {
      //callbackQueue_.callAvailable(ros::WallDuration(0));
      //rclcpp::spin_node_once ();
    }

    bool exiting_;

    unsigned queueSize_;

    std::shared_ptr<rclcpp::Node> nodeHandle_;
    std::shared_ptr<rclcpp::Node> nodeHandlePrivate_;
    image_transport::ImageTransport imageTransport_;

    State state_;
    std::string trackerType_;

    image_t image_;

    std::string cameraPrefix_;
    std::string rectifiedImageTopic_;
    std::string cameraInfoTopic_;
    std::string parameterInfo_;
    
    boost::filesystem::path modelPath_;

    image_transport::CameraSubscriber cameraSubscriber_;

    boost::recursive_mutex mutex_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr resultPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transformationPublisher_;
//    tf2_ros::TransformBroadcaster tfBroadcaster_;
    rclcpp::Publisher<visp_tracker::msg::MovingEdgeSites>::SharedPtr movingEdgeSitesPublisher_;
    rclcpp::Publisher<visp_tracker::msg::KltPoints>::SharedPtr kltPointsPublisher_;

    rclcpp::Service<visp_tracker::srv::Init>::SharedPtr initService_;

    std_msgs::msg::Header header_;
    std::shared_ptr<std_msgs::msg::Header> header2_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> info_;

    vpKltOpencv kltTracker_;
    vpMe movingEdge_;
    vpCameraParameters cameraParameters_;
    vpMbGenericTracker tracker_;

    unsigned lastTrackedImage_;

    /// \brief Helper used to check that subscribed topics exist.
    rclcpp::TimerBase::SharedPtr advertisement_timer_;
    //image_proc::AdvertisementChecker checkInputs_;

    vpHomogeneousMatrix cMo_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> transformBroadcaster_;
    
    std::string worldFrameId_;
    bool compensateRobotMotion_;

    std::string childFrameId_;

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr objectPositionHintSubscriber_;
    
    std::shared_ptr<geometry_msgs::msg::TransformStamped> objectPositionHint_;
    
    friend class ModelBasedSettingsConfig;
    friend class ModelBasedSettingsEdgeConfig;
    friend class ModelBasedSettingsKltConfig;
    
    // parameters 
    double angle_appear_;
    double angle_disappear_;
    int mask_size_;
    int range_;
    double threshold_;
    double mu1_;
    double mu2_;
    double sample_step_;
    int strip_;
    double first_threshold_;
    int mask_border_;
    int max_features_;
    int window_size_;
    double quality_;
    double min_distance_;
    double harris_;
    int size_block_;
    int pyramid_lvl_;

  };
} // end of namespace visp_tracker.

#endif //! VISP_TRACKER_TRACKER_HH
