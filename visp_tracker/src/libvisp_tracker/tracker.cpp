#include <stdexcept>
#include <chrono>

#include <boost/filesystem/fstream.hpp>
#include <boost/format.hpp>
#include <boost/scope_exit.hpp>
#include <boost/version.hpp>

//#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
//#include <image_proc/advertisement_checker.h>
#include <image_transport/image_transport.hpp>
//#include <ros/param.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_map.hpp>
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rosidl_runtime_cpp/bounded_vector.hpp"

#include <image_transport/transport_hints.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>

#include <boost/bind.hpp>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpCameraParameters.h>

#include "tracker.hh"

#include "conversion.hh"
#include "callbacks.hh"
#include "file.hh"
#include "names.hh"

// TODO:
// - add a topic allowing to suggest an estimation of the cMo
// - handle automatic reset when tracking is lost.
// - add a ROS2 executor to deal with properly spinning the node

namespace visp_tracker
{
    ModelBasedSettingsConfig::ModelBasedSettingsConfig( Tracker * theTracker ) {
      angle_appear_       = theTracker->angle_appear_;
      angle_disappear_    = theTracker->angle_disappear_;
      mask_size_          = theTracker->angle_disappear_;
      range_              = theTracker->mask_size_;
      threshold_          = theTracker->threshold_;
      mu1_                = theTracker->mu1_;
      mu2_                = theTracker->mu2_;
      sample_step_        = theTracker->sample_step_;
      strip_              = theTracker->strip_;
      first_threshold_    = theTracker->first_threshold_;
      mask_border_        = theTracker->mask_border_;
      max_features_       = theTracker->max_features_;
      window_size_        = theTracker->window_size_;
      quality_            = theTracker->quality_;
      min_distance_       = theTracker->min_distance_;
      harris_             = theTracker->harris_;
      size_block_         = theTracker->size_block_;
      pyramid_lvl_        = theTracker->pyramid_lvl_;
  }
  
  ModelBasedSettingsEdgeConfig::ModelBasedSettingsEdgeConfig( Tracker * theTracker ) { 
      angle_appear_       = theTracker->angle_appear_;
      angle_disappear_    = theTracker->angle_disappear_;
      mask_size_          = theTracker->angle_disappear_;
      range_              = theTracker->mask_size_;
      threshold_          = theTracker->threshold_;
      mu1_                = theTracker->mu1_;
      mu2_                = theTracker->mu2_;
      sample_step_        = theTracker->sample_step_;
      strip_              = theTracker->strip_;
      first_threshold_    = theTracker->first_threshold_;
  }

  ModelBasedSettingsKltConfig::ModelBasedSettingsKltConfig( Tracker * theTracker ) {
      angle_appear_       = theTracker->angle_appear_;
      angle_disappear_    = theTracker->angle_disappear_;
      mask_border_        = theTracker->mask_border_;
      max_features_       = theTracker->max_features_;
      window_size_        = theTracker->window_size_;
      quality_            = theTracker->quality_;
      min_distance_       = theTracker->min_distance_;
      harris_             = theTracker->harris_;
      size_block_         = theTracker->size_block_;
      pyramid_lvl_        = theTracker->pyramid_lvl_;
  }
 
  void
  Tracker::initCallback(const std::shared_ptr<rmw_request_id_t> request_header, 
                        const std::shared_ptr<visp_tracker::srv::Init::Request> req,
                        std::shared_ptr<visp_tracker::srv::Init::Response> res)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialization request received.");

    res->initialization_succeed = false;

    // If something goes wrong, rollback all changes.
    BOOST_SCOPE_EXIT((res)(&tracker_)(&state_)
                     (&lastTrackedImage_)(&trackerType_))
    {
      if(!res.get()->initialization_succeed)
      {
        tracker_.resetTracker();
        state_ = WAITING_FOR_INITIALIZATION;
        lastTrackedImage_ = 0;

      }
    } BOOST_SCOPE_EXIT_END;

    std::string fullModelPath;
    boost::filesystem::ofstream modelStream;
    std::string modeldescription;
    
    // Load model from parameter.
    if (!makeModelFile(modelStream, fullModelPath, modeldescription))
      return;

    tracker_.resetTracker();

    // Common parameters
    convertInitRequestToVpMbTracker(req, tracker_);

    if(trackerType_!="klt"){ // for mbt and hybrid
      convertInitRequestToVpMe(req, tracker_, movingEdge_);
    }

    if(trackerType_!="mbt"){ // for klt and hybrid
      convertInitRequestToVpKltOpencv(req, tracker_, kltTracker_);
    }

    //
    // Assuming all parameters have default values, and ROS2 parameters are used, updateConfig port is not needed
    #if 0
    if(trackerType_=="mbt+klt"){ // Hybrid Tracker reconfigure
      visp_tracker::ModelBasedSettingsConfig config;
      convertVpMbTrackerToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsConfig>(tracker_, config);
      reconfigureSrv_->updateConfig(config); // set parameters on configuration service ....
      convertVpMeToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsConfig>(movingEdge_, tracker_, config);
      reconfigureSrv_->updateConfig(config);
      convertVpKltOpencvToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsConfig>(kltTracker_, tracker_, config);
      reconfigureSrv_->updateConfig(config);
    }
    else if(trackerType_=="mbt"){ // Edge Tracker reconfigure
      visp_tracker::ModelBasedSettingsEdgeConfig config;
      convertVpMbTrackerToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsEdgeConfig>(tracker_, config);
      reconfigureEdgeSrv_->updateConfig(config);
      convertVpMeToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsEdgeConfig>(movingEdge_, tracker_, config);
      reconfigureEdgeSrv_->updateConfig(config);
    }
    else{ // KLT Tracker reconfigure
      visp_tracker::ModelBasedSettingsKltConfig config;
      convertVpMbTrackerToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsKltConfig>(tracker_, config);
      reconfigureKltSrv_->updateConfig(config);
      convertVpKltOpencvToModelBasedSettingsConfig<visp_tracker::ModelBasedSettingsKltConfig>(kltTracker_, tracker_, config);
      reconfigureKltSrv_->updateConfig(config);
    }
    #endif 
    
    state_ = WAITING_FOR_INITIALIZATION;
    lastTrackedImage_ = 0;

    // Load the model.
    try
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"), "Trying to load the model Tracker: " << fullModelPath);
      tracker_.loadModel(fullModelPath.c_str());
      modelStream.close();
    }
    catch(...)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to load the model: " << fullModelPath);
      return;
    }
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Model has been successfully loaded.");

    // Load the initial cMo.
    transformToVpHomogeneousMatrix(cMo_, req->initial_cmo);

    // Enable covariance matrix.
    tracker_.setCovarianceComputation(true);

    // Try to initialize the tracker.
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Initializing tracker with cMo:\n" << cMo_);
    try
    {
      // Bug between setPose() and initFromPose() not present here due to previous call to resetTracker()
      tracker_.initFromPose(image_, cMo_);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tracker successfully initialized.");

      //movingEdge.print();
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), convertVpMbTrackerToRosMessage(tracker_));
      // - Moving edges.
      if(trackerType_!="klt")
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), convertVpMeToRosMessage(tracker_, movingEdge_));

      if(trackerType_!="mbt")
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), convertVpKltOpencvToRosMessage(tracker_,kltTracker_));
    }
    catch(const std::string& str)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Tracker initialization has failed: " << str);
    }

    // Initialization is valid.
    res->initialization_succeed = true;
    state_ = TRACKING;
    return;
  }

  void
  Tracker::updateMovingEdgeSites(std::shared_ptr<visp_tracker::msg::MovingEdgeSites> sites)
  {
    if (!sites)
      return;

    rclcpp::Clock clock;
    std::list<vpMbtDistanceLine*> linesList;

    if(trackerType_!="klt") { // For mbt and hybrid
      tracker_.getLline(linesList, 0);

      std::list<vpMbtDistanceLine*>::iterator linesIterator = linesList.begin();

      bool noVisibleLine = true;
      for (; linesIterator != linesList.end(); ++linesIterator)
      {
        vpMbtDistanceLine* line = *linesIterator;

#if VISP_VERSION_INT >= VP_VERSION_INT(3,0,0) // ViSP >= 3.0.0
        if (line && line->isVisible() && ! line->meline.empty())
#else
        if (line && line->isVisible() && line->meline)
#endif
        {
#if VISP_VERSION_INT >= VP_VERSION_INT(3,0,0) // ViSP >= 3.0.0
          for(unsigned int a = 0 ; a < line->meline.size() ; a++)
          {
            if(line->meline[a] != NULL) {
              std::list<vpMeSite>::const_iterator sitesIterator = line->meline[a]->getMeList().begin();
              if (line->meline[a]->getMeList().empty())
                RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("rclcpp"), clock, 10, "no moving edge for a line");
              for (; sitesIterator != line->meline[a]->getMeList().end(); ++sitesIterator)
              {
#elif VISP_VERSION_INT >= VP_VERSION_INT(2,10,0) // ViSP >= 2.10.0
          std::list<vpMeSite>::const_iterator sitesIterator = line->meline->getMeList().begin();
          if (line->meline->getMeList().empty())
            RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("rclcpp"), clock, 10, "no moving edge for a line");
          for (; sitesIterator != line->meline->getMeList().end(); ++sitesIterator)
          {
#else
          std::list<vpMeSite>::const_iterator sitesIterator = line->meline->list.begin();
          if (line->meline->list.empty())
            RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("rclcpp"), clock, 10, "no moving edge for a line");
          for (; sitesIterator != line->meline->list.end(); ++sitesIterator)
          {
#endif
            visp_tracker::msg::MovingEdgeSite movingEdgeSite;
            movingEdgeSite.x = sitesIterator->ifloat;
            movingEdgeSite.y = sitesIterator->jfloat;
#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0) // ViSP < 2.10.0
            movingEdgeSite.suppress = sitesIterator->suppress;
#endif
            sites->moving_edge_sites.push_back (movingEdgeSite);
          }
          noVisibleLine = false;
        }
#if VISP_VERSION_INT >= VP_VERSION_INT(3,0,0) // ViSP >= 3.0.0
      }
    }
#endif
  }
  if (noVisibleLine)
  RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("rclcpp"), clock, 10, "no distance lines");
}
}

void
Tracker::updateKltPoints(std::shared_ptr<visp_tracker::msg::KltPoints> klt)
{
  if (!klt)
    return;

#if VISP_VERSION_INT < VP_VERSION_INT(2,10,0) // ViSP < 2.10.0
  vpMbHiddenFaces<vpMbtKltPolygon> *poly_lst;
  std::map<int, vpImagePoint> *map_klt;

  if(trackerType_!="mbt") { // For klt and hybrid
    poly_lst = &dynamic_cast<vpMbKltTracker*>(tracker_)->getFaces();

    for(unsigned int i = 0 ; i < poly_lst->size() ; i++)
    {
      if((*poly_lst)[i])
      {
        map_klt = &((*poly_lst)[i]->getCurrentPoints());

        if(map_klt->size() > 3)
        {
          for (std::map<int, vpImagePoint>::iterator it=map_klt->begin(); it!=map_klt->end(); ++it)
          {
            visp_tracker::KltPoint kltPoint;
            kltPoint.id = it->first;
            kltPoint.i = it->second.get_i();
            kltPoint.j = it->second.get_j();
            klt->klt_points_positions.push_back (kltPoint);
          }
        }
      }
    }
  }
#else // ViSP >= 2.10.0
  std::list<vpMbtDistanceKltPoints*> poly_lst;
  std::map<int, vpImagePoint> *map_klt;

  if(trackerType_!="mbt") { // For klt and hybrid
    poly_lst = tracker_.getFeaturesKlt();

    for(std::list<vpMbtDistanceKltPoints*>::const_iterator it=poly_lst.begin(); it!=poly_lst.end(); ++it){
      map_klt = &((*it)->getCurrentPoints());

      if((*it)->polygon->isVisible()){
        if(map_klt->size() > 3)
        {
          for (std::map<int, vpImagePoint>::iterator it=map_klt->begin(); it!=map_klt->end(); ++it)
          {
            visp_tracker::msg::KltPoint kltPoint;
            kltPoint.id = it->first;
            kltPoint.i = it->second.get_i();
            kltPoint.j = it->second.get_j();
            klt->klt_points_positions.push_back (kltPoint);
          }
        }
      }
    }
  }
#endif
}

//
//  Check for topic existance.  Replacement for ROS1 advertisement_checker 
//
void Tracker::checkInputs()
{
  auto current_topics = this->get_topic_names_and_types();
  
  for (auto it=current_topics.begin(); it!=current_topics.end(); ++it) {
    if( it->first == rectifiedImageTopic_ ) {
      advertisement_timer_->cancel();
      return;
    }
  }
  
  RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "The input topic '%s' is not yet advertised", rectifiedImageTopic_.c_str());
}

Tracker::Tracker(const rclcpp::NodeOptions & options,
                 std::shared_ptr<rclcpp::Node> nh,
                 std::shared_ptr<rclcpp::Node> privateNh,
                 volatile bool& exiting,
                 unsigned queueSize)
  : Node("tracker", options),
    exiting_ (exiting),
    queueSize_(queueSize),
    nodeHandle_(nh),
    nodeHandlePrivate_(privateNh),
    imageTransport_(nodeHandle_),
    state_(WAITING_FOR_INITIALIZATION),
//    image_(),
//    cameraPrefix_(),
//    rectifiedImageTopic_(),
//    cameraInfoTopic_(),
//    modelPath_(),
//    cameraSubscriber_(),
//    mutex_ (),
    //reconfigureSrv_(mutex_, nodeHandlePrivate_),
    //reconfigureSrv_(NULL),
    //reconfigureKltSrv_(NULL),
    //reconfigureEdgeSrv_(NULL),
//    resultPublisher_(),
//    transformationPublisher_(),
//    movingEdgeSitesPublisher_(),
//    kltPointsPublisher_(),
//    initService_(),
//    header_(),
//    info_(),
//    kltTracker_(),
//    movingEdge_(),
//    cameraParameters_(),
//    lastTrackedImage_(),
    //checkInputs_(nodeHandle_, ros::this_node::getName()),
//    cMo_ (),
//    listener_ (),
//    worldFrameId_ (),
    compensateRobotMotion_ (false)
//    transformBroadcaster_ (),
//    childFrameId_ ()
//    objectPositionHintSubscriber_ (),
//    objectPositionHint_ ()
{
  // Set cMo to identity.
  cMo_.eye();

  // Parameters.
  this->declare_parameter<std::string>("camera_prefix", "");
  this->declare_parameter<std::string>("tracker_type", "mbt" );
  this->declare_parameter<std::string>("frame_id", "object_position");
  this->declare_parameter<std::string>("world_frame_id", "/odom");
  this->declare_parameter<std::string>("parameter_info", "config/parameter_info.yaml");
  this->declare_parameter<bool>("compensate_robot_motion", false);

  rclcpp::Parameter camera_prefix_param = this->get_parameter("camera_prefix");
  rclcpp::Parameter tracker_type_param = this->get_parameter("tracker_type");
  rclcpp::Parameter frame_id_param = this->get_parameter("frame_id");
  rclcpp::Parameter world_frame_id_param = this->get_parameter("world_frame_id");
  rclcpp::Parameter parameter_info_param = this->get_parameter("parameter_info");
  rclcpp::Parameter compensate_robot_motion_param = this->get_parameter("compensate_robot_motion");

  cameraPrefix_ = camera_prefix_param.as_string();
  trackerType_ = tracker_type_param.as_string();
  childFrameId_ = frame_id_param.as_string();
  worldFrameId_ = world_frame_id_param.as_string();
  parameterInfo_ = parameter_info_param.as_string();
  compensateRobotMotion_ = compensate_robot_motion_param.as_bool();
  
  // nodeHandlePrivate_.param<std::string>("camera_prefix", cameraPrefix_, "");
  // nodeHandlePrivate_.param<std::string>("tracker_type", trackerType_, "mbt");
  
  if(trackerType_=="mbt")
    tracker_.setTrackerType(vpMbGenericTracker::EDGE_TRACKER);
  else if(trackerType_=="klt")
    tracker_.setTrackerType(vpMbGenericTracker::KLT_TRACKER);
  else
    tracker_.setTrackerType(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);

  if (cameraPrefix_.empty ())
  {
    RCLCPP_FATAL
        (rclcpp::get_logger("rclcpp"), "The camera_prefix parameter not set.\n"
         "Please relaunch the tracker while setting this parameter, i.e.\n"
         "$ rosrun visp_tracker tracker _camera_prefix:=/my/camera");
    rclcpp::shutdown ();
    return;
  }
  // Create global /camera_prefix param to avoid to remap in the launch files the tracker_client and tracker_viewer nodes
  //nodeHandle_.setParam("camera_prefix", cameraPrefix_);
  this->set_parameter(camera_prefix_param);
  //nodeHandle_.param<std::string>("frame_id", childFrameId_, "object_position");

  // Robot motion compensation.
  //nodeHandle_.param<std::string>("world_frame_id", worldFrameId_, "/odom");
  //nodeHandle_.param<bool>
  //    ("compensate_robot_motion", compensateRobotMotion_, false);

  // Compute topic and services names.
  //rectifiedImageTopic_ =
  //    ros::names::resolve(cameraPrefix_ + "/image_rect");

  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  transformBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  
  // Check for subscribed topics.
  using namespace std::chrono_literals;
  advertisement_timer_ = this->create_wall_timer( 60s, std::bind(&Tracker::checkInputs, this));

  // Result publisher.
  resultPublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(visp_tracker::object_position_covariance_topic, queueSize_ );

  transformationPublisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(visp_tracker::object_position_topic, queueSize_ );

  // Moving edge sites_ publisher.
  movingEdgeSitesPublisher_ = this->create_publisher<visp_tracker::msg::MovingEdgeSites>(visp_tracker::moving_edge_sites_topic, queueSize_ );

  // Klt_points_ publisher.
  kltPointsPublisher_ = this->create_publisher<visp_tracker::msg::KltPoints>(visp_tracker::klt_points_topic, queueSize_ );

  // Camera subscriber.
  cameraSubscriber_ =
      imageTransport_.subscribeCamera
      (rectifiedImageTopic_, queueSize_,
       bindImageCallback(image_, header2_, info_));

  // Object position hint subscriber.
  //typedef boost::function<
   //   void (const geometry_msgs::msg::TransformStampedConstPtr&)>
   //   objectPositionHintCallback_t;
  //objectPositionHintCallback_t callback =
   //   boost::bind (&Tracker::objectPositionHintCallback, this, _1);
  //objectPositionHintSubscriber_ =
  //    nodeHandle_.subscribe<geometry_msgs::msg::TransformStamped>
  //    ("object_position_hint", queueSize_, callback);
  objectPositionHintSubscriber_ = this->create_subscription<geometry_msgs::msg::TransformStamped>( "object_position_hint", queueSize_, std::bind(&Tracker::objectPositionHintCallback, this, std::placeholders::_1));

  // Parameter initialization
  if(trackerType_=="mbt+klt"){ // Hybrid Tracker reconfigure
    auto angle_appear_desc = rcl_interfaces::msg::ParameterDescriptor();
    angle_appear_desc.set__name( "angle_appear" ).set__type(rclcpp::ParameterType::PARAMETER_DOUBLE).set__description("Maximal angle value to consider an appearing face");
    
    auto angle_appear_range = rcl_interfaces::msg::FloatingPointRange();
    angle_appear_range.set__from_value(0.0).set__to_value(90.0);
    
    rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1 > valid_range;
    valid_range.push_back( angle_appear_range );
    angle_appear_desc.set__floating_point_range( valid_range );

    this->declare_parameter<double>("angle_appear", 65.0, angle_appear_desc);
    
#if 0
    this->declare_parameter<double>("angle_disappear", "");
    this->declare_parameter<int>("mask_size", "");
    this->declare_parameter<int>("range", "");
    this->declare_parameter<double>("threshold", "");
    this->declare_parameter<double>("mu1", "");
    this->declare_parameter<double>("mu2", "");
    this->declare_parameter<double>("sample_step", "");
    this->declare_parameter<int>("strip", "");
    this->declare_parameter<double>("first_threshold", "");
    this->declare_parameter<int>("mask_border", "");
    this->declare_parameter<int>("max_features", "");
    this->declare_parameter<int>("window_size", "");
    this->declare_parameter<double>("quality", "");
    this->declare_parameter<double>("min_distance", "");
    this->declare_parameter<double>("harris", "");
    this->declare_parameter<int>("size_block", "");
    this->declare_parameter<int>("pyramid_lvl", "");

    rclcpp::Parameter angle_appear_param = this->get_parameter("angle_appear");
    rclcpp::Parameter angle_disappear_param = this->get_parameter("angle_disappear");
    rclcpp::Parameter mask_size_param = this->get_parameter("mask_size");
    rclcpp::Parameter range_param = this->get_parameter("range");
    rclcpp::Parameter threshold_param = this->get_parameter("threshold");
    rclcpp::Parameter mu1_param = this->get_parameter("mu1");
    rclcpp::Parameter mu2_param = this->get_parameter("mu2");
    rclcpp::Parameter sample_step_param = this->get_parameter("sample_step");
    rclcpp::Parameter strip_param = this->get_parameter("strip");
    rclcpp::Parameter first_threshold_param = this->get_parameter("first_threshold");
    rclcpp::Parameter mask_border_param = this->get_parameter("mask_border");
    rclcpp::Parameter max_features_param = this->get_parameter("max_features");
    rclcpp::Parameter window_size_param = this->get_parameter("window_size");
    rclcpp::Parameter quality_param = this->get_parameter("quality");
    rclcpp::Parameter min_distance_param = this->get_parameter("min_distance");
    rclcpp::Parameter harris_param = this->get_parameter("harris");
    rclcpp::Parameter size_block_param = this->get_parameter("size_block");
    rclcpp::Parameter pyramid_lvl_param = this->get_parameter("pyramid_lvl");

    angle_appear_ = angle_appear_param.as_double();
    angle_disappear_ = angle_disappear_param.as_double();
    mask_size_ = mask_size_param.as_int();
    range_ = range_param.as_int();
    threshold_ = threshold_param.as_double();
    mu1_ = mu1_param.as_double();
    mu2_ = mu2_param.as_double();
    sample_step_ = sample_step.as_double();
    strip_ = strip_param.as_int();
    first_threshold_ = first_threshold_param.as_double();
    mask_border_ = mask_border_param.as_int();
    max_features_ = max_features_param.as_int();
    window_size_ = window_size_param.as_int();
    quality_ = quality_param.as_double();
    min_distance_ = min_distance_param.as_double();
    harris_ = harris_param.as_double();
    size_block_ = size_block_param.as_int();
    pyramid_lvl_ = pyramid_lvl_param.as_int();
#endif
  }
  else if(trackerType_=="mbt"){ // Edge Tracker reconfigure
#if 0
    this->declare_parameter<double>("angle_appear", "");
    this->declare_parameter<double>("angle_disappear", "");
    this->declare_parameter<int>("mask_size", "");
    this->declare_parameter<int>("range", "");
    this->declare_parameter<double>("threshold", "");
    this->declare_parameter<double>("mu1", "");
    this->declare_parameter<double>("mu2", "");
    this->declare_parameter<double>("sample_step", "");
    this->declare_parameter<int>("strip", "");
    this->declare_parameter<double>("first_threshold", "");

    rclcpp::Parameter angle_appear_param = this->get_parameter("angle_appear");
    rclcpp::Parameter angle_disappear_param = this->get_parameter("angle_disappear");
    rclcpp::Parameter mask_size_param = this->get_parameter("mask_size");
    rclcpp::Parameter range_param = this->get_parameter("range");
    rclcpp::Parameter threshold_param = this->get_parameter("threshold");
    rclcpp::Parameter mu1_param = this->get_parameter("mu1");
    rclcpp::Parameter mu2_param = this->get_parameter("mu2");
    rclcpp::Parameter sample_step_param = this->get_parameter("sample_step");
    rclcpp::Parameter strip_param = this->get_parameter("strip");
    rclcpp::Parameter first_threshold_param = this->get_parameter("first_threshold");

    angle_appear_ = angle_appear_param.as_double();
    angle_disappear_ = angle_disappear_param.as_double();
    mask_size_ = mask_size_param.as_int();
    range_ = range_param.as_int();
    threshold_ = threshold_param.as_double();
    mu1_ = mu1_param.as_double();
    mu2_ = mu2_param.as_double();
    sample_step_ = sample_step.as_double();
    strip_ = strip_param.as_int();
    first_threshold_ = first_threshold_param.as_double();
#endif
  }
  else{ // KLT Tracker reconfigure
#if 0
    this->declare_parameter<double>("angle_appear", "");
    this->declare_parameter<double>("angle_disappear", "");
    this->declare_parameter<int>("mask_border", "");
    this->declare_parameter<int>("max_features", "");
    this->declare_parameter<int>("window_size", "");
    this->declare_parameter<double>("quality", "");
    this->declare_parameter<double>("min_distance", "");
    this->declare_parameter<double>("harris", "");
    this->declare_parameter<int>("size_block", "");
    this->declare_parameter<int>("pyramid_lvl", "");
    
    rclcpp::Parameter angle_appear_param = this->get_parameter("angle_appear");
    rclcpp::Parameter angle_disappear_param = this->get_parameter("angle_disappear");
    rclcpp::Parameter mask_border_param = this->get_parameter("mask_border");
    rclcpp::Parameter max_features_param = this->get_parameter("max_features");
    rclcpp::Parameter window_size_param = this->get_parameter("window_size");
    rclcpp::Parameter quality_param = this->get_parameter("quality");
    rclcpp::Parameter min_distance_param = this->get_parameter("min_distance");
    rclcpp::Parameter harris_param = this->get_parameter("harris");
    rclcpp::Parameter size_block_param = this->get_parameter("size_block");
    rclcpp::Parameter pyramid_lvl_param = this->get_parameter("pyramid_lvl");
    
    angle_appear_ = angle_appear_param.as_double();
    angle_disappear_ = angle_disappear_param.as_double();
    mask_border_ = mask_border_param.as_int();
    max_features_ = max_features_param.as_int();
    window_size_ = window_size_param.as_int();
    quality_ = quality_param.as_double();
    min_distance_ = min_distance_param.as_double();
    harris_ = harris_param.as_double();
    size_block_ = size_block_param.as_int();
    pyramid_lvl_ = pyramid_lvl_param.as_int();
#endif
  }
        
  // Dynamic reconfigure ROS2 replacement.
  callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&Tracker::parametersCallback, this, std::placeholders::_1));

  // Wait for the image to be initialized.
  waitForImage();
  if (this->exiting())
    return;
  if (!image_.getWidth() || !image_.getHeight())
    throw std::runtime_error("failed to retrieve image");

  // Tracker initialization.
  initializeVpCameraFromCameraInfo(cameraParameters_, info_);

  // Double check camera parameters.
  if (cameraParameters_.get_px () == 0.
      || cameraParameters_.get_px () == 1.
      || cameraParameters_.get_py () == 0.
      || cameraParameters_.get_py () == 1.
      || cameraParameters_.get_u0 () == 0.
      || cameraParameters_.get_u0 () == 1.
      || cameraParameters_.get_v0 () == 0.
      || cameraParameters_.get_v0 () == 1.)
    RCLCPP_WARN (rclcpp::get_logger("rclcpp"), "Dubious camera parameters detected.\n"
              "\n"
              "It seems that the matrix P from your camera\n"
              "calibration topics is wrong.\n"
              "The tracker will continue anyway, but you\n"
              "should double check your calibration data,\n"
              "especially if the model re-projection fails.\n"
              "\n"
              "This warning is triggered is px, py, u0 or v0\n"
              "is set to 0. or 1. (exactly).");

  tracker_.setCameraParameters(cameraParameters_);
  tracker_.setDisplayFeatures(false);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), cameraParameters_);

  initService_ = this->create_service<visp_tracker::srv::Init>(visp_tracker::init_service, std::bind(&Tracker::initCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

}

Tracker::~Tracker()
{
  #if 0
  if(reconfigureSrv_ != NULL)
    delete reconfigureSrv_;

  if(reconfigureKltSrv_ != NULL)
    delete reconfigureKltSrv_;

  if(reconfigureEdgeSrv_ != NULL)
    delete reconfigureEdgeSrv_;
  #endif
}

void Tracker::spin()
{
  rclcpp::Rate loopRateTracking(100);
  tf2::Transform transform;
  std_msgs::msg::Header lastHeader;

  while (!exiting())
  {
    // When a camera sequence is played several times,
    // the seq id will decrease, in this case we want to
    // continue the tracking.
    //if (header_->seq < lastHeader.seq)
    //  lastTrackedImage_ = 0;

   // if (lastTrackedImage_ < header_->seq)
    {
      //lastTrackedImage_ = header_->seq;

      // If we can estimate the camera displacement using tf,
      // we update the cMo to compensate for robot motion.
      if (compensateRobotMotion_)
        try
      {
        geometry_msgs::msg::TransformStamped stampedTransform;
        stampedTransform = tfBuffer_->lookupTransform
            (worldFrameId_, header_.frame_id,
            tf2::TimePointZero
            );
//            (header_.frame_id, // camera frame name
//             header_.stamp,    // current image time
//             header_.frame_id, // camera frame name
//             lastHeader.stamp, // last processed image time
//             worldFrameId_,    // frame attached to the environment
//             stampedTransform
//             );
        vpHomogeneousMatrix newMold;
        transformToVpHomogeneousMatrix (newMold, stampedTransform);
        cMo_ = newMold * cMo_;

        mutex_.lock();
        tracker_.setPose(image_, cMo_);
        mutex_.unlock();
      }
      catch(tf2::TransformException& e)
      {
        mutex_.unlock();
      }

      // If we are lost but an estimation of the object position
      // is provided, use it to try to reinitialize the system.
      if (state_ == LOST)
      {
        // If the last received message is recent enough,
        // use it otherwise do nothing.
        if (this->get_clock()->now() - objectPositionHint_->header.stamp
            < rclcpp::Duration (1,0))
          transformToVpHomogeneousMatrix
              (cMo_, objectPositionHint_->transform);

        mutex_.lock();
        tracker_.setPose(image_, cMo_);
        mutex_.unlock();
      }

      // We try to track the image even if we are lost,
      // in the case the tracker recovers...
      if (state_ == TRACKING || state_ == LOST)
        try
      {
        mutex_.lock();
        // tracker_->setPose(image_, cMo_); // Removed as it is not necessary when the pose is not modified from outside.
        tracker_.track(image_);
        tracker_.getPose(cMo_);
        mutex_.unlock();
      }
      catch(...)
      {
        rclcpp::Clock clock;
        mutex_.unlock();
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("rclcpp"), clock, 10, "tracking lost");
        state_ = LOST;
      }

      // Publish the tracking result.
      if (state_ == TRACKING)
      {
        std::shared_ptr<geometry_msgs::msg::Transform> transformMsg = std::make_shared<geometry_msgs::msg::Transform>();
        vpHomogeneousMatrixToTransform(transformMsg, cMo_);

        // Publish position.
        if (transformationPublisher_->get_subscription_count() > 0)
        {
          geometry_msgs::msg::TransformStamped objectPosition;
          objectPosition.header = header_;
          objectPosition.child_frame_id = childFrameId_;
          objectPosition.transform = *transformMsg;
          transformationPublisher_->publish(objectPosition);
        }

        // Publish result.
        if (resultPublisher_->get_subscription_count	() > 0)
        {
          std::unique_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> result =
              std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();

          result->header = header_;
          result->pose.pose.position.x =
              transformMsg->translation.x;
          result->pose.pose.position.y =
              transformMsg->translation.y;
          result->pose.pose.position.z =
              transformMsg->translation.z;

          result->pose.pose.orientation.x =
              transformMsg->rotation.x;
          result->pose.pose.orientation.y =
              transformMsg->rotation.y;
          result->pose.pose.orientation.z =
              transformMsg->rotation.z;
          result->pose.pose.orientation.w =
              transformMsg->rotation.w;
          const vpMatrix& covariance =
              tracker_.getCovarianceMatrix();
          for (unsigned i = 0; i < covariance.getRows(); ++i)
            for (unsigned j = 0; j < covariance.getCols(); ++j)
            {
              unsigned idx = i * covariance.getCols() + j;
              if (idx >= 36)
                continue;
              result->pose.covariance[idx] = covariance[i][j];
            }
          resultPublisher_->publish(*result);
        }

        // Publish moving edge sites.
        if (movingEdgeSitesPublisher_->get_subscription_count	() > 0)
        {
          std::shared_ptr<visp_tracker::msg::MovingEdgeSites> sites =
              std::make_shared<visp_tracker::msg::MovingEdgeSites>();
          updateMovingEdgeSites(sites);
          sites->header = header_;
          movingEdgeSitesPublisher_->publish(*sites);
        }
        // Publish KLT points.
        if (kltPointsPublisher_->get_subscription_count	() > 0)
        {
          std::shared_ptr<visp_tracker::msg::KltPoints> klt =
              std::make_shared<visp_tracker::msg::KltPoints>();
          updateKltPoints(klt);
          klt->header = header_;
          kltPointsPublisher_->publish(*klt);
        }

        // Publish to tf.
        transform.setOrigin
            (tf2::Vector3(transformMsg->translation.x,
                         transformMsg->translation.y,
                         transformMsg->translation.z));
        transform.setRotation
            (tf2::Quaternion
             (transformMsg->rotation.x,
              transformMsg->rotation.y,
              transformMsg->rotation.z,
              transformMsg->rotation.w));
        geometry_msgs::msg::TransformStamped transform_to_broadcast;
        
        transform_to_broadcast.header.stamp = header_.stamp;
        transform_to_broadcast.header.frame_id = header_.frame_id;
        transform_to_broadcast.child_frame_id = childFrameId_;
        transform_to_broadcast.transform = *transformMsg;
        
        transformBroadcaster_->sendTransform(transform_to_broadcast);
      }
    }

    lastHeader = header_;
    spinOnce();
    loopRateTracking.sleep();

  }
}

// Make sure that we have an image *and* associated calibration
// data.
void
Tracker::waitForImage()
{
  rclcpp::Rate loop_rate(10);
  while (!exiting()
         && (!image_.getWidth() || !image_.getHeight())
         && (!info_ || info_->k[0] == 0.))
  {
    //RCLCPP_INFO_THROTTLE(rclcpp::get_logger("rclcpp"), 1, "waiting for a rectified image...");
    spinOnce();
    loop_rate.sleep();
  }
}

void
Tracker::objectPositionHintCallback
(const std::shared_ptr<geometry_msgs::msg::TransformStamped> transform)
{
  objectPositionHint_ = transform;
}

    
rcl_interfaces::msg::SetParametersResult Tracker::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
      
  for (const auto &aParameter: parameters)
  {
    std::string strParameterName = aParameter.get_name();
    rclcpp::ParameterType strParameterType = aParameter.get_type();

    if( strParameterName == "angle_appear" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        angle_appear_ = aParameter.as_double();
      }
    }
    else if( strParameterName == "angle_disappear" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        angle_disappear_ = aParameter.as_double();
      }
    }
    else if( strParameterName == "mask_size" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_INTEGER) {
        mask_size_ = aParameter.as_int();
      }
    }
    else if( strParameterName == "range" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_INTEGER) {
        range_ = aParameter.as_int();
      }
    }
    else if( strParameterName == "threshold" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        threshold_ = aParameter.as_double();
      }
    }
    else if( strParameterName == "mu1" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        mu1_ = aParameter.as_double();
      }
    }
    else if( strParameterName == "mu2" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        mu2_ = aParameter.as_double();
      }
    }
    else if( strParameterName == "sample_step" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        sample_step_ = aParameter.as_double();
      }
    }
    else if( strParameterName == "strip" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_INTEGER) {
        strip_ = aParameter.as_int();
      }
    }
    else if( strParameterName == "first_threshold" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        first_threshold_ = aParameter.as_double();
      }
    }
    else if( strParameterName == "mask_border" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_INTEGER) {
        mask_border_ = aParameter.as_int();
      }
    }
    else if( strParameterName == "max_features" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_INTEGER) {
        max_features_ = aParameter.as_int();
      }
    }
    else if( strParameterName == "window_size" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_INTEGER) {
        window_size_ = aParameter.as_int();
      }
    }
    else if( strParameterName == "quality" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        quality_ = aParameter.as_double();
      }
    }
    else if( strParameterName == "min_distance" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        min_distance_ = aParameter.as_double();
      }
    }
    else if( strParameterName == "harris" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        harris_ = aParameter.as_double();
      }
    }
    else if( strParameterName == "size_block" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_INTEGER) {
        size_block_ = aParameter.as_int();
      }
    }
    else if( strParameterName == "pyramid_lvl" ) {
      if( strParameterType == rclcpp::ParameterType::PARAMETER_INTEGER) {
        pyramid_lvl_ = aParameter.as_int();
      }
    }
  }
  
  if(trackerType_=="mbt+klt"){ // Hybrid Tracker reconfigure
    std::shared_ptr<ModelBasedSettingsConfig> config = std::make_shared<ModelBasedSettingsConfig>( this );
    
    reconfigureCallbackAndInitViewer( 
        this, std::ref(tracker_),
        std::ref(image_), std::ref(movingEdge_), std::ref(kltTracker_),
        std::ref(mutex_), config, 1 );
  }
  else if(trackerType_=="mbt"){ // Edge Tracker reconfigure
    std::shared_ptr<ModelBasedSettingsEdgeConfig> config = std::make_shared<ModelBasedSettingsEdgeConfig>( this );
    
    reconfigureEdgeCallbackAndInitViewer( 
        this, std::ref(tracker_),
        std::ref(image_), std::ref(movingEdge_),
        std::ref(mutex_), config, 1 );
  }  
  else{ // KLT Tracker reconfigure
    std::shared_ptr<ModelBasedSettingsKltConfig> config = std::make_shared<ModelBasedSettingsKltConfig>( this );
    
    reconfigureKltCallbackAndInitViewer(
        this, std::ref(tracker_),
        std::ref(image_), std::ref(kltTracker_),
        std::ref(mutex_), config, 1 );
  }
}

bool
Tracker::create_parameter( const visp_tracker::parameter_info &param ) {

    if( param.type == rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1 > dummy_range;

      rcl_interfaces::msg::IntegerRange integer_range = rcl_interfaces::build<rcl_interfaces::msg::IntegerRange>()
              .from_value( param.start_val.n )
              .to_value( param.end_val.n )
              .step( 0 );
              
      rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1 > valid_range;
      valid_range.push_back( integer_range );
      
      rcl_interfaces::msg::ParameterDescriptor parameter_desc = rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
              .name( param.name )
              .type( rclcpp::ParameterType::PARAMETER_INTEGER )
              .description(param.description)
              .additional_constraints( "" )
              .read_only( false )
              .dynamic_typing( false )
              .floating_point_range( dummy_range )
              .integer_range( valid_range );
              
      this->declare_parameter<int>(param.name, param.default_val.d, parameter_desc);
    }
    else if( param.type == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      rcl_interfaces::msg::FloatingPointRange floating_point_range = rcl_interfaces::build<rcl_interfaces::msg::FloatingPointRange>()
              .from_value( param.start_val.d )
              .to_value( param.end_val.d )
              .step( 0.0 );
              
      rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1 > valid_range;
      valid_range.push_back( floating_point_range );
      
      rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1 > dummy_range;

      rcl_interfaces::msg::ParameterDescriptor parameter_desc = rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
              .name( param.name )
              .type( rclcpp::ParameterType::PARAMETER_DOUBLE )
              .description(param.description)
              .additional_constraints( "" )
              .read_only( false )
              .dynamic_typing( false )
              .floating_point_range( valid_range )
              .integer_range( dummy_range );
              
      this->declare_parameter<double>(param.name, param.default_val.d, parameter_desc);
    }
    else
    {
      RCLCPP_FATAL (rclcpp::get_logger("rclcpp"), "Parameter type is not handled, currently only Integer and Double supported" );
      return false;
    }
    
    return true;
#if 0

      rcl_interfaces::msg::FloatingPointRange angle_appear_range = rcl_interfaces::build<rcl_interfaces::msg::FloatingPointRange>()
              .from_value( param.start_val.d )
              .to_value( param.end_val.d )
              .step( 0.0 );
              
      rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::FloatingPointRange, 1 > valid_range;
      valid_range.push_back( angle_appear_range );
      
      rosidl_runtime_cpp::BoundedVector<rcl_interfaces::msg::IntegerRange, 1 > dummy_range;

      rcl_interfaces::msg::ParameterDescriptor angle_appear_desc = rcl_interfaces::build<rcl_interfaces::msg::ParameterDescriptor>()
              .name( param.name )
              .type( rclcpp::ParameterType::PARAMETER_DOUBLE )
              .description(param.description)
              .additional_constraints( "" )
              .read_only( false )
              .dynamic_typing( false )
              .floating_point_range( valid_range )
              .integer_range( dummy_range );
              
      this->declare_parameter<double>(param.name, param.default_val.d, param.description);

#endif
}

} // end of namespace visp_tracker.

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
//RCLCPP_COMPONENTS_REGISTER_NODE(visp_tracker::Tracker)
