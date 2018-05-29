/* Body Tracker Node using Nuitrack library

   Publish data messages:
   
   1. body_tracking_position_pub_ custom message:  <body_tracker_msgs::BodyTracker>
   Includes:
   2D position of person relative to head camera; allows for fast, smooth tracking
     Joint.proj:  position in normalized projective coordinates
     (x, y from 0.0 to 1.0, z is real)
     Astra Mini FOV: 60 horz, 49.5 vert (degrees)

   3D position of the neck joint in relation to robot (using TF)
     Joint.real: position in real world coordinates
     Useful for tracking person in 3D
   
   2. body_tracking_skeleton_pub_ custom message: <body_tracker_msgs::Skeleton>
   Includes:
   Everyting in BodyTracker message above, plus 3D position of upper body. 
     joints in relation to robot (using TF)
     Joint.real: position in real world coordinates

   3. marker_pub_  message: <visualization_msgs::Marker>
   Publishes 3d markers for selected joints.  Visible as markers in RVIZ

*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include "ros/console.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>  // setprecision

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include <visualization_msgs/Marker.h>
#include "body_tracker_msgs/BodyTracker.h"  // Publish custom message
#include "body_tracker_msgs/Skeleton.h"  // Publish custom message

// If Camera mounted on Pan/Tilt head
//#include "sensor_msgs/JointState.h"
#include "dynamixel_msgs/JointState.h"

//For Nuitrack SDK
#include "nuitrack/Nuitrack.h"
#define KEY_JOINT_TO_TRACK    JOINT_LEFT_COLLAR // JOINT_TORSO // JOINT_NECK

namespace nuitrack_body_tracker
{
  using namespace tdv::nuitrack;

  class nuitrack_body_tracker_node 
  {
  public:

    nuitrack_body_tracker_node(std::string name) :
      _name(name)
    {
      ROS_INFO("%s: Starting...", _name.c_str());
      bool initialized = false;

      ros::NodeHandle nodeHandle("~");
      nodeHandle.param<std::string>("camera_depth_frame",camera_depth_frame_,"camera_depth_frame");

      // Publishers and Subscribers

      // Publish tracked person in 2D and 3D
      // 2D: x,y in camera frame.   3D: x,y,z in world coordinates
      body_tracking_position_pub_ = nh_.advertise<body_tracker_msgs::BodyTracker>
        ("body_tracker/position", 1); 

      // Publish tracked person upper body skeleton for advanced uses
      body_tracking_skeleton_pub_ = nh_.advertise<body_tracker_msgs::Skeleton>
        ("body_tracker/skeleton", 1);

      // Publish markers to show where robot thinks person is in RViz
      marker_pub_ = nh_.advertise<visualization_msgs::Marker>
        ("body_tracker/marker", 1);


      // OPTIONAL: Publish tracked person as a 2DPose message, for camera servo tracking
      // NOTE: we send person ID in the "theta" slot
      // body_tracking_pose2d_pub_ = nh_.advertise<geometry_msgs::Pose2D>
      //  ("body_tracker/pose2d", 1); 

      // OPTIONAL: Publish tracked person as a basic 3D Pose message
      // body_tracking_pose3d_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
      //  ("body_tracker/pose", 1); 

      // OPTIONAL: Publish gestures to indicate active user to track
      // TODO fix this KLUDGE - using Pose2d, need custom message!
      // body_tracking_gesture_pub_ = nh_.advertise<geometry_msgs::Pose2D>
      //  ("body_tracker/gesture", 1); 


      // Subscribe to servo messages, so provided data is as "real time" as possible for smooth tracking
      //servo_pan_sub_ = nh_.subscribe("/head_pan_controller/state", 1, &nuitrack_body_tracker_node::servoPanCallback, this);

    }

    ~nuitrack_body_tracker_node()
    {
      ROS_INFO("nuitrack_body_tracker_node shutting down");
    }

    ///////////////////////////////////////////////////////////////////////////
    // Nuitrack callbacks
    // Copy depth frame data, received from Nuitrack, to texture to visualize
    void onNewDepthFrame(DepthFrame::Ptr frame)
    {
	    // std::cout << "Nuitrack: onNewDepthFrame callback" << std::endl;
    }

    /* Not used (yet?)
    void onNewRGBFrame(RGBFrame::Ptr frame)
    {
	    std::cout << "Nuitrack: onNewRGBFrame callback" << std::endl;
    }
    */

    void onUserUpdate(tdv::nuitrack::UserFrame::Ptr frame)
    {
	    // std::cout << "Nuitrack: onUserUpdate callback" << std::endl;
    }

    void onSkeletonUpdate(SkeletonData::Ptr userSkeletons)
    {
	    // std::cout << "Nuitrack: onSkeletonUpdate callback" << std::endl;
	    auto skeletons = userSkeletons->getSkeletons();
	    for (auto skeleton: skeletons)
	    {
	      // std::cout << "Nuitrack: Skeleton.id = " << skeleton.id << std::endl;

        // Use KEY_JOINT_TO_TRACK to determine if we have a good lock on the person
        float tracking_confidence = skeleton.joints[KEY_JOINT_TO_TRACK].confidence;
        if (tracking_confidence < 0.15)
        {
  	      std::cout << "Nuitrack: ID " << skeleton.id << " Low Confidence (" 
            << tracking_confidence << "), skipping"  << std::endl;
          continue;  // assume none of the joints are valid 
        }


        // Fill in message data from Nuitracker SDK data
        // camera z,x,y coordinates are mapped to ROS x,y,z coordinates 
        // All values are relative to camera position in meters (ie, in camera's TF frame)
        // ROS x = camera z - distance to person
        // ROS y = camera x - side to side
        // ROS z = camera y - vertical height, *relative to camera position*

        ///////////////////////////////////////////////////////////////
        // Position data in 2D and 3D for tracking people
        body_tracker_msgs::BodyTracker_ <body_tracker_msgs::BodyTracker> position_data;

        // position_data.frame_id = camera_depth_frame_;
        position_data.body_id = skeleton.id;
        position_data.tracking_status = 0; // TODO
        position_data.gesture = -1; // No gesture

        if(skeleton.id != last_id_)
        {
          ROS_INFO("%s: detected person ID %d", _name.c_str(), skeleton.id);
          last_id_ = skeleton.id;
        }

        ///////////////////////////////////////////////////////////////
        // 2D position for camera servo tracking
        const float ASTRA_MINI_FOV_X = -1.047200; // (60 degrees horizontal)
        const float ASTRA_MINI_FOV_Y = -0.863938; // (49.5 degrees vertical)

        // Convert projection to radians
        // proj is 0.0 (left) --> 1.0 (right)
        geometry_msgs::Pose2D track2d;
        track2d.x = (skeleton.joints[KEY_JOINT_TO_TRACK].proj.x - 0.5) * ASTRA_MINI_FOV_X;
        track2d.y = (skeleton.joints[KEY_JOINT_TO_TRACK].proj.y - 0.5) * ASTRA_MINI_FOV_Y;
        track2d.theta = (float)skeleton.id;

        position_data.position2d.x = 
          (skeleton.joints[KEY_JOINT_TO_TRACK].proj.x - 0.5) * ASTRA_MINI_FOV_X;
        position_data.position2d.y = 
          (skeleton.joints[KEY_JOINT_TO_TRACK].proj.y - 0.5) * ASTRA_MINI_FOV_Y;
        position_data.position2d.z = skeleton.joints[KEY_JOINT_TO_TRACK].proj.z / 1000.0;

        
        std::cout << std::setprecision(4) << std::setw(7) 
          << "Nuitrack: " << "2D Tracking"  
          << " x: " << track2d.x 
          << " y: " << track2d.y
          << " ID: " << track2d.theta
          << std::endl;
        

        // Publish pose2D
        // body_tracking_pose2d_pub_.publish(track2d); 



        ///////////////////////////////////////////////////////////////
        // Basic Pose for person location tracking
        // This is for compatability with other trackers, which use PoseStamped messages
/***
        geometry_msgs::PoseStamped body_pose;
        body_pose.header.frame_id = camera_depth_frame_;
        body_pose.header.stamp = ros::Time::now();

        body_pose.pose.position.x = skeleton.joints[KEY_JOINT_TO_TRACK].real.z / 1000.0;
        body_pose.pose.position.y = skeleton.joints[KEY_JOINT_TO_TRACK].real.x / -1000.0;
        body_pose.pose.position.z = skeleton.joints[KEY_JOINT_TO_TRACK].real.y / 1000.0;

***/
        /*
        std::cout << std::setprecision(4) << std::setw(7) 
          << "Nuitrack: " << "KEY_JOINT_TO_TRACK"  
          << " x: " << (float)body_pose.pose.position.x 
          << " y: " << body_pose.pose.position.y
          << " z: " << body_pose.pose.position.z
          << "  Confidence: " << skeleton.joints[KEY_JOINT_TO_TRACK].confidence
          << std::endl;
        */

        // Publish pose 
        // body_tracking_pose3d_pub_.publish(body_pose); // this is position only!



        ///////////////////////////////////////////////////////////////
        // Skeleton Data for publishing more detail
        body_tracker_msgs::Skeleton_ <body_tracker_msgs::Skeleton> skeleton_data;

        // skeleton_data.frame_id = camera_depth_frame_;
        skeleton_data.body_id = skeleton.id;
        skeleton_data.tracking_status = 0; // TODO

        //skeleton_data.centerOfMass.x = 0.0;
        //skeleton_data.centerOfMass.y = 0.0;
        //skeleton_data.centerOfMass.z = 0.0;

        // *** POSITION 3D ***
        position_data.position3d.x = skeleton.joints[KEY_JOINT_TO_TRACK].real.z / 1000.0;
        position_data.position3d.y = skeleton.joints[KEY_JOINT_TO_TRACK].real.x / 1000.0;
        position_data.position3d.z = skeleton.joints[KEY_JOINT_TO_TRACK].real.y / 1000.0;
 
       
        skeleton_data.joint_position_head.x = skeleton.joints[JOINT_HEAD].real.z / 1000.0;
        skeleton_data.joint_position_head.y = skeleton.joints[JOINT_HEAD].real.x / 1000.0;
        skeleton_data.joint_position_head.z = skeleton.joints[JOINT_HEAD].real.y / 1000.0;

        skeleton_data.joint_position_neck.x = skeleton.joints[JOINT_NECK].real.z / 1000.0;
        skeleton_data.joint_position_neck.y = skeleton.joints[JOINT_NECK].real.x / 1000.0;
        skeleton_data.joint_position_neck.z = skeleton.joints[JOINT_NECK].real.y / 1000.0;

        skeleton_data.joint_position_spine_top.x = skeleton.joints[JOINT_TORSO].real.z / 1000.0;
        skeleton_data.joint_position_spine_top.y = skeleton.joints[JOINT_TORSO].real.x / 1000.0;
        skeleton_data.joint_position_spine_top.z = skeleton.joints[JOINT_TORSO].real.y / 1000.0;

        skeleton_data.joint_position_spine_mid.x = skeleton.joints[JOINT_WAIST].real.z / 1000.0;
        skeleton_data.joint_position_spine_mid.y = skeleton.joints[JOINT_WAIST].real.x / 1000.0;
        skeleton_data.joint_position_spine_mid.z = skeleton.joints[JOINT_WAIST].real.y / 1000.0;

        skeleton_data.joint_position_spine_bottom.x = 0.0;
        skeleton_data.joint_position_spine_bottom.y = 0.0;
        skeleton_data.joint_position_spine_bottom.z = 0.0;

        skeleton_data.joint_position_left_shoulder.x = skeleton.joints[JOINT_LEFT_SHOULDER].real.z / 1000.0;
        skeleton_data.joint_position_left_shoulder.y = skeleton.joints[JOINT_LEFT_SHOULDER].real.x / 1000.0;
        skeleton_data.joint_position_left_shoulder.z = skeleton.joints[JOINT_LEFT_SHOULDER].real.y / 1000.0;

        skeleton_data.joint_position_left_elbow.x = skeleton.joints[JOINT_LEFT_ELBOW].real.z / 1000.0;
        skeleton_data.joint_position_left_elbow.y = skeleton.joints[JOINT_LEFT_ELBOW].real.x / 1000.0;
        skeleton_data.joint_position_left_elbow.z = skeleton.joints[JOINT_LEFT_ELBOW].real.y / 1000.0;

        skeleton_data.joint_position_left_hand.x = skeleton.joints[JOINT_LEFT_HAND].real.z / 1000.0;
        skeleton_data.joint_position_left_hand.y = skeleton.joints[JOINT_LEFT_HAND].real.x / 1000.0;
        skeleton_data.joint_position_left_hand.z = skeleton.joints[JOINT_LEFT_HAND].real.y / 1000.0;

        skeleton_data.joint_position_right_shoulder.x = skeleton.joints[JOINT_RIGHT_SHOULDER].real.z / 1000.0;
        skeleton_data.joint_position_right_shoulder.y = skeleton.joints[JOINT_RIGHT_SHOULDER].real.x / 1000.0;
        skeleton_data.joint_position_right_shoulder.z = skeleton.joints[JOINT_RIGHT_SHOULDER].real.y / 1000.0;

        skeleton_data.joint_position_right_elbow.x = skeleton.joints[JOINT_RIGHT_ELBOW].real.z / 1000.0;
        skeleton_data.joint_position_right_elbow.y = skeleton.joints[JOINT_RIGHT_ELBOW].real.x / 1000.0;
        skeleton_data.joint_position_right_elbow.z = skeleton.joints[JOINT_RIGHT_ELBOW].real.y / 1000.0;

        skeleton_data.joint_position_right_hand.x = skeleton.joints[JOINT_RIGHT_HAND].real.z / 1000.0;
        skeleton_data.joint_position_right_hand.y = skeleton.joints[JOINT_RIGHT_HAND].real.x / 1000.0;
        skeleton_data.joint_position_right_hand.z = skeleton.joints[JOINT_RIGHT_HAND].real.y / 1000.0;

        // Hand:  open (0), grasping (1), waving (2)
        /* TODO - see which of these actually work
	      GESTURE_WAVING          = 0,
	      GESTURE_SWIPE_LEFT      = 1,
	      GESTURE_SWIPE_RIGHT     = 2,
	      GESTURE_SWIPE_UP        = 3,
	      GESTURE_SWIPE_DOWN      = 4,
	      GESTURE_PUSH            = 5,
        in MSG:  -1 = none, 0 = waving, 1 = right fist, 2 = left fist
        By experimentation:
          Gesture 0:  Wave
          Gesture 3:  Hand down (go)
          Gesture 4:  Hand up, in stopping motion (stop)

        */

        skeleton_data.gesture = -1; // No gesture
        position_data.gesture = -1; 
        for (int i = 0; i < userGestures_.size(); ++i)
        {
          if( (userGestures_[i].userId == skeleton.id) && // match the person being reported
              (userGestures_[i].type > -1) )              // Not already reported             
          {
            skeleton_data.gesture = userGestures_[i].type; // TODO - map Nuitrack to my MSG enum
            position_data.gesture = userGestures_[i].type; // TODO - map Nuitrack to my MSG enum
		        printf("Reporting Gesture %d for User %d\n", 
              userGestures_[i].type, userGestures_[i].userId);
            userGestures_[i].type = (tdv::nuitrack::GestureType)(-1); // clear so we don't report old gestures
          }

        }

        ////////////////////////////////////////////////////
        // Publish custom position and skeleton messages

        body_tracking_position_pub_.publish(position_data); // position data
        body_tracking_skeleton_pub_.publish(skeleton_data); // full skeleton data

        // Publish skeleton markers

        PublishMarker(  // show marker at KEY_JOINT_TO_TRACK location
          1, // ID
          position_data.position3d.x, 
          position_data.position3d.y, 
          position_data.position3d.z, 
          1.0, 0.0, 0.0 ); // r,g,b

        PublishMarker(
          3, // ID
          skeleton_data.joint_position_head.x,
          skeleton_data.joint_position_head.y,
          skeleton_data.joint_position_head.z,
          0.7, 0.0, 0.7 ); // r,g,b

        PublishMarker(
          4, // ID
          skeleton_data.joint_position_spine_top.x,
          skeleton_data.joint_position_spine_top.y,
          skeleton_data.joint_position_spine_top.z,
          0.0, 0.0, 1.0 ); // r,g,b

        PublishMarker(
          5, // ID
          skeleton_data.joint_position_spine_mid.x,
          skeleton_data.joint_position_spine_mid.y,
          skeleton_data.joint_position_spine_mid.z,
          0.0, 1.0, 0.0 ); // r,g,b

 	    }

    }


    void onHandUpdate(HandTrackerData::Ptr handData)
    {
	    // std::cout << "Nuitrack: onHandUpdate callback" << std::endl;
    }


    void onNewGesture(GestureData::Ptr gestureData)
    {
	    //std::cout << "Nuitrack: onNewGesture callback" << std::endl;

	    userGestures_ = gestureData->getGestures(); // Save for use in next skeleton frame
	    for (int i = 0; i < userGestures_.size(); ++i)
	    {
		    printf("onNewGesture: Gesture Recognized %d for User %d\n", userGestures_[i].type, userGestures_[i].userId);

        // This kludge uses Pose2D message to pass userID and gesture
        // Better to use the defined custom messages!
        /*
        geometry_msgs::Pose2D gesture;
        gesture.x = userGestures_[i].type;  // Passing gesture in "x"
        gesture.y = 0.0;
        gesture.theta = (float)userGestures_[i].userId; // Passing ID in "theta"

        body_tracking_gesture_pub_.publish(gesture); 
        */
	    }

    }


    void PublishMarker(int id, float x, float y, float z, 
                       float color_r, float color_g, float color_b)
    {
      // Display marker for RVIZ to show where robot thinks person is
      // For Markers info, see http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

      // ROS_INFO("DBG: PublishMarker called");
      //if( id != 1)
      // printf ("DBG PublishMarker called for %f, %f, %f\n", x,y,z);

      visualization_msgs::Marker marker;
      marker.header.frame_id = camera_depth_frame_;
      marker.header.stamp = ros::Time::now();
      marker.lifetime = ros::Duration(3.0); // seconds
      // Any marker sent with the same namespace and id will overwrite the old one
      marker.ns = _name;
      marker.id = id; // This must be id unique for each marker

      uint32_t shape = visualization_msgs::Marker::SPHERE;
      marker.type = shape;

      // Set the marker action.  Options are ADD, DELETE, and DELETEALL
      marker.action = visualization_msgs::Marker::ADD;
      marker.color.r = color_r;
      marker.color.g = color_g; 
      marker.color.b = color_b;
      marker.color.a = 1.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = 0.1; // size of marker in meters
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;  

      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;

      // ROS_INFO("DBG: Publishing Marker");
      marker_pub_.publish(marker);

    }

    /*
    void servoPanCallback(const dynamixel_msgs::JointState::ConstPtr& msg)
    {
      // Called on each pan servo update
      double current_pos = msg->current_pos;
      printf ("\n\nDBG PAN SERVO: %f\n\n", current_pos);

    }
    */

    // Publish 2D position of person, relative to camera
    // useful for direct control of servo pan/tilt for tracking
    // Example: publishJoint2D("JOINT_NECK", joints[JOINT_NECK]);

    void publishJoint2D(const char *name, const tdv::nuitrack::Joint& joint)
    {
      const float ASTRA_MINI_FOV_X =  1.047200; // (60 degrees horizontal)
      const float ASTRA_MINI_FOV_Y = -0.863938; // (49.5 degrees vertical)
      if (joint.confidence < 0.15)
      {
        return;  // ignore low confidence joints
      }

      // Convert projection to radians
      // proj is 0.0 (left) --> 1.0 (right)
      float radians_x = (joint.proj.x - 0.5) * ASTRA_MINI_FOV_X;
      float radians_y = (joint.proj.y - 0.5) * ASTRA_MINI_FOV_Y;
      std::cout << std::setprecision(4) << std::setw(7) 
        << "Nuitrack: " << name  
        << " x: " << joint.proj.x << " (" << radians_x << ")  y: " 
        << joint.proj.y << " (" << radians_y << ")" 
        // << "  Confidence: " << joint.confidence 
        << std::endl;

      // Future? Add in servo position to get absolute position relative to the robot body
      // This allows the subscriber to use these values directly for servo control
    }


    ///////////////////////////////////////////////////////////////////////////
    void Init(const std::string& config)
    {
	    // Initialize Nuitrack first, then create Nuitrack modules
      ROS_INFO("%s: Initializing...", _name.c_str());
	    // std::cout << "Nuitrack: Initializing..." << std::endl;

	    std::cout << 
      "\n============ IGNORE ERRORS THAT SAY 'Couldnt open device...' ===========\n" 
      << std::endl;

	    try
	    {
		    tdv::nuitrack::Nuitrack::init(config);
	    }
	    catch (const tdv::nuitrack::Exception& e)
	    {
		    std::cerr << 
        "Can not initialize Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
		    exit(EXIT_FAILURE);
	    }

	    // Create all required Nuitrack modules

	    std::cout << "Nuitrack: DepthSensor::create()" << std::endl;
	    depthSensor_ = tdv::nuitrack::DepthSensor::create();
	    // Bind to event new frame
	    depthSensor_->connectOnNewFrame(std::bind(
      &nuitrack_body_tracker_node::onNewDepthFrame, this, std::placeholders::_1));
	
      outputMode_ = depthSensor_->getOutputMode();
	    width_ = outputMode_.xres;
	    height_ = outputMode_.yres;
      last_id_ = -1;
      std::cout << "========= Nuitrack: GOT DEPTH SENSOR =========" << std::endl;
	    std::cout << "Nuitrack: Depth:  width = " << width_ << "  height = " << height_ << std::endl;

      /*  Color frame not currently used
	    std::cout << "Nuitrack: ColorSensor::create()" << std::endl;
	    colorSensor_ = tdv::nuitrack::ColorSensor::create();
	    // Bind to event new frame
	    colorSensor_->connectOnNewFrame(std::bind(
        &nuitrack_body_tracker_node::onNewRGBFrame, this, std::placeholders::_1));
	    */

	    std::cout << "Nuitrack: UserTracker::create()" << std::endl;
	    userTracker_ = tdv::nuitrack::UserTracker::create();
	    // Bind to event update user tracker
	    userTracker_->connectOnUpdate(std::bind(
        &nuitrack_body_tracker_node::onUserUpdate, this, std::placeholders::_1));
	
	    std::cout << "Nuitrack: SkeletonTracker::create()" << std::endl;
	    skeletonTracker_ = tdv::nuitrack::SkeletonTracker::create();
	    // Bind to event update skeleton tracker
	    skeletonTracker_->connectOnUpdate(std::bind(
        &nuitrack_body_tracker_node::onSkeletonUpdate, this, std::placeholders::_1));
	
	    std::cout << "Nuitrack: HandTracker::create()" << std::endl;
	    handTracker_ = tdv::nuitrack::HandTracker::create();
	    // Bind to event update Hand tracker
	    handTracker_->connectOnUpdate(std::bind(
        &nuitrack_body_tracker_node::onHandUpdate, this, std::placeholders::_1));
	
	    std::cout << "Nuitrack: GestureRecognizer::create()" << std::endl;
	    gestureRecognizer_ = tdv::nuitrack::GestureRecognizer::create();
	    gestureRecognizer_->connectOnNewGestures(std::bind(
        &nuitrack_body_tracker_node::onNewGesture, this, std::placeholders::_1));

      ROS_INFO("%s: Init complete.  Waiting for frames...", _name.c_str());

    }

    void Run()
    {
	    // Initialize Nuitrack first, then create Nuitrack modules
      ROS_INFO("%s: Running...", _name.c_str());
	    // std::cout << "Nuitrack: Running..." << std::endl;
      // Start Nuitrack
      try
      {
          tdv::nuitrack::Nuitrack::run();
      }
      catch (const tdv::nuitrack::Exception& e)
      {
          std::cerr << "Can not start Nuitrack (ExceptionType: " 
            << e.type() << ")" << std::endl;
          return;
      }

      ROS_INFO("%s: Waiting for person to be detected...", _name.c_str());

      // Run Loop
      ros::Rate r(30); // hz
      while (ros::ok())
      {
	        // std::cout << "Nuitrack: Looping..." << std::endl;

          try
          {
              // Wait for new person tracking data
              tdv::nuitrack::Nuitrack::waitUpdate(skeletonTracker_);
          }
          catch (tdv::nuitrack::LicenseNotAcquiredException& e)
          {
              std::cerr << "LicenseNotAcquired exception (ExceptionType: " 
                << e.type() << ")" << std::endl;
              break;
          }
          catch (const tdv::nuitrack::Exception& e)
          {
              std::cerr << "Nuitrack update failed (ExceptionType: " 
                << e.type() << ")" << std::endl;
          }

	        // std::cout << "Nuitrack: Sleeping..." << std::endl;
          ros::spinOnce();
          r.sleep();
      }

      // Release Nuitrack
      try
      {
          tdv::nuitrack::Nuitrack::release();
      }
      catch (const tdv::nuitrack::Exception& e)
      {
          std::cerr << "Nuitrack release failed (ExceptionType: " 
            << e.type() << ")" << std::endl;
      }

    } // Run()


  private:
    /////////////// DATA MEMBERS /////////////////////

    std::string _name;
    ros::NodeHandle nh_;
    std::string camera_depth_frame_;
	  int width_, height_;
    int last_id_;
    ros::Publisher body_tracking_position_pub_;
    ros::Publisher body_tracking_skeleton_pub_;
    ros::Publisher marker_pub_;
    //ros::Publisher body_tracking_pose2d_pub_;
    //ros::Publisher body_tracking_pose3d_pub_;
    //ros::Publisher body_tracking_gesture_pub_;
    //ros::Subscriber servo_pan_sub_;

	  tdv::nuitrack::OutputMode outputMode_;
	  std::vector<tdv::nuitrack::Gesture> userGestures_;
	  tdv::nuitrack::DepthSensor::Ptr depthSensor_;
	  tdv::nuitrack::ColorSensor::Ptr colorSensor_;
	  tdv::nuitrack::UserTracker::Ptr userTracker_;
	  tdv::nuitrack::SkeletonTracker::Ptr skeletonTracker_;
	  tdv::nuitrack::HandTracker::Ptr handTracker_;
	  tdv::nuitrack::GestureRecognizer::Ptr gestureRecognizer_;

  };
};  // namespace nuitrack_body_tracker


  // The main entry point for this node.
  int main( int argc, char *argv[] )
  {
    using namespace nuitrack_body_tracker;
    ros::init( argc, argv, "nuitrack_body_tracker" );
    nuitrack_body_tracker_node node(ros::this_node::getName());
	  node.Init("");
		node.Run();

    return 0;
  }




