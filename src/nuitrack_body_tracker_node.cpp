/* Body Tracker Node using Nuitrack library

   Publish data messages:
   
   1. body_tracking_position_pub_ custom message:  <body_tracker_msgs::BodyTracker>
   Includes:
   2D position of person relative to head camera; allows for fast, smooth tracking
     Joint.proj:  position in normalized projective coordinates
     (x, y from 0.0 to 1.0, z is real)
     Astra Mini FOV: 60 horz, 49.5 vert (degrees)
     
     body_tracking_array_pub_ : multiple person detections in one message

   3D position of the person's neck joint in relation to robot (using TF)
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
#include <sstream>
#include <iostream>
#include <iomanip>  // setprecision

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <body_tracker_msgs/BodyTracker.h>       // Publish custom message
#include <body_tracker_msgs/BodyTrackerArray.h>  // Custom message, multiple people 
#include <body_tracker_msgs/Skeleton.h>          // Publish custom message

// If Camera mounted on Pan/Tilt head
//#include "sensor_msgs/JointState.h"
#include "dynamixel_msgs/JointState.h"

//For Nuitrack SDK
#include "nuitrack/Nuitrack.h"
#define KEY_JOINT_TO_TRACK    JOINT_LEFT_COLLAR // JOINT_TORSO // JOINT_NECK

// For Face JSON parsing
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

const bool ENABLE_PUBLISHING_FRAMES = true;

namespace nuitrack_body_tracker
{
  using namespace tdv::nuitrack;
  //namespace pt = boost::property_tree;

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
      nodeHandle.param<std::string>("camera_color_frame",camera_color_frame_,"camera_color_frame");

      // Publishers and Subscribers

      // Publish tracked person in 2D and 3D
      // 2D: x,y in camera frame.   3D: x,y,z in world coordinates
      body_tracking_position_pub_ = nh_.advertise<body_tracker_msgs::BodyTracker>
        ("body_tracker/position", 1); 

      body_tracking_array_pub_ =
        nh_.advertise<body_tracker_msgs::BodyTrackerArray>
        ("body_tracker_array/position", 1); 

      // Publish tracked person upper body skeleton for advanced uses
      body_tracking_skeleton_pub_ = nh_.advertise<body_tracker_msgs::Skeleton>
        ("body_tracker/skeleton", 1);

      // Publish markers to show where robot thinks person is in RViz
      marker_pub_ = nh_.advertise<visualization_msgs::Marker>
        ("body_tracker/marker", 1);

      // Publish the depth frame for other nodes
      depth_image_pub_ = nh_.advertise<sensor_msgs::Image>
        ("camera/depth/image", 1);
      color_image_pub_ = nh_.advertise<sensor_msgs::Image>
        ("camera/color/image", 1);

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
      //ROS_INFO("DBG: Nuitrack::onNewDepthFrame()");

      if(!ENABLE_PUBLISHING_FRAMES)
      {
        return;
      }

      sensor_msgs::Image msg;
      int _width = frame->getCols(); 
      int _height = frame->getRows();

      const uint16_t* depthPtr = frame->getData();

      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = camera_depth_frame_;
      msg.height = _height; 
      msg.width = _width; 
      msg.encoding = "rgb8";  // see sensor_msgs::image_encodings
      msg.is_bigendian = false;

      msg.step = 3 * _width; // sensor_msgs::ImagePtr row step size

      for (size_t row = 0; row < _height; ++row)
      {
        for (size_t col = 0; col < _width; ++col )
        {
          uint16_t depthValue = *(depthPtr+ col) >> 5;

          // RGB are all the same for depth (monochrome)
          msg.data.push_back(depthValue); 
          msg.data.push_back(depthValue);
          msg.data.push_back(depthValue);

        }
        depthPtr += _width; // Next row
      }

      depth_image_pub_.publish(msg);

    }

    void onNewColorFrame(RGBFrame::Ptr frame)
    {
      //ROS_INFO("DBG: Nuitrack::onNewColorFrame()");

      if(!ENABLE_PUBLISHING_FRAMES)
      {
        return;
      }

      sensor_msgs::Image msg;
      int _width = frame->getCols(); 
      int _height = frame->getRows(); 

      const tdv::nuitrack::Color3* colorPtr = frame->getData();

      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = camera_color_frame_;
      msg.height = _height; 
      msg.width = _width;  
      msg.encoding = "rgb8";  //sensor_msgs::image_encodings::TYPE_16UC1;
      msg.is_bigendian = false;

      msg.step = 3 * _width; // sensor_msgs::ImagePtr row step size

      for (size_t row = 0; row < _height; ++row)
      {
        for (size_t col = 0; col < _width; ++col )
        {
          msg.data.push_back((colorPtr + col)->red); 
          msg.data.push_back((colorPtr + col)->green);
          msg.data.push_back((colorPtr + col)->blue);

        }
        colorPtr += _width; // Next row
      }

      color_image_pub_.publish(msg);
    }


    void onUserUpdate(tdv::nuitrack::UserFrame::Ptr frame)
    {
      // std::cout << "Nuitrack: onUserUpdate callback" << std::endl;
    }


    void onSkeletonUpdate(SkeletonData::Ptr userSkeletons)
    {
      // std::cout << "Nuitrack: onSkeletonUpdate callback" << std::endl;

      // Message for array of body detections
      body_tracker_msgs::BodyTrackerArray  body_tracker_array_msg;
      body_tracker_array_msg.header.frame_id = camera_depth_frame_;
      ros::Time frame_time_stamp = ros::Time::now();
      body_tracker_array_msg.header.stamp = frame_time_stamp;

//      body_tracker_msgs::BodyTrackerArray_ 
//        <body_tracker_msgs::BodyTrackerArray> foo; // body_tracker_array_msg;


      // process skeletons for each user found
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
        body_tracker_msgs::BodyTracker position_data;
//        body_tracker_msgs::BodyTracker_ <body_tracker_msgs::BodyTracker> position_data;

        position_data.body_id = skeleton.id;
        position_data.tracking_status = 0; // TODO
        position_data.gesture = -1; // No gesture
        position_data.face_found = false;
        position_data.face_left = 0;
        position_data.face_top = 0;
        position_data.face_width = 0;
        position_data.face_height = 0;
        position_data.face_age = 0;
        position_data.face_gender = 0;

        //if(skeleton.id != last_id_)
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



        ///////////////////////////////////////////////////////////////
        // Face Data
        // if the same ID as skeleton id, publish face data too

        std::string face_info = tdv::nuitrack::Nuitrack::getInstancesJson();
        //std::cout << face_info; //This will print the entire json object.
        // Good examples at: http://zenol.fr/blog/boost-property-tree/en.html

        try
        {
          std::stringstream ss;
          ss << face_info;
          boost::property_tree::ptree root;
          boost::property_tree::read_json(ss, root);

          // Find all instances of objects (usually people)
          for(boost::property_tree::ptree::value_type &instance : root.get_child("Instances"))
          {
            std::string json_id_str = "";
            std::string json_class_str = "";
            int json_id = -1;

            for (boost::property_tree::ptree::value_type &found_object : instance.second)
            {

              if( "id" == found_object.first)
              {
                json_id_str = found_object.second.data();
                json_id = found_object.second.get_value<int>();
                std::cout << "FIELD: id = " << json_id_str << " = " << json_id << std::endl;

              }
              else if( "class" == found_object.first)
              {
                std::cout << "FIELD: class = " << found_object.second.data() << std::endl;
                json_class_str = found_object.second.data();

              }
              else if( "face" == found_object.first)
              {

                // See if we found a face ID that matches current skeleton
                //if( (json_class_str == "human") && (json_id_str != "") )
                if( !( (json_class_str == "human") && (json_id == skeleton.id) ))
                {
                  std::cout << "FACE ID (" << json_id << ") DOES NOT MATCH SKELETON (" <<
                    skeleton.id << ")... SKIPPING  (or object != Human?)" << std::endl;
                }
                else
                {

                  boost::property_tree::ptree face = found_object.second; // subtree
                  if(face.empty()) 
                  {
                    std::cout << "Face tree is empty!" << std::endl;
                  }
                  else
                  {
                    // this is a face subtree
                    std::cout << "FACE FOUND " << std::endl;
                    position_data.face_found = true;
                    float face_left, face_top, face_width, face_height;
                    face_left = face_top = face_width = face_height = 0.0;

                    for(boost::property_tree::ptree::value_type &rectangle : face.get_child("rectangle"))
                    {
                      // Face bounding box from 0.0 -> 1.0 (from top left of image) 
                      // convert to pixel position before publishing              
                      std::string name = rectangle.first;
                      std::string val = rectangle.second.data();
                      std::cout << "FACE RECTANGLE: " << name << " : " << val << std::endl;
                      
                      if( rectangle.first == "left")
                      {
                        face_left = rectangle.second.get_value<float>();
                        position_data.face_left = (int)((float)frame_width_ * face_left);
                      }                      
                      if( rectangle.first == "top")
                      {
                        face_top = rectangle.second.get_value<float>();
                        position_data.face_top = (int)((float)frame_width_ * face_top);
                      }                      
                      if( rectangle.first == "width")
                      {
                        face_width = rectangle.second.get_value<float>();
                        position_data.face_width = (int)((float)frame_width_ * face_width);
                      }                      
                      if( rectangle.first == "height")
                      {
                        face_height = rectangle.second.get_value<float>();
                        position_data.face_height = (int)((float)frame_width_ * face_height);
                      }                      
                    }
                    
                    // Get center of the face bounding box and convert projection to radians
                    // proj is 0.0 (left) --> 1.0 (right)
                    
                    float face_center_proj_x = face_left + (face_width / 2.0);
                    float face_center_proj_y = face_top + (face_height / 2.0);
                    position_data.face_center.x = (face_center_proj_x - 0.5) * ASTRA_MINI_FOV_X;
                    position_data.face_center.y =  (face_center_proj_y - 0.5) * ASTRA_MINI_FOV_Y;
                    // just use the skeleton location 
                    position_data.face_center.z = skeleton.joints[JOINT_HEAD].real.z / 1000.0;
                    
                    //std::cout << "DBG face_center_proj = " << face_center_proj_x << ", " <<
                    //  face_center_proj_y << std::endl;
                      
                    //std::cout << "DBG face_center_ROS = " << position_data.face_center.x << ", " <<
                    //  position_data.face_center.y << std::endl;

                    
                    
                    for(boost::property_tree::ptree::value_type &angles : face.get_child("angles"))
                    {
                      // rectangle is set of std::pair
                      std::string name = angles.first;
                      std::string val = angles.second.data();
                      //std::cout << "FACE ANGLES: " << name << " : " << val << std::endl;
                      // Not currently published for ROS (future)

                    }
                    for(boost::property_tree::ptree::value_type &age : face.get_child("age"))
                    {
                      // rectangle is set of std::pair
                      std::string name = age.first;
                      std::string val = age.second.data();
                      std::cout << "FACE AGE: " << name << " : " << val << std::endl;
                      
                      if( age.first == "years")
                      {
                        float face_age = age.second.get_value<float>();
                        position_data.face_age = (int)face_age;
                      }                      

                    }

                    std::string gender = face.get<std::string>("gender");
                    std::cout << "GENDER: " << gender << std::endl;
                    if("male" == gender)
                    {
                      position_data.face_gender = 1;
                    }
                    else if("female" == gender)
                    {
                      position_data.face_gender = 2;
                    }

                  }
                }
              }
            }
          }
        }
        catch (std::exception const& e)
        {
          std::cerr << e.what() << std::endl;
        }


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
        // Publish custom position and skeleton messages for each person found

              
        body_tracking_position_pub_.publish(position_data); // position data
        body_tracking_skeleton_pub_.publish(skeleton_data); // full skeleton data

        // Msg with array of position data for each person detected
        body_tracker_array_msg.detected_list.push_back(position_data); 


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

      ////////////////////////////////////////////////////
      // Publish custom array message with position info for all people found
      body_tracking_array_pub_.publish(body_tracker_array_msg);
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
        printf("onNewGesture: Gesture Recognized %d for User %d\n", 
          userGestures_[i].type, userGestures_[i].userId);

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

      // Set config values.  Overrides $NUITRACK_HOME/data/nuitrack.config

      // Align depth and color 
      Nuitrack::setConfigValue("DepthProvider.Depth2ColorRegistration", "true");

      // Realsense Depth Module - force to 848x480 @ 60 FPS
      Nuitrack::setConfigValue("Realsense2Module.Depth.Preset", "5");
      Nuitrack::setConfigValue("Realsense2Module.Depth.RawWidth", "848");
      Nuitrack::setConfigValue("Realsense2Module.Depth.RawHeight", "480");
      Nuitrack::setConfigValue("Realsense2Module.Depth.ProcessWidth", "848");
      Nuitrack::setConfigValue("Realsense2Module.Depth.ProcessHeight", "480");
      Nuitrack::setConfigValue("Realsense2Module.Depth.FPS", "60");

      // Realsense RGB Module - force to 848x480 @ 60 FPS
      Nuitrack::setConfigValue("Realsense2Module.RGB.RawWidth", "848");
      Nuitrack::setConfigValue("Realsense2Module.RGB.RawHeight", "480");
      Nuitrack::setConfigValue("Realsense2Module.RGB.ProcessWidth", "848");
      Nuitrack::setConfigValue("Realsense2Module.RGB.ProcessHeight", "480");
      Nuitrack::setConfigValue("Realsense2Module.RGB.FPS", "60");

      // Enable face tracking
      Nuitrack::setConfigValue("Faces.ToUse", "true");

      //Options for debug
      //Nuitrack::setConfigValue("Skeletonization.ActiveUsers", "1");
      //Nuitrack::setConfigValue("DepthProvider.Mirror", "true");


      // Create all required Nuitrack modules

      std::cout << "Nuitrack: DepthSensor::create()" << std::endl;
      depthSensor_ = tdv::nuitrack::DepthSensor::create();
      // Bind to event new frame
      depthSensor_->connectOnNewFrame(std::bind(
      &nuitrack_body_tracker_node::onNewDepthFrame, this, std::placeholders::_1));
  
      std::cout << "Nuitrack: ColorSensor::create()" << std::endl;
      colorSensor_ = tdv::nuitrack::ColorSensor::create();
      // Bind to event new frame
      colorSensor_->connectOnNewFrame(std::bind(
      &nuitrack_body_tracker_node::onNewColorFrame, this, std::placeholders::_1));
  
      outputMode_ = depthSensor_->getOutputMode();
      OutputMode colorOutputMode = colorSensor_->getOutputMode();
      if (colorOutputMode.xres > outputMode_.xres)
        outputMode_.xres = colorOutputMode.xres;
      if (colorOutputMode.yres > outputMode_.yres)
        outputMode_.yres = colorOutputMode.yres;

      // Use the larger of color or depth frame size
      frame_width_ = outputMode_.xres;
      frame_height_ = outputMode_.yres;
      last_id_ = -1;
      std::cout << "========= Nuitrack: GOT DEPTH SENSOR =========" << std::endl;
      std::cout << "Nuitrack: Depth:  width = " << frame_width_ << 
        "  height = " << frame_height_ << std::endl;

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
    std::string camera_color_frame_;    
    int frame_width_, frame_height_;
    int last_id_;
    ros::Publisher body_tracking_position_pub_;
    ros::Publisher body_tracking_array_pub_;
    ros::Publisher body_tracking_skeleton_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher depth_image_pub_;
    ros::Publisher color_image_pub_;

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
    //tdv::nuitrack::getInstancesJson::Ptr getInstancesJson;


    /* Note from http://download.3divi.com/Nuitrack/doc/Instance_based_API.html
    Face modules are by default disabled. To enable face modules, open nuitrack.config file and set Faces.ToUse and DepthProvider.Depth2ColorRegistration to true.
    */

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




