# NuiTrack Body Tracker

# Info
   This is a ROS Node to provide functionality of the NuiTrack SDK (https://nuitrack.com)
   
   - NuiTrack is NOT FREE, but reasonably inexpensive, and works with a number of cameras (see their webpage)
   - Also see Orbbec Astra SDK as a possible alternative (but only for Orbbec cameras)

   Publishes 3 messages:
   
   1. body_tracking_position_pub_ custom message:  <body_tracker_msgs::BodyTracker>
   Includes:
   2D position of person relative to head camera; allows for fast, smooth tracking
     Joint.proj:  position in normalized projective coordinates
     (x, y from 0.0 to 1.0, z is real)
     Astra Mini FOV: 60 horz, 49.5 vert (degrees)

   3D position of the shoulder joint in relation to robot (using TF)
     Joint.real: position in real world coordinates
     Useful for tracking person in 3D
   
   2. body_tracking_skeleton_pub_ custom message: <body_tracker_msgs::Skeleton>
   Includes:
   Everyting in BodyTracker message above, plus 3D position of upper body 
   joints in relation to robot (using TF)

   3. marker_pub_  message: <visualization_msgs::Marker>
   Publishes 3d markers for selected joints.  Visible as markers in RVIZ.


# Installation Instructions / Prerequisites

  - Clone body_tracker_msgs into your Catkin workspace 
    - catkin_make to confirm complies OK
  - Clone this project into your Catkin workspace
  
  - Remove OpenNI - it conflicts with the version supplied by NuiTrack!
    -   sudo apt-get purge --auto-remove openni-utils
  - Download BOTH the nuitrack linux drivers and the Nuitrack SDK

  - Install Linux drivers:
    -   sudo dpkg -i <downloaded-package-name>.deb

    - Install Nuitrack SDK:
    -  extract with GUI, copy to home directory

# Test NuiTrack SDK
  - REBOOT!  If you run into errors, it is probably becuse you did not reboot.
  - Follow instructions at: ~/NuitrackSDK/Examples/nuitrack_gl_sample/README.txt



