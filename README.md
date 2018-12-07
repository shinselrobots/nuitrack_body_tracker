# Nuitrack Body Tracker

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

## Follow instructions on Nuitrack website
  - http://download.3divi.com/Nuitrack/doc/Installation_page.html

### Summary instructions below, but bits might not be up to date!  Go to the website to get the latest SDK.


  - Clone body_tracker_msgs into your Catkin workspace 
    - catkin_make to confirm complies OK

  - Clone this project into your Catkin workspace
  
  - Remove OpenNI - it conflicts with the version supplied by Nuitrack!
    -   sudo apt-get purge --auto-remove openni-utils

  - Download BOTH the nuitrack linux drivers and the Nuitrack SDK

  - Install Nuitrack Linux drivers:
    -   sudo dpkg -i nuitrack-ubuntu-amd64.deb
    -   sudo reboot
    -   confirm environment variables set correctly:
        - echo $NUITRACK_HOME    (should be /usr/etc/nuitrack)
        - echo $LD_LIBRARY_PATH  (should include /usr/local/lib/nuitrack)

  - Install Nuitrack SDK (NuitrackSDK.zip)
    - Download from: http://download.3divi.com/Nuitrack/
    - mkdir ~/sdk/NuitrackSDK
    - cp NuitrackSDK.zip ~/NuitrackSDK
    - extract ZIP archive with ubuntu Archive Manager (double click the zip file)
    - delete the zip file

  - Edit CMakeLists.txt if you installed the SDK to a different location:
    set(NUITRACK_SDK_PATH /home/system/sdk/NuitrackSDK)

# NOTE: Nuitrack install may break other RealSense applications!
  - See this discussion: 
    - https://community.nuitrack.com/t/nuitrack-prevents-other-realsense-apps-from-working/893
    - if needed: export LD_LIBRARY_PATH="{$LD_LIBRARY_PATH}:/usr/local/lib/nuitrack"


# Option: Edit Nuitrack Config parameters (now done in the node code)
  - sudo vi $NUITRACK_HOME/data/nuitrack.config
  - set Faces.ToUse and DepthProvider.Depth2ColorRegistration to true

# Test NuiTrack SDK
  - If you run into errors, it is probably becuse you did not reboot after driver install

  - Try the license tool, it will test the system:
    - sudo -E nuitrack_license_tool
    - click on "compatibility test"
    - if you have a license, enter it after the test completes.

  - Follow instructions at: ~/sdk/NuitrackSDK/Examples/nuitrack_gl_sample/README.txt




