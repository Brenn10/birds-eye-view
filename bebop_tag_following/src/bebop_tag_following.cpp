/******************************************************************************
* bebopTagFollowing class
* This class is designed to track a tag using ar_track_alvar and move a Parrot
* Bebop. It also communicates with a TurtleBot about tag detection status.
*
* Author: Shannon Hood
* Maintainer: Brennan Cain
* Last modified: 13 Jan 2018
*/

#include "bebop_tag_following.h"
#include "PIDController.h"
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

#include "gmapping/dynamic_map.h"
#include "nav_msgs/GetMap.h"
using namespace std;
using namespace Eigen;

bebopTagFollowing::bebopTagFollowing(ros::NodeHandle& nh) {
  tagDetected = false;
  droneSeenOnce = false;
  objectSeenOnce= false;
  dispatch_state=following;
  //publishers & services
  turtleCmd = nh.advertise<geometry_msgs::Twist>("/turtlebot/commands/velocity",1);
  drone_comms = nh.advertise<std_msgs::UInt8>("/drone_comms", 1);
  cmdVel = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
  takeOff = nh.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
  land = nh.advertise<std_msgs::Empty>("/bebop/land", 1);
  dispatchPubber = nh.advertise<std_msgs::U?Int8>("/birdseye/dispatch_state",1);
  flattrim = nh.serviceClient<std_srvs::Empty>("/bebop/flattrim");

  //subscribers
  usbTagPose = nh.subscribe("/usb_cam_tracker/ar_pose_marker", 5, &bebopTagFollowing::usbCamTagCallback, this);
  bebopTagPose = nh.subscribe("/bebop_cam_tracker/ar_pose_marker", 5, &bebopTagFollowing::bebopCamTagCallback, this);
  orbSlamSub = nh.subscribe("/birdseye/orbslam_path", 1, &bebopTagFollowing::orbSlamCallback, this);
  clSub = nh.subscribe("/birdseye/cl_ugv_path", 1, &bebopTagFollowing::clCallback, this);
  navPathCallback = nh.subscribe("/birdseye/quad_return_path",1,&bebopTagFollowing::navPathCallback,this);

  //PID controllers
  z_pid= new PIDController(0.0,.4,-0.1,0.0);
  yaw_pid= new PIDController(0.0,.02,0.005,0.0);

  usleep(1000*1000); //wait 1 second to ensure commands are published
  std_srvs::Empty srv;
  flattrim.call(srv);
  usleep(100);
  initialize();
}

/*****************************************************************************
* Function: initialize
* Input: none
*
* Makes the drone take off.
*/
void bebopTagFollowing::initialize() {
  startTime = ros::Time::now().toSec();
  state.x = state.y = state.z = 0;

  std_msgs::Empty blank;
  takeOff.publish(blank);
  usleep(100);

  // Let Turtlebot know to start
  std_msgs::UInt8 turtleInfo;
  turtleInfo.data = NO_TAG_FOUND;
  drone_comms.publish(turtleInfo);
  usleep(100);

  ROS_INFO("Bebop initialization finished");
}

/******************************************************************************
* Function: shutDown
* Input: none
*
* Called on exit. Clear all drone commands and land the drone
*/
void bebopTagFollowing::shutDown() {
  std_msgs::Empty blank;
  hover();
  land.publish(blank);
}

/******************************************************************************
* Function: hover
* Input: none
*
* Makes the drone take hover in one spot by sending 0's as velocities.
*/
void bebopTagFollowing::hover() {
  geometry_msgs::Twist cmdT;
  cmdT.linear.z = 0;
  cmdT.linear.x = 0;
  cmdT.linear.y = 0;
  cmdT.angular.x = cmdT.angular.y = cmdT.angular.z = 0;
  cmdVel.publish(cmdT);
}

/******************************************************************************
* Function: orbSlamCallback
* Input: ORB-SLAM path
*
* Saves the last pose of the UAV computed using ORB-SLAM
*/
void bebopTagFollowing::orbSlamCallback(const nav_msgs::Path::ConstPtr& msg) {
  lastOrbPose = msg->poses[msg->poses.size()-1].pose;
}

/******************************************************************************
* Function: clCallback
* Input: cooperative localization path
*
* Saves the last pose of the UGV computed using cooperative localization
*/
void bebopTagFollowing::clCallback(const nav_msgs::Path::ConstPtr& msg) {
  lastUGVPose = msg->poses[msg->poses.size()-1].pose;
}

/*
gives the distance between two angles in degrees
*/
double ang_dist(double a,double b)
{
  double dist= b-a;
  while(dist<-180)
  dist+=360;
  while(dist>180)
  dist-=360;
  return dist;
}


/**************************************************************************************
* Function: usbTagFollowingCallback
* Input: ar_track_alvar msg
* TODO: find the tag we want to follow and only use that tag
*
* If the drone is using its bottom camera, then it will move towards the tag and
* maintain its height.
*/
void bebopTagFollowing::usbCamTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  currentTime = ros::Time::now().toSec();
  std_msgs::UInt8 turtleInfo;
  ostringstream s;
  std_msgs::String command;
  //if fully dispatched

  if(dispatch_state==following){
    if(!msg->markers.empty()) {
      ROS_WARN("Tag Spotted");
      droneSeenOnce = true;
      timeDelta = currentTime-droneLastSeen;
      droneLastSeen = currentTime;

      if(!tagDetected) {
        printedWarn = false;
        tagDetected = true;
      }

      // Save velocities
      state.vx = -(state.x + msg->markers[0].pose.pose.position.x) / timeDelta; // state velocity
      state.vy = -(state.y + msg->markers[0].pose.pose.position.y) / timeDelta; // state velocity

      // Save x, y, z pose
      state.x = msg->markers[0].pose.pose.position.x;
      state.y = msg->markers[0].pose.pose.position.y;
      state.z = msg->markers[0].pose.pose.position.z;

      // Get message as quaternion, then get roll, pitch, yaw
      tf::Quaternion q(
        msg->markers[0].pose.pose.orientation.x,
        msg->markers[0].pose.pose.orientation.y,
        msg->markers[0].pose.pose.orientation.z,
        msg->markers[0].pose.pose.orientation.w
      );
      tf::Matrix3x3 m(q);
      double t_r, t_p, t_y;
      m.getRPY(t_r, t_p, t_y);
      // Transform x, y, z, and yaw to drones distance from tag
      state.x = -state.x; // Originally, forward is pos and backward is neg.
      state.y = -state.y; // Originally, left is neg and right is pos.
      state.z = -(state.z-HEIGHT_OVER_TAG);
      state.yaw = t_y * 180 / M_PI;
      state.yaw = (state.yaw-90);
      if(state.yaw<-180.0) {
        state.yaw = 360.0+state.yaw;
      }
      ROS_INFO("Tag at: x=%1.2f  y=%1.2f  z=%1.2f  yaw=%1.2f",
      state.x, state.y, state.z, state.yaw);
      ROS_INFO("Tag vel: vx=%1.3f  vy=%1.3f", state.vx, state.vy);

      // Let Turtlebot know a tag is found
      turtleInfo.data = FOUND_TAG;
      drone_comms.publish(turtleInfo);
      usleep(100);

      //update PIDs
      z_pid->update(state.z);
      yaw_pid->update(-state.yaw);
      geometry_msgs::Twist cmdT;

      //pid controllers based on last and this state (accurate)
      cmdT.linear.x = state.x*0.25+.8*state.vx;
      cmdT.linear.y = state.y*0.25+.8*state.vy;


      // rotate velocities the same as yaw (to have them point to the robot)
      double yaw_rad = state.yaw*M_PI/180;
      Vector2d v(cmdT.linear.x,cmdT.linear.y);
      Matrix2d T; //standard rotation matrix
      T << cos(yaw_rad),sin(yaw_rad),
      -sin(yaw_rad), cos(yaw_rad);
      v = T*v;

      cmdT.linear.x = v[0];
      cmdT.linear.y = v[1];

      //get commands from pid controllers
      cmdT.angular.z = yaw_pid->signal;
      cmdT.linear.z = z_pid->signal;
      cmdT.angular.x = cmdT.angular.y = 0;


      cmdVel.publish(cmdT);
    }

    else { // There are no tags detected
      else if(currentTime-droneLastSeen>10) {
        if(!printedWarn and droneSeenOnce) {
          tagDetected = false;
          printedWarn = true;
          turtleInfo.data = LOST_TAG;
          drone_comms.publish(turtleInfo);
          usleep(100);
          ROS_ERROR("Failed recover, attempting retrieval.");
          hover();
          setDispatchState(returning);
        }
      }
      //if recently lost, try to recover using simple mechanism
      else if(currentTime-droneLastSeen>1) {
        ROS_WARN("Lost tag, attempting recovery.");

        geometry_msgs::Twist cmdT;
        cmdT.linear.z=.1;
        cmdT.angular.x = cmdT.angular.y = cmdT.angular.z = cmdT.linear.x = cmdT.linear.y = 0;
        cmdVel.publish(cmdT);
        geometry_msgs::Twist turtleCmdT;
        turtleCmdT.angular.z=.3;
        turtleCmdT.angular.x = turtleCmdT.angular.y = turtleCmdT.linear.z = turtleCmdT.linear.x = turtleCmdT.linear.y = 0;
        turtleCmd.publish(turtleCmdT);
      }
    }
  }
  else if(dispatch_state==returning){
    if(!msg->makers.empty())){ //successful return
      setDispatchState(following);
    }
  }
}

/**************************************************************************************
* Function: bebopTagFollowingCallback
* Input: ar_track_alvar msg
* TODO: find the tag we want to follow and only use that tag
*
* If the drone is using its bottom camera, then it will move towards the tag and
* maintain its height.
*/
void bebopTagFollowing::bebopCamTagCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  currentTime = ros::Time::now().toSec();
  std_msgs::UInt8 turtleInfo;
  ostringstream s;
  std_msgs::String command;

  if(!msg->markers.empty() and dispatch_state==following) //if following and object spotted, investigate
  {
    setDispatchState(dispatched);
    yaw_pid->reset();
  }
  if(dispatch_state==dispatched)
  {
    if(!msg->markers.empty())
    {
      ROS_WARN("Object Spotted");

      objState.x = msg->markers[0].pose.pose.position.x;
      objState.y = msg->markers[0].pose.pose.position.y;
      objState.z = msg->markers[0].pose.pose.position.z;


      ROS_INFO("Object at: x=%1.2f  y=%1.2f  z=%1.2f",
      state.x, state.y, state.z);

      if(pow(objState.x,2)+pow(objState.y,2)+pow(objState.z,2)<1)
      {
        ROS_WARN("Quad has arrived at the object, returning");
        setDispatchState(returning);
        yaw_pid->reset();
        return;
      }
      double angleOff = atan2(objState.x,objState.z);
      yaw_pid->update(angleOff);
      Vector2d v(1,0);
      Matrix2d T; //standard rotation matrix
      T << cos(angleOff),sin(angleOff),
      -sin(angleOff), cos(angleOff);
      v = T*v;

      cmdT.linear.x = v[0];
      cmdT.linear.y = v[1];
      cmdT.angular.z = yaw_pid->signal;

      cmdT.linear.z=cmdT.angular.x=cmdT.angular.y=0;

      cmdVel.publish(cmdT);
    }
    else //stop for a few seconds then return to robot
    {
      if(currentTime-objectLastSeen<3)
      {
        cmdT.angular.x = cmdT.angular.y = cmdT.angular.z = cmdT.linear.x = cmdT.linear.y = cmdT.linear.z = 0;
        cmdVel.publish(cmdT);
      }
      else
      {
        setDispatchState(returning);
        yaw_pid->reset();
      }
    }
  }
}

void bebopTagFollowing::navPathCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  //TODO: update instance var containing waypoints
}

void bebopTagFollowing::setDispatchState(Dispatch new_disp_state)
{
  std_msgs::UInt8 msg;
  msg.data=new_disp_state;
  dispatchPubber.publish(msg);
  dispatch_state=new_disp_state
}
/******************************************************************************
* Function: main
* Input: none required
*
* Runs this node, allowing the Bebop to start following tags
*/
main(int argc, char **argv) {
  ros::init(argc, argv, "pilot");
  ros::NodeHandle n;
  bebopTagFollowing pilot(n);
  //  ros::Rate rate(15);
  // while(ros::ok()) {
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  ros::spin();

  pilot.shutDown();
}
