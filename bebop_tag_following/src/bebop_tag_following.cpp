/******************************************************************************
* bebopTagFollowing class
* This class is designed to track a tag using ar_track_alvar and move a Parrot
* Bebop. It also communicates with a TurtleBot about tag detection status.
*
* Author: Shannon Hood
* Last modified: 13 Feb 2017
*/

#include "bebop_tag_following.h"
#include "PIDController.h"
#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

bebopTagFollowing::bebopTagFollowing(ros::NodeHandle& nh) {
  tagDetected = false;
  seenOnce = false;
  dispatch_state=following;
  //publishers & services
  drone_comms = nh.advertise<std_msgs::UInt8>("/drone_comms", 1);
  cmdVel = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
  takeOff = nh.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
  land = nh.advertise<std_msgs::Empty>("/bebop/land", 1);
  flattrim = nh.serviceClient<std_srvs::Empty>("/bebop/flattrim");

  //subscribers
  tagPose = nh.subscribe("/ar_pose_marker", 5, &bebopTagFollowing::positionCallback, this);
  orbSlamSub = nh.subscribe("/birdseye/orbslam_path", 1, &bebopTagFollowing::orbSlamCallback, this);
  clSub = nh.subscribe("/birdseye/cl_ugv_path", 1, &bebopTagFollowing::clCallback, this);
  dispatchSub = nh.subscribe("/birdseye/dispatch",1,&bebopTagFollowing::dispatchDrone,this);
  dispatchSub = nh.subscribe("/birdseye/retrieve",1,&bebopTagFollowing::retrieveDrone,this);

  //PID controllers
  z_pid= new PIDController(0.0,.4,-0.1,0.0);
  yaw_pid= new PIDController(0.0,.02,0.005,0.0);

  usleep(1000*1000); //wait 1 second to ensure commands are published
  std_srvs::Empty srv;
  flattrim.call(srv);
  usleep(100);
  initialize();
}

/******************************************************************************
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

  std::cout << "{INFO}: Bebop initialization finished" << std::endl;
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
* Function: positionCallback
* Input: ar_track_alvar msg
* TODO: find the tag we want to follow and only use that tag
*
* If the drone is using its bottom camera, then it will move towards the tag and
* maintain its height.
*/
void bebopTagFollowing::positionCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
  currentTime = ros::Time::now().toSec();
  std_msgs::UInt8 turtleInfo;
  ostringstream s;
  std_msgs::String command;
  //if fully dispatched
  //TODO: implement explore and retireval
  if(dispatch_state>=3){
    hover();
  }
  else if(!msg->markers.empty()) {//if not dispatched
    seenOnce = true;
    timeDelta = currentTime-lastSeen;
    lastSeen = currentTime;

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
    tf::Quaternion q(msg->markers[0].pose.pose.orientation.x,
                     msg->markers[0].pose.pose.orientation.y,
                     msg->markers[0].pose.pose.orientation.z,
                     msg->markers[0].pose.pose.orientation.w);
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

    switch(dispatch_state)
    {
      case following: //if not in a dispatch state
      {
        z_pid->update(state.z);
        yaw_pid->update(-state.yaw);
        geometry_msgs::Twist cmdT;

        //pid controllers based on last and this state (accurate)
        cmdT.linear.x = state.x*0.25+.8*state.vx;
        cmdT.linear.y = state.y*0.25+.8*state.vy;


        // rotate velocities the same as yaw (to have them point to the robot)
        double yaw_rad = state.yaw*M_PI/180;
        Vector2d v(cmdT.linear.x,cmdT.linear.y);
        Matrix2d T; //standard rotation matrix a
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
        break;
      }
      case turning://in turning state
      {
        z_pid->update(state.z);
        yaw_pid->update(ang_dist(state.yaw,disp_angle));
        geometry_msgs::Twist cmdT;

        //pid controllers based on last and this state (accurate)
        cmdT.linear.x = state.x*0.25+.8*state.vx;
        cmdT.linear.y = state.y*0.25+.8*state.vy;


        // rotate velocities the same as yaw (to have them point to the robot)
        double yaw_rad = state.yaw*M_PI/180;
        Vector2d v(cmdT.linear.x,cmdT.linear.y);
        Matrix2d T; //standard rotation matrix a
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
        if(abs(ang_dist(state.yaw,disp_angle))<5)
        {
          ROS_WARN("Drone leaving");
          dispatch_state=leaving;
        }
        break;
      }
      case leaving://in leaving state(i'll fly away, fly away)
      {

        z_pid->update(state.z);
        yaw_pid->update(ang_dist(state.yaw,disp_angle));
        geometry_msgs::Twist cmdT;

        //pid controllers based on last and this state (accurate)
        cmdT.linear.x = disp_vel;//TODO: figure out which is forward

        //get commands from pid controllers
        cmdT.angular.z = yaw_pid->signal;
        cmdT.linear.z = z_pid->signal;
        cmdT.angular.x = cmdT.angular.y = cmdT.linear.y = 0;


        cmdVel.publish(cmdT);
        break;
      }
    }
  }

  else { // There are no tags detected
    //if in leaving mode and no tags detected, that means we are now dispatched
    if(dispatch_state==leaving) {
      ROS_WARN("Drone has been dispatched");
      dispatch_state=dispatched;
      hover();
    }
    //if time last seen was N seconds+ ago, we failwed recover
    // TODO: implement auto retrieval if lost for long time (and hope we dont hit a wall)
    else if(currentTime-lastSeen>5) {
      if(!printedWarn) {
        tagDetected = false;
        printedWarn = true;
        turtleInfo.data = LOST_TAG;
        drone_comms.publish(turtleInfo);
        usleep(100);
        ROS_ERROR("Failed recover, please teleop to recover.");
        hover();
      }
      if(seenOnce) {

        hover();
      }
    }
    //if recently lost, try to recover using simple mechanism
    else {
      ROS_WARN("Lost tag, attempting recovery.");

      geometry_msgs::Twist cmdT;

      //move in direction of error, clip vel to +/-.1m/s
      cmdT.linear.x = min(.1,max(-.1,state.x*.1));
      cmdT.linear.y = min(.1,max(-.1,state.y*.1));


      // rotate velocities the same as yaw (to have them point to the robot)
      double yaw_rad = state.yaw*M_PI/180;
      Vector2d v(cmdT.linear.x,cmdT.linear.y);
      Matrix2d T; //standard rotation matrix a
      T << cos(yaw_rad),sin(yaw_rad),
           -sin(yaw_rad), cos(yaw_rad);
      v = T*v;

      //decompose vector
      cmdT.linear.x = v[0];
      cmdT.linear.y = v[1];

      cmdT.angular.x = cmdT.angular.y = cmdT.angular.z = cmdT.linear.z = 0;


      cmdVel.publish(cmdT);
    }
  }
}

/* Dispatch teh drone in a given direction

TODO: tell robot to stop
TODO: implement dispatch
*/
void bebopTagFollowing::dispatchDrone( const bebop_tag_following::Dispatch::ConstPtr& msg)
{
  // Let Turtlebot know a tag is found
  if(dispatch_state==following)
  {
    ROS_WARN("Telling drone to get lost");
    std_msgs::UInt8 turtleInfo;
    turtleInfo.data = DRONE_DISPATCHED;
    disp_angle=msg->yaw;
    disp_vel=msg->lin_vel;

    dispatch_state=turning;

    drone_comms.publish(turtleInfo);
  }
}

void bebopTagFollowing::retrieveDrone( const bebop_tag_following::Dispatch::ConstPtr& msg)
{

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
