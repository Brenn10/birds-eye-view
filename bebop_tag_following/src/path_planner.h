#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"

#ifndef PATH_PLANNER_BSC_H
#define PATH_PLANNER_BSC_H

class pathPlanner {
  public:
    pathPlanner(ros::NodeHandle& nh);

    //callbacks
    void mapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& inMap);
    void quadPoseUpdate(const nav_msgs::Path::ConstPtr& path);//last pose is the current
    void turtlePoseUpdate(const nav_msgs::Path::ConstPtr& path);//last pose is the current
    void dispatchState(const std_msgs::UInt8::ConstPtr& state);
  private:

    bool returning;
    bool quadSet,turtleSet;

    geometry_msgs::Pose lastQuadPose;
    geometry_msgs::Pose lastTurtlePose;
    geometry_msgs::PoseArray waypoints;

    //Subscriptions
    ros::Subscriber mapSub;
    ros::Subscriber quadPoseSub;
    ros::Subscriber turtlePoseSub;
    ros::Subscriber dispStateSub;

    //publishers
    ros::Publisher waypointPub;
}
