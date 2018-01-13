#include "path_planner.h"

pathPlanner::pathPlanner(ros::NodeHandle& nh)
{
  dispatched=false;
  quadSet=true;
  turtleSet=false;

  mapSub = nh.subscribe("map",1,&pathPlanner::mapUpdate,this);
  quadPoseSub = nh.subscriber("/birdseye/orbslam_path",1,&pathPlanner::quadPoseUpdate,this);
  turtlePoseSub = nh.subscriber("/birdseye/cl_ugv_path",1,&pathPlanner::turtlePoseUpdate,this);
  dispStateSub = nh.subscriber("/birdseye/dispatch_state",1,&pathPlanner::dispatchState,this);

  waypointPub = nh.advertise<geometry_msgs::PoseArray>("/birdseye/returnWaypoints");

}

void pathPlanner::mapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& inMap)
{
  //TODO: Do path pathPlanner
  /*
  UAV pose -> lastQuadPose.pose.(x,y,z)
  UGV pose -> lastTurtlePose.pose.(x,y,z)
  Occupancy Map -> inMap.data[][]
  Map info -> inMap.info.(resolution(m/cell),width(cells),height(cells),origin)
    origin is of geometry_msgs::Pose type
  */

}

void pathPlanner::quadPoseUpdate(const nav_msgs::Path::ConstPtr& path)
{

}

void pathPlanner::turtlePoseUpdate(const nav_msgs::Path::ConstPtr& path)
{

}

void pathPlanner::dispatchState(const std_msgs::UInt8::ConstPtr& state)
{
  returning = (state.data == 2); //state=2 means returning 
}
