//
// Created by vlad on 05.02.2021.
//
#include <ros/ros.h>
#include <ros/console.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cds_msgs/PathStamped.h>
#include <nav_msgs/Odometry.h>

#include "planner_cds_core/mission.h"

#include <chrono>

double yaw(geometry_msgs::Quaternion q){
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


class Planner{
private:
    ros::Subscriber     taskSub;
    ros::Subscriber     gridSub;
    ros::Subscriber     odomSub;
    ros::Publisher      trajPub;

    tf2_ros::Buffer&        tfBuffer;
    nav_msgs::OccupancyGrid grid;
    nav_msgs::Odometry      odom;

    Mission mission;

    std::string odomTopic;
    std::string taskTopic;
    std::string gridTopic;
    std::string pathTopic;

    std::string searchType;

    bool gridSet;
    cds_msgs::PathStamped path;
public:
    Planner(tf2_ros::Buffer& _tfBuffer);
    void setTask(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
    void setGrid(const nav_msgs::OccupancyGrid::ConstPtr& gridMsg);
    void setOdom(const nav_msgs::Odometry::ConstPtr& odomMsg);

    bool plan();
    void fillPath(std::list<Node> lppath);
    void transformPath();
    void publish();
    geometry_msgs::Pose transformPoseToTargetFrame(geometry_msgs::Pose poseIn, std::string poseFrame, std::string targetFrame);
    geometry_msgs::Pose rescaleToGrid(geometry_msgs::Pose Pose);
    geometry_msgs::Pose rescaleFromGrid(geometry_msgs::Pose Pose);

};

Planner::Planner(tf2_ros::Buffer& _tfBuffer): tfBuffer(_tfBuffer){
    ros::NodeHandle nh;


    nh.param<std::string>("/node_params/odom_topic", odomTopic, "some_odom");
    nh.param<std::string>("/node_params/task_topic", taskTopic, "some_task");
    nh.param<std::string>("/node_params/grid_topic", gridTopic, "some_grid");
    nh.param<std::string>("/node_params/path_topic", pathTopic, "some_path");

    nh.param<std::string>("/node_params/search_type", searchType, "dijkstra");

    taskSub                     = nh.subscribe<geometry_msgs::PoseStamped>      (taskTopic,
                                                                                50,
                                                                                &Planner::setTask,
                                                                                this);

    gridSub                     = nh.subscribe<nav_msgs::OccupancyGrid>         (gridTopic,
                                                                                50,
                                                                                &Planner::setGrid,
                                                                                this);

    odomSub                     = nh.subscribe<nav_msgs::Odometry>              (odomTopic,
                                                                                50,
                                                                                &Planner::setOdom,
                                                                                this);

    trajPub                     = nh.advertise<cds_msgs::PathStamped>           (pathTopic,
                                                                                50);


    mission.createSearch(searchType);
}

void Planner::setTask(const geometry_msgs::PoseStamped::ConstPtr& goalMsg) {
    if(gridSet){
        auto transformedPose = transformPoseToTargetFrame(goalMsg->pose, goalMsg->header.frame_id, grid.header.frame_id);
        transformedPose = rescaleToGrid(transformedPose);
        mission.setGoal(int(transformedPose.position.x),
                        int(transformedPose.position.y));
        if(!plan()) ROS_WARN_STREAM("Planning error! Resulted path is empty.");
        publish();
    }else{
        ROS_WARN_STREAM("No grid received! Cannot set task.");
    }
}

void Planner::setGrid(const nav_msgs::OccupancyGrid::ConstPtr& gridMsg) {
    this->grid = *gridMsg;
    if(mission.getMap(gridMsg)) {
        gridSet = true;
    }else {
        ROS_WARN_STREAM("Cannot set map!");
        gridSet = false;
    }
}

void Planner::setOdom(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    this->odom = *odomMsg;
    if(gridSet){
        auto transformedPose = transformPoseToTargetFrame(odomMsg->pose.pose, odomMsg->header.frame_id, grid.header.frame_id);
        transformedPose = rescaleToGrid(transformedPose);
        mission.setGoal(int(transformedPose.position.x),
                        int(transformedPose.position.y));
    }else{
        ROS_WARN_STREAM("No grid received! Cannot set odometry.");
    }
}

bool Planner::plan(){
    auto start = std::chrono::system_clock::now();

    mission.startSearch();

    auto finish = std::chrono::system_clock::now();
    double time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(finish - start).count()) / 1000000000;
    std::cout << "Planning time: " << time << std::endl;

    auto lppath = mission.getLPPath();
    if (lppath.size() != 0){
        fillPath(lppath);
        transformPath();
        return true;
    }
    else{
        return false;
    }
}

void Planner::fillPath(std::list<Node> lppath){
    path.points.clear();
    path.header.frame_id = grid.header.frame_id;
    cds_msgs::Point point;
    for (auto node : lppath){
        point.pose.position.x = node.i;
        point.pose.position.y = node.j;
        path.points.push_back(point);
    }
}

void Planner::transformPath(){
    geometry_msgs::TransformStamped transform;
    std::string pathFrame = path.header.frame_id;
    try {
        transform = tfBuffer.lookupTransform(pathFrame, odom.header.frame_id, ros::Time(0));
    }catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        //return poseIn;
    }
    auto angle = yaw(transform.transform.rotation);
    double angle_orig;
    geometry_msgs::Pose point;
    for(int i=0; i<path.points.size(); ++i) {
        point = path.points[i].pose;
        point = rescaleFromGrid(point);
        point.position.x += grid.info.origin.position.x;
        point.position.y += grid.info.origin.position.y;

        angle_orig = yaw(point.orientation);
        angle_orig -= angle;
        point.orientation.x = 0;
        point.orientation.y = 0;
        point.orientation.z = sin(angle_orig / 2);
        point.orientation.w = cos(angle_orig / 2);

        path.points[i].pose = point;
    }
    //!For some reason it did not work as expected:
//        path.path_with_metadata[i].pose = transformPoseToTargetFrame(point, pathFrame, targetFrame);


    reverse(path.points.begin(), path.points.end());
    path.header.frame_id = odom.header.frame_id;
}

void Planner::publish(){
    path.header.stamp = ros::Time::now();
    trajPub.publish(path);
}

geometry_msgs::Pose Planner::transformPoseToTargetFrame(geometry_msgs::Pose poseIn, std::string poseFrame, std::string targetFrame){
    geometry_msgs::TransformStamped transform;
    try {
        transform = tfBuffer.lookupTransform(poseFrame, targetFrame, ros::Time(0));
    }catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return poseIn;
    }

    geometry_msgs::Pose poseOut;
    poseOut = poseIn;
    poseOut.position.x -= transform.transform.translation.x;
    poseOut.position.y -= transform.transform.translation.y;
    auto angle = yaw(transform.transform.rotation);
    auto new_x = poseOut.position.x*cos(angle) + poseOut.position.y*sin(angle);
    auto new_y = -poseOut.position.x*sin(angle) + poseOut.position.y*cos(angle);
    poseOut.position.x = new_x;
    poseOut.position.y = new_y;


    auto angle_orig = yaw(poseIn.orientation);
    angle_orig -= angle;
    poseOut.orientation.x = 0;
    poseOut.orientation.y = 0;
    poseOut.orientation.z = sin(angle_orig/2);
    poseOut.orientation.w = cos(angle_orig/2);
//
//    tf2::Quaternion q_orig, q_rot, q_new;
//    tf2::convert(poseOut.orientation , q_orig);
//    tf2::convert(transform.transform.rotation, q_rot);
//    q_new = q_rot*q_orig;
//    q_new.normalize();
//    tf2::convert(q_new, poseOut.orientation);

    return poseOut;
}

geometry_msgs::Pose Planner::rescaleToGrid(geometry_msgs::Pose Pose){
    Pose.position.x = int(Pose.position.x / grid.info.resolution);
    Pose.position.y = int(Pose.position.y / grid.info.resolution);
    return Pose;
}

geometry_msgs::Pose Planner::rescaleFromGrid(geometry_msgs::Pose Pose){
    Pose.position.x = (Pose.position.x) * grid.info.resolution;
    Pose.position.y = (Pose.position.y) * grid.info.resolution;
    return Pose;
}



int main(int argc, char **argv){
    ros::init(argc, argv, "planner");
    tf2_ros::Buffer tfBuffer(ros::Duration(5)); //todo: remake with pointers
    tf2_ros::TransformListener tfListener(tfBuffer);


    Planner AStar_Planner(tfBuffer);
    ros::Rate r(5);

    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}
