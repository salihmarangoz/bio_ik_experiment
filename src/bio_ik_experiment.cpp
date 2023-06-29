#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <bio_ik/bio_ik.h>
#include <visualization_msgs/Marker.h>

using moveit::planning_interface::MoveGroupInterface;
using moveit::core::MoveItErrorCode;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bio_ik_experiment");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //ros::AsyncSpinner spinner(4);
  //spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  auto robot_model = robot_model_loader.getModel();
  auto joint_model_group = robot_model->getJointModelGroup("xarm7");
  auto tip_names = joint_model_group->getSolverInstance()->getTipFrames();

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_model->getModelFrame(),"/moveit_visual_markers"));


  tf2::Vector3 target(0.4, 0.0, 0.5);

  double rate = 5.0;
  ros::Rate r(rate);
  while (ros::ok())
  {
    //kinematics::KinematicsQueryOptions opts;
    bio_ik::BioIKKinematicsQueryOptions opts;
    //opts.replace =  ????
    opts.return_approximate_solution = true; // optional

    // These three goals are for look at goal with min/max distance limits
    auto* lookat_goal = new bio_ik::LookAtGoal();
    lookat_goal->setLinkName("link_eef");
    lookat_goal->setTarget(target);
    lookat_goal->setAxis(tf2::Vector3(0.0, 0.0, 1.0));
    lookat_goal->setWeight(1.0);
    opts.goals.emplace_back(lookat_goal);

    auto max_dist = new bio_ik::MaxDistanceGoal();
    max_dist->setLinkName("link_eef");
    max_dist->setTarget(target);
    max_dist->setDistance(0.2);
    max_dist->setWeight(20.0);
    opts.goals.emplace_back(max_dist);

    auto min_dist = new bio_ik::MinDistanceGoal();
    min_dist->setLinkName("link_eef");
    min_dist->setTarget(target);
    min_dist->setDistance(0.1);
    min_dist->setWeight(20.0);
    opts.goals.emplace_back(min_dist);

    // To minimize the distance between different poses in the long term
    // Also helps with the self-collision
    auto *center_goal = new bio_ik::CenterJointsGoal();
    center_goal->setWeight(0.2);
    opts.goals.emplace_back(center_goal);

    // Balancing the robot using the weight information of the links
    auto *balance_goal = new bio_ik::BalanceGoal();
    balance_goal->setWeight(1.0);
    opts.goals.emplace_back(balance_goal);

    robot_state::RobotState robot_state_ik(robot_model);
    robot_state_ik.setToRandomPositions();                  // SALIH: Setting this to random because we want to disable prior to find different solutions
    bool ok = robot_state_ik.setFromIK(
                joint_model_group,
                EigenSTL::vector_Isometry3d(),
                std::vector<std::string>(),
                1/rate,
                moveit::core::GroupStateValidityCallbackFn(),
                opts
              );
    visual_tools_->publishRobotState(robot_state_ik);

    // Publish marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = robot_model->getModelFrame();
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = target.getX();
    marker.pose.position.y = target.getY();
    marker.pose.position.z = target.getZ();
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);

    ros::spinOnce();
    r.sleep();
  }

  ros::waitForShutdown();
}