#include <ros/ros.h>
#include <moveit_msgs/GraspPlanning.h>
#include <moveit_msgs/CollisionObject.h>
#include <mutex>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>

// MoveIt!
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// Grasp 
#include <moveit_grasps/two_finger_grasp_generator.h>
#include <moveit_grasps/two_finger_grasp_data.h>
#include <moveit_grasps/two_finger_grasp_filter.h>
#include <moveit_grasps/grasp_planner.h>
#include <moveit_grasps/grasp_generator.h>

class PlanDOPEGrasp
{
public:
  PlanDOPEGrasp(ros::NodeHandle &n)
  { 
    ros::NodeHandle nh("~");

    nh.param("ee_group_name", ee_group_name_, std::string("hand1"));
    ROS_INFO_STREAM_NAMED("pick place", "End Effector: " << ee_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>("world");
    visual_tools_->setMarkerTopic("/rviz_visual_tools");
    visual_tools_->loadMarkerPub();
    visual_tools_->loadRobotStatePub("/display_robot_state");
    visual_tools_->loadTrajectoryPub("/display_planned_path");
    visual_tools_->loadSharedRobotState();
    visual_tools_->getSharedRobotState()->setToDefaultValues();
    visual_tools_->enableBatchPublishing();
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    visual_tools_->hideRobot();
    visual_tools_->trigger();

    grasp_visuals_ = std::make_shared<rviz_visual_tools::RvizVisualTools>("world");
    grasp_visuals_->setMarkerTopic("/grasp_visuals");
    grasp_visuals_->loadMarkerPub();
    grasp_visuals_->enableBatchPublishing();
    grasp_visuals_->deleteAllMarkers();
    grasp_visuals_->trigger();

    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    grasp_data_ =
        std::make_shared<moveit_grasps::TwoFingerGraspData>(nh, ee_group_name_, visual_tools_->getRobotModel());
    if (!grasp_data_->loadGraspData(nh, ee_group_name_))
    {
      ROS_ERROR_STREAM("Failed to load Grasp Data parameters.");
      exit(-1);
    }

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    grasp_generator_ = std::make_shared<moveit_grasps::TwoFingerGraspGenerator>(visual_tools_, true);
    grasp_generator_->setVerbose(true);
  }

  void generate_grasps()
  {
    // --- calculate grasps
    // this is using standard frame orientation: x forward, y left, z up, relative to object bounding box center
    // ---------------------------------------------------------------------------------------------
    // Generate grasps for objects
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
    bool success = planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
    const moveit::core::JointModelGroup* arm_jmg = visual_tools_->getRobotModel()->getJointModelGroup("arm1");
    const moveit::core::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup("gripper1");

    // get table height
    std::vector<std::string> object_ids;
    object_ids.push_back("bowl");
    geometry_msgs::Pose object_pose = planning_scene_interface.getObjectPoses({object_ids}).at("bowl");
    ROS_INFO_STREAM("abetSoup pose in pick(): " << object_pose.position.x << ", " 
                                              << object_pose.position.y << ", " 
                                              << object_pose.position.z);

    // Generate set of grasps for one object
    double depth = 0.13;  
    double width = 0.13;  //y is height
    double height = 0.06;
    // 16.403600692749023,21.343700408935547,7.179999828338623
    // 8.3555002212524414, 7.1121001243591309, 6.6055998802185059

    object_pose.position.z += 0.05;
    object_pose.orientation.x = 0;
    object_pose.orientation.y = 0;
    object_pose.orientation.z = 0;
    object_pose.orientation.w = 1;

    grasp_visuals_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);
    grasp_visuals_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);
    grasp_visuals_->trigger();

    std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
    grasp_candidates.clear();

    // Configure the desired types of grasps
    moveit_grasps::TwoFingerGraspCandidateConfig grasp_generator_config =
        moveit_grasps::TwoFingerGraspCandidateConfig();
    grasp_generator_config.disableAll();
    grasp_generator_config.enable_face_grasps_ = true;
    grasp_generator_config.enable_edge_grasps_ = true;
    grasp_generator_config.generate_x_axis_grasps_ = false;
    grasp_generator_config.generate_y_axis_grasps_ = false;
    grasp_generator_config.generate_z_axis_grasps_ = true;

    // ---------------------------------------------------------------------------------------------
    // Set the ideal grasp pose to be centered and 0.5m above (for visualization) with an orientation of roll = 3.14

    // We set custom grasp score weights
    moveit_grasps::TwoFingerGraspScoreWeightsPtr grasp_score_weights =
        std::make_shared<moveit_grasps::TwoFingerGraspScoreWeights>();
    grasp_score_weights->orientation_x_score_weight_ = 2.0;
    grasp_score_weights->orientation_y_score_weight_ = 2.0;
    grasp_score_weights->orientation_z_score_weight_ = 2.0;
    grasp_score_weights->translation_x_score_weight_ = 1.0;
    grasp_score_weights->translation_y_score_weight_ = 1.0;
    grasp_score_weights->translation_z_score_weight_ = 1.0;
    // Finger gripper specific weights. (Note that we do not need to set the suction gripper specific weights for our
    // finger gripper)
    grasp_score_weights->depth_score_weight_ = 2.0;
    grasp_score_weights->width_score_weight_ = 2.0;
    grasp_generator_->setGraspScoreWeights(grasp_score_weights);

    // Publish world coordinate system
    grasp_visuals_->publishAxisLabeled(Eigen::Isometry3d::Identity(), "world frame");
    visual_tools_->trigger();

    grasp_generator_->setGraspCandidateConfig(grasp_generator_config);
    grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), depth, width, height, grasp_data_,
                                      grasp_candidates);

    // Filter the grasp for only the ones that are reachable
    ROS_INFO_STREAM_NAMED("test", "Filtering grasps kinematically");
    bool filter_pregrasps = true;

    moveit_grasps::TwoFingerGraspFilterPtr grasp_filter_ = std::make_shared<moveit_grasps::TwoFingerGraspFilter>(visual_tools_->getSharedRobotState(), visual_tools_); 
    std::size_t valid_grasps = grasp_filter_->filterGrasps(grasp_candidates, planning_scene_monitor_, arm_jmg,
                                                        visual_tools_->getSharedRobotState(), filter_pregrasps, "bowl");


    for(auto grasp_candidate: grasp_candidates)
    {
        ros::Time grasp_stamp = grasp_candidate->grasp_.pre_grasp_posture.header.stamp;
        if (grasp_candidate->isValid())
          grasp_candidates_.push_front(std::make_pair(grasp_candidate->grasp_, grasp_stamp));
    }
  }

  bool plan_dope_grasp(moveit_msgs::GraspPlanning::Request &req, moveit_msgs::GraspPlanning::Response &res)
  {
    ROS_INFO("<<<<< IN SERVICE CALL : plan_dope_grasp >>>>>");
    std::lock_guard<std::mutex> lock(m_);
    generate_grasps();
 
    {
      for (auto grasp_candidate:grasp_candidates_)
      {
        res.grasps.push_back(grasp_candidate.first);
      }
      grasp_candidates_.clear();
    }

    if(res.grasps.empty())
    {
      ROS_INFO("No valid grasp found.");
      res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    else
    {
      ROS_INFO("%ld grasps found.", res.grasps.size());
      res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }

    return true;
  }

private:
  std::deque<std::pair<moveit_msgs::Grasp, ros::Time>> grasp_candidates_;
  std::mutex m_;

  // Grasp generator
  moveit_grasps::TwoFingerGraspGeneratorPtr grasp_generator_;
  // Robot-specific data for generating grasps
  moveit_grasps::TwoFingerGraspDataPtr grasp_data_;
  // Tool for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  rviz_visual_tools::RvizVisualToolsPtr grasp_visuals_;
  
  // Which arm should be used
  std::string ee_group_name_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_grasps_service");
  ros::NodeHandle n;
  PlanDOPEGrasp p_dope_g(n);

  ros::ServiceServer ss = n.advertiseService("plan_grasps", &PlanDOPEGrasp::plan_dope_grasp, &p_dope_g);
  ros::spin();

  return 0;
}
