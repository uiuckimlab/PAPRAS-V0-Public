/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  Copyright (c) 2018, DFKI GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Kazuki Shin, Sankalp Yamsani */

// ROS
#include <ros/ros.h>

// MoveIt!
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Other
#include <sstream>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <vision_msgs/Detection3DArray.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// Gazebo Object Spawn
#include "gazebo_msgs/SpawnModel.h"
#include "geometry_msgs/Pose.h"
#include <tinyxml.h>
#include <sstream>

std::string tf_prefix_ = "robot3/";
constexpr char LOGNAME[] = "moveit_task_constructor_papras";
// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

#define SPAWN_URDF_OBJECT_TOPIC "gazebo/spawn_urdf_model"
#define SPAWN_SDF_OBJECT_TOPIC "gazebo/spawn_sdf_model"

enum state
{
    ST_INIT,
    ST_PAUSED,
    ST_ARM_TO_HOME_START,
    ST_SPAWN_URDF,
    ST_SPAWN_SDF,
    ST_MTC,
    ST_DONE
};

enum ObjectID
{
    CUP = 1,
    KETTLE = 2,
    CARAFE = 3,
    PLATE = 4,
    TABLE = 5,
};

std::string id_to_string(int id)
{
    switch (id)
    {
    case ObjectID::CUP:
        return "cup";
    case ObjectID::KETTLE:
        return "kettle";
    case ObjectID::CARAFE:
        return "carafe";
    case ObjectID::PLATE:
        return "plate";
    case ObjectID::TABLE:
        return "table";
    default:
        std::cerr << "No such ObjectID: " << id << std::endl;
        return "";
    }
}

bool paused = true;
bool failed = false;

int updatePlanningScene(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                        ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &group)
{
    // get objects from object detection
    bool found_cup = false;
    while (!found_cup)
    {
        if (!ros::ok())
            return 0;

        vision_msgs::Detection3DArrayConstPtr detections =
            ros::topic::waitForMessage<vision_msgs::Detection3DArray>("dope/detected_objects", nh, ros::Duration(30.0));
        if (!detections)
        {
            ROS_ERROR("Timed out while waiting for a message on topic detected_objects!");
            return 1;
        }

        // add objects to planning scene
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        for (auto &&det3d : detections->detections)
        {
            if (det3d.results.empty())
            {
                ROS_ERROR("Detections3D message has empty results!");
                return 1;
            }

            if (det3d.results[0].id == ObjectID::CUP)
                found_cup = true;

            moveit_msgs::CollisionObject co;
            co.header = detections->header;
            co.id = id_to_string(det3d.results[0].id);
            co.operation = moveit_msgs::CollisionObject::ADD;
            co.primitives.resize(1);
            co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
            co.primitives[0].dimensions.resize(geometric_shapes::solidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>());
            co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = det3d.bbox.size.x + 10; //0.04;
            co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = det3d.bbox.size.y + 10; //0.04;
            co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = det3d.bbox.size.z + 10; //0.04;
            co.primitive_poses.resize(1);
            co.primitive_poses[0] = det3d.bbox.center;

            collision_objects.push_back(co);
        }

        if (!found_cup)
            ROS_INFO_THROTTLE(1.0, "Still waiting for cup...");

        planning_scene_interface.applyCollisionObjects(collision_objects);
    }

    // detach all objects
    auto attached_objects = planning_scene_interface.getAttachedObjects();
    for (auto &&object : attached_objects)
    {
        group.detachObject(object.first);
    }
    return 0;
}

bool pause_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (!paused)
    {
        ROS_INFO_STREAM("Pause statemachine after current state is completed");
        paused = true;
    }
    return true;
}

bool continue_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    if (paused || failed)
    {
        paused = false;
        failed = false;
        ROS_INFO_STREAM("Continue statemachine");
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_n_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    state task_state = ST_INIT;
    state paused_state = ST_INIT;

    geometry_msgs::Pose base_pick_pose;
    geometry_msgs::Pose base_handover_pose;
    geometry_msgs::Pose base_place_pose;
    geometry_msgs::Pose base_home_pose;
    std::string world_name;
    bool handover_planned;

    // Load rosparams
    ros::NodeHandle nh_priv("~");
    std::string param_path;

    // pause service
    ros::ServiceServer pause_state = nh.advertiseService("pause_statemachine", pause_service);

    // continue service
    ros::ServiceServer continue_state = nh.advertiseService("continue_statemachine", continue_service);

    // GAZEBO SERVICE
    ros::ServiceClient gazebo_spawn_urdf_obj = nh.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_URDF_OBJECT_TOPIC);
    ros::ServiceClient gazebo_spawn_sdf_obj = nh.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_SDF_OBJECT_TOPIC);

    moveit::planning_interface::MoveGroupInterface group("arm3");
    // MOVE IT
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    // PLANNING INTERFACE
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    while (ros::ok())
    {
        if ((paused || failed) && !(task_state == ST_PAUSED))
        {
            ROS_INFO_STREAM("PAUSED in state " << task_state);
            ROS_INFO_STREAM("Call service continue_statemachine to resume");
            paused_state = task_state;
            task_state = ST_PAUSED;
        }
        switch (task_state)
        {
        case ST_INIT:
        {
            ROS_INFO_STREAM("ST_INIT");
            group.setPlanningTime(10.0);
            group.setPlannerId("RRTConnect");

            task_state = ST_ARM_TO_HOME_START;
            break;
        }
        case ST_PAUSED:
        {
            if (!paused && !failed)
            {
                task_state = paused_state;
                ROS_INFO_STREAM("Next state: " << task_state);
            }
            ros::Duration(0.1).sleep();
            break;
        }
        case ST_ARM_TO_HOME_START:
        {
            ROS_INFO_STREAM("ST_ARM_TO_HOME_START");

            /* ******************* MOVE ARM TO HOME ****************************** */
            // plan
            group.setPlannerId("RRTConnect");
            group.setNamedTarget("rest");
            moveit::planning_interface::MoveItErrorCode error_code = group.plan(plan);
            if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("Planning to home pose SUCCESSFUL");
            }
            else
            {
                ROS_ERROR("Planning to home pose FAILED");
                failed = true;
                break;
            }

            // move
            error_code = group.execute(plan);
            if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                ROS_INFO("Moving to home pose SUCCESSFUL");
                task_state = ST_SPAWN_SDF;
            }
            else
            {
                ROS_ERROR("Moving to home pose FAILED");
                failed = true;
            }
            break;
        }
        case ST_SPAWN_URDF:
        {
            ROS_INFO_STREAM("ST_SPAWN_URDF");
            gazebo_spawn_urdf_obj.waitForExistence();

            gazebo_msgs::SpawnModel spawn_model;
            spawn_model.request.model_name = "box1";

            // load urdf file
            std::string urdf_filename = std::string("/opt/catkin_ws/src/PAPRAS/papras_description/urdf/spot_mount.urdf.xacro");
            ROS_INFO("loading file: %s", urdf_filename.c_str());
            // read urdf / gazebo model xml from file
            TiXmlDocument xml_in(urdf_filename);
            xml_in.LoadFile();
            std::ostringstream stream;
            stream << xml_in;
            spawn_model.request.model_xml = stream.str(); // load xml file
            ROS_INFO("XML string: %s", stream.str().c_str());

            spawn_model.request.robot_namespace = "";
            geometry_msgs::Pose pose;
            pose.position.x = pose.position.y = 0;
            pose.position.z = 1;
            pose.orientation.w = 1.0;
            pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
            spawn_model.request.initial_pose = pose;
            spawn_model.request.reference_frame = "world";

            if (gazebo_spawn_urdf_obj.call(spawn_model))
            {
                ROS_INFO("Spawn in simulation SUCCESSFUL");
                task_state = ST_DONE;
                break;
            }
            else
            {
                ROS_ERROR("Failed to spawn model in sim! Error msg:%s", spawn_model.response.status_message.c_str());
                failed = true;
            }
            break;
        }
        case ST_SPAWN_SDF:
        {
            ROS_INFO_STREAM("ST_SPAWN_SDF");
            gazebo_spawn_sdf_obj.waitForExistence();

            gazebo_msgs::SpawnModel spawn_model;
            spawn_model.request.model_name = "box2";
            spawn_model.request.model_xml = "<sdf version ='1.4'>\
                <model name ='cylinder'>\
                    <pose>0.01 -0.2 0.81 0 0 0</pose>\
                    <link name ='cup_link'>\
                    <pose>0.01 -0.2 0.81 0 0 0</pose>\
                    <collision name ='collision'>\
                        <geometry>\
                        <cylinder><length>0.20</length>\
                                    <radius>0.02</radius>\
                        </cylinder>\
                        </geometry>\
                    </collision>\
                    <visual name ='visual'>\
                        <geometry>\
                        <cylinder><length>0.20</length>\
                                    <radius>0.02</radius>\
                        </cylinder>\
                        </geometry>\
                    </visual>\
                    </link>\
                </model>\
                </sdf>";
            ROS_INFO("XML string: %s", spawn_model.request.model_xml.c_str());

            spawn_model.request.robot_namespace = "";
            geometry_msgs::Pose pose;
            pose.position.x = pose.position.y = 0;
            pose.position.z = 1;
            pose.orientation.w = 1.0;
            pose.orientation.x = pose.orientation.y = pose.orientation.z = 0;
            spawn_model.request.initial_pose = pose;
            spawn_model.request.reference_frame = "world";

            if (gazebo_spawn_sdf_obj.call(spawn_model))
            {
                ROS_INFO("Spawn in simulation SUCCESSFUL");
                task_state = ST_DONE;
                break;
            }
            else
            {
                ROS_ERROR("Failed to spawn model in sim! Error msg:%s", spawn_model.response.status_message.c_str());
                failed = true;
            }
            break;
        }
        case ST_MTC:
        {
            ROS_INFO_STREAM("ST_MTC");

            ros::WallDuration(5.0).sleep();
            task_state = ST_DONE;
            break;
        }
        case ST_DONE:
        {
            ROS_INFO_STREAM("ST_DONE");
            return 0;
        }
        default:
        {
            ROS_INFO_STREAM("Unknown state");
            return 1;
        }
        }
    }
    return 1;
}
