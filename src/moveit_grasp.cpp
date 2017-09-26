#include <iostream>

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_simple_grasps/simple_grasps.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <agile_grasp/Grasp.h>
#include <agile_grasp/Grasps.h>

#define ARM_PLANNING_GROUP "arm"
#define HEAD_PLANNING_GROUP "head"
#define BASE_PLANNING_GROUP "base"
#define WHOLE_BODY_PLANNING_GROUP "whole_body_weighted"

// Not using this one
// https://github.com/atenpas/grasp_selection - grasp selection Baxter
// Calculates own Inverse Kinematics

// Not using this one
// https://github.com/atenpas/grasp_selection/blob/master/src/grasp_selection/reaching.cpp

class Manipulation {
	public:
		Manipulation() : move_whole(WHOLE_BODY_PLANNING_GROUP), move_arm(ARM_PLANNING_GROUP) {}
		~Manipulation() {}

		void init() {
            nh_ = ros::NodeHandle("~");
            int queueSize = 5;

            if (!grasp_data_.loadRobotGraspData(nh_, "gripper")) {
            	ros::shutdown();
            }

            // https://github.com/ros-planning/moveit_tutorials/blob/indigo-devel/doc/pr2_tutorials/planning/src/move_group_interface_tutorial.cpp

            display_publisher_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

            grasp_sub_ = nh_.subscribe ("/find_grasps/grasps", 
                queueSize, &Manipulation::graspCallback, this);
		}

		void graspCallback(const agile_grasp::Grasps& grasps) {
			grasps_ = grasps;	
			has_grasps_ = true;
		}

		void resetArm() {
			std::map< std::string, double > joint_values;
			joint_values["arm_lift_joint"] = 0.0;
			joint_values["arm_flex_joint"] = 0.0;
			joint_values["arm_roll_joint"] = 0.0;
			joint_values["wrist_flex_joint"] = -1.57;
			joint_values["wrist_roll_joint"] = 0.0;
			move_arm.setJointValueTarget(joint_values);
			move_arm.move();
			ROS_INFO("Pick: set initial pose");
		}

		bool pickUpObject() {
			if (!has_grasps_) {
				ROS_INFO("No grasps available");
				return false;
			}

			agile_grasp::Grasp grasp_ = grasps_.grasps[0];

			// Get this from agile graps grasps_
			geometry_msgs::Pose target_pose;
			//target_pose.orientation = grasp_.approach;
			target_pose.orientation.w = 1.0;
			target_pose.orientation.x = grasp_.approach.x;
			target_pose.orientation.y = grasp_.approach.y;
			target_pose.orientation.z = grasp_.approach.z;
			target_pose.position.x = grasp_.center.x;
			target_pose.position.y = grasp_.center.y;
			target_pose.position.z = grasp_.center.z;
			move_arm.setPoseTarget(target_pose);

			moveit::planning_interface::MoveGroup::Plan my_plan;
			bool success = move_arm.plan(my_plan);

			ROS_INFO("Visualizing plan (pose goal) %s",success?"":"FAILED");    
  			
  			/* Sleep to give Rviz time to visualize the plan. */
  			sleep(5.0);

  			// Visualising plan

  			if (1)
  			{
  				ROS_INFO("Visualizing plan");    
  				display_trajectory_.trajectory_start = my_plan.start_state_;
  				display_trajectory_.trajectory.push_back(my_plan.trajectory_);
  				display_publisher_.publish(display_trajectory_);
    			/* Sleep to give Rviz time to visualize the plan. */
  				sleep(5.0);
  			}
 
  			// Move
  			//move_arm.move();	

  			/*
  			// With objects

  			moveit_msgs::CollisionObject collision_object;
  			collision_object.header.frame_id = move_arm.getPlanningFrame();

  			collision_object.id = "Table";

  			// Define a box

  			shape_msgs::SolidPrimitive primitive;
  			primitive.type = primitive.BOX;
  			primitive.dimensions.resize(3);
  			primitive.dimensions[0] = 0.4;
  			primitive.dimensions[1] = 0.1;
  			primitive.dimensions[2] = 0.4;

  			// Pose for box

  			geometry_msgs::Pose box_pose;
  			box_pose.orientation.w = 1.0;
  			box_pose.position.x =  0.6;
  			box_pose.position.y = -0.4;
  			box_pose.position.z =  1.2;

  			collision_object.primitives.push_back(primitive);
  			collision_object.primitive_poses.push_back(box_pose);
  			collision_object.operation = collision_object.ADD;

  			std::vector<moveit_msgs::CollisionObject> collision_objects;  
  			collision_objects.push_back(collision_object);  

  			ROS_INFO("Add an object into the world");  
  			planning_scene_interface_.addCollisionObjects(collision_objects);
  
  			// Sleep so we have time to see the object in RViz
  			sleep(2.0);

		  	// Planning with collision detection can be slow.  Lets set the planning time
		  	// to be sure the planner has enough time to plan around the box.  10 seconds
		  	// should be plenty.
  			move_arm.setPlanningTime(10.0);

  			// Now when we plan a trajectory it will avoid the obstacle
  
  			move_arm.setStartState(*move_arm.getCurrentState());
  			move_arm.setPoseTarget(target_pose);
  			success = move_arm.plan(my_plan);

			// Now, let's attach the collision object to the robot.
  			ROS_INFO("Attach the object to the robot");  
  			move_arm.attachObject(collision_object.id);  
  			sleep(4.0);

  			// Now, let's detach the collision object from the robot.
  			ROS_INFO("Detach the object from the robot");  
  			move_arm.detachObject(collision_object.id);  
  			sleep(4.0);

  			// Now, let's remove the collision object from the world.
  			ROS_INFO("Remove the object from the world");  
  			std::vector<std::string> object_ids;
  			object_ids.push_back(collision_object.id);  
  			planning_scene_interface_.removeCollisionObjects(object_ids);
  			sleep(4.0);
  			*/

			return success;
		}

	private:
        ros::NodeHandle nh_;

  		// robot-specific data for generating grasps
		moveit_simple_grasps::GraspData grasp_data_;

		moveit::planning_interface::MoveGroup move_whole;
		moveit::planning_interface::MoveGroup move_arm;

		ros::Publisher display_publisher_;
		moveit_msgs::DisplayTrajectory display_trajectory_;

		moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

		ros::Subscriber grasp_sub_;

		agile_grasp::Grasps grasps_;
		bool has_grasps_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_grasp");
    ROS_INFO("Node started");

    Manipulation manip;
    manip.init();

    std::cout << "Enter command: ";
    std::string comm;

    while (getline(std::cin, comm)) {
    	if (comm == "r") {
    		manip.resetArm();
    	}
    	else if (comm == "p") {
    		manip.pickUpObject();
    	}
    	std::cout << "Enter command:";
    }

    ros::spin();
    ROS_INFO("Shutting down");

    ros::shutdown();
    return 0;
}