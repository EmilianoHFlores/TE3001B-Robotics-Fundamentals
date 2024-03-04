#include <functional>
#include <memory>
#include <thread>

#include "xarm_as_interfaces/action/move_arm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


#include "xarm_as/visibility_control.h"

//moveit2
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace xarm_as
{
class MoveArmActionServer : public rclcpp::Node
{
public:
    using MoveArm = xarm_as_interfaces::action::MoveArm;
    using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;
   
  XARM_AS_PUBLIC
  explicit MoveArmActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_arm_action_server", options)
  { 
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<MoveArm>(
      this,
      "move_arm",
      std::bind(&MoveArmActionServer::handle_goal, this, _1, _2),
      std::bind(&MoveArmActionServer::handle_cancel, this, _1),
      std::bind(&MoveArmActionServer::handle_accepted, this, _1));
    
  }

private:
    rclcpp_action::Server<MoveArm>::SharedPtr action_server_;
    move_group = moveit::planning_interface::MoveGroupInterface(this, "xarm6");
    planning_scene_interface = moveit::planning_interface::PlanningSceneInterface();
    
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveArm::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with target %s", goal->target.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveArm> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&MoveArmActionServer::execute, this, _1), goal_handle}.detach();
    }
    
    void execute(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        move_group = moveit::planning_interface::MoveGroupInterface(this, "xarm6");
        planning_scene_interface = moveit::planning_interface::PlanningSceneInterface();
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        const auto goal_pose = goal->target_pose;
        const auto goal_state = goal->target_state;
        const auto goal_joints = goal->target_joints;
        const auto planning_time = goal->planning_time;
        const auto num_planning_attempts = goal->num_planning_attempts;
        const auto velocity = goal->velocity;
        const auto acceleration = goal->acceleration;
        const auto positionTolerance = goal->position_tolerance;
        const auto orientationTolerance = goal->orientation_tolerance;

        auto feedback = std::make_shared<MoveArm::Feedback>();
        auto result = std::make_shared<MoveArm::Result>();
    
        
        move_group.setPlanningTime(planning_time);
        move_group.setNumPlanningAttempts(num_planning_attempts);
        move_group.setMaxVelocityScalingFactor(velocity);
        move_group.setMaxAccelerationScalingFactor(acceleration);
        move_group.setGoalPositionTolerance(positionTolerance);
        move_group.setGoalOrientationTolerance(orientationTolerance);
        move_group.setPlanningTime(planning_time);
        move_group.setNumPlanningAttempts(num_planning_attempts);

        if (goal_state != "")
        {
            move_group.setNamedTarget(goal_state);
        }
        else if (goal_pose.position.x != 0.0 || goal_pose.position.y != 0.0 || goal_pose.position.z != 0.0)
        {
            geometry_msgs::Pose target_pose;
            target_pose.position.x = goal_pose.position.x;
            target_pose.position.y = goal_pose.position.y;
            target_pose.position.z = goal_pose.position.z;
            target_pose.orientation.x = goal_pose.orientation.x;
            target_pose.orientation.y = goal_pose.orientation.y;
            target_pose.orientation.z = goal_pose.orientation.z;
            target_pose.orientation.w = goal_pose.orientation.w;
            move_group.setPoseTarget(target_pose);
        }
        else if (goal_joints.size() > 0)
        {
            move_group.setJointValueTarget(goal_joints);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No target specified");
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan movePlan;
        
        RCLCPP_INFO(this->get_logger(), "Planning...");
        bool success = (move_group.plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(this->get_logger(), "Plan result: %d", success);
        if(success)
        {
            RCLCPP_INFO(this->get_logger(), "Executing...");
            move_group.execute(movePlan);
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        else
        {
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_INFO(this->get_logger(), "Goal aborted");
        }
    }

    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

}; // class MoveArmActionServer
}  // namespace xarm_as

RCLCPP_COMPONENTS_REGISTER_NODE(xarm_as::MoveArmActionServer)



/*#include <functional>
#include <memory>
#include <thread>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

namespace action_tutorials_cpp
{
class FibonacciActionServer : public rclcpp::Node
{
public:
  using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class FibonacciActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)*/