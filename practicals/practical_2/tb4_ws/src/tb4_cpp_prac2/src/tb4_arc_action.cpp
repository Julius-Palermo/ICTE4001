#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/action/drive_arc.hpp"
#include "nav_msgs/msg/odometry.hpp"


class TB4ArcActionServer : public rclcpp::Node
{
public:
  using Drive_Arc= irobot_create_msgs::action::DriveArc;

  explicit TB4ArcActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  :Node("tb4_arc_action_server", options)
  {
    /*TODO  TASK - MILESTONE #2.2 Initialise the command velocity publisher share pointer*/

    /*TODO  TASK - MILESTONE #2.3 Initialise the odometry subsriber share pointer, and bing the call back function
      "odom_callback"
    */

    /*TODO  TASK - MILESTONE #2.4
      Initialsie the drive arc action server with name as "drive_arc_prac2", and bind call back functions for
      handling of accepting a goal, cancelling a action, and process the accepted goal
    */

    // Publisher of topic /cmd_vel
        this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::SystemDefaultsQoS());
    using namespace std::placeholders;

    // Subscribe to the /odom topic
    this->odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom",
    rclcpp::SensorDataQoS(),
    std::bind(&TB4ArcActionServer::odom_callback, this, std::placeholders::_1)
    );

    // Action server with name as "drive_distance_prac1", and bind callback functions
    this->action_server_ = rclcpp_action::create_server<Drive_Arc>(
        this,
        "drive_arc_prac2",
        std::bind(&TB4ArcActionServer::handle_goal, this, _1, _2),
        std::bind(&TB4ArcActionServer::handle_cancel, this, _1),
        std::bind(&TB4ArcActionServer::handle_accepted, this, _1));

  }
private:
  /* TODO TASK - MILESTONE #2.1
  Define shared pointers for 
    - action server for drive arc defined in irobot_create_msgs, 
    - command velocity publisher
    - odometry subscriber
  */
  // Define drive arc server
  rclcpp_action::Server<Drive_Arc>::SharedPtr action_server_;
  // Define a command velocity publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // Define odometry subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  // odometry pointer
  nav_msgs::msg::Odometry::SharedPtr odom_;

  // odometry subscriber callback
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  // Callback function for handling goals
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const irobot_create_msgs::action::DriveArc::Goal> goal
  );

  // Callback function for handling cancellation:
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle);

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle);

  // Action processing and update
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle);
};


void TB4ArcActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  /*TODO TASK - MILESTONE #3.1
    save the odom_msg to the class member variable odom_   
  */
  odom_ = odom_msg;
  //odom_recieved = true;
}

/*TODO TASK - MILESTONE #4.1
  complete the  call back function of "TB4ArcActionServer::handle_accepted" that handling the accepted goal 
*/
void TB4ArcActionServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&TB4ArcActionServer::execute, this, _1), goal_handle}.detach();
}
/* TODO TASK - MILESTONE #4.2
  complete the  call back function of "TB4ArcActionServer::handle_cancel" that cancel the goal 
  */
rclcpp_action::CancelResponse TB4ArcActionServer::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}
/*TODO TASK - MILESTONE #4.3
  complete the  call back function of "TB4ArcActionServer::handle_goal" that accept goal, 
  you should also print the goal details in the terminal 
*/
rclcpp_action::GoalResponse TB4ArcActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const irobot_create_msgs::action::DriveArc::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), 
    "Received goal request with translate_direction at %d, angle at %f radians, radius %f m, max speed %f m/s", 
    goal->translate_direction,
    goal->angle,
    goal->radius,
    goal->max_translation_speed);
  (void)uuid;
  if (goal ->radius < 0 || goal->max_translation_speed < 0) // reject goal if negative values used
  {
    RCLCPP_INFO(this->get_logger(), "goal rejected cannot be negative");
    RCLCPP_WARN(this->get_logger(), "goal rejected cannot be negative");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
/* TODO TASKS - MILESTONE #5.1 ~ #5.3
  complete the  thread function "execute" to proccess the goal in the action request
*/
void TB4ArcActionServer::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<Drive_Arc>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Drive_Arc::Feedback>();
  auto & remaining_angle_travel = feedback->remaining_angle_travel;
  auto result = std::make_shared<Drive_Arc::Result>();

  // 5.1: build the velocity command from the goal
  double linear_speed = goal->max_translation_speed;
  double angular_speed = linear_speed / goal->radius;
  if (goal->translate_direction < 0) {
    angular_speed = -angular_speed;
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed;
  cmd_vel.angular.z = angular_speed;

  int pub_freq = 100;
  rclcpp::Rate loop_rate(pub_freq);

  //5.2: figure out how many cycles the arc will take
  double arc_length = std::fabs(goal->angle) * goal->radius;
  int count = static_cast<int>(pub_freq * arc_length / goal->max_translation_speed);

  geometry_msgs::msg::PoseStamped pose_stamped;

  for (int i = 0; (i < count) && rclcpp::ok(); ++i) {
    while(!odom_){
      // RCLCPP_INFO(this->get_logger(), "waiting for odom");
    }
  // if (odom_recieved == true){
    pose_stamped.header = odom_->header;
    pose_stamped.pose = odom_->pose.pose;
  // }else{
  //   cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
  //   result->pose = pose_stamped;
  //   goal_handle->canceled(result);
  //   RCLCPP_INFO(this->get_logger(), "Goal canceled");
  //   return;

  // }
  // odom_recieved = false;

    if (goal_handle->is_canceling()) {
      cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
      pose_stamped.header = odom_->header;
      pose_stamped.pose = odom_->pose.pose;
      result->pose = pose_stamped;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    //5.3: publish feedback
    remaining_angle_travel = std::fabs(goal->angle) - std::fabs(angular_speed) * i / pub_freq;

    cmd_vel_publisher_->publish(cmd_vel);
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  //stop the robot once the arc is complete
  cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());

  if (rclcpp::ok()) {
    pose_stamped.header = odom_->header;
    pose_stamped.pose = odom_->pose.pose;
    result->pose = pose_stamped;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TB4ArcActionServer>());
	rclcpp::shutdown();
	return 0;
}