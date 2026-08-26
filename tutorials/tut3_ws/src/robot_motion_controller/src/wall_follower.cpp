#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#define PI 3.14159265358

class WallFollower : public rclcpp::Node
{
public:
    WallFollower(): Node("wall_follower")
    {
        auto wall_side_desc = rcl_interfaces::msg::ParameterDescriptor{};
        wall_side_desc.description = "A positive value indicates that the wall will be on the left side of the robot, otherwise on the right";
        auto buffer_zone_desc = rcl_interfaces::msg::ParameterDescriptor{};
        buffer_zone_desc.description = "A positive value used to determine whether the tracking control is on or off";
        // Declare parameters
        this->declare_parameter<float>("following_distance", 0.7);
        this->declare_parameter<int8_t>("wall_side", 1, wall_side_desc);
        this->declare_parameter<float>("buffer_zone", 0.4, buffer_zone_desc);
        this->declare_parameter<float>("forward_velocity", 0.4);
        this->declare_parameter<float>("angle_control_gain_1", 1.0);
        this->declare_parameter<float>("angle_control_gain_2", 1.0);
        this->declare_parameter<float>("distance_control_gain", 0.5);
        // Get parameter values
        this->get_parameter("following_distance", following_distance_);
        this->get_parameter("wall_side", wall_side_);
        this->get_parameter("buffer_zone", buffer_zone_);
        this->get_parameter("forward_velocity", forward_velocity_);
        this->get_parameter("angle_control_gain_1", angle_control_gain_1_);
        this->get_parameter("angle_control_gain_2", angle_control_gain_2_);
        this->get_parameter("distance_control_gain", distance_control_gain_);
        // Print parameter values
        RCLCPP_INFO(this->get_logger(), "following_distance: %.2f", following_distance_);
        RCLCPP_INFO(this->get_logger(), "wall_side: %d", wall_side_);
        RCLCPP_INFO(this->get_logger(), "buffer_zone: %.2f", buffer_zone_);
        RCLCPP_INFO(this->get_logger(), "forward_velocity: %.2f", forward_velocity_);
        RCLCPP_INFO(this->get_logger(), "angle_control_gain_1: %.2f", angle_control_gain_1_);
        RCLCPP_INFO(this->get_logger(), "angle_control_gain_2: %.2f", angle_control_gain_2_);
        RCLCPP_INFO(this->get_logger(), "distance_control_gain: %.2f", distance_control_gain_);

        if(wall_side_>0)
            following_angle_ = PI/2;
        else
            following_angle_ = -PI/2;

        // Creating cmd_vel publisher
        this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            rclcpp::SystemDefaultsQoS());

        // Creating scan subscriber
        using namespace std::placeholders;
        this->scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&WallFollower::scan_callback, this, _1)
        );
    }
private:
    float following_distance_;
    int8_t wall_side_;
    float buffer_zone_;
    float following_angle_;
    float forward_velocity_;
    float angle_control_gain_1_;
    float angle_control_gain_2_;
    float distance_control_gain_;

    // Define a command velocity publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    // Define a laser scan subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
};

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    // Finds the smallest element in the range, and return the iterator to the smallest element
    auto min_distance = std::min_element(scan_msg->ranges.begin(), scan_msg->ranges.end());
    // Get the value of the smallest element
    float min_value = *min_distance;
    // Returns the number of hops from the begin to the iterator of the smallest element.
    int min_index = std::distance(scan_msg->ranges.begin(), min_distance);
    // Use the index to calculate the angle where the smallest range is measured.
    float min_angle = (min_index - 320)*2*PI/640.0;

    geometry_msgs::msg::Twist cmd_vel_msg;

    /*
    The magic number 12 is from the simulation setup, i.e., the range of the lidar sensor is from
    0.164 to 12.
    If min_value<12, the lidar sensor has a valid measurement.
    */
   if(min_value<12)
   {
        /* The robot is moving towards to the closed target at speed of forward_velocity_*/
        if(min_value>(following_distance_+buffer_zone_))
        {
            if(abs(min_angle)>PI/4.0)
            {
                if(min_angle>PI/4.0)
                    cmd_vel_msg.angular.z = 1.0;
                else
                    cmd_vel_msg.angular.z = -1.0;
            }
            else
            {
                cmd_vel_msg.angular.z = 0;
                cmd_vel_msg.linear.x = forward_velocity_;
            }
        }
        else
        {
            if(wall_side_>0)
                cmd_vel_msg.angular.z = angle_control_gain_1_*(min_angle - following_angle_) + angle_control_gain_2_*(min_value - following_distance_);
            else
                cmd_vel_msg.angular.z = angle_control_gain_1_*(min_angle - following_angle_) - angle_control_gain_2_*(min_value - following_distance_);
            
            cmd_vel_msg.linear.x = forward_velocity_ + distance_control_gain_*(min_value - following_distance_);
        }
   }
   else // No valid measurement
   {
        RCLCPP_INFO(this->get_logger(), "No Object is Detected");
        cmd_vel_msg.linear.x = 0.2;
   }
   // publish command velocity
   cmd_vel_publisher_->publish(cmd_vel_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollower>());
    rclcpp::shutdown();
    return 0;
}