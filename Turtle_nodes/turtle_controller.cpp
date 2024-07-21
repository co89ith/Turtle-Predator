#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include <random>
#include <cmath>
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
// #include <vector>
// #include <chrono>
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

class TurtleControllerNode : public rclcpp::Node // Modity name
{
    public:
        TurtleControllerNode() : Node("turtle_controller"), name_("turtle1") //Modify name
        {
            this->declare_parameter("catch_closest_turtle_first", true);
            catch_closest_turtle_first_ = this->get_parameter("catch_closest_turtle_first").as_bool();

            this->declare_parameter("Kp_angle", 5.0);
            Kp_ang = this->get_parameter("Kp_angle").as_double();

            this->declare_parameter("Kp_x", 2.0);
            Kp_x = this->get_parameter("Kp_x").as_double();


            //Subscribe to alive_turles topic
            alive_turtle_subscriber_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>(
                    "alive_turles", 10, 
                    std::bind(&TurtleControllerNode::callbackAliveTurtle,this, 
                    std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Turtle Subscriber has been started.");

            // Subscribe to /pose topic
            pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
                    name_ + "/pose", 10, 
                    std::bind(&TurtleControllerNode::callbackPose,this, 
                    std::placeholders::_1));

            // Publish the /cmd_vel topic
            turtle1_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>
                (name_ + "/cmd_vel", 10);

            // Run motion control
            control_loop_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10), std::bind(&TurtleControllerNode::MotionControl, this)
            );
        }

    private:
        // Distance Calculation
        double getDistance(my_robot_interfaces::msg::Turtle turtle)
        {
            double lx = turtle.x - pose_.x;
            double ly = turtle.y - pose_.y;
            return std::sqrt(lx*lx + ly*ly);

        }

        // Subscribe the alive turtles' list, then find the target_turtle
        void callbackAliveTurtle(const my_robot_interfaces::msg::TurtleArray::SharedPtr msg)
            {
                if (catch_closest_turtle_first_)
                {
                    if (!msg->turtles.empty())
                    {
                        my_robot_interfaces::msg::Turtle closest_turtle = msg->turtles.at(0);
                         closest_turtle_dist = getDistance(closest_turtle);

                        // Update the closesy turtle
                        for (int i = 1; i < (int)msg->turtles.size(); i++)
                        {
                            double dist = getDistance(msg->turtles.at(i));
                            if (dist < closest_turtle_dist)
                            {
                                closest_turtle_dist = dist;
                                closest_turtle = msg->turtles.at(i);
                            }
                        }
                        target_turtle = closest_turtle;
                    }
                }
                else
                {
                    target_turtle = msg->turtles.at(0);
                }

                // RCLCPP_INFO(this->get_logger(), "%s is targeted, at [ %f, %f ], distance = %f", target_turtle.turtle_name.c_str(), target_turtle.x, target_turtle.y, closest_turtle_dist);
        
            }

        // Subscribe the turtle 1 pose
        void callbackPose(const turtlesim::msg::Pose::SharedPtr pose)
            {
                pose_ = *pose.get();
                turtlesim_up_ = true;
            }

        // 
        // void PublishT1CmdVel(double dx, double dtheta)
        // {
        //     auto msg = geometry_msgs::msg::Twist();
        //     msg.linear.x = dx;
        //     msg.angular.z = dtheta;
        //     turtle1_vel_publisher_ ->publish(msg);
        // }

        //  motion control 
        void MotionControl()
        {
            if (!turtlesim_up_ || target_turtle.turtle_name == "")
            {
                return;
            }

            double lx = target_turtle.x - pose_.x;
            double ly = target_turtle.y - pose_.y;
            dist = std::sqrt(lx*lx + ly*ly);

            RCLCPP_INFO(this->get_logger(), "%s at [ %f, %f], turtle0 at [%f, %f]", 
                target_turtle.turtle_name.c_str(), target_turtle.x, target_turtle.y, pose_.x, pose_.y);

            auto msg = geometry_msgs::msg::Twist(); // velocity message

            if (dist > 0.5)
            {
                // Transation 
                msg.linear.x = Kp_x*dist;

                //Rotation 
                double angle = std::atan2(ly, lx);
                double angle_diff = angle - pose_.theta;

                if (angle_diff > M_PI)
                {
                    angle_diff -= 2*M_PI;
                }
                else if (angle_diff < -M_PI)
                {
                    angle_diff += 2*M_PI;
                }
                msg.angular.z = Kp_ang*angle_diff; // P control 
            }
            else // Reach the target
            {
                //Stop turtle
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;

                // call kill service
                RCLCPP_INFO(this->get_logger(), "%s is killed, distance = %f", target_turtle.turtle_name.c_str(), dist);

                catch_turtle_threads.push_back(
                    std::make_shared<std::thread>(
                        std::bind(&TurtleControllerNode::callCatchTurleService, this, target_turtle.turtle_name)
                    ));

                // clear the target
                target_turtle.turtle_name = "";
            }

            // Publish the turtle 1 cmd_vel
            turtle1_vel_publisher_->publish(msg);
        }

        // Call Catch Turtle service
        void callCatchTurleService(std::string turtle_name)
        {
            auto client = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");
            while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), 
                "Waiting for the CatchTurtle server to be up...");
            }

            auto request = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();
            request->name = turtle_name;

            auto future = client->async_send_request(request);
            try
            {
                auto response = future.get();
                if (!response->success){
                    RCLCPP_INFO(this->get_logger(), "failed to catch turtle");
                }
            }
            catch (const std::exception &e)
            {   
                RCLCPP_ERROR(this->get_logger(), "CatchTurtle Service Call failed");
            }

        }

    std::string name_;
    turtlesim::msg::Pose pose_;
    bool turtlesim_up_;
    double Kp_ang;
    double Kp_x;
    double closest_turtle_dist;
    double dist ;
    my_robot_interfaces::msg::Turtle target_turtle;
    bool catch_closest_turtle_first_;

    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtle_subscriber_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle1_vel_publisher_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    std::vector<std::shared_ptr<std::thread>> catch_turtle_threads;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>(); //Modify name
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}