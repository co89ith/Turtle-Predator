#include "rclcpp/rclcpp.hpp"
#include <random>
#include <cmath>
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
// #include <vector>
// #include <chrono>
#include "turtlesim/msg/pose.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

class TurtleSpawnerNode : public rclcpp::Node // Modity name
{
    public:
    TurtleSpawnerNode() : Node("turtle_spawner"), gen(rd()), dist1(0.0, 11.0), dist2(0.0, 2*M_PI) //Modify name
    {
        this->declare_parameter("spawn_frequency",0.5);
        this->declare_parameter("turtle_name_prefix", "turtle");

        double spawn_fr = this->get_parameter("spawn_frequency").as_double();
        turle_name_pref = this->get_parameter("turtle_name_prefix").as_string();

        // Publisher 
        publisher_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>("alive_turles",10);
        publisher_timer_ = this->create_wall_timer(std::chrono::milliseconds((int) (1000.0 / spawn_fr)),
                                            std::bind(&TurtleSpawnerNode::TurtleSpawnerGenerator,this));
        RCLCPP_INFO(this->get_logger(), "Turtle Spawner publisher has been started.");

        // Service
        service_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>(
            "catch_turtle", 
            std::bind(
                &TurtleSpawnerNode::callbackCatchTurtle, 
                this, std::placeholders::_1,  std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Turtle catch service is started");



    }

    private:
    // Publisher
    void publish_alive_turle()
    {
        auto message = my_robot_interfaces::msg::TurtleArray();
        message.turtles = alive_turtles_;
        publisher_ ->publish(message);
    }

    void TurtleSpawnerGenerator()
        {
            auto turtle_name = turle_name_pref + std::to_string(counter_++);
            double x = dist1(gen);
            double y = dist1(gen);
            double theta = dist2(gen);

            // Call Spawn Service to spaw the new turtle
            spawn_turtle_threads_.push_back(
                std::make_shared<std::thread>(
                    std::bind(&TurtleSpawnerNode::callSpawnTurtleService, this, x, y, theta)
                )
            );

        }

    void callSpawnTurtleService(double x, double y, double theta){
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), 
                "Waiting for the spawn server to be up...");
            }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;    
        request->theta = theta;

        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            if (response->name != ""){
                my_robot_interfaces::msg::Turtle new_turtle;
                new_turtle.turtle_name = response->name;
                new_turtle.x = x;
                new_turtle.y = y;
                new_turtle.theta = theta;
                alive_turtles_.push_back(new_turtle);
                publish_alive_turle();
                RCLCPP_INFO(this->get_logger(), "%s is alive", response->name.c_str());

            }
        }
        catch (const std::exception &e)
        {   
            RCLCPP_ERROR(this->get_logger(), "Spawn Service Call failed");
        }
    }


    // Service
    void callbackCatchTurtle(const my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr request,
                             const my_robot_interfaces::srv::CatchTurtle::Response::SharedPtr response)
    {
        kill_turtle_threads_.push_back(
            std::make_shared<std::thread>(std::bind(&TurtleSpawnerNode::callkillTurtleService, this, request->name))
        );
        response->success = true;
    }

    void callkillTurtleService(std::string turtle2kill_name){
        auto client = this->create_client<turtlesim::srv::Kill>("kill");
        while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), 
                "Waiting for kill server to be up...");
            }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turtle2kill_name;

        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            for (int i = 0; i < (int)alive_turtles_.size(); i++)
            {
                if (alive_turtles_.at(i).turtle_name == turtle2kill_name)
                {
                    alive_turtles_.erase(alive_turtles_.begin() + i);
                    publish_alive_turle();
                    RCLCPP_INFO(this->get_logger(), "%s is killed", turtle2kill_name.c_str());
                    break;
                }
            }
        }
        catch (const std::exception &e)
        {   
            RCLCPP_ERROR(this->get_logger(), "Kill Service Call failed");
        }
    }


    int counter_ = 0;
    std::string turle_name_pref;
    std::random_device rd;
    std::mt19937 gen;
    std::uniform_int_distribution<> dist1;
    std::uniform_int_distribution<> dist2;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_;
    rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr publisher_timer_;

    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr service_timer_;

    std::vector<std::shared_ptr<std::thread>> spawn_turtle_threads_;
    std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>(); //Modify name
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

