#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "robot_position_interface/action/robot_position.hpp"
#include "robot_position_interface/msg/robot_position_cancel.hpp"
#include <exception>

using RobotPositionGoalHandle = rclcpp_action::ClientGoalHandle
                                    <robot_position_interface::action::RobotPosition>;
using namespace std::placeholders;

class RobotPositionClient: public rclcpp::Node{

    private:

        rclcpp_action::Client<robot_position_interface::action::RobotPosition>::SharedPtr client;
        RobotPositionGoalHandle::SharedPtr goal_handle;
        rclcpp::Subscription<robot_position_interface::msg::RobotPositionCancel>::SharedPtr cancel_sub;

        void get_response_callback(const RobotPositionGoalHandle::SharedPtr &goal_handle){
        
            std::string msg;

            msg = !goal_handle ? "Goal was rejected by the server":"Goal was accepted by the server";

            this->goal_handle = goal_handle;
            RCLCPP_INFO(this->get_logger(), msg.c_str());

        }

        void get_feedback_msg(const RobotPositionGoalHandle::SharedPtr &goal_handle,
                              const std::shared_ptr<const robot_position_interface::action
                                    ::RobotPosition::Feedback> feedback){

            (void) goal_handle;
            int current_time = feedback->current_position;
            RCLCPP_INFO(this->get_logger(), "Current position: %d", current_time);

        }

        void get_result(const RobotPositionGoalHandle::WrappedResult &result){
            
            int position = result.result.get()->position;
            std::string message = result.result.get()->message;

            auto result_code = result.code;

            switch(result_code){

                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this -> get_logger(), "Goal was succeded");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
            }

            RCLCPP_INFO(this->get_logger(), "Position: %d", position);
            RCLCPP_INFO(this->get_logger(), "Message: %s", message.c_str());
            this->goal_handle.reset();

        }

        void cancel_goal_callback(const robot_position_interface::msg::
                                    RobotPositionCancel::SharedPtr){

            if(this->goal_handle){

                this->client->async_cancel_goal(this->goal_handle);
                RCLCPP_INFO(this->get_logger(), "Sending request to cancel goal");
                this->goal_handle.reset();
            }

        }

    public:

        RobotPositionClient():rclcpp::Node("robot_position_client_node"){

            this -> client = rclcpp_action::create_client<robot_position_interface::action::RobotPosition>
                                                        (this, "robot_position");
            this->cancel_sub = this->create_subscription<robot_position_interface::msg::
                                     RobotPositionCancel>("cancel_goal_client", 10,
                                     std::bind(&RobotPositionClient::cancel_goal_callback, this, _1));

            RCLCPP_INFO(this->get_logger(), "Robot Position Client has been started");
        }

        void send_goal(int position, int velocity){

            this->client->wait_for_action_server();
            auto goal = robot_position_interface::action::RobotPosition::Goal();
            goal.position = position;
            goal.velocity = velocity;
            auto options = rclcpp_action::Client<robot_position_interface::action::
                            RobotPosition>::SendGoalOptions();

            options.feedback_callback = std::bind(&RobotPositionClient::get_feedback_msg, this, _1, _2);
            options.goal_response_callback = std::bind(&RobotPositionClient::get_response_callback, this, _1);
            options.result_callback = std::bind(&RobotPositionClient::get_result, this, _1);

            this->client->async_send_goal(goal, options);

            RCLCPP_INFO(this->get_logger(), "Sending a goal to the server");

        }
    
};

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);

    try{
        auto robot_client = std::make_shared<RobotPositionClient>();
        robot_client -> send_goal(93, 3);
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(robot_client);
        executor.spin();
    }

    catch(std::exception &ex){std::cerr << "Exception: " << ex.what() << std::endl;}

}

