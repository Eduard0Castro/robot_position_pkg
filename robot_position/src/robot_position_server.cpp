#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_position_interface/action/robot_position.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <exception>

using namespace std::placeholders;
using RobotPosition = robot_position_interface::action::RobotPosition;
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::
                                    LifecycleNodeInterface::CallbackReturn;

class RobotPositionServer: public rclcpp_lifecycle::LifecycleNode {

    private:

        rclcpp_action::Server<robot_position_interface::action::RobotPosition>::SharedPtr server_; 
        std::shared_ptr<rclcpp_action::ServerGoalHandle<RobotPosition>> goal_handle;
        rclcpp_action::GoalUUID preempted_goal_uuid;
        rclcpp::CallbackGroup::SharedPtr callback_group; 
        std::mutex mutex;
        int current_position = 50;
        const int max_position = 100;
        const int min_position = 0;
        bool activation = false;
        bool execute_thread_on = false;

        rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,
                                                  std::shared_ptr<const RobotPosition::Goal> goal){

            RCLCPP_INFO(this->get_logger(), "Goal received");
            (void) uuid;

            if(goal->position > this->max_position or goal->position < this->min_position){
                
                RCLCPP_INFO(this->get_logger(), "Invalid goal values. Rejecting the goal");
                return rclcpp_action::GoalResponse::REJECT;

            }
            else if (!this->activation){
                RCLCPP_INFO(this->get_logger(), "Node is not activated yet");
                return rclcpp_action::GoalResponse::REJECT;
            }

            {
                std::lock_guard<std::mutex> lock(this->mutex);
                if(this->goal_handle and this->goal_handle->is_active()){
                    this->preempted_goal_uuid = this->goal_handle->get_goal_id();
                    RCLCPP_INFO(this->get_logger(), "Accepting new goal and aborting current goal");
                }

                else RCLCPP_INFO(this->get_logger(), "Goal accept");
            }

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        
        }

        rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<rclcpp_action::
                                                    ServerGoalHandle<RobotPosition>> goal_handle){

            RCLCPP_INFO(this->get_logger(), "Cancel request received");
            (void) goal_handle;

            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_goal_callback(const std::shared_ptr<rclcpp_action::
                                    ServerGoalHandle<RobotPosition>> goal_handle){

            this->execute_goal(goal_handle);
        }

        void execute_goal(const std::shared_ptr<rclcpp_action::
                            ServerGoalHandle<RobotPosition>> goal_handle){
            
            RCLCPP_INFO(this->get_logger(), "Executing the goal");

            {
                std::lock_guard<std::mutex> lock(this->mutex);
                this->goal_handle = goal_handle;
            }

            const int position = goal_handle -> get_goal() -> position;
            const int velocity = this->current_position < position ? 
                                    abs(goal_handle->get_goal()->velocity): 
                                    - abs(goal_handle->get_goal()->velocity);

            auto result = std::make_shared<RobotPosition::Result>();
            auto feedback = std::make_shared<RobotPosition::Feedback>();

            this->execute_thread_on = true;

            rclcpp::Rate loop_rate(1);
            
            for (this->current_position = this->current_position; 
                 this->target_position(this->current_position, position); 
                 this->current_position += velocity){

                {
                    if(goal_handle->is_canceling()){
                        if(this-> current_position != position){
                            result->position = this->current_position;
                            result->message = "Goal canceled";
                            goal_handle->canceled(result);
                            RCLCPP_INFO(this->get_logger(), "Current position: %d", current_position);
                            RCLCPP_WARN(this->get_logger(), "Goal canceled");
                        }
                        else break;
                        execute_thread_on = false;
                        return;
                    }

                    if(this->preempted_goal_uuid == goal_handle->get_goal_id()){
                        result->position = this->current_position;
                        result->message = "Goal aborted";
                        goal_handle->abort(result);
                        RCLCPP_INFO(this->get_logger(), "Current position: %d", current_position);
                        RCLCPP_WARN(this->get_logger(), "Goal aborted");
                        execute_thread_on = false;
                        return;
                    }


                    RCLCPP_INFO(this->get_logger(), "Current position: %d", this->current_position);
                    feedback->current_position = this->current_position;
                    goal_handle->publish_feedback(feedback);
                    if (abs(position - this->current_position) < abs(velocity)) break;
                    loop_rate.sleep();
                }
            }
            int difference = position - this->current_position;

            if(abs(difference) <= abs(velocity)) this->current_position += difference;

            result->position = this->current_position;
            result->message = "Finish goal";
            RCLCPP_INFO(this->get_logger(), "Reached position: %d", this->current_position);
            goal_handle->succeed(result);
            execute_thread_on = false;

        }

        bool target_position(int current_position, int position){

            if(this->current_position < position){
                if (current_position < position) return true;
                else return false;
            }
            else{
                if (current_position > position) return true;
                else return false;
            }
        }

        LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state){

            (void) previous_state;

            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.dynamic_typing = true;

            this->declare_parameter("robot_name", "robot_position", descriptor);
            std::string robot_name = this->get_parameter("robot_name").as_string();

            this->callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            this->server_ = rclcpp_action::create_server<RobotPosition>(this, 
                                            "robot_position_" + robot_name, 
                                            std::bind(&RobotPositionServer::goal_callback, this, _1, _2),
                                            std::bind(&RobotPositionServer::cancel_callback, this, _1), 
                                            std::bind(&RobotPositionServer::handle_goal_callback, this, _1),
                                            rcl_action_server_get_default_options(),
                                            this->callback_group);

            RCLCPP_INFO(this->get_logger(), "Action server has been started");

            return LifecycleCallbackReturn::SUCCESS;
        }

        LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state){

            (void) previous_state;
            while(this->execute_thread_on){
                RCLCPP_INFO(this->get_logger(), "Waiting execute_goal thread finish");
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
            this->server_.reset();

            this->undeclare_parameter("robot_name");
            RCLCPP_INFO(this->get_logger(), "Action server has been destroyed");

            return LifecycleCallbackReturn::SUCCESS;
        }

        LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state){

            (void) previous_state;
            this->activation = true;
            rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);

            RCLCPP_INFO(this->get_logger(), "Action server is activate");
            return LifecycleCallbackReturn::SUCCESS;
        }

        LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state){

            rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
            (void) previous_state;
            this->activation = false;

            {
                std::lock_guard<std::mutex> lock(this->mutex);
                if(this->goal_handle and this->goal_handle->is_active()){
                    this->preempted_goal_uuid = this->goal_handle->get_goal_id();
                    RCLCPP_INFO(this->get_logger(), "Aborting the current goal");
                }

            }
            RCLCPP_INFO(this->get_logger(), "Action server is deactivate");
            return LifecycleCallbackReturn::SUCCESS;
        }

        LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state){
            (void) previous_state;
            this->activation = false;
            this->server_.reset();

            RCLCPP_INFO(this->get_logger(), "Shutting down application");

            return LifecycleCallbackReturn::SUCCESS;
        }
    

    
    public:

        RobotPositionServer():rclcpp_lifecycle::LifecycleNode("robot_position_server_node"){

            RCLCPP_INFO(this->get_logger(), "Robot position server has been initialized");
        }
};

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    try{    

        auto robot_server = std::make_shared<RobotPositionServer>();
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(robot_server->get_node_base_interface());
        executor.spin();
    }

    catch(std::exception &ex){std::cerr << "Exception: " << ex.what() << std::endl;}
    
    rclcpp::shutdown();
    return 0;

}

