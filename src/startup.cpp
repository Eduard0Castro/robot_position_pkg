#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

class ControlLifecycle: public rclcpp::Node{

    public:

        ControlLifecycle(): rclcpp::Node("control_lifecycle_node"){

            std::vector<std::string> nodes_name = declare_parameter<std::vector<std::string>>("node_name", 
                                                                                        {"robot_position"});
            for (long unsigned int i = 0; i < nodes_name.size(); i++){
                std::string service_name = "/"+nodes_name[i]+"/change_state";
                this->clients.push_back(this->create_client<lifecycle_msgs::srv::ChangeState>(service_name));
            }

            RCLCPP_INFO(this->get_logger(), "Control lifecycle node has been initialized");

        }

        void initialize_sequence (){

            auto transition = lifecycle_msgs::msg::Transition();
            transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

            RCLCPP_INFO(this->get_logger(), "Transition from unconfigure to inactive");
            this->change_state(transition);

            rclcpp::sleep_for(std::chrono::seconds(2));

            transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
            
            RCLCPP_INFO(this->get_logger(), "Transition from inactive to active");
            this->change_state(transition);
            
        }

    private:

        std::vector<rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> clients;
        
        void change_state(lifecycle_msgs::msg::Transition transition){

            for (long unsigned int i = 0; i < this->clients.size(); i++){

                this->clients[i]->wait_for_service(std::chrono::seconds(1));

                auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

                request->transition = transition;
                auto future = this->clients[i]->async_send_request(request);

                this->get_future_response(future);

            }
        }

        void get_future_response(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::FutureAndRequestId &future){

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();

                if(response.get()->success) RCLCPP_INFO(this->get_logger(), "Transition was a success");
                else RCLCPP_INFO(this->get_logger(), "Transition request failed");
                
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to call change_state service");
            }
        }
};


int main(int argc, char **argv){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlLifecycle>();
    node->initialize_sequence();
    rclcpp::shutdown();
    
    return 0;
}