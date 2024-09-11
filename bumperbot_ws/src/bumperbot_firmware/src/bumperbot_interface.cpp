#include "bumperbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace bumperbot_interface{
    
    BumperbotInterface::BumperbotInterface(){}

    BumperbotInterface::~BumperbotInterface(){
        if(arduino_.IsOpen()){
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),"Something went wrong while closing the connection with port" << port_);
            }
            
        }
    }

    CallbackReturn BumperbotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info){
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if(result != CallbackReturn::SUCCESS){
            return result;
        }

        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface"),"Something went wrong no serial port provided, ABORTING");
            return CallbackReturn::FAILURE;
        }
        
        velocity_commands_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());
        positon_states_.reserve(info_.joints.size());
        last_run_ = rclcpp::Clock().now();

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> BumperbotInterface::export_state_interfaces(){
        std::vector<hardware_interface::StateInterface> state_interfaces;

        //we put in the state_interfaces vector the joint name, the interface type, and the position of that joint i
        for(size_t i=0; i < info_.joints.size(); i++){
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name
                                                                             ,hardware_interface::HW_IF_POSITION
                                                                             , &positon_states_.at(i)));
        }

        //we put in the state_interfaces vector the joint name, the interface type, and the velocity of that joint i
        for(size_t i=0; i < info_.joints.size(); i++){
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name
                                                                             ,hardware_interface::HW_IF_VELOCITY
                                                                             , &velocity_states_.at(i)));
        }

        return state_interfaces;
    }

    //the purpose is to communicate the interface we use in order to receive movement commands for the motor
    std::vector<hardware_interface::CommandInterface> BumperbotInterface::export_command_interfaces(){
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for(size_t i=0; i < info_.joints.size(); i++){
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints.at(i).name
                                                                             ,hardware_interface::HW_IF_VELOCITY
                                                                             , &velocity_commands_.at(i)));
        }

        return command_interfaces;
    }

    CallbackReturn BumperbotInterface::on_activate(const rclcpp_lifecycle::State &previous_state){

        RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Activation State: Starting robot Hardware...");

        //intialize the vectors
        velocity_commands_ = {0.0, 0.0};
        positon_states_ = {0.0, 0.0};
        velocity_states_ = {0.0, 0.0};

        //open the serial communication with arduino
        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"), "Activation State: Something went wrong while interacting with port" << port_);
            return CallbackReturn::FAILURE;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Activation State: Hardware Started ready to take command");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn BumperbotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state){

        RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "DeActivation State: Stopping robot Hardware...");
        if(arduino_.IsOpen()){
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface")," DeActivation State: Something went wrong no serial port provided, ABORTING");
                return CallbackReturn::FAILURE;
            }
            
        }
        return CallbackReturn::SUCCESS;
    }

    //this is where all the logic will happen

    //read messages from the arduino
    hardware_interface::return_type BumperbotInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period){
        if(arduino_.IsDataAvailable()){

            auto dt = (rclcpp::Clock().now() - last_run_).seconds();
            std::string message;
            arduino_.ReadLine(message);

            std::stringstream ss(message);
            std::string res;
            int multiplier = 1;
            //checking the on serial we are getting from the arduino, if the rotation at(1) is positive then p if no the n, if the wheel at(0) is right then r if not then l
            //example: rp2.5,ln5.1 --- rad/s 
            while (std::getline(ss,res,','))
            {
                multiplier = res.at(1) == 'p' ? 1 : -1;
                if(res.at(0) == 'r'){
                    velocity_states_.at(0) = multiplier * std::stod(res.substr(2 , res.size()));
                    //get the position form the velocity
                    positon_states_.at(0) += velocity_states_.at(0) * dt;
                }
                else if(res.at(0) == 'l'){
                    velocity_states_.at(1) = multiplier * std::stod(res.substr(2 , res.size()));
                    positon_states_.at(1) += velocity_states_.at(1) * dt;
                }
            }
            last_run_ = rclcpp::Clock().now();
        }
        return hardware_interface::return_type::OK;
    }

    //write messages to the arduino
    hardware_interface::return_type BumperbotInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period){

        std::stringstream message_stream;
        char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
        char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';

        std::string compensate_zeros_right = "";
        std::string compensate_zeros_left = "";

        if(std::abs(velocity_commands_.at(0)) < 10.0){
            compensate_zeros_right = "0";
        }

        if(std::abs(velocity_commands_.at(1)) < 10.0){
            compensate_zeros_left = "0";
        }

        message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0)) <<
            ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1)) << ",";

        try
        {
            arduino_.Write(message_stream.str());
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("BumperbotInterface")," Writing State: Something went wrong while sending mesage"<< message_stream.str() <<  "on the port: "<< port_);
            return hardware_interface::return_type::ERROR;
        }
        
        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(bumperbot_interface::BumperbotInterface, hardware_interface::SystemInterface)