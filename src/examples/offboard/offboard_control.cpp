/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include "std_msgs/msg/string.hpp"
#include <math.h>

float X, Y, Z, vx, vy, vz, heading;
float X_home, Y_home, Z_home, heading_home;
float X_tmp, Y_tmp, Z_tmp, vx_tmp, vy_tmp, vz_tmp;
float lat,lon,alt;
float lat_home, lon_home, alt_home;
float radius = 1;
uint16_t mission_status = 0;
bool flag_control_offboard_enabled = false;
bool offboard_signal_received = false; // true if offboard_signal from "pixhawk" is received,

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
		//vehicle_status_subcriber =
        //	this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos_profile, std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));
		
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in");
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in");
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in");
		//vehicle_status_subcriber =
        //	this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos_profile, std::bind(&OffboardControl::vehicle_status_callback, this, std::placeholders::_1));
		
#endif

		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});


		subscription_local = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		    "/fmu/vehicle_local_position/out",10,
		    [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
				LocalPositionCallback(msg);
	    	});
		subscription_vehicle_control_mode = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
		    "/fmu/vehicle_control_mode/out",10,
		    [this](const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
				VehicleControlModeCallback(msg);
	    	});
		// subscription_GPS = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
		// 	"fmu/vehicle_global_position/out",10,
		// 	[this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
		// 		GlobalPositionCallback(msg);
		// 	});
		latest_local_position_msg_ = std::make_shared<px4_msgs::msg::VehicleLocalPosition>();
		//latest_global_position_msg_ = std::make_shared<px4_msgs::msg::VehicleGlobalPosition>();

		offboard_setpoint_counter_ = 0;
		
		// loop!!
		auto timer_callback = [this]() -> void {
			if(flag_control_offboard_enabled == true && offboard_signal_received == false){ // if offboard enabled and if it's the first time that mmu received the signal, initialize and start offboard command
				//this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				//enter this if-statement only once!!
				offboard_signal_received = true; //mark that mmu has received the offboard command
				set_home_pos();
				this->arm(); 
				offboard_setpoint_counter_ = 0; // reset the offboard counter
				
			}
			// if(flag_control_offboard_enabled == false && offboard_signal_received == true){ // this means that the remote switch was at offboard mode but changed to other mode(e.g., position mode)
			// 	//reset the offboard_signal_received == false
			// 	offboard_signal_received = false;
			// }
			//-------------------------------------------------//
			//LocalPositionCallback(latest_local_position_msg_);
			//GlobalPositionCallback(latest_global_position_msg_);
			//-------------------------------------------------//
			
			//print current local position every 1 second
			if(offboard_setpoint_counter_ %50 ==0){
				print_current_position();
			}

			// loop keep sending signals regardless of the flight mode
            // offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint(offboard_setpoint_counter_);
			

			if(offboard_setpoint_counter_ == 1000 && offboard_signal_received == true) { //landing at t = 20s after the offboard mode arming!
				//this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 6);//PX4_CUSTOM_SUB_MODE_AUTO_LAND
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
				//this->land();
				RCLCPP_INFO(this->get_logger(), "Land command send");
				// std::cout <<"X home (m): " << X_home <<std::endl;
				// std::cout <<"Y home (m): " << Y_home <<std::endl;
				// std::cout <<"Z home (m): " << Z_home <<std::endl;
				// std::cout <<"Heading (rad): " << heading_home <<std::endl;
				print_current_position();
				//mission_status = 1; // change to landing mode (Goto Home position)
				//this->disarm();
			}

           	// stop the counter after reaching 2001
			if (offboard_setpoint_counter_ < 2001) {
				offboard_setpoint_counter_++;
			}

			
		};
		timer_ = this->create_wall_timer(20ms, timer_callback);
	}

	void arm() const;
	void disarm() const;
	//void land() const;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

	// Below added
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_local;
	rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr subscription_GPS;
	rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr subscription_vehicle_control_mode;
	std::shared_ptr<px4_msgs::msg::VehicleLocalPosition> latest_local_position_msg_;
	std::shared_ptr<px4_msgs::msg::VehicleGlobalPosition> latest_global_position_msg_;
    // Above added

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent
	
	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint(uint64_t offboard_setpoint_counter_) const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0, float param3 = 0.0) const;

	//Below added
	void LocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr &msg);
	void GlobalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr &msg);
	void VehicleControlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr &msg);
	void set_home_pos();
	void print_current_position();
	//Above added
};
/**
 * @brief Member function to handle the vehicle control mode callback logic
*/
void OffboardControl::VehicleControlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr &msg) {
	flag_control_offboard_enabled = msg->flag_control_offboard_enabled;
	//std::cout << "\n\n";
	//std::cout << "offboard mode status : " << flag_control_offboard_enabled << std::endl;
	// std::cout << "\n\n\n\n\n";
	// std::cout << "RECEIVED VEHICLE Local POSITION DATA" << std::endl;
	// std::cout << "==================================" << std::endl;
	// std::cout << "ts: " << msg->timestamp << std::endl;
	// std::cout << "X: " << X << std::endl;
	// std::cout << "Y: " << Y << std::endl;
	// std::cout << "Z: " << Z << std::endl;
}
/**
 * @brief Member function to handle the local position callback logic
*/
void OffboardControl::LocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr &msg) {
	X = msg->x;
	Y = msg->y;
	Z = msg->z;
	vx = msg->vx;
	vy = msg->vy;
	vz = msg->vz;
	heading = msg->heading;
	//std::cout << "Heading: " << heading << std::endl;
	
	// std::cout << "\n\n\n\n\n";
	// std::cout << "RECEIVED VEHICLE Local POSITION DATA" << std::endl;
	// std::cout << "==================================" << std::endl;
	// std::cout << "ts: " << msg->timestamp << std::endl;
	// std::cout << "X: " << X << std::endl;
	// std::cout << "Y: " << Y << std::endl;
	// std::cout << "Z: " << Z << std::endl;
}
/**
 * @brief Member function to handle the local position callback logic
*/
void OffboardControl::GlobalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr &msg) {
	lat = msg->lat;
	lon = msg->lon;
	alt = msg->alt;
	
	std::cout << "\n\n\n\n\n";
	std::cout << "RECEIVED VEHICLE GLOBAL POSITION DATA"   << std::endl;
	std::cout << "=================================="   << std::endl;
	std::cout << "ts: "  << msg->timestamp    << std::endl;
	std::cout << "lat: " << lat  << std::endl;
	std::cout << "lon: " << lon << std::endl;
	std::cout << "alt: " << alt  << std::endl;
}
void OffboardControl::set_home_pos(){
	std::cout << "Local Home position Set" << std::endl;
	X_home = X;
	Y_home = Y;
	Z_home = Z;
	heading_home = heading;
	std::cout <<"X home (m): " << X_home <<std::endl;
	std::cout <<"Y home (m): " << Y_home <<std::endl;
	std::cout <<"Z home (m): " << Z_home <<std::endl;
	std::cout <<"Heading (rad): " << heading_home <<std::endl;

	// vehicle_command_publisher_->publish(msg);
}
void OffboardControl::print_current_position(){
	std::cout << "Current Local position" << std::endl;
	std::cout <<"X home (m): " << X<<std::endl;
	std::cout <<"Y home (m): " << Y <<std::endl;
	std::cout <<"Z home (m): " << Z <<std::endl;
	std::cout <<"Heading (rad): " << heading <<std::endl;

	// vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Send a command to land the vehicle
 */
// void OffboardControl::land() const {
// 	//publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0);
// 	RCLCPP_INFO(this->get_logger(), "Land command send");

// 	TrajectorySetpoint msg{};
// 	msg.timestamp = timestamp_.load();
// 	msg.x = X_home;
// 	msg.y = Y_home;
// 	msg.z = Z_home;
// 	msg.yaw = 0; 
// 	trajectory_setpoint_publisher_->publish(msg);
// }

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 2 meters.
 */
void OffboardControl::publish_trajectory_setpoint(uint64_t offboard_setpoint_counter_) const {
	offboard_setpoint_counter_ = offboard_setpoint_counter_;
	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	if(mission_status == 0){ 
		if(offboard_setpoint_counter_ <= 500){ // go toward (0,0,-2) slowly, for 10 seconds
			msg.x = X_home + 0.0;
			msg.y = Y_home + 0.0;
			msg.z = Z_home - 0.004*offboard_setpoint_counter_; 
			msg.yaw = heading_home; 
		}
		else{
			msg.x = X_home + 0.0;
			msg.y = Y_home + 0.0;
			msg.z = Z_home - 2.0;
			msg.yaw = heading_home; 
		}


		// if(offboard_setpoint_counter_<=250){//hovering
		// 	msg.x = radius;
		// 	msg.y = 0.0;
		// 	msg.z = -2.0;
		// 	msg.yaw = 0; 
		// }
		// else{
		// 	msg.x = radius * cos(0.0041888*2*(offboard_setpoint_counter_ - 250));
		// 	msg.y = radius * sin(0.0041888*2*(offboard_setpoint_counter_ - 250));
		// 	msg.z = -2.0;
		// 	msg.yaw = 0; 
		// }
		
	}
	else{ // Should not reach here
		std::cout <<"Something is wrong!!" <<std::endl;
	}
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2, float param3) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	if(param3 > 0){
		msg.param3 = param3;
	}
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Print local position of vehicle
 */
// void OffboardControl::sub_vehicle_local_position() const {
// 	sub_vehicle_local_position
// }

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
