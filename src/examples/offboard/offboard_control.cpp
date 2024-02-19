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
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include "std_msgs/msg/string.hpp"
#include <math.h>
#include <cmath>

float X, Y, Z, vx, vy, vz, heading;
float X_del, Y_del, Z_del; // offset from the home position (NED)!!
float X_del_dyn, Y_del_dyn, Z_del_dyn, vx_dyn, vy_dyn, vz_dyn, heading_dyn; //offset from the home position, but written in frame from Dynamics (compared to NED, x = x , y = -y ,z = -z)
float X_home, Y_home, Z_home, heading_home;
float X_tmp, Y_tmp, Z_tmp, vx_tmp, vy_tmp, vz_tmp, X_ddot_tmp, Y_ddot_tmp, Z_ddot_tmp;
float lat,lon,alt;
float lat_home, lon_home, alt_home;
float radius = 1;
float mass_drone = 1.535; //The parameter needs to be set!! // 1.535 kg for Gazebo sitl iris quadrotor
float pos_acceleration[3] = {0.0, 0.0, 0.0}; // Vehicle x,y,z acceleration, which will be delivered to the pixhawk to follow
float vehicle_thrust[3] = {0.0, 0.0, 0.0}; // Vehicle_thrust_setpoint
const double pi = 3.14159265358979;
// nominal
float theta_nom=0.0;
float phi_nom=0.0;
float Del_U1_nom=0.0;
float U1_nom = 0.0;
float x_del_desired, y_del_desired, z_del_desired; // Desired trajectiry in NED frmae (PX4)
float x_del_desired_dyn, y_del_desired_dyn, z_del_desired_dyn; // Desired trajectory in ENU frame (Dynamics)
float a0=0.1;
float a1=1; // Q-filter coefficient
float tau = 0.1; // DOB Q-filter parameter(Sampling time for this code)
// DOB output before saturation. After saturated, this value will be subtracted from the outloop controller's value
float theta_tmp, phi_tmp, DelU1_tmp;

// input after all calculation. This will head to the saturation block again before execution, to ensure safety
float theta_now, theta_mi1, theta_mi2;
float phi_now, phi_mi1, phi_mi2;
float DelU1_now, DelU1_mi1, DelU1_mi2;

// DOB intermediate qx,qy,qz. for state estimation block
// ex ) qx_pl2 means qx[time+2], and qx_pl1 means q1[time+1]
float qx_pl2, qx_pl1, qx_now;
float qy_pl2, qy_pl1, qy_now;
float qz_pl2, qz_pl1, qz_now;

// DOB intermediate p_theta, p_phi, p_DelU1
// ex ) p_theta_pl1 is the variable that stores p_theta[time+1]
float p_theta_pl1, p_theta_now, p_theta_mi1;
float p_phi_pl1, p_phi_now, p_phi_mi1;
float p_DelU1_pl1, p_DelU1_now, p_DelU1_mi1;


float g = 9.81; // gravity constant

/*-- Position Control LQR Gain --//
K = [ 8.4545	-0.0000	0.0000	1.5616	-0.0000	0.0000
	0.0000	-8.4545	0.0000	-0.0000	-1.5616	0.0000
	-0.0000	-0.0000	9.7431	-0.0000	-0.0000	7.7076 ]
*/
// the below Control Gain should be re-calculated if the parameters & Gain matrix are revised!!
float K_pos[3][6] = {
	{0.7974,	0.0,	0.0,	0.4756,		0.0,	0.0},
	{0.0,	-0.7974,	0.0,	0.0,	-0.4756,	0.0},
	{0.0,	0.0,	4.9536,		0.0,		0.0,	3.9547}
};
uint16_t mission_status = 1; // change here manually to confirm what mission to handle!!
/* 
mission_status = 0; //(takeoff->circle trajectory->auto landing) [PID position control]
mission_status = 1; //(takeoff->circle trajectory->auto landing) [LQR Position control]
*/
bool flag_control_offboard_enabled = false;
bool flag_armed = false;
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
		vehicle_attitude_setpoint_publisher_ =
			this->create_publisher<VehicleAttitudeSetpoint>("fmu/vehicle_attitude_setpoint/in", 10);
		vehicle_thrust_setpoint_publisher_ =
			this->create_publisher<VehicleThrustSetpoint>("fmu/vehicle_thrust_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
		
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in");
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in");
		vehicle_attitude_setpoint_publisher_ =
			this->create_publisher<VehicleAttitudeSetpoint>("fmu/vehicle_attitude_setpoint/in", 10);
		vehicle_thrust_setpoint_publisher_ =
			this->create_publisher<VehicleThrustSetpoint>("fmu/vehicle_thrust_setpoint/in", 10);
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
		set_home_pos(); //initialize one time
		// loop!!
		auto timer_callback = [this]() -> void {
			if(flag_armed==true){
				if(flag_control_offboard_enabled == true && offboard_signal_received == false){ // if offboard enabled and if it's the first time that mmu received the signal, initialize and start offboard command
				//enter this if-statement only once!!
				offboard_signal_received = true; //mark that mmu has received the offboard command
				set_home_pos(); // initialize home position when start!!
				//this->arm(); 
				offboard_setpoint_counter_ = 0; // reset the offboard counter
				}
			}
			
			//print current local position every 2 second
			if(offboard_setpoint_counter_%200 ==0){
				//print_current_position();
			}

			// loop keep sending signals regardless of the flight mode
            // offboard_control_mode needs to be paired with trajectory_setpoint
			
			publish_offboard_control_mode(offboard_setpoint_counter_);
			publish_trajectory_setpoint(offboard_setpoint_counter_);
			

			if(offboard_setpoint_counter_ == 5000 && offboard_signal_received == true) { 
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 6);//PX4_CUSTOM_SUB_MODE_AUTO_LAND
				//this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
				//this->land();
				RCLCPP_INFO(this->get_logger(), "Land command send");
	 			print_current_position();
			}

           	// stop the counter after reaching 5001
			if (offboard_setpoint_counter_ < 5001) {
				offboard_setpoint_counter_++;
			}

			
		};
		timer_ = this->create_wall_timer(10ms, timer_callback); // 100 Hz 
	}

	void arm() const;
	void disarm() const;
	//void land() const;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
	rclcpp::Publisher<VehicleThrustSetpoint>::SharedPtr vehicle_thrust_setpoint_publisher_;
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
	
	void publish_offboard_control_mode(uint64_t offboard_setpoint_counter_) const;
	void publish_trajectory_setpoint(uint64_t offboard_setpoint_counter_) const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0, float param3 = 0.0) const;

	//Below added
	void LocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr &msg);
	void GlobalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr &msg);
	void VehicleControlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr &msg);
	void set_home_pos();
	void print_current_position();
	float wrapToPi(float angle) const;
	//Above added
};

float OffboardControl::wrapToPi(float angle) const{
    // Normalize the angle to be between -π and π
    angle = fmodf(angle + M_PI, 2.0f * M_PI) - M_PI;

    return angle;
}

/**
 * @brief Member function to handle the vehicle control mode callback logic
*/
void OffboardControl::VehicleControlModeCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr &msg) {
	flag_control_offboard_enabled = msg->flag_control_offboard_enabled;
	flag_armed = msg->flag_armed;
	
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

	//offset from home position
	X_del = X - X_home;
	Y_del = Y - Y_home;
	Z_del = Z - Z_home;


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
	std::cout << "\n Current Local position" << std::endl;
	std::cout <<"X (m): " << X<<std::endl;
	std::cout <<"Y (m): " << Y <<std::endl;
	std::cout <<"Z (m): " << Z <<std::endl;
	std::cout <<"Heading (rad): " << heading <<std::endl;
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
void OffboardControl::publish_offboard_control_mode(uint64_t offboard_setpoint_counter_) const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	if(mission_status == 0){
		msg.position = true;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
	}
	else if(mission_status ==1){ // Change here!!!!
		if(offboard_setpoint_counter_<=1000){ // use position message (PID) for takeoff
			msg.position = true;
			msg.velocity = false;
			msg.acceleration = false;
			msg.attitude = false;
			msg.body_rate = false;
		}
		else if(offboard_setpoint_counter_ <= 4000){ // use LQR position control
			msg.position = false;
			msg.velocity = false;
			msg.acceleration = true; //
			msg.attitude = false; //
			msg.body_rate = false;
		}
		if(offboard_setpoint_counter_<=5000){ // use position message (PID) for Landing
			msg.position = true;
			msg.velocity = false;
			msg.acceleration = false;
			msg.attitude = false;
			msg.body_rate = false;
		}
	}
	
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
	VehicleAttitudeSetpoint msg_a{};
	VehicleThrustSetpoint msg_t{};
	msg.timestamp = timestamp_.load();
	msg_a.timestamp = timestamp_.load();
	msg_t.timestamp = timestamp_.load();
	if(mission_status == 0){ //takeoff -> circle motion -> landing (PID)
		if(flag_control_offboard_enabled == false){ // Even though offboard control mode is disabled, any signals need to be sent over 2 Hz
			msg.x = X_home + 0.0;
			msg.y = Y_home + 0.0;
			msg.z = Z_home - 0.0;
			msg.yaw = heading_home;
			trajectory_setpoint_publisher_->publish(msg);
		}
		else if(offboard_setpoint_counter_ <= 500){ // go upward to (0,0,-2) slowly, from t = 0 ~ 10s
			msg.x = X_home + 0.0;
			msg.y = Y_home + 0.0;
			msg.z = Z_home - 0.004*offboard_setpoint_counter_; 
			msg.yaw = heading_home; 
			trajectory_setpoint_publisher_->publish(msg);
		}
		else if(offboard_setpoint_counter_ <= 750){ // go position (0,1,-2) to start circle motion, from t = 10s ~ 15s
			msg.x = X_home + 0.0;
			msg.y = Y_home + 1.0;
			msg.z = Z_home - 2.0; 
			msg.yaw = heading_home;
			trajectory_setpoint_publisher_->publish(msg);
		}
		else if(offboard_setpoint_counter_ <= 1500){ // circle motion, from t = 15s ~ 30s
			msg.x = X_home + radius * sin(0.0041888*2*(offboard_setpoint_counter_ - 750));
			msg.y = Y_home + radius * cos(0.0041888*2*(offboard_setpoint_counter_ - 750));
			msg.z = Z_home - 2.0;
			msg.yaw = heading_home;
			trajectory_setpoint_publisher_->publish(msg);
		}
		else if(offboard_setpoint_counter_ <= 1750){ // move to the center, from t = 30s~35s
			msg.x = X_home + 0.0;
			msg.y = Y_home + 0.0;
			msg.z = Z_home - 2.0; 
			msg.yaw = heading_home;
			trajectory_setpoint_publisher_->publish(msg);
		}
		else if(offboard_setpoint_counter_ <= 2125){ // descending to (0,0,-0.5) before landing , for safety 
			msg.x = X_home + 0.0;
			msg.y = Y_home + 0.0;
			msg.z = Z_home - 2.0 + 0.004*(offboard_setpoint_counter_ - 1750); // 
			msg.yaw = heading_home;
			trajectory_setpoint_publisher_->publish(msg);
		}
	}
	else if(mission_status == 1){ // mission using LQR!!! Acceleration command is used
		if(flag_control_offboard_enabled == false){ // Even though offboard control mode is disabled, any signals need to be sent over 2 Hz
			msg.x = X_home + 0.0;
			msg.y = Y_home + 0.0;
			msg.z = Z_home - 0.0;
			msg.yaw = heading_home;
			trajectory_setpoint_publisher_->publish(msg);
		}
		else if(offboard_setpoint_counter_ <= 500){ // Before LQR command, go upward to (0,0,-2) slowly, from t = 0 ~ 5s
			msg.x = X_home + 0.0;
			msg.y = Y_home + 0.0;
			msg.z = Z_home - 0.5 - 0.004*offboard_setpoint_counter_;
			msg.yaw = heading_home;
			trajectory_setpoint_publisher_->publish(msg);
		}
		else if(offboard_setpoint_counter_ <= 1000){ // go position (0,1,-2) to start circle motion, from t = 5s~10s
			msg.x = X_home + 0.0;
			msg.y = Y_home + 1.0;
			msg.z = Z_home - 2.5; 
			msg.yaw = heading_home;
			trajectory_setpoint_publisher_->publish(msg);

			//fill DOB values for warm-start //need to be modified for better result!!
			qx_pl1 = qx_now = Y_del;
			qy_pl1 = qy_now = X_del;
			qz_pl1 = qz_now = -Z_del;
			p_theta_now = theta_now = p_theta_mi1 = 0.0;
			p_phi_now = phi_now = p_phi_mi1 = 0.0;
			p_DelU1_now = DelU1_now = p_DelU1_mi1 = 0.0;
			
		}
		else if(offboard_setpoint_counter_ <= 4000){ // Circular motion with radius 1m using LQR at 2.5m height. t = 10s ~ 40s
			// --- Desired Trajectory in NED frame--- // 
			x_del_desired =radius * sin(0.002094*(offboard_setpoint_counter_ - 1000));
			y_del_desired =radius * cos(0.002094*(offboard_setpoint_counter_ - 1000));	
			z_del_desired = -2.5; // 2.5m height

			// Cooordinate change : From NED To the frame from Dynamics (X = Y, Y = X, Z = -Z)
			// For dynamics frame, positive Z means upward 
			X_del_dyn = Y_del;
			Y_del_dyn = X_del;
			Z_del_dyn = -Z_del;

			vx_dyn = vy;
			vy_dyn = vx;
			vz_dyn = -vz;

			x_del_desired_dyn = y_del_desired;
			y_del_desired_dyn = x_del_desired;
			z_del_desired_dyn = -z_del_desired;

			heading_dyn = wrapToPi(M_PI/2 - heading);

			
			//------DOB PART --------------------------------------//
			// fill DOB intermediate values (State estimator block)
			qx_pl2 = (2-a1)*qx_pl1 + a0*X_del_dyn - (1-a1+a0)*qx_now;
			qy_pl2 = (2-a1)*qy_pl1 + a0*Y_del_dyn - (1-a1+a0)*qy_now;
			qz_pl2 = (2-a1)*qz_pl1 + a0*Z_del_dyn - (1-a1+a0)*qz_now;

			// DOB lower block output (after Pn_inverse)
			theta_tmp = (qx_pl2 - 2*qx_pl1 + qx_now)/(g*tau*tau);
			phi_tmp = -(qy_pl2 - 2*qy_pl1 + qy_now)/(g*tau*tau);
			DelU1_tmp = (qz_pl2 - 2*qz_pl1 + qz_now)*mass_drone/(tau*tau);

			// DOB p part
			p_theta_pl1 = -(a1-2)*p_theta_now + a0*theta_mi1 - (1-a1+a0)*p_theta_mi1;
			p_phi_pl1 = -(a1-2)*p_phi_now + a0*phi_mi1 - (1-a1+a0)*p_phi_mi1;
			p_DelU1_pl1 = -(a1-2)*p_DelU1_now + a0*DelU1_mi1 - (1-a1+a0)*p_DelU1_mi1;

			// temporary output of DOB before it meets the outloop controller
			theta_tmp = theta_tmp - p_theta_now;
			phi_tmp = phi_tmp - p_phi_now;
			DelU1_tmp = DelU1_tmp - p_DelU1_now;

			//saturate the DOB output
			theta_tmp = std::clamp(theta_tmp, -0.35f, 0.35f); // theta 0.35rad = 20 \degree
			phi_tmp = std::clamp(phi_tmp, -0.35f, 0.35f); // phi 0.35rad = 20 \degree
			DelU1_tmp = std::clamp(DelU1_tmp, -10.0f, 10.0f); // +-10 N centered at hovering thrust
			//--------------------------------------------------------//


			//-----Nominal Outloop Controller part --------------------//
			// Caclulate nominal control from outloop Controller (LQR)

			// theta_nom = K_pos[0][0]*(X_del_dyn- x_del_desired_dyn) + K_pos[0][1]*(Y_del_dyn-y_del_desired_dyn) + K_pos[0][2]*(Z_del_dyn -z_del_desired_dyn) + K_pos[0][3]*vx_dyn + K_pos[0][4]*vy_dyn + K_pos[0][5]*vz_dyn;
			// phi_nom = K_pos[1][0]*(X_del_dyn- x_del_desired_dyn)  + K_pos[1][1]*(Y_del_dyn-y_del_desired_dyn) + K_pos[1][2]*(Z_del_dyn -z_del_desired_dyn) + K_pos[1][3]*vx_dyn + K_pos[1][4]*vy_dyn + K_pos[1][5]*vz_dyn;
			// Del_U1_nom = K_pos[2][0]*(X_del_dyn- x_del_desired_dyn)  + K_pos[2][1]*(Y_del_dyn-y_del_desired_dyn)+ K_pos[2][2]*(Z_del_dyn -z_del_desired_dyn) + K_pos[2][3]*vx_dyn + K_pos[2][4]*vy_dyn + K_pos[2][5]*vz_dyn;

			theta_nom = K_pos[0][0]*(x_del_desired_dyn - X_del_dyn) + K_pos[0][1]*(y_del_desired_dyn- Y_del_dyn) + K_pos[0][2]*(z_del_desired_dyn - Z_del_dyn) + K_pos[0][3]*(0 - vx_dyn) + K_pos[0][4]*(0-vy_dyn) + K_pos[0][5]*(0-vz_dyn);
			phi_nom = K_pos[1][0]*(x_del_desired_dyn - X_del_dyn)  + K_pos[1][1]*(y_del_desired_dyn- Y_del_dyn) + K_pos[1][2]*(z_del_desired_dyn - Z_del_dyn) + K_pos[1][3]*(0 - vx_dyn) + K_pos[1][4]*(0-vy_dyn) + K_pos[1][5]*(0-vz_dyn);
			Del_U1_nom = K_pos[2][0]*(x_del_desired_dyn - X_del_dyn)  + K_pos[2][1]*(y_del_desired_dyn- Y_del_dyn)+ K_pos[2][2]*(z_del_desired_dyn - Z_del_dyn) + K_pos[2][3]*(0 - vx_dyn) + K_pos[2][4]*(0-vy_dyn) + K_pos[2][5]*(0-vz_dyn);
			// // reverse sign! Since u = - K (x-x_desired)
			// theta_nom *=-1;
			// phi_nom *=-1;
			// Del_U1_nom *=-1; //
			
			// theta_nom = std::clamp(theta_nom, -0.35f, 0.35f); // theta 0.35rad = 20 \degree
			// phi_nom = std::clamp(phi_nom, -0.35f, 0.35f); // phi 0.35rad = 20 \degree
			// Del_U1_nom = std::clamp(Del_U1_nom, -10.0f, 10.0f); // +-10 N centered at hovering thrust0
			//----------------------------------------------------------//
			theta_tmp = - theta_tmp;
			phi_tmp = - phi_tmp;
			DelU1_tmp = - DelU1_tmp;

			// //SET Below 0 to Not use DOB!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Cutoff DOB to test only LQR
			// theta_tmp = 0;
			// phi_tmp = 0;
			// DelU1_tmp = 0;

			//Calculate input !!!
			theta_now = theta_nom - theta_tmp;
			phi_now = phi_nom - phi_tmp;
			DelU1_now = Del_U1_nom - DelU1_tmp;

			// Clip values ! 
			theta_now = std::clamp(theta_now, -0.35f, 0.35f); // theta 0.35rad = 20 \degree
			phi_now = std::clamp(phi_now, -0.35f, 0.35f); // phi 0.35rad = 20 \degree
			DelU1_now = std::clamp(DelU1_now, -10.0f, 10.0f); // +-10 N centered at hovering thrust
			
			//--------Conversion--------------------------------------//

			//calculate x,y,z acceleration value (in dynamics frame)
			X_ddot_tmp = (cos(phi_now)*sin(theta_now)*cos(heading_dyn) +sin(phi_now)*sin(heading_dyn))*(mass_drone*g + DelU1_now)/mass_drone;
			Y_ddot_tmp = (cos(phi_now)*sin(theta_now)*sin(heading_dyn) -sin(phi_now)*cos(heading_dyn))*(mass_drone*g + DelU1_now)/mass_drone;
			Z_ddot_tmp = -g + (cos(phi_now)*cos(theta_now))*(mass_drone*g + DelU1_now)/mass_drone;

			// Convert to NED frame (X=Y, Y=X, Z=-Z)
			pos_acceleration[0] = Y_ddot_tmp;
			pos_acceleration[1] = X_ddot_tmp;
			pos_acceleration[2] = -Z_ddot_tmp;

			
			std::copy(std::begin(pos_acceleration), std::end(pos_acceleration), msg.acceleration.begin());
			msg.yaw = heading_home;
			msg.x = NAN;
			msg.y = NAN;
			msg.z = NAN;
			msg.vx = NAN;
			msg.vy = NAN;
			msg.vz = NAN;

			if(offboard_setpoint_counter_% 100==0){
				std::cout << "\nz-acceleration (Desired, NED) : " << pos_acceleration[2] << std::endl;
				std::cout << "current z position : " << Z <<std::endl;
				std::cout << "phi_tmp: " << phi_tmp<< std::endl;
			}

			trajectory_setpoint_publisher_->publish(msg);
			
			// --------- time shift !!!!------------------//
			theta_mi2 = theta_mi1;
			theta_mi1 = theta_now;

			phi_mi2 = phi_mi1;
			phi_mi1 = phi_now;

			DelU1_mi2 = DelU1_mi1;
			DelU1_mi1 = DelU1_now;

			//
			qx_now = qx_pl1;
			qx_pl1 = qx_pl2;

			qy_now = qy_pl1;
			qy_pl1 = qy_pl2;

			qz_now = qz_pl1;
			qz_pl1 = qz_pl2;
			//
			p_theta_mi1 = p_theta_now;
			p_theta_now = p_theta_pl1;

			p_phi_mi1 = p_phi_now;
			p_phi_now = p_phi_pl1;

			p_DelU1_mi1 = p_DelU1_now;
			p_DelU1_now = p_DelU1_pl1;
			//---------------------------------------------//
			

		}
		else if(offboard_setpoint_counter_ <= 4500){ // move to the center, from t = 40s~45s
			msg.x = X_home + 0.0;
			msg.y = Y_home + 0.0;
			msg.z = Z_home - 2.5; 
			msg.yaw = heading_home;
			trajectory_setpoint_publisher_->publish(msg);
		}
		else if(offboard_setpoint_counter_ <= 5000){ // descending to (0,0,-0.5) before landing , for safety 
			if(offboard_setpoint_counter_%200==0){
				RCLCPP_INFO(this->get_logger(), "Descending Before Landing for Safety");
			}
			//std::cout << "LAND Command Send" << std::endl;
			msg.x = X_home + 0.0;
			msg.y = Y_home + 0.0;
			msg.z = Z_home - 2.5 + 0.004 * (offboard_setpoint_counter_ - 4500); // 
			msg.yaw = heading_home; 
			trajectory_setpoint_publisher_->publish(msg);
		}
	}
	else{ // Should not reach here
		std::cout <<"Something is wrong!!" <<std::endl;
	}
	
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
