/*
Purpose: Concurrent Robot (ROS 2 Jazzy Port) - ArduPilot SITL Edition
Last updated: Restored Asynchronous Flight Loop Timer (Removed Blocking Deadlocks)
*/

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <math.h>
#include <string>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <atomic>

#include "con_plan_exec_pkg/basics.h"
#include "con_plan_exec_pkg/config.h"
#include "con_plan_exec_pkg/debug.h"

#include "con_plan_exec_pkg/srv/plan_for_horizon.hpp"
#include "con_plan_exec_pkg/srv/share_local_information.hpp"
#include "con_plan_exec_pkg/srv/stop_robot.hpp"

#include "con_plan_exec_pkg/msg/discrete_clock.hpp"
#include "con_plan_exec_pkg/msg/cell_info.hpp"

// --- MAVROS INCLUDES ---
#include <mavros_msgs/msg/state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/qos.hpp>

#define WS_OBS_ROBS_FILE_NAME "ws_obs_robs.txt"

using namespace std;
using namespace std::placeholders;

class RobotClass : public rclcpp::Node
{
	public:
		int rob_id;
		float loc_x;
		float loc_y;
		float loc_theta;
		uint ws_size_x;
		uint ws_size_y;
		float_mat ws_lidar;

		uint req_id;
		bool first_req_sent;
        rclcpp::Client<con_plan_exec_pkg::srv::ShareLocalInformation>::SharedPtr lv_sc;
		float_mat lv;
		map<pair<uint, uint>, float> cell_info_map;

        rclcpp::Service<con_plan_exec_pkg::srv::PlanForHorizon>::SharedPtr path_ss;
		plan_vec_t path;
		uint path_len;
		uint path_len_rem;

		uint start_from_clk;
		uint send_lv_at_clk;
		uint prev_clk;
		uint cur_clk;
		uint cur_intrvl;

        std::mutex robot_mutex;
        rclcpp::Service<con_plan_exec_pkg::srv::StopRobot>::SharedPtr stop_rob_ss;
        rclcpp::Subscription<con_plan_exec_pkg::msg::DiscreteClock>::SharedPtr clk_sub;

        // --- MAVROS VARIABLES ---
        std::atomic<bool> fcu_connected{false};
        std::atomic<bool> is_airborne{false};
        std::atomic<float> current_altitude{0.0};

        mavros_msgs::msg::State current_state;
        geometry_msgs::msg::PoseStamped local_pos; 
        geometry_msgs::msg::PoseStamped current_target_pose;
        float CELL_SIZE = 5.0;
        float TAKEOFF_ALT = 5.0;

        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub;
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;

        // FIXED: Restored your asynchronous flight loop timer
        rclcpp::TimerBase::SharedPtr flight_loop_timer;

		RobotClass() : Node("robot_node")
		{
            this->declare_parameter("rid", 0);
            this->declare_parameter("ws_x", 21);
            this->declare_parameter("ws_y", 20);
            this->declare_parameter("cell_size", 5.0);
            this->declare_parameter("takeoff_alt", 5.0);

            rob_id = this->get_parameter("rid").as_int();
            ws_size_x = this->get_parameter("ws_x").as_int();
            ws_size_y = this->get_parameter("ws_y").as_int();
            CELL_SIZE = this->get_parameter("cell_size").as_double();
            TAKEOFF_ALT = this->get_parameter("takeoff_alt").as_double();

			req_id = 0;
			first_req_sent = false;

            lv_sc = this->create_client<con_plan_exec_pkg::srv::ShareLocalInformation>("/concpp_node/share_workspace");

			cell_info_map.clear();
			path_len_rem = 0;
			start_from_clk = prev_clk = send_lv_at_clk = 0;

            populateLiDARValues();
            updateLocalView();
            startServiceServers();

            clk_sub = this->create_subscription<con_plan_exec_pkg::msg::DiscreteClock>(
                "/concpp_node/discrete_clock", 1000, std::bind(&RobotClass::readClock, this, _1));

            // Absolute Topic Names
			std::string state_topic = "/drone" + std::to_string(rob_id) + "/state";
			std::string local_pos_topic = "/drone" + std::to_string(rob_id) + "/local_position/pose";
			std::string local_pos_pub_topic = "/drone" + std::to_string(rob_id) + "/setpoint_position/local";
			std::string arming_topic = "/drone" + std::to_string(rob_id) + "/cmd/arming";
			std::string set_mode_topic = "/drone" + std::to_string(rob_id) + "/set_mode";

            state_sub = this->create_subscription<mavros_msgs::msg::State>(
                state_topic, 10, std::bind(&RobotClass::state_cb, this, _1));

            local_pos_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                local_pos_topic, rclcpp::SensorDataQoS(),
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    current_altitude = msg->pose.position.z;
                    local_pos = *msg; 
                });

            local_pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(local_pos_pub_topic, 10);
            arming_client = this->create_client<mavros_msgs::srv::CommandBool>(arming_topic);
            set_mode_client = this->create_client<mavros_msgs::srv::SetMode>(set_mode_topic);

            current_target_pose.pose.position.x = loc_x * CELL_SIZE;
            current_target_pose.pose.position.y = loc_y * CELL_SIZE;
            current_target_pose.pose.position.z = TAKEOFF_ALT;

            // FIXED: Restored your 10Hz continuous flight loop
            flight_loop_timer = this->create_wall_timer(
                std::chrono::milliseconds(100), std::bind(&RobotClass::flight_loop, this));

            std::thread(&RobotClass::takeoff_routine, this).detach();
		}

        void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
            current_state = *msg;
            fcu_connected = msg->connected;
        }

        // FIXED: The non-blocking publisher that runs in the background
        void flight_loop() {
            if (!is_airborne) return; 
            current_target_pose.header.stamp = this->get_clock()->now();
            current_target_pose.header.frame_id = "map";
            local_pos_pub->publish(current_target_pose);
        }

        void takeoff_routine() {
            RCLCPP_INFO(this->get_logger(), "R_%d Waiting for MAVROS Heartbeat...", rob_id);
            
            while (rclcpp::ok() && !fcu_connected) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            RCLCPP_INFO(this->get_logger(), "R_%d Connected! AWAITING MANUAL TAKEOFF (Ground Control)...", rob_id);

            // Wait for you to manually take off
            while (rclcpp::ok() && current_altitude < (TAKEOFF_ALT - 0.5)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }

            RCLCPP_INFO(this->get_logger(), "R_%d Swarm Altitude Reached (%.1f m)! Autonomy Engaged.", rob_id, current_altitude.load());

            is_airborne = true; // STARTS THE PUBLISHER THREAD
            std::this_thread::sleep_for(std::chrono::seconds(2));

            std::lock_guard<std::mutex> lock(robot_mutex);
            first_req_sent = true;

            RCLCPP_INFO(this->get_logger(), "R_%d Triggering Central Server communication...", rob_id);
            sendLocalView();
        }

		void populateLiDARValues()
		{
			std::string ws_obs_robs_file_path = ament_index_cpp::get_package_share_directory("con_plan_exec_pkg") + "/input/" + WS_OBS_ROBS_FILE_NAME;
			std::ifstream ws_obs_robs_file(ws_obs_robs_file_path.c_str());

			float data;
			int ws_row_id = 0, ws_col_id = 0;
            float_vec current_row;

			while(ws_obs_robs_file >> data) {
				ws_obs_robs_file.ignore(); 
				if(data > 0.5) {
					if(rob_id == (int)(data - 1)) {
						loc_x = ws_row_id;
						loc_y = ws_col_id;
						loc_theta = 0;
					}
                    current_row.push_back(0.5);
				} else {
                    current_row.push_back(data);
                }

				ws_col_id++;
				if(ws_col_id == (int)ws_size_y) { 
					ws_col_id = 0;
					ws_row_id++;
                    ws_lidar.push_back(current_row);
                    current_row.clear();
				}
			}
			ws_obs_robs_file.close();

           

			RCLCPP_INFO(this->get_logger(), "R_%d (%f, %f) in %d x %d", rob_id, loc_x, loc_y, ws_size_x, ws_size_y);

		    float_mat lv_init(ws_size_x, float_vec(ws_size_y, -1));
		    lv = lv_init;
		}

		void readLiDAR(int nbr_x, int nbr_y)
		{
			if((0 <= nbr_x) && (nbr_x < (int)ws_size_x) && (0 <= nbr_y) && (nbr_y < (int)ws_size_y))
				if(lv[nbr_x][nbr_y] == -1)
				{
					float lidar_val = ws_lidar[nbr_x][nbr_y];
					lv[nbr_x][nbr_y] = lidar_val;
					pair<uint, uint> cell_loc_key(nbr_x, nbr_y);
					if(cell_info_map.find(cell_loc_key) == cell_info_map.end())
					{
						pair<pair<uint, uint>, float> cell_info(cell_loc_key, lidar_val);
						cell_info_map.insert(cell_info);
					}
				}
		}

		void updateLocalView()
		{
			int cur_x = int(loc_x);
	        int cur_y = int(loc_y);

	        readLiDAR(cur_x + 1, cur_y);
	        readLiDAR(cur_x, cur_y + 1);
	        readLiDAR(cur_x - 1, cur_y);
	        readLiDAR(cur_x, cur_y - 1);

			lv[cur_x][cur_y] = 1;

			pair<uint, uint> cell_loc_key(cur_x, cur_y);
			if(cell_info_map.find(cell_loc_key) == cell_info_map.end())
			{
				pair<pair<uint, uint>, float> cell_info(cell_loc_key, 1);
				cell_info_map.insert(cell_info);
			}
			else
				cell_info_map.at(cell_loc_key) = 1;
		}

		void startServiceServers()
		{
			std::string path_srv_name = "/robot_" + std::to_string(rob_id) + "/share_plan";
            path_ss = this->create_service<con_plan_exec_pkg::srv::PlanForHorizon>(
                path_srv_name, std::bind(&RobotClass::receivePath, this, _1, _2));

			std::string stop_rob_srv_name = "/stop_robot_" + to_string(rob_id);
            stop_rob_ss = this->create_service<con_plan_exec_pkg::srv::StopRobot>(
                stop_rob_srv_name, std::bind(&RobotClass::stopRobot, this, _1, _2));
		}

		void readClock(const con_plan_exec_pkg::msg::DiscreteClock::SharedPtr msg)
		{
			{
                std::lock_guard<std::mutex> lock(robot_mutex);
				cur_clk = msg->clk_val;

				if(prev_clk < cur_clk)
				{
					prev_clk = cur_clk;

					if(cur_clk >= start_from_clk && first_req_sent)
					{
						if(cur_clk < send_lv_at_clk)
						{
							goToCardinalState(path[cur_clk - start_from_clk + 1], cur_clk);
							path_len_rem--;
						}
						else if(cur_clk == send_lv_at_clk)
							sendLocalView();
						else if(path_len)
						{
							RCLCPP_ERROR(this->get_logger(), "R_%d LV_%d delayed! CLK(%d) > SEND_LV_CLK(%d)", rob_id, req_id, cur_clk, send_lv_at_clk);
							rclcpp::shutdown();
						}
					}
				}
				cur_intrvl = cur_clk / INTRVL_LEN_FAC;
			}
		}

		void sendLocalView()
		{
			path.clear();
			path_len = 0;

			if(path_len_rem)
			{
				RCLCPP_ERROR(this->get_logger(), "R_%d RPL = %d", rob_id, path_len_rem);
				rclcpp::shutdown();
			}

            auto lv_srv = std::make_shared<con_plan_exec_pkg::srv::ShareLocalInformation::Request>();
			lv_srv->robot_id = rob_id;
			lv_srv->horizon = req_id;

            lv_srv->x = loc_x;
            lv_srv->y = loc_y;
            lv_srv->theta = loc_theta;

			con_plan_exec_pkg::msg::CellInfo cell_info_tmp;
			map<pair<uint, uint>, float>::iterator cell_info_map_it;

			for(cell_info_map_it = cell_info_map.begin(); cell_info_map_it != cell_info_map.end(); ++cell_info_map_it)
			{
				cell_info_tmp.cell_x = (cell_info_map_it->first).first;
				cell_info_tmp.cell_y = (cell_info_map_it->first).second;
				cell_info_tmp.cell_type = cell_info_map_it->second;
				lv_srv->lv_short.push_back(cell_info_tmp);
			}

            RCLCPP_INFO(this->get_logger(), "R_%d Sending Data to Central Brain (Req: %d)...", rob_id, req_id);
            lv_sc->async_send_request(lv_srv);
            req_id++;
            RCLCPP_INFO(this->get_logger(), "R_%d Data Sent! Waiting for flight path.", rob_id);
		}

		void receivePath(const std::shared_ptr<con_plan_exec_pkg::srv::PlanForHorizon::Request> req,
                         std::shared_ptr<con_plan_exec_pkg::srv::PlanForHorizon::Response> res)
		{
			{
                std::lock_guard<std::mutex> lock(robot_mutex);
				uint sfi_tmp = req->start_from_intrvl;

				if(cur_intrvl < sfi_tmp)
				{
					path_len_rem = path_len = req->plans.size() - 1;
					motionPlan tmp_plan;

					for(uint state_id = 0; state_id <= path_len; state_id++)
					{
						tmp_plan.location_x = req->plans[state_id].x;
						tmp_plan.location_y = req->plans[state_id].y;
						tmp_plan.theta = req->plans[state_id].theta;
						path.push_back(tmp_plan);
					}

					start_from_clk = sfi_tmp * INTRVL_LEN_FAC;
					send_lv_at_clk = start_from_clk + path_len;
					cell_info_map.clear();
				}
				else
				{
					RCLCPP_ERROR(this->get_logger(), "R_%d Response delayed! CI (%d) >= SFI (%d)", rob_id, cur_intrvl, sfi_tmp);
					rclcpp::shutdown();
				}
				res->rcvd_intrvl = cur_intrvl;
		    }
		}

		void goToCardinalState(motionPlan state, uint t)
		{
			float loc_x_next = state.location_x;
			float loc_y_next = state.location_y;
			float loc_theta_next = state.theta;

			if((loc_x == loc_x_next) && (loc_y == loc_y_next)) {
				RCLCPP_INFO(this->get_logger(), "R_%d W (%d, %d) @ CLK = %d", rob_id, int(loc_x_next), int(loc_y_next), t + 1);
			}
			else if(loc_x == loc_x_next) {
				if((loc_y + 1) == loc_y_next) RCLCPP_INFO(this->get_logger(), "R_%d MN (%d, %d) @ CLK = %d", rob_id, int(loc_x_next), int(loc_y_next), t + 1);
				else if((loc_y - 1) == loc_y_next) RCLCPP_INFO(this->get_logger(), "R_%d MS (%d, %d) @ CLK = %d", rob_id, int(loc_x_next), int(loc_y_next), t + 1);
			}
			else if(loc_y == loc_y_next) {
				if((loc_x + 1) == loc_x_next) RCLCPP_INFO(this->get_logger(), "R_%d ME (%d, %d) @ CLK = %d", rob_id, int(loc_x_next), int(loc_y_next), t + 1);
				else if((loc_x - 1) == loc_x_next) RCLCPP_INFO(this->get_logger(), "R_%d MW (%d, %d) @ CLK = %d", rob_id, int(loc_x_next), int(loc_y_next), t + 1);
			}

			loc_x = loc_x_next;
			loc_y = loc_y_next;
			loc_theta = loc_theta_next;

			updateLocalView();

            // FIXED: Non-blocking. Just sets target and immediately releases the thread
            current_target_pose.pose.position.x = loc_x * CELL_SIZE;
            current_target_pose.pose.position.y = loc_y * CELL_SIZE;
            current_target_pose.pose.position.z = TAKEOFF_ALT;
		}

		void stopRobot(const std::shared_ptr<con_plan_exec_pkg::srv::StopRobot::Request> req,
                       std::shared_ptr<con_plan_exec_pkg::srv::StopRobot::Response> res)
		{
            (void)res;
			if(req->stop_robot)
			{
				RCLCPP_INFO(this->get_logger(), "R_%d Coverage Completed (%d)", rob_id, req_id);
				rclcpp::shutdown();
			}
		}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<RobotClass>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
	return 0;
}
