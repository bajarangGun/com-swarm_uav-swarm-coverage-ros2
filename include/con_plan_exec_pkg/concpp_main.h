/*
Purpose: Concurrent Coverage Planner (ROS 2 Jazzy Port)
*/

#pragma once

#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <stack>
#include <thread>
#include <chrono>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "con_plan_exec_pkg/gap.h"
#include "con_plan_exec_pkg/srv/plan_for_horizon.hpp"
#include "con_plan_exec_pkg/srv/share_local_information.hpp"
#include "con_plan_exec_pkg/srv/stop_robot.hpp"
#include "con_plan_exec_pkg/msg/discrete_clock.hpp"
#include "con_plan_exec_pkg/rviz.h"

#define RESULT_PER_CALL_FILE_NAME "resultPerCall.txt"
#define COMP_TIME_INFO_FILE_NAME "comp_time_info.txt"
#define RESULT_FILE "result.txt"

using namespace std;

class ConCPP_MAIN : public rclcpp::Node
{
	public:
		ConCPP_MAIN(); 

		void getLocalView(const std::shared_ptr<con_plan_exec_pkg::srv::ShareLocalInformation::Request> req, 
                          std::shared_ptr<con_plan_exec_pkg::srv::ShareLocalInformation::Response> res);
		void handleRequest(con_plan_exec_pkg::srv::ShareLocalInformation::Request req);
		void updateGlobalView(con_plan_exec_pkg::srv::ShareLocalInformation::Request req);
		void printGlobalView();
		void checkCoveragePlanningCriteria();
		void stopCoveragePlanning();
		void startCoveragePlanning(float_mat ws_old, bool_mat assigned_goals_old, uint obs_count_old, uint unassigned_goal_count_old, uint cov_count_old, loc_vec req_rob_states);
		void callRobot(uint call_id_old, uint start_from_intrvl_old, uint path_len_old, loc_vec path_old, uint rob_id);
		void printCallInformation(uint req_count_old, uint TH_req_old, uint obs_count_old, uint unassigned_goal_count_old, uint cov_count_old, double comp_time, rclcpp::Time comp_begun_at, rclcpp::Time comp_ended_at, uint comp_begin_clk, uint comp_end_clk, double cfp_time, rclcpp::Time cfp_begun_at, rclcpp::Time cfp_ended_at, uint cfp_begin_clk, uint cfp_end_clk, uint fsbl_comp_intrvl, uint look_ahead_intrvl_bkp, uint cur_intrvl, uint act_rob_count, int max_pl, bool flag_send_paths);
		void writeClkFile(uint clk);
		void initializeClock();
		void writeClock();
		void incrementClock();
		void stopRobots();

		string op_dir_path;	

		int ws_size_x;		
		int ws_size_y;
		int rob_count;		
		int TH_req;			

		float_mat ws;					
		bool_mat assigned_goals;		
		uint obs_count;
		uint unassigned_goal_count;
		uint cov_count;

		int req_count;						
		int req_count_old;
		bool_vec S_req;						
		bool_vec S_req_old;
		robot_status_vec_t rob_states;		
		robot_status_vec_t rob_states_old;	

		rclcpp::Time mis_begun_at;			

		uint clock;					
		rclcpp::TimerBase::SharedPtr clk_timer;		
        rclcpp::Publisher<con_plan_exec_pkg::msg::DiscreteClock>::SharedPtr write_clk_pub; 

		rclcpp::Service<con_plan_exec_pkg::srv::ShareLocalInformation>::SharedPtr lv_ss; 
		bool flag_running;				
		uint call_id;					

		loc_mat paths;					
		uint start_from_intrvl;			
		uint_vec stops_at_clk;
		uint last_req_exp_from_rob;		
		uint last_req_exp_at_clk;		
		
		double tot_comp_time;			
		
        std::mutex core_mutex; // Replaces #pragma omp critical
        cRVIZ grid_visualizer;
};
