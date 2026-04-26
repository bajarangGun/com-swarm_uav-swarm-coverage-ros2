/*
Purpose: Concurrent Coverage Planner (ROS 2 Jazzy Port)
*/

#include "con_plan_exec_pkg/concpp_main.h"
#include "con_plan_exec_pkg/rviz.h"

using namespace std;
using namespace std::placeholders;

ConCPP_MAIN::ConCPP_MAIN() : Node("concpp_node")
{
    this->declare_parameter("ws_x", 100);
    this->declare_parameter("ws_y", 100);
    this->declare_parameter("rc", 2);
    this->declare_parameter("comm_range", 6.0);
    

    ws_size_x = this->get_parameter("ws_x").as_int();
    ws_size_y = this->get_parameter("ws_y").as_int();
    rob_count = this->get_parameter("rc").as_int();
    
	TH_req = rob_count;
	cout << "Workspace size = " << ws_size_x << " x " << ws_size_y << endl;

	cout << "#Robots = " << rob_count << " (QC)" << endl;

	float_mat ws_init(ws_size_x, float_vec(ws_size_y, -1));
	ws = ws_init;
	grid_visualizer.init_rviz(ws_size_x, ws_size_y, ws, this);
	bool_mat assigned_goals_init(ws_size_x, bool_vec(ws_size_y, false));
	assigned_goals = assigned_goals_init;

	obs_count = unassigned_goal_count = cov_count = 0;
	req_count = req_count_old = 0;

	bool_vec S_req_init(rob_count, false);
	S_req = S_req_old = S_req_init;

	robot_status rob_state_init;
	rob_state_init.x = -1; rob_state_init.y = -1; rob_state_init.theta = 0;
	robot_status_vec_t rob_states_init(rob_count, rob_state_init);
	rob_states = rob_states_old = rob_states_init;

	loc_mat paths_init(rob_count, loc_vec());
    paths = paths_init;

	start_from_intrvl = 1;
	uint_vec stops_at_clk_init(rob_count, 0);
	stops_at_clk = stops_at_clk_init;
	flag_running = false;
	call_id = 0;

    lv_ss = this->create_service<con_plan_exec_pkg::srv::ShareLocalInformation>(
        "/concpp_node/share_workspace", std::bind(&ConCPP_MAIN::getLocalView, this, _1, _2));

    write_clk_pub = this->create_publisher<con_plan_exec_pkg::msg::DiscreteClock>("/concpp_node/discrete_clock", 1000);
	tot_comp_time = 0;
	last_req_exp_at_clk = 0;

	op_dir_path = ament_index_cpp::get_package_share_directory("con_plan_exec_pkg") + "/input/";

	std::string rpc_file_path = op_dir_path + RESULT_PER_CALL_FILE_NAME;
	ofstream rpc_file(rpc_file_path.c_str()); rpc_file.close();

	std::string cti_file_path = op_dir_path + COMP_TIME_INFO_FILE_NAME;
	ofstream cti_file(cti_file_path.c_str()); cti_file.close();

	std::string capl_file_path = op_dir_path + COL_AVRT_PATH_LENGTHS_FILE;
	ofstream capl_file(capl_file_path.c_str()); capl_file.close();

	initializeClock();
    std::thread(&ConCPP_MAIN::writeClock, this).detach();
}

void ConCPP_MAIN::getLocalView(const std::shared_ptr<con_plan_exec_pkg::srv::ShareLocalInformation::Request> req, 
                               std::shared_ptr<con_plan_exec_pkg::srv::ShareLocalInformation::Response> res)
{
    std::thread(&ConCPP_MAIN::handleRequest, this, *req).detach();
}

void ConCPP_MAIN::handleRequest(con_plan_exec_pkg::srv::ShareLocalInformation::Request req)
{
    {
        std::lock_guard<std::mutex> lock(core_mutex);
        req_count++;

        if(!call_id && (req_count == rob_count))
        {
            mis_begun_at = this->get_clock()->now();
            clk_timer = this->create_wall_timer(
                std::chrono::milliseconds(MOT_PREM_EXEC_TIME * 1000),
                std::bind(&ConCPP_MAIN::incrementClock, this));
        }
        
        int rob_id = int(req.robot_id);
        S_req[rob_id] = true;
        rob_states[rob_id].x = int(req.x);
        rob_states[rob_id].y = int(req.y);
        rob_states[rob_id].theta = double(req.theta);

        updateGlobalView(req);
    }
	checkCoveragePlanningCriteria();
}

void ConCPP_MAIN::updateGlobalView(con_plan_exec_pkg::srv::ShareLocalInformation::Request req)
{
	size_t lv_short_size = req.lv_short.size();
	for(size_t lv_index = 0; lv_index < lv_short_size; lv_index++)
	{
		con_plan_exec_pkg::msg::CellInfo cell_info_tmp = req.lv_short[lv_index];
		float lv = float(cell_info_tmp.cell_type);
		float gv_old = ws[cell_info_tmp.cell_x][cell_info_tmp.cell_y];

		if(lv == 0)	
		{
			if(gv_old == -1)
			{
				ws[cell_info_tmp.cell_x][cell_info_tmp.cell_y] = 0;
				obs_count++;
			}
		}
		else if(lv == 0.5)
		{
			if(gv_old == -1)
			{
				ws[cell_info_tmp.cell_x][cell_info_tmp.cell_y] = 0.5;
				unassigned_goal_count++;
			}
		}
		else
		{
			if(gv_old != 1)
			{
				ws[cell_info_tmp.cell_x][cell_info_tmp.cell_y] = 1;
				cov_count++;

				if(gv_old == 0)
					obs_count--;
				else if((gv_old == 0.5) && !assigned_goals[cell_info_tmp.cell_x][cell_info_tmp.cell_y])
					unassigned_goal_count--;
			}
		}
	}
	grid_visualizer.update_rviz(ws, this);
}

void ConCPP_MAIN::printGlobalView()
{
	std::string gv_file_path = op_dir_path + "gv_" + std::to_string(call_id) + ".txt";
	ofstream gv_file;
	gv_file.open(gv_file_path.c_str());

	for(uint row_id = 0; row_id < (uint)ws_size_x; row_id++)
	{
		for(uint col_id = 0; col_id < (uint)ws_size_y; col_id++)
		{
			gv_file << ws[row_id][col_id] << ", ";
		}
		gv_file << endl;
	}
	gv_file.close();
}

void ConCPP_MAIN::checkCoveragePlanningCriteria()
{
    std::lock_guard<std::mutex> lock(core_mutex);
    if(!flag_running)
    {
        if(unassigned_goal_count)
        {
            if((!call_id && (req_count == rob_count)) || (call_id && (req_count >= TH_req)))
            {
                bool flag_robs_changed = false;

                if(req_count_old != req_count)
                    flag_robs_changed = true;
                else
                    for(int rob_id = 0; rob_id < rob_count; rob_id++)
                        if((!S_req_old[rob_id] && S_req[rob_id]) || (S_req_old[rob_id] && !S_req[rob_id]))
                        {
                            flag_robs_changed = true;
                            break;
                        }
                        else if(S_req_old[rob_id] && S_req[rob_id])
                        {
                            robot_status rob_state_old = rob_states_old[rob_id];
                            robot_status rob_state = rob_states[rob_id];

                            if((rob_state_old.x != rob_state.x) || (rob_state_old.y != rob_state.y) || (rob_state_old.theta != rob_state.theta))
                            {
                                flag_robs_changed = true;
                                break;
                            }
                        }

                if(flag_robs_changed)
                {
                    flag_running = true;

                    float_mat ws_old = ws;
                    bool_mat assigned_goals_old = assigned_goals;
                    uint obs_count_old = obs_count;
                    uint unassigned_goal_count_old = unassigned_goal_count;
                    uint cov_count_old = cov_count;

                    S_req_old = S_req;
                    req_count_old = req_count;
                    req_count = 0;		

                    rob_states_old = rob_states;

                    robot_status rob_state;
                    struct loc rob_init_state;
                    loc_vec req_rob_states;

                    for(int rob_id = 0; rob_id < rob_count; rob_id++)
                        if(S_req_old[rob_id])
                        {
                            S_req[rob_id] = false;	
                            rob_state = rob_states_old[rob_id];

                            rob_init_state.x = rob_state.x;
                            rob_init_state.y = rob_state.y;
                            rob_init_state.theta = (int)rob_state.theta;

                            req_rob_states.push_back(rob_init_state);
                        }

                    std::thread(&ConCPP_MAIN::startCoveragePlanning, this, ws_old, assigned_goals_old, obs_count_old, unassigned_goal_count_old, cov_count_old, req_rob_states).detach();
                }
            }
        }
        else if(req_count == rob_count)
        {
            stopCoveragePlanning();
        }
    }
}

void ConCPP_MAIN::startCoveragePlanning(float_mat ws_old, bool_mat assigned_goals_old, uint obs_count_old, uint unassigned_goal_count_old, uint cov_count_old, loc_vec req_rob_states)
{
	bool_mat ws_graph(ws_size_x, bool_vec(ws_size_y, true));
	loc_vec unassigned_goal_locs;
	struct loc goal_loc;

	for(uint row_id = 0; row_id < (uint)ws_size_x; row_id++)
		for(uint col_id = 0; col_id < (uint)ws_size_y; col_id++)
		{
			float gv_old = ws_old[row_id][col_id];

			if((gv_old == -1) || (gv_old == 0))
				ws_graph[row_id][col_id] = false;
			else if((gv_old == 0.5) && !assigned_goals_old[row_id][col_id])
			{
				goal_loc.x = row_id;
				goal_loc.y = col_id;
				unassigned_goal_locs.push_back(goal_loc);
			}
		}

	int_vec opt_goal_vec;
	uint act_count = 0;
	rclcpp::Time comp_begun_at = this->get_clock()->now();
	
    uint comp_begun_at_clk;
    {
        std::lock_guard<std::mutex> lock(core_mutex);
	    comp_begun_at_clk = clock;
    }
        float comm_range = this->get_parameter("comm_range").as_double();
	GAP gap_obj;
	paths = gap_obj.get_cost_optimal_paths(ws_size_x, ws_size_y, ws_graph, req_count_old, req_rob_states, unassigned_goal_count_old, unassigned_goal_locs, call_id, S_req_old, paths, opt_goal_vec, act_count,comm_range);
	
	rclcpp::Time comp_ended_at = this->get_clock()->now();
	rclcpp::Duration comp_time = comp_ended_at - comp_begun_at;

	uint comp_ended_at_clk;
    {
        std::lock_guard<std::mutex> lock(core_mutex);
	    comp_ended_at_clk = clock;
    }

	uint fsbl_comp_intrvl = 0;
	uint look_ahead_intrvl = 1;
	bool flag_send_paths = false;

	while(!flag_send_paths)
	{
		rclcpp::Time cfp_begun_at = this->get_clock()->now();
		uint cfp_begun_at_clk;
        
        {
            std::lock_guard<std::mutex> lock(core_mutex);
		    cfp_begun_at_clk = clock;
        }

		paths = gap_obj.get_col_free_paths(ws_size_x, ws_size_y, req_count_old, call_id, S_req_old, opt_goal_vec, paths, act_count, look_ahead_intrvl, start_from_intrvl, fsbl_comp_intrvl,comm_range);

		rclcpp::Time cfp_ended_at = this->get_clock()->now();
		rclcpp::Duration cfp_time = cfp_ended_at - cfp_begun_at;

		uint cfp_ended_at_clk;
		uint cur_intrvl;
		uint look_ahead_intrvl_bkp = look_ahead_intrvl;

		uint act_rob_count = 0;
		uint max_pl = 0;
		uint TH_req_old = TH_req;

		{
            std::lock_guard<std::mutex> lock(core_mutex);
			cfp_ended_at_clk = clock;
			cur_intrvl = cfp_ended_at_clk / INTRVL_LEN_FAC;

            // SANITY CLAMP: Override the ghost memory bug from the math logic 
            // to guarantee the drones never go to sleep for 8 hours
            if (start_from_intrvl > cur_intrvl + 5) {
                start_from_intrvl = cur_intrvl + 1;
            }

			if(cur_intrvl < start_from_intrvl)
			{
				flag_send_paths = true;
				TH_req = 0;

				loc_vec path;
				uint path_len;
				struct loc rob_goal_state;
				uint send_lv_at_clk;

				for(int rob_id = 0; rob_id < rob_count; rob_id++)
					if(S_req_old[rob_id])
					{
						path = paths[rob_id];
						path_len = path.size() - 1;

			    		if(path_len)
			    		{
			    			act_rob_count++;
		    				rob_goal_state = path[path_len];
			    			assigned_goals[rob_goal_state.x][rob_goal_state.y] = true;

			    			if(ws[rob_goal_state.x][rob_goal_state.y] != 1)
								unassigned_goal_count--;

							if(max_pl < path_len)
								max_pl = path_len;

							send_lv_at_clk = start_from_intrvl * INTRVL_LEN_FAC + path_len;
							stops_at_clk[rob_id] = send_lv_at_clk - 1;

							if(last_req_exp_at_clk < send_lv_at_clk)
							{
								last_req_exp_at_clk = send_lv_at_clk;
								last_req_exp_from_rob = rob_id;
							}

                            std::thread(&ConCPP_MAIN::callRobot, this, call_id, start_from_intrvl, path_len, path, rob_id).detach();
			    		}
			    		else
		    			{
			    			S_req[rob_id] = true;
			    			req_count++;
			    			TH_req++;
			    		}
			    	}
			    	else
			    		if(stops_at_clk[rob_id] < cfp_ended_at_clk)
			    			TH_req++;

			    if(!TH_req)
			    {
			    	bool init = true;
			    	uint stops_at_clk_min = 0;

			    	for(int rob_id = 0; rob_id < rob_count; rob_id++)
			    	{
			    		uint stops_at_clk_tmp = stops_at_clk[rob_id];

			    		if(stops_at_clk_tmp >= cfp_ended_at_clk)
			    			if(init)
			    			{
			    				stops_at_clk_min = stops_at_clk_tmp;
			    				init = false;
			    			}
			    			else if(stops_at_clk_min > stops_at_clk_tmp)
			    				stops_at_clk_min = stops_at_clk_tmp;
			    	}

			    	for(int rob_id = 0; rob_id < rob_count; rob_id++)
			    		if(stops_at_clk[rob_id] == stops_at_clk_min)
			    			TH_req++;
			    }
			}
			else
			{
				TH_req = req_count_old;
				rclcpp::Time cfp_exp_end_at = cfp_ended_at + cfp_time + rclcpp::Duration::from_seconds(0.001);
				rclcpp::Duration cfp_exp_time = cfp_exp_end_at - mis_begun_at;
				uint cfp_exp_end_at_clk = cfp_exp_time.seconds() / MOT_PREM_EXEC_TIME;
				uint cfp_exp_end_at_intrvl = cfp_exp_end_at_clk / INTRVL_LEN_FAC;
				uint intrvl_dif = cfp_exp_end_at_intrvl - cur_intrvl;
				look_ahead_intrvl = 1 + intrvl_dif;
			}
		}

		printCallInformation(req_count_old, TH_req_old, obs_count_old, unassigned_goal_count_old, cov_count_old, comp_time.seconds(), comp_begun_at, comp_ended_at, comp_begun_at_clk, comp_ended_at_clk, cfp_time.seconds(), cfp_begun_at, cfp_ended_at, cfp_begun_at_clk, cfp_ended_at_clk, fsbl_comp_intrvl, look_ahead_intrvl_bkp, cur_intrvl, act_rob_count, max_pl, flag_send_paths);
		call_id++;
		tot_comp_time += comp_time.seconds() + cfp_time.seconds();
		comp_time = rclcpp::Duration::from_seconds(0);
	}

    {
        std::lock_guard<std::mutex> lock(core_mutex);
	    flag_running = false;
    }
    std::thread(&ConCPP_MAIN::checkCoveragePlanningCriteria, this).detach();
}

void ConCPP_MAIN::callRobot(uint call_id_old, uint start_from_intrvl_old, uint path_len_old, loc_vec path_old, uint rob_id)
{
    auto pfh_srv = std::make_shared<con_plan_exec_pkg::srv::PlanForHorizon::Request>();
	pfh_srv->hor_id = call_id;
	pfh_srv->start_from_intrvl = start_from_intrvl;

	con_plan_exec_pkg::msg::PlanInstance pi_msg;
	struct loc rob_state;

	for(uint state_id = 0; state_id <= path_len_old; state_id++)
	{
		rob_state = path_old[state_id];
		pi_msg.x = rob_state.x;
		pi_msg.y = rob_state.y;
		pi_msg.theta = rob_state.theta;
		pfh_srv->plans.push_back(pi_msg);
	}

	std::string path_srv_name = "/robot_" + to_string(rob_id) + "/share_plan";
    auto path_sc = this->create_client<con_plan_exec_pkg::srv::PlanForHorizon>(path_srv_name);
	
    // Block thread and force packet to physically leave the network card before deleting request
    if (path_sc->wait_for_service(std::chrono::seconds(2))) {
        auto result_future = path_sc->async_send_request(pfh_srv);
        result_future.wait();
    }
}

void ConCPP_MAIN::printCallInformation(uint req_count_old, uint TH_req_old, uint obs_count_old, uint unassigned_goal_count_old, uint cov_count_old, double comp_time, rclcpp::Time comp_begun_at, rclcpp::Time comp_ended_at, uint comp_begun_at_clk, uint comp_ended_at_clk, double cfp_time, rclcpp::Time cfp_begun_at, rclcpp::Time cfp_ended_at, uint cfp_begun_at_clk, uint cfp_ended_at_clk, uint fsbl_comp_intrvl, uint look_ahead_intrvl_bkp, uint cur_intrvl, uint act_rob_count, int max_pl, bool flag_send_paths)
{
	fstream rpc_file;
	std::string rpc_file_path = op_dir_path + RESULT_PER_CALL_FILE_NAME;
	rpc_file.open(rpc_file_path.c_str(), fstream::app);

	std::string cti_file_path = op_dir_path + COMP_TIME_INFO_FILE_NAME;
	fstream cti_file;
	cti_file.open(cti_file_path.c_str(), fstream::app);

	if(call_id)
	{
		rpc_file << endl;
		cti_file << endl;
	}

	rpc_file << "Call = " << call_id << ", #Req = " << req_count_old << " # " << TH_req_old << ", #O = " << obs_count_old << ", #UG = " << unassigned_goal_count_old << ", #C = " << cov_count_old << ", CT = " << fixed << (comp_time + cfp_time) << ", FCI = " << fsbl_comp_intrvl << ", LAI = " << look_ahead_intrvl_bkp << ", SFI = " << start_from_intrvl << ", CI = " << cur_intrvl << ", #Act = " << act_rob_count << ", Max PL = " << max_pl << ", " << flag_send_paths;
	cti_file << "Call = " << call_id << ", COP = " << fixed << comp_time << " # " << fixed << comp_begun_at.seconds() << " : " << fixed << comp_ended_at.seconds() << " # " << comp_begun_at_clk << " : " << comp_ended_at_clk << ", CFP = " << fixed << cfp_time << " # " << fixed << cfp_begun_at.seconds() << " : " << fixed << cfp_ended_at.seconds() << " # " << cfp_begun_at_clk << " : " << cfp_ended_at_clk;

	rpc_file.close();
	cti_file.close();

	cout << "\nCall = " << call_id << ", #Req = " << req_count_old << " # " << TH_req_old << ", #UG = " << unassigned_goal_count_old << ", #C = " << cov_count_old << ", CT = " << fixed << (comp_time + cfp_time) << ", " << fsbl_comp_intrvl << " + " << look_ahead_intrvl_bkp << " = " << start_from_intrvl << " > " << cur_intrvl << ", #Act = " << act_rob_count << ", Max PL = " << max_pl << ", " << flag_send_paths << endl;
}

void ConCPP_MAIN::stopCoveragePlanning()
{
	clk_timer->cancel();
	rclcpp::Time mis_ended_at = this->get_clock()->now();
	rclcpp::Duration mis_time = mis_ended_at - mis_begun_at;
	stopRobots();

	ofstream r_file;
	std::string r_file_path = op_dir_path + RESULT_FILE;
	r_file.open(r_file_path.c_str());
	r_file << "Workspace Size = " << ws_size_x << " x " << ws_size_y << ", #Cov = " << cov_count << ", #Robots = " << rob_count << ", #Calls = " << call_id << ", Total Computation Time = " << fixed << tot_comp_time << ", Mission Time = " << fixed << mis_time.seconds() << " # " << fixed << mis_begun_at.seconds() << " : " << fixed << mis_ended_at.seconds();
	
	
	r_file << ", QuadCopter";
	

	r_file.close();

	cout << "\n\n******************** Coverage Completed ********************";
	cout << "\n#Calls = " << call_id << ", #Cov = " << cov_count << ", TCT = " << tot_comp_time << ", MT = " << mis_time.seconds() << endl;
	rclcpp::shutdown();
}

void ConCPP_MAIN::writeClkFile(uint clk)
{
	ofstream clk_file;
	std::string clk_file_path = op_dir_path + "clock.txt";
	clk_file.open(clk_file_path.c_str());
	clk_file << clk;
	clk_file.close();
}

void ConCPP_MAIN::initializeClock()
{
	clock = 0;
	writeClkFile(clock);
}

void ConCPP_MAIN::writeClock()
{
	con_plan_exec_pkg::msg::DiscreteClock dc_msg;
	rclcpp::Rate loop_rate(LOOP_RATE);

	while(rclcpp::ok())
	{
        {
            std::lock_guard<std::mutex> lock(core_mutex);
		    dc_msg.clk_val = clock;
        }
		write_clk_pub->publish(dc_msg);
		loop_rate.sleep();
	}
}

void ConCPP_MAIN::incrementClock()
{
    std::lock_guard<std::mutex> lock(core_mutex);
    clock++;
    writeClkFile(clock);
}

void ConCPP_MAIN::stopRobots()
{
    auto sr_srv = std::make_shared<con_plan_exec_pkg::srv::StopRobot::Request>();
	sr_srv->stop_robot = true;

    // --- NEW: Keep the transmitters alive in memory! ---
    std::vector<rclcpp::Client<con_plan_exec_pkg::srv::StopRobot>::SharedPtr> clients;

	for(int rob_id = 0; rob_id < rob_count; rob_id++)
	{
		std::string sr_srv_name = "/stop_robot_" + to_string(rob_id);
        auto sr_sc = this->create_client<con_plan_exec_pkg::srv::StopRobot>(sr_srv_name);
        
        clients.push_back(sr_sc); // Prevent the transmitter from being destroyed
        sr_sc->async_send_request(sr_srv);
	}
    
    // Gives the ROS 2 executor time to actually send the packets from the alive clients
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

int main(int argc, char *argv[])
{	
	rclcpp::init(argc, argv);
    auto concpp_main_obj = std::make_shared<ConCPP_MAIN>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(concpp_main_obj);
    executor.spin();

	rclcpp::shutdown();
	return 0;
}
