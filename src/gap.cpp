/*
Purpose: Asynchronous CPP
Last updated: 
Last updated on: 20 april 2026
To do: 
Author: Divyansh Singh
*/


#include "con_plan_exec_pkg/gap.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <string>
#include <cmath>
#include <cfloat>

//#define COMM_RANGE_GRID 6


//==================================================1. Optimal Costs : START
void GAP::showQueue(queue<struct loc> BFS_QUEUE)
{
	while(!BFS_QUEUE.empty())
	{
		struct loc temp_loc = BFS_QUEUE.front(); 
        BFS_QUEUE.pop();

		cout << "(" << temp_loc.x << ", " << temp_loc.y << ", " << temp_loc.theta << ") ";
	}
}


void GAP::showMatrix(int_mat v_2d)
{
	int v_2d_size = v_2d[0].size();
	int i, j;

	//for(j = v_2d_size - 1; j >= 0; j--)
	for(i = 0; i < v_2d_size; i++)
	{
		//for(i = 0; i < v_2d_size; i++)
		for(j = 0; j < v_2d_size; j++)
		{
			cout << v_2d[i][j] << " ";
		}

		cout << endl;
	}
}


int_mat GAP::bfs_drone(int ws_size_x, int ws_size_y, bool_mat ws_graph, struct loc cur_rob_state)
{
	int cur_rob_x = cur_rob_state.x;
	int cur_rob_y = cur_rob_state.y;
	// int cur_rob_theta = cur_rob_state.theta;
	//cout << "(" << cur_rob_x << ", " << cur_rob_y << ", " << cur_rob_theta <<")";

	
	int_mat cost_mat(ws_size_x, int_vec(ws_size_y, COST_INF));
	

	//color[cur_rob_x][cur_rob_y] = 2;
	cost_mat[cur_rob_x][cur_rob_y] = 0;

	//==================================================EnQueue : START
	queue<struct loc> BFS_QUEUE;
	struct loc temp_loc;
	temp_loc.x = cur_rob_x;
	temp_loc.y = cur_rob_y;
	// temp_loc.theta = cur_rob_theta;
	BFS_QUEUE.push(temp_loc);

	//printf("After enqueue...\n");showQueue(BFS_QUEUE);
	//==================================================EnQueue : END
	//direction[cur_rob_x][cur_rob_y] = cur_rob_theta;

	while(!BFS_QUEUE.empty())
	{
		//==================================================DeQueue : START
		temp_loc = BFS_QUEUE.front(); 
    	BFS_QUEUE.pop();

		int cur_x = temp_loc.x;		//Coordinate of the current cell
		int cur_y = temp_loc.y;
		// int cur_theta = temp_loc.theta;

		//printf("After dequeue...\n");showQueue(BFS_QUEUE);
		//==================================================DeQueue : END

		// if(direction[cur_x][cur_y] == cur_theta)
		// {
			int nbr_x = cur_x + 1;		//Right neighbor
			int nbr_y = cur_y;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						
						cost_mat[nbr_x][nbr_y] = temp_dist;
						

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

					
						//==================================================EnQueue : END
					}
				// }
			}

			nbr_x = cur_x;		//Top neighbor
			nbr_y = cur_y + 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END
					}
				// }
			}

			nbr_x = cur_x - 1;		//Left neighbor
			nbr_y = cur_y;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
					
						cost_mat[nbr_x][nbr_y] = temp_dist;
						

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("After enqueue...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END
					}
				// }
			}

			nbr_x = cur_x;		//Bottom neighbor
			nbr_y = cur_y - 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						//predecessor_x[nbr_x][nbr_y] = cur_x;
						//predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("After enqueue...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END
					}
				// }
			}

		
	}

	
	return cost_mat;
}
bool GAP::is_network_connected(struct loc goal_loc, int eval_rob_id, loc_vec robs_states, bool_vec S_r, loc_mat paths_old, float r_max)
{
    int rob_count = S_r.size();
    loc_vec swarm_locs(rob_count);

    int req_idx = 0;
    for (int k = 0; k < rob_count; k++) {
        if (k == eval_rob_id) {
            swarm_locs[k] = goal_loc; // Pretend the evaluating drone is already at the goal
            if (S_r[k]) req_idx++;    
        } else if (S_r[k]) {
            swarm_locs[k] = robs_states[req_idx]; // Other participants haven't moved yet
            req_idx++;
        } else {
            if (paths_old[k].size() > 0) {
                swarm_locs[k] = paths_old[k].back();
            } else {
                swarm_locs[k] = goal_loc; // Failsafe fallback
            }
        }
    }

    bool_vec in_tree(rob_count, false);
    vector<float> min_dist(rob_count, FLT_MAX);

    in_tree[0] = true; // Start the tree with Drone 0
    for (int i = 1; i < rob_count; i++) {
        float dx = swarm_locs[0].x - swarm_locs[i].x;
        float dy = swarm_locs[0].y - swarm_locs[i].y;
        min_dist[i] = sqrt(dx*dx + dy*dy);
    }

    for (int step = 1; step < rob_count; step++) {
        float best_dist = FLT_MAX;
        int best_node = -1;

        for (int i = 0; i < rob_count; i++) {
            if (!in_tree[i] && min_dist[i] < best_dist) {
                best_dist = min_dist[i];
                best_node = i;
            }
        }

        if (best_dist > r_max) {
            return false;
        }

        in_tree[best_node] = true;

        for (int i = 0; i < rob_count; i++) {
            if (!in_tree[i]) {
                float dx = swarm_locs[best_node].x - swarm_locs[i].x;
                float dy = swarm_locs[best_node].y - swarm_locs[i].y;
                float dist = sqrt(dx*dx + dy*dy);
                if (dist < min_dist[i]) {
                    min_dist[i] = dist;
                }
            }
        }
    }
    return true; // All drones successfully connected within the threshold
}

int_mat GAP::compute_optimal_costs(bool_mat ws_graph, loc_vec robs_states, loc_vec goals_locs, bool_vec S_r, loc_mat paths_old,float comm_range)
{
	uint ws_size_x = ws_graph.size();			
	uint ws_size_y = ws_graph[0].size();
	int num_of_reqs = robs_states.size();
	int goal_count = goals_locs.size();
	int_mat opt_cost_mat(num_of_reqs, int_vec(goal_count, -1));

	// --- NEW: Calculate the Center of Mass (CoM) of the Swarm ---
	float com_x = 0;
	float com_y = 0;
	for (int i = 0; i < num_of_reqs; i++) {
		com_x += robs_states[i].x;
		com_y += robs_states[i].y;
	}
	if (num_of_reqs > 0) {
		com_x /= num_of_reqs;
		com_y /= num_of_reqs;
	}
	// ------------------------------------------------------------

	int req_idx = 0;
	for(uint rob_id = 0; rob_id < S_r.size(); rob_id++)
	{
		if (!S_r[rob_id]) continue; // Skip non-participants

		struct loc cur_rob_state = robs_states[req_idx];
		
		int_mat cost_mat = bfs_drone(ws_size_x, ws_size_y, ws_graph, cur_rob_state);

		for(int goal_index = 0; goal_index < goal_count; goal_index++)
		{
			struct loc cur_goal_loc = goals_locs[goal_index];
			int travel_cost = cost_mat[cur_goal_loc.x][cur_goal_loc.y];

			if (travel_cost != COST_INF) {
				
				// --- NEW: The Elastic Tension Penalty ---
				float dist_to_com = sqrt(pow(cur_goal_loc.x - com_x, 2) + pow(cur_goal_loc.y - com_y, 2));
				int tension_penalty = 0;
				
				// If goal is stretching past 50% of the tether from the center, add rubber-band penalty
				if (dist_to_com > (comm_range * 0.5)) {
					tension_penalty = (int)(pow((dist_to_com - (comm_range * 0.5)), 2) * 10); 
				}
				
				int final_cost = travel_cost + tension_penalty;
				// ----------------------------------------

				// Instead of COST_INF, we add a massive, but valid, mathematical penalty.
				// This allows the Hungarian algorithm to still process the matrix without crashing or going rogue.
				bool connected = is_network_connected(cur_goal_loc, rob_id, robs_states, S_r, paths_old, comm_range);
				if (!connected) {
					final_cost += 5000; 
				}

				opt_cost_mat[req_idx][goal_index] = final_cost;
			} else {
				opt_cost_mat[req_idx][goal_index] = COST_INF;
			}
		}
		req_idx++;
	}

	return opt_cost_mat;
}
//==================================================1. Optimal Costs : END


loc_vec GAP::optimal_path_bfs_drone(int ws_size_x, int ws_size_y, bool_mat ws_graph, struct loc rob_loc, struct loc goal_loc)
{
	int cur_rob_x = rob_loc.x;
	int cur_rob_y = rob_loc.y;
	// int cur_rob_theta = rob_loc.theta;
	//cout << "Robot (" << cur_rob_x << ", " << cur_rob_y << ")\t\t";

	int cur_goal_x = goal_loc.x;
	int cur_goal_y = goal_loc.y;
	//cout << "Goal (" << cur_goal_x << ", " << cur_goal_y << ")" << endl;

	//int_mat color(ws_size_x, int_vec(ws_size_y, 1));		//1 : White, 2 : Grey, 3 : Black
	int_mat cost_mat(ws_size_x, int_vec(ws_size_y, COST_INF));
	int_mat predecessor_x(ws_size_x, int_vec(ws_size_y, -1));		//Predecessor cell of a cell
	int_mat predecessor_y(ws_size_x, int_vec(ws_size_y, -1));
	// int_mat direction(ws_size_x, int_vec(ws_size_y, -1));

	//color[cur_rob_x][cur_rob_y] = 2;
	cost_mat[cur_rob_x][cur_rob_y] = 0;

	//==================================================EnQueue : START
	queue<struct loc> BFS_QUEUE;
	BFS_QUEUE.push(rob_loc);
	//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
	//==================================================EnQueue : END
	// direction[cur_rob_x][cur_rob_y] = cur_rob_theta;

	while(!BFS_QUEUE.empty())
	{
		//==================================================DeQueue : START
		struct loc temp_loc = BFS_QUEUE.front(); 
    	BFS_QUEUE.pop();

		int cur_x = temp_loc.x;		//Coordinate of the current cell
		int cur_y = temp_loc.y;
		// int cur_theta = temp_loc.theta;

		//printf("\nAfter dequeue...\n");showQueue(BFS_QUEUE);
		//==================================================DeQueue : END

		// if(direction[cur_x][cur_y] == cur_theta)
		// {
			int nbr_x = cur_x + 1;		//Right neighbor
			int nbr_y = cur_y;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						predecessor_x[nbr_x][nbr_y] = cur_x;
						predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						
					}
				// }
			}

			nbr_x = cur_x;		//Top neighbor
			nbr_y = cur_y + 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						predecessor_x[nbr_x][nbr_y] = cur_x;
						predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						
					}
				// }
			}

			nbr_x = cur_x - 1;		//Left neighbor
			nbr_y = cur_y;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						predecessor_x[nbr_x][nbr_y] = cur_x;
						predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue3...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END

						//if((nbr_x == cur_goal_x) && (nbr_y == cur_goal_y))
						//	break;
					}
				// }
			}

			nbr_x = cur_x;		//Bottom neighbor
			nbr_y = cur_y - 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				

					int temp_dist = cost_mat[cur_x][cur_y] + 1;
					int nbr_dst = cost_mat[nbr_x][nbr_y];

					// if(nbr_dst > temp_dist + num_of_rot)
					if(nbr_dst > temp_dist)
					{
						// cost_mat[nbr_x][nbr_y] = temp_dist + num_of_rot;
						cost_mat[nbr_x][nbr_y] = temp_dist;
						predecessor_x[nbr_x][nbr_y] = cur_x;
						predecessor_y[nbr_x][nbr_y] = cur_y;
						// direction[nbr_x][nbr_y] = temp_theta;

						//==================================================EnQueue : START
						temp_loc.x = nbr_x;
						temp_loc.y = nbr_y;
						// temp_loc.theta = temp_theta;
						BFS_QUEUE.push(temp_loc);

						//printf("\nAfter enqueue4...\n");showQueue(BFS_QUEUE);
						//==================================================EnQueue : END

						//if((nbr_x == cur_goal_x) && (nbr_y == cur_goal_y))
						//	break;
					}
				// }
			}

			//color[cur_x][cur_y] = 3;printf("Direction Matrix...\n");showMatrix(direction);printf("\n");
		// }
	}

	
	
	//==================================================Backtract : START
	loc_vec opt_path;
	struct loc temp_loc;
	temp_loc.x = cur_goal_x;
	temp_loc.y = cur_goal_y;
	// temp_loc.theta = direction[cur_goal_x][cur_goal_y];
	opt_path.push_back(temp_loc);

	// int last_theta = temp_loc.theta;
	int pred_cell_x = predecessor_x[cur_goal_x][cur_goal_y];
	int pred_cell_y = predecessor_y[cur_goal_x][cur_goal_y];

	while(1)
	{
		temp_loc.x = pred_cell_x;
		temp_loc.y = pred_cell_y;
		

		opt_path.push_back(temp_loc);
		// last_theta = temp_loc.theta;
		
		if((pred_cell_x == cur_rob_x) && (pred_cell_y == cur_rob_y))
			break;
		else
		{
			int pred_cell_x_bkp = pred_cell_x;
			pred_cell_x = predecessor_x[pred_cell_x][pred_cell_y];
			pred_cell_y = predecessor_y[pred_cell_x_bkp][pred_cell_y];
		}
	}
	
	reverse(opt_path.begin(), opt_path.end());
	//==================================================Backtract : END

	return opt_path;
}


loc_mat GAP::compute_optimal_paths(bool_mat ws_graph, loc_vec robs_states, loc_vec goals_locs, int_vec opt_goal_vec, bool_vec S_r, loc_mat paths_old, uint &active_count)
{
	uint ws_size_x = ws_graph.size();			// Size of the workspace
	uint ws_size_y = ws_graph[0].size();		// Size of the workspace
	uint rob_count = S_r.size();				// Total number of robots
	uint req_rob_id = 0;						// Requester robot ID
	loc_mat opt_paths;						// Optimal Paths

	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
	{
		loc_vec opt_path;
		
		if(S_r[rob_id])
		{
			struct loc rob_state = robs_states[req_rob_id];
			int opt_goal_id = opt_goal_vec[req_rob_id];

			if(opt_goal_id == -1)		// Inactive
				opt_path.push_back(rob_state);
			else
			{
				active_count++;

				struct loc goal_loc = goals_locs[opt_goal_id];

				opt_path = optimal_path_bfs_drone(ws_size_x, ws_size_y, ws_graph, rob_state, goal_loc);
			}

			req_rob_id++;
		}
		else
		{
			loc_vec path_old = paths_old[rob_id];
			uint path_old_len = path_old.size() - 1;

			for(uint state_id = 0; state_id <= path_old_len; state_id++)
				opt_path.push_back(path_old[state_id]);
		}

		opt_paths.push_back(opt_path);
	}

	return opt_paths;
}


bool GAP::is_type1_crossover_path_pair(int rob_1, int rob_2, struct loc rob1_start_loc, loc_vec path_2)
{
	int path2_len = path_2.size() - 1;		// Length of path_2

	if(path2_len > 1)
	{
		for(int i = 1; i < path2_len; i++)
		{
			struct loc path2_loc = path_2[i];

			if((path2_loc.x == rob1_start_loc.x) && (path2_loc.y == rob1_start_loc.y))
			{
				// cout << "\nR_" << rob_1 << " ! R_" << rob_2;
				return true;
			}
		}

		return false;
	}
	else
		return false;
}


bool GAP::is_type2_crossover_path_pair(int rob_1, int rob_2, loc_vec path_1, loc_vec path_2)
{
	int path1_len = path_1.size() - 1;		// Length of path_1
	int path2_len = path_2.size() - 1;		// Length of path_2

	if((path1_len > 1) && (path2_len > 1))
	{
		struct loc path1_start_loc = path_1[0];
		struct loc path2_start_loc = path_2[0];
		bool flag_start1_found, flag_start2_found;
		flag_start1_found = flag_start2_found = false;

		for(int i = 1; i < path1_len; i++)
		{
			struct loc path1_loc = path_1[i];

			if((path1_loc.x == path2_start_loc.x) && (path1_loc.y == path2_start_loc.y))
			{
				flag_start2_found = true;
				break;
			}
		}

		for(int i = 1; i < path2_len; i++)
		{
			struct loc path2_loc = path_2[i];

			if((path2_loc.x == path1_start_loc.x) && (path2_loc.y == path1_start_loc.y))
			{
				flag_start1_found = true;
				break;
			}
		}

		if(flag_start1_found && flag_start2_found)
		{
			// cout << "\nP_" << rob_1 << " x P_" << rob_2;
			return true;
		}
		else
			return false;
	}
	else
		return false;
}


bool GAP::is_nested_path_pair(int rob_1, int rob_2, loc_vec path_1, loc_vec path_2)
{
	int path_1_len = path_1.size() - 1;		// Length of path_1
	int path_2_len = path_2.size() - 1;		// Length of path_2
	struct loc path_1_start_loc = path_1[0];
	struct loc path_1_goal_loc = path_1[path_1_len];
	bool flag_start1_found, flag_goal1_found, r1_in_r2;
	flag_start1_found = flag_goal1_found = r1_in_r2 = false;

	for(int i = 1; i < path_2_len; i++)		// Is R_1 nested in R_2?
	{
		struct loc path_2_loc = path_2[i];

		if(!flag_start1_found && (path_1_start_loc.x == path_2_loc.x) && (path_1_start_loc.y == path_2_loc.y))
			flag_start1_found = true;
		else if(!flag_goal1_found && (path_1_goal_loc.x == path_2_loc.x) && (path_1_goal_loc.y == path_2_loc.y))
			flag_goal1_found = true;

		r1_in_r2 = flag_start1_found && flag_goal1_found;

		if(r1_in_r2)
		{
			// cout << "\nR_" << rob_2 << " > R_" << rob_1;
			return true;
		}
	}

	return false;
}





void GAP::compute_residual_paths(loc_mat &opt_paths, bool_vec S_r, uint look_ahead_intrvl, uint &start_from_intrvl, uint &fsbl_comp_intrvl)
{
	uint cur_clk;		// Current value of the clock
	std::string clock_file_path = ament_index_cpp::get_package_share_directory("con_plan_exec_pkg") + OUTPUT_DIR + CLOCK_FILE;

	#pragma omp critical
	{
		ifstream ifs;
		ifs.open(clock_file_path.c_str());
		ifs >> cur_clk;
		ifs.close();
	}
	
	fsbl_comp_intrvl = cur_clk / INTRVL_LEN_FAC;		// Current interval ID
	uint start_from_intrvl_new = fsbl_comp_intrvl + look_ahead_intrvl;

	if(start_from_intrvl_new > start_from_intrvl)
	{
		uint rob_count = S_r.size();					// Total number of robots

		for(uint rob_id = 0; rob_id < rob_count; rob_id++)
			if(!S_r[rob_id])
			{
				uint intrvl_dif = start_from_intrvl_new - start_from_intrvl;		// Difference in interval
				uint state_count = intrvl_dif * INTRVL_LEN_FAC;
				uint path_len = opt_paths[rob_id].size() - 1;
				uint state_del_count;

				if(state_count <= path_len)
					state_del_count = state_count;
				else
					state_del_count = path_len;
				
				for(uint state_id = 1; state_id <= state_del_count; state_id++)
					opt_paths[rob_id].erase(opt_paths[rob_id].begin());
			}

		start_from_intrvl = start_from_intrvl_new;
	}
}


loc_vec GAP::adjust_path(struct loc init_state, loc_vec opt_path)
{
	loc_vec path;
	path.push_back(init_state);
	uint path_len = opt_path.size() - 1;
	struct loc opt_path_loc;
	uint k = path_len - 1;

	do
	{
		opt_path_loc = opt_path[k];

		if((init_state.x == opt_path_loc.x) && (init_state.y == opt_path_loc.y))
			break;

		k--;
	}
	while(k > 0);

	opt_path_loc = opt_path[++k];

	if(init_state.theta != opt_path_loc.theta)			// Needs Rotations
	{
		uint tmp_theta = (init_state.theta + 1) % 4;
		uint lr_count = 1;			// Left rotation counter

		while(tmp_theta != opt_path_loc.theta)
		{
			tmp_theta = (tmp_theta + 1) % 4;
			lr_count++;
		}

		uint rr_count = 4 - lr_count;			// Right rotation counter
		uint r_count = 1;		// Rotation counter

		if(lr_count <= rr_count)		// Rotate left
		{
			tmp_theta = init_state.theta;

			while(r_count <= lr_count)
			{
				tmp_theta = (tmp_theta + 1) % 4;
				init_state.theta = tmp_theta;
				path.push_back(init_state);
				r_count++;
			}
		}
		else
		{
			tmp_theta = init_state.theta;

			while(r_count <= rr_count)
			{
				tmp_theta = (tmp_theta - 1 + 4) % 4;
				init_state.theta = tmp_theta;
				path.push_back(init_state);
				r_count++;
			}
		}
	}

	for(uint l = k; l <= path_len; l++)
		path.push_back(opt_path[l]);

	return path;
}


bool GAP::test_path(int nearest_rob_id, loc_vec nearest_rob_path, vec_int goal_vec_new, loc_mat feasible_paths, bool_vec S_r)
{
	// int num_of_robs = goal_vec_new.size();
	int rob_count = S_r.size();

	for(int rob_id = 0; rob_id < rob_count; rob_id++)
		if(nearest_rob_id != rob_id)
		{
			loc_vec rob_path = feasible_paths[rob_id];

			if(S_r[rob_id])		// Requester
			{
				int rob_id_req = -1;

				for(uint rob_id2 = 0; rob_id2 <= rob_id; rob_id2++)
					if(S_r[rob_id2])
						rob_id_req++;

				if((goal_vec_new[rob_id_req] != -1) && 
					(is_nested_path_pair(nearest_rob_id, rob_id, nearest_rob_path, rob_path) || 
					is_nested_path_pair(rob_id, nearest_rob_id, rob_path, nearest_rob_path)))
					return false;
			}
			else		// Non-requester
			{
				if(rob_path.size() - 1)			// Non-requester Active
				{
					if(is_nested_path_pair(rob_id, nearest_rob_id, rob_path, nearest_rob_path))
						return false;
				}
				else if(is_type1_crossover_path_pair(rob_id, nearest_rob_id, rob_path[0], nearest_rob_path))		// Non-requester Inactive
					return false;
			}
		}

	return true;
}


loc_mat GAP::get_feasible_paths(int_vec &opt_goal_vec, loc_mat opt_paths, uint &killed_count, uint &revived_count, uint &revived_goal_count, int ws_size_x, int ws_size_y, bool_vec S_r, uint look_ahead_intrvl, uint &start_from_intrvl, uint &fsbl_comp_intrvl)
{
	compute_residual_paths(opt_paths, S_r, look_ahead_intrvl, start_from_intrvl, fsbl_comp_intrvl);

	//================================================== Detection : START
	// int num_of_robs = opt_goal_vec.size();		// Number of robots
	int rob_count = S_r.size();						// Total number of robots
	bool_vec is_killed_rob(rob_count, false);		// Is R_i killed?
	vec_int killed_robs;							// The set of killed robots
	// cout << "\nKilled";

	for(int i = 0; i < rob_count; i++)		// Type-2 crossover path and Nested path
	{
		int i_req = -1;

		if(S_r[i])
			for(uint k = 0; k <= i; k++)
				if(S_r[k])
					i_req++;

		for(int j = 0; j < rob_count; j++)
		{
			int j_req = -1;

			if(S_r[j])
				for(uint k = 0; k <= j; k++)
					if(S_r[k])
						j_req++;

			if((i != j) && ((S_r[i] && (opt_goal_vec[i_req] != -1)) || (!S_r[i] && (opt_paths[i].size() - 1))) && (S_r[j] && opt_goal_vec[j_req] != -1) && 
				(!is_killed_rob[i] || !is_killed_rob[j]) && 
				((S_r[i] && S_r[j] && is_type2_crossover_path_pair(i, j, opt_paths[i], opt_paths[j])) || 
					(((!S_r[i] && S_r[j]) || (S_r[i] && S_r[j])) && is_nested_path_pair(i, j, opt_paths[i], opt_paths[j]))))
			{
				if(S_r[i] && !is_killed_rob[i])
				{
					is_killed_rob[i] = true;
					killed_robs.push_back(i);
					killed_count++;
					// cout << " R_" << i;
				}

				if(S_r[j] && !is_killed_rob[j])
				{
					is_killed_rob[j] = true;
					killed_robs.push_back(j);
					killed_count++;
					// cout << " R_" << j;
				}
			}
		}
	}

	for(int i = 0; i < rob_count; i++)		// Adds inactive robots
	{
		int i_req = -1;

		if(S_r[i])
			for(uint k = 0; k <= i; k++)
				if(S_r[k])
					i_req++;

		if((S_r[i] && (opt_goal_vec[i_req] == -1)) || (!S_r[i] && !(opt_paths[i].size() - 1)))
			killed_robs.push_back(i);
	}
	
	while(!killed_robs.empty())
	{
		int i = killed_robs.back();
		killed_robs.pop_back();

		for(int j = 0; j < rob_count; j++)
			if(S_r[j])
			{
				int j_req = -1;

				for(uint k = 0; k <= j; k++)
					if(S_r[k])
						j_req++;

				if((i != j) && (opt_goal_vec[j_req] != -1) && !is_killed_rob[j] && is_type1_crossover_path_pair(i, j, opt_paths[i][0], opt_paths[j]))
				{
					is_killed_rob[j] = true;
					killed_robs.push_back(j);
					killed_count++;
					// cout << " R_" << j;
				}
			}
	}

	// cout << "\n\nKilled...";

	// for(int i = 0; i < rob_count; i++)
	// 	if(is_killed_rob[i])
	// 		cout << " R_" << i;
	//================================================== Detection : END
	//================================================== Correction : START
	int_mat W_r(ws_size_x, vec_int(ws_size_y, -1));						// Workspace with Inactive/Killed Robot IDs
	bool_mat W_killed_goals(ws_size_x, bool_vec(ws_size_y, false));		// Goals of killed robots
	// vec_int goal_vec_new(rob_count, -1);									// Goals of Inactive/Killed robots
	vec_int goal_vec_new(opt_goal_vec.size(), -1);							// Goals of Inactive/Killed robots
	loc_mat feasible_paths;												// The set of feasible paths

	for(int i = 0; i < rob_count; i++)		// Initialization
	{
		loc_vec path;

		if(S_r[i])
		{
			int i_req = -1;

			for(uint i2 = 0; i2 <= i; i2++)
				if(S_r[i2])
					i_req++;

			if((opt_goal_vec[i_req] != -1) && !is_killed_rob[i])		// Active unkilled
				path = opt_paths[i];
			else
			{
				struct loc start_state = opt_paths[i][0];
				W_r[(int)start_state.x][(int)start_state.y] = i;
				path.push_back(start_state);

				if(is_killed_rob[i])
				{
					int path_len = opt_paths[i].size() - 1;
					struct loc goal_state = opt_paths[i][path_len];
					W_killed_goals[(int)goal_state.x][(int)goal_state.y] = true;		// Needs to be visited
				}
			}
		}
		else
			path = opt_paths[i];

		feasible_paths.push_back(path);
	}

	for(int i = 0; i < rob_count; i++)
		if(S_r[i] && is_killed_rob[i])
		{
			int i_req = -1;

			for(uint i2 = 0; i2 <= i; i2++)
				if(S_r[i2])
					i_req++;

			// cout << "\nTry reassigning G_" << opt_goal_vec[i] << " (of R_" << i;
			loc_vec path_i = opt_paths[i];
			int path_i_len = path_i.size() - 1;

			for(int loc_id = path_i_len - 1; loc_id >= 0; loc_id--)
			{
				struct loc path_i_loc = path_i[loc_id];
				int nearest_rob = W_r[(int)path_i_loc.x][(int)path_i_loc.y];		// The nearest robot to the goal opt_goal_vec[i]

				if(nearest_rob != -1)
				{
					// cout << ") to R_" << nearest_rob;
					loc_vec path;

					if(nearest_rob == i)
						path = path_i;
					else
						path = adjust_path(opt_paths[nearest_rob][0], path_i);

					if(test_path(nearest_rob, path, goal_vec_new, feasible_paths, S_r))
					{
						W_r[(int)path_i_loc.x][(int)path_i_loc.y] = -1;		// Revived
						int nearest_rob_req = -1;

						for(uint j = 0; j <= nearest_rob; j++)
							if(S_r[j])
								nearest_rob_req++;

						// goal_vec_new[nearest_rob] = opt_goal_vec[i];
						goal_vec_new[nearest_rob_req] = opt_goal_vec[i_req];
						
						int path_len = path.size() - 1;

						for(int loc_id = 1; loc_id <= path_len; loc_id++)
						{
							struct loc path_state = path[loc_id];
							feasible_paths[nearest_rob].push_back(path_state);
							W_killed_goals[(int)path_state.x][(int)path_state.y] = false;		// To be visited
						}

						revived_count++;
						// cout << ": OK";
					}

					break;
				}
			}
		}
	//================================================== Correction : END
	// cout << "\n\nRevived...";

	for(int i = 0; i < rob_count; i++)
		if(S_r[i])
		{
			int i_req = -1;

			for(uint i2 = 0; i2 <= i; i2++)
				if(S_r[i2])
					i_req++;
			
			if((opt_goal_vec[i_req] == -1) || is_killed_rob[i])
			{
				int goal_id_new = goal_vec_new[i_req];
				opt_goal_vec[i_req] = goal_id_new;

				// if(goal_id_new != -1)
					// cout << " R_" << i;
			}
		}

	revived_goal_count = killed_count;

	for(uint i = 0; i < ws_size_x; i++)
		for(uint j = 0; j < ws_size_y; j++)
			if(W_killed_goals[i][j])
				revived_goal_count--;

	return feasible_paths;
}
//==================================================3. Optimal Paths : END


//==================================================4. Partial Order : START
bool_mat GAP::compute_partial_orders(uint rob_count, loc_mat paths, bool_vec S_r)
{
	bool_mat partial_order(rob_count, bool_vec(rob_count, false));		// Initialized to false

	for(int rob_id1 = 0; rob_id1 < rob_count; rob_id1++)
	{
		struct loc start_state1 = paths[rob_id1][0];
		int path1_len = paths[rob_id1].size() - 1;
		struct loc goal_state1 = paths[rob_id1][path1_len];

		for(uint rob_id2 = 0; rob_id2 < rob_count; rob_id2++)
			if((rob_id1 != rob_id2) && (S_r[rob_id1] || S_r[rob_id2]))
			{
				uint path2_len = paths[rob_id2].size() - 1;

				if(path2_len)		// Active robot
					for(uint state_id2 = 0; state_id2 <= path2_len; state_id2++)
					{
						struct loc state2 = paths[rob_id2][state_id2];

						if((start_state1.x == state2.x) && (start_state1.y == state2.y))		// Check if S_2..........S_1..........G_2
						{
							partial_order[rob_id1][rob_id2] = true;
							// cout << "\nS_" << rob_id1 << " > " << "P_" << rob_id2;
						}

						if((goal_state1.x == state2.x) && (goal_state1.y == state2.y))		// Check if S_2..........G_1..........G_2
						{
							partial_order[rob_id2][rob_id1] = true;
							// cout << "\nG_" << rob_id1 << " > " << "P_" << rob_id2;
						}
					}
			}
	}

	return partial_order;
}
//==================================================4. Partial Order : END


//==================================================5. Total Order : START
int_vec GAP::compute_total_order(int num_of_robs, bool_mat adj, bool_mat &adj_residue)
{
	int_vec in_degrees(num_of_robs, 0);
	bool_vec visited(num_of_robs, false);
	queue<int> Q;

	for(int i = 0; i < num_of_robs; i++)
	{
		for(int j = 0; j < num_of_robs; j++)
		{
			if(adj[i][j])
			{
				in_degrees[j]++;
			}
		}
	}

	//cout << "\nIn-degrees...\n";

	for(int i = 0; i < num_of_robs; i++)
	{
		//cout << in_degrees[i] << " ";

		if(!in_degrees[i])
		{
			Q.push(i);
			//cout << "Pushed R_" << i << " ";
			visited[i] = true;
		}
	}

	// cout << endl;
	//int_vec total_order;
	int_vec total_order(num_of_robs, -1);		//Initialized with invalid robot id (-1)
	uint to_index = 0;

	while(!Q.empty()) 
    { 
        int robot_index = Q.front(); 
        Q.pop();
        //cout << "\nPopped R_" << robot_index << endl;
        
        //total_order.push_back(robot_index);
        total_order[to_index++] = robot_index;

        for(int j = 0; j < num_of_robs; j++)
        {
        	if(adj[robot_index][j] && !visited[j])
        	{
        		in_degrees[j]--;
        		adj_residue[robot_index][j] = false;		//Traversed

        		if(!in_degrees[j])
				{
					Q.push(j);
					//cout << "Pushed R_" << j << " ";
					visited[j] = true;
				}
        	}
        }

        /*cout << "\n\nIn-degrees...\n";

		for(int i = 0; i < num_of_robs; i++)
		{
			cout << in_degrees[i] << " ";
		}

		cout << endl;*/
    }

	return total_order;
}


void GAP::break_dependency_cycles(int_vec &opt_goal_vec, bool_mat po_residue, bool_vec S_r)
{
	int num_of_robs = opt_goal_vec.size();
	uint rob_count = po_residue[0].size();

	//================================================== Directed Cyclic Graph (DCG) to Directed Acyclic Graph (DAG) : START
	int_vec opt_goal_vec_l, opt_goal_vec_r;		// Left & Right Vectors of opt_goal_vec
	opt_goal_vec_l = opt_goal_vec_r = opt_goal_vec;

	uint count_l, count_r;
	count_l = count_r = 0;

	int i_index, j_index;
	i_index = -1;

	for(int i = 0; i < rob_count; i++)
	{
		if(S_r[i])
			i_index++;

		j_index = -1;

		for(int j = 0; j < rob_count; j++)
		{
			if(S_r[j])
				j_index++;

			if(po_residue[i][j])
			{
				// cout << "po_residue[" << i << "][" << j << "] i_index = " << i_index << " j_index = " << j_index;

				if(i < j)
				{
					// cout << " Left\n";

					if(S_r[i] && (opt_goal_vec_r[i_index] != -1))
					{
						opt_goal_vec_r[i_index] = -1;		//Goal Unassigned
						count_l++;
					}
					else if(S_r[j] && (opt_goal_vec_r[j_index] != -1))
					{
						opt_goal_vec_r[j_index] = -1;		//Goal Unassigned
						count_l++;
					}
				}
				else if(i > j)
				{
					// cout << " Right\n";

					if(S_r[i] && (opt_goal_vec_l[i_index] != -1))
					{
						opt_goal_vec_l[i_index] = -1;		//Goal Unassigned
						count_r++;
					}
					else if(S_r[j] && (opt_goal_vec_l[j_index] != -1))
					{
						opt_goal_vec_l[j_index] = -1;		//Goal Unassigned
						count_r++;
					}
				}
			}
		}
	}

	/*cout << "\nOG:\t\t";
	for(uint i = 0; i < num_of_robs; i++)
		cout << opt_goal_vec[i] << " ";

	cout << "count_l = " << count_l << " count_r = " << count_r;*/

	if(count_l && count_r)
	{
		if(count_l >= count_r)
			opt_goal_vec = opt_goal_vec_l;
		else
			opt_goal_vec = opt_goal_vec_r;
	}
	else if(count_l)
		opt_goal_vec = opt_goal_vec_r;
	else if(count_r)
		opt_goal_vec = opt_goal_vec_l;

	/*cout << "\nLeft:\t";
	for(uint i = 0; i < num_of_robs; i++)
		cout << opt_goal_vec_l[i] << " ";
	
	cout << "\nRight:\t";
	for(uint i = 0; i < num_of_robs; i++)
		cout << opt_goal_vec_r[i] << " ";*/
	//================================================== Directed Cyclic Graph (DCG) to Directed Acyclic Graph (DAG) : END
}


void GAP::adjust_dependent_paths(int_vec opt_goal_vec, loc_mat &paths, bool_vec S_r, uint &active_count)
{
	uint rob_count = S_r.size();
	uint req_rob_id = 0;

	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
		if(S_r[rob_id])
		{
			if(opt_goal_vec[req_rob_id] == -1)
			{
				active_count--;
				uint path_len = paths[rob_id].size() - 1;

				for(uint state_id = path_len; state_id > 0; state_id--)
					paths[rob_id].erase(paths[rob_id].begin() + state_id);
			}

			req_rob_id++;
		}
		else
			active_count--;

	// paths_old = paths;
}
//==================================================5. Total Order : END


//==================================================6. Timeoffsets : START
int_vec GAP::compute_start_time_offsets(int_vec total_order, loc_mat paths, bool_vec S_r, int_vec &opt_goal_vec, bool &flag_inactivated_robot)
{
	int rob_count = total_order.size();
	int_vec sto_vec(rob_count, 0);				// Initialized to 0

	#ifdef COMP_STO_NEW
		bool_vec skip_vec(rob_count, false);		// Initialized to false
	#endif

	int to_index = 1;

	while(to_index < rob_count)
	{
		int rob_id = total_order[to_index];
		int sto_rob_id = 0;
		int path_len_rob_id = paths[rob_id].size() - 1;
		int pred_index = to_index - 1;

		while(pred_index >= 0)
		{
			int pred_id = total_order[pred_index];		// Total order predecessor robot index

			#ifdef COMP_STO_NEW
				if(skip_vec[pred_id])		// Recently inactivated by a non-requester
					pred_index--;
				else
			#endif
			{
				int sto_pred_id = sto_vec[pred_id];
				int path_len_pred_id = paths[pred_id].size() - 1;
				bool flag_scc, flag_hoc;
				flag_scc = flag_hoc = false;

				//================================================== Same Cell Collision (SCC) : START
				int time_max = sto_rob_id + path_len_rob_id;

				if(time_max < (sto_pred_id + path_len_pred_id))
					time_max = sto_pred_id + path_len_pred_id;

				struct loc cell_rob_id, cell_pred_id;

				for(int time = 0; time <= time_max; time++)
				{
					if(time <= sto_rob_id)
						cell_rob_id = paths[rob_id][0];
					else if(time <= (sto_rob_id + path_len_rob_id))
						cell_rob_id = paths[rob_id][time - sto_rob_id];
					else
						cell_rob_id = paths[rob_id][path_len_rob_id];

					if(time <= sto_pred_id)
						cell_pred_id = paths[pred_id][0];
					else if(time <= (sto_pred_id + path_len_pred_id))
						cell_pred_id = paths[pred_id][time - sto_pred_id];
					else
						cell_pred_id = paths[pred_id][path_len_pred_id];

					if((cell_rob_id.x == cell_pred_id.x) && (cell_rob_id.y == cell_pred_id.y))		// Same Cell Collision
					{
						flag_scc = true;
						break;
					}
				}
				//================================================== Same Cell Collision (SCC) : END
				//================================================== Head-on Collision (HoC) : START
				int time_min = sto_rob_id + path_len_rob_id;

				if(time_min > (sto_pred_id + path_len_pred_id))
					time_min = sto_pred_id + path_len_pred_id;

				struct loc prev_cell_rob_id, prev_cell_pred_id;
				prev_cell_rob_id = paths[rob_id][0];
				prev_cell_pred_id = paths[pred_id][0];

				for(int time = 1; time <= time_min; time++)
				{
					if(time <= sto_rob_id)
						cell_rob_id = paths[rob_id][0];
					else if(time <= (sto_rob_id + path_len_rob_id))
						cell_rob_id = paths[rob_id][time - sto_rob_id];

					if(time <= sto_pred_id)
						cell_pred_id = paths[pred_id][0];
					else if(time <= (sto_pred_id + path_len_pred_id))
						cell_pred_id = paths[pred_id][time - sto_pred_id];

					if((cell_rob_id.x == prev_cell_pred_id.x) && (cell_rob_id.y == prev_cell_pred_id.y) && (cell_pred_id.x == prev_cell_rob_id.x) && (cell_pred_id.y == prev_cell_rob_id.y))		// Head-on Collision
					{
						flag_hoc = true;
						break;
					}

					prev_cell_rob_id = cell_rob_id;
					prev_cell_pred_id = cell_pred_id;
				}
				//================================================== Head-on Collision (HoC) : END
				
				if(flag_scc || flag_hoc)
				{
					if(S_r[rob_id])
					{
						sto_rob_id++;
						pred_index = to_index - 1;
					}
					else if(S_r[pred_id])
					{
						int pred_id2 = -1;

						for(uint pred_id_tmp = 0; pred_id_tmp <= pred_id; pred_id_tmp++)
							if(S_r[pred_id_tmp])
								pred_id2++;
						
						opt_goal_vec[pred_id2] = -1;
						// cout << "\nInactivated R_" << pred_id2 << " for R_" << rob_id;
						flag_inactivated_robot = true;

						#ifdef COMP_STO_NEW
							skip_vec[pred_id] = true;
							pred_index--;
						#else
							return sto_vec;
						#endif
					}
				}
				else
					pred_index--;
			}
		}
		
		sto_vec[rob_id] = sto_rob_id;
		to_index++;
	}

	return sto_vec;
}
//==================================================6. Timeoffsets : END


//==================================================7. Optimal Trajectories : START
loc_mat GAP::compute_optimal_trajectories(int num_of_robs, int_vec start_time_offsets, loc_mat paths, bool_vec S_r, uint call_id)
{
	uint rob_count = S_r.size();
	//================================================== Save : START
	std::string con_plan_exec_pkg_path = ament_index_cpp::get_package_share_directory("con_plan_exec_pkg");
	std::string capl_file_name = con_plan_exec_pkg_path + OUTPUT_DIR + COL_AVRT_PATH_LENGTHS_FILE;	
	fstream capl_file;
	capl_file.open(capl_file_name.c_str(), ios::app);
	capl_file << "========== Call = " + to_string(call_id) << endl;

	for(int i = 0; i < rob_count; i++)
		if(S_r[i])
		{
			int sum_of_path_len_and_time_offset = (paths[i].size() - 1) + start_time_offsets[i];
			capl_file << i << ":" << (paths[i].size() - 1) << "," << start_time_offsets[i] << "," << sum_of_path_len_and_time_offset << endl;
		}

	capl_file.close();
	//================================================== Save : END
	//================================================== Paths : START
	loc_mat trajectories(rob_count, loc_vec());
	struct loc robot_loc;

	for(int robot_index = 0; robot_index < rob_count; robot_index++)
	{
		int time_offset_robot_index;

		if(S_r[robot_index])
			time_offset_robot_index = start_time_offsets[robot_index];
		else
			time_offset_robot_index = 0;

		loc_vec path_robot_index = paths[robot_index];
		int path_len_robot_index = path_robot_index.size() - 1;

		robot_loc.x = path_robot_index[0].x;
		robot_loc.y = path_robot_index[0].y;
		robot_loc.theta = path_robot_index[0].theta;
		trajectories[robot_index].push_back(robot_loc);

		for(int time = 0; time < (time_offset_robot_index + path_len_robot_index); time++)
		{
			if(time >= time_offset_robot_index)
			{
				robot_loc.x = path_robot_index[time - time_offset_robot_index + 1].x;
				robot_loc.y = path_robot_index[time - time_offset_robot_index + 1].y;
				robot_loc.theta = path_robot_index[time - time_offset_robot_index + 1].theta;
			}
			else		// Append initial location
			{
				robot_loc.x = path_robot_index[0].x;
				robot_loc.y = path_robot_index[0].y;
				robot_loc.theta = path_robot_index[0].theta;
			}

			trajectories[robot_index].push_back(robot_loc);
		}
	}
	//================================================== Paths : END

	return trajectories;
}


void GAP::monitor_paths(int ws_size_x, int ws_size_y, int num_of_robs, loc_mat trajectories)
{
	int_vec path_len_vec(num_of_robs, -1);
	int hor_len = trajectories[0].size() - 1;		// Horizon Length
	path_len_vec[0] = hor_len;

	for(int rob_id = 1; rob_id < num_of_robs; rob_id++)
	{
		int path_len = trajectories[rob_id].size() - 1;
		path_len_vec[rob_id] = path_len;

		if(hor_len < path_len)		// Maximum path length
			hor_len = path_len;
	}

	//================================================== Detect Motion Inconsistency : START
	for(uint rob_id = 0; rob_id < num_of_robs; rob_id++)
	{
		struct loc rob_state_prev = trajectories[rob_id][0], rob_state;

		for(uint time = 1; time <= path_len_vec[rob_id]; time++)
		{
			rob_state = trajectories[rob_id][time];

			if(!(((rob_state.x == rob_state_prev.x) && (rob_state.y == rob_state_prev.y))
					|| ((rob_state.x == rob_state_prev.x + 1) && (rob_state.y == rob_state_prev.y))
					|| ((rob_state.x == rob_state_prev.x - 1) && (rob_state.y == rob_state_prev.y))
					|| ((rob_state.x == rob_state_prev.x) && (rob_state.y == rob_state_prev.y + 1))
					|| ((rob_state.x == rob_state_prev.x) && (rob_state.y == rob_state_prev.y - 1))))
				{
					cout << "\nMonitor failed! MI R_" << rob_id << " (" << rob_state_prev.x << ", " << rob_state_prev.y << ") t = " << time - 1 << " (" << rob_state.x << ", " << rob_state.y << ") t = " << time;
					exit(1);
				}

			rob_state_prev = rob_state;
		}
	}
	//================================================== Detect Motion Inconsistency : End
	//================================================== Detect Same Cell Collsion : START
	for(int time = 0; time <= hor_len; time++)
	{
		int_mat ws_cells(ws_size_x, int_vec(ws_size_y, -1));
		
		for(int rob_id = 0; rob_id < num_of_robs; rob_id++)
		{
			int path_len = path_len_vec[rob_id];
			struct loc rob_loc;

			if(time <= path_len)
				rob_loc = trajectories[rob_id][time];
			else
				rob_loc = trajectories[rob_id][path_len];

			if(ws_cells[rob_loc.x][rob_loc.y] != -1)		// Same Cell Collision
			{
				cout << "\nMonitor failed! SCC R_" << rob_id << " <> R_" << ws_cells[rob_loc.x][rob_loc.y] << " (" << rob_loc.x << ", " << rob_loc.y << ") @ t = " << time << "\n";
				exit(1);
			}
			else
				ws_cells[rob_loc.x][rob_loc.y] = rob_id;
		}
	}
	//================================================== Detect Same Cell Collsion : END
	//================================================== Detect Head-on Collsion : START
	for(int time = 1; time <= hor_len; time++)
		for(int rob_id1 = 0; rob_id1 < num_of_robs - 1; rob_id1++)
			for(int rob_id2 = rob_id1 + 1; rob_id2 < num_of_robs; rob_id2++)
			{
				int path1_len = path_len_vec[rob_id1];
				int path2_len = path_len_vec[rob_id2];
				struct loc cell_1, cell_2, cell_1_prev, cell_2_prev;

				if(time <= path1_len)
					cell_1 = trajectories[rob_id1][time];
				else
					cell_1 = trajectories[rob_id1][path1_len];

				if(time <= path2_len)
					cell_2 = trajectories[rob_id2][time];
				else
					cell_2 = trajectories[rob_id2][path2_len];

				if(time - 1 <= path1_len)
					cell_1_prev = trajectories[rob_id1][time - 1];
				else
					cell_1_prev = trajectories[rob_id1][path1_len];

				if(time - 1 <= path2_len)
					cell_2_prev = trajectories[rob_id2][time - 1];
				else
					cell_2_prev = trajectories[rob_id2][path2_len];

				if((cell_1.x == cell_2_prev.x) && (cell_1.y == cell_2_prev.y) && (cell_2.x == cell_1_prev.x) && (cell_2.y == cell_1_prev.y))		// Head-on Collision
				{
					cout << "\nMonitor failed! HoC R_" << rob_id1 << " <> R_" << rob_id2 << " (" << cell_1.x << ", " << cell_1.y << ") <> (" << cell_2.x << ", " << cell_2.y << ") @ t = " << time << "\n";
					exit(1);
				}
			}
	//================================================== Detect Head-on Collsion : END
}
//==================================================7. Optimal Trajectories : END


loc_mat GAP::get_cost_optimal_paths(int ws_size_x, int ws_size_y, bool_mat ws_graph, uint num_of_robs, loc_vec robs_states, uint goal_count, loc_vec goals_locs, uint call_id, bool_vec S_r, loc_mat paths_old, int_vec &opt_goal_vec, uint &active_count,float comm_range )
{
	uint rob_count = S_r.size();		// Total number of robots

	#ifdef DEBUG_WS_GRAPH
		cout << "\nWS_Graph...\n";

		for(uint row_id = 0; row_id < ws_size_x; row_id++)
		{
			for(uint col_id = 0; col_id < ws_size_y; col_id++)
				cout << ws_graph[row_id][col_id] << " ";

			cout << endl;
		}
	#endif

	#ifdef DEBUG_ROBS_STATES
		cout << "\nRobot states...";
		uint rob_id = 0;

		for(uint rob_id_tmp = 0; rob_id_tmp < rob_count; rob_id_tmp++)
			if(S_r[rob_id_tmp])
			{
				struct loc rob_state = robs_states[rob_id];
				cout << "\nR_" << rob_id << " (" << rob_state.x << ", " << rob_state.y << ", " << rob_state.theta << "): Path_" << rob_id_tmp;
				rob_id++;
			}
	#endif

	#ifdef DEBUG_GOALS_LOCS
		cout << "\nGoals...";

		for(uint goal_id = 0; goal_id < goal_count; goal_id++)
		{
			struct loc goal_loc = goals_locs[goal_id];
			cout << "\nG_" << goal_id << " (" << goal_loc.x << ", " << goal_loc.y << ")";
		}
	#endif

	std::string con_plan_exec_pkg_path = ament_index_cpp::get_package_share_directory("con_plan_exec_pkg");
	ofstream debug_file;
	std::string debug_filename;

	//================================================== 1. Optimal Costs : START
	int_mat opt_cost_mat = compute_optimal_costs(ws_graph, robs_states, goals_locs, S_r, paths_old,comm_range);
	// cout << "\nOC...";

	#ifdef DEBUG_OC
		debug_filename = con_plan_exec_pkg_path + OUTPUT_DIR + DEBUG_OC_FILENAME + to_string(call_id) + CSV_EXT;
		debug_file.open(debug_filename.c_str());

		for(uint rob_id = 0; rob_id < num_of_robs; rob_id++)
		{
			for(uint goal_id = 0; goal_id < goal_count; goal_id++)
			{
				// cout << opt_cost_mat[rob_id][goal_id] << " ";
				debug_file << opt_cost_mat[rob_id][goal_id];

				if((goal_id + 1) != goal_count)
					debug_file << ",";
			}

			// cout << endl;
			debug_file << endl;
		}

		debug_file.close();
	#endif
	//================================================== 1. Optimal Costs : END
	//================================================== 2. Optimal Goals : START
	//================================================== 2. Optimal Goals : START
	
	// 1. MAKE THE BACKUP MATRIX HERE
	int_mat opt_cost_mat_backup = opt_cost_mat; 

	MUNKRES_ALGO ma_obj;
	opt_goal_vec = ma_obj.munkres(opt_cost_mat, num_of_robs, goal_count);
	
	// 2. CHECK THE BACKUP MATRIX FOR INFINITY COSTS
	for(uint req_idx = 0; req_idx < num_of_robs; req_idx++) {
		int assigned_goal = opt_goal_vec[req_idx];
		// We now check opt_cost_mat_backup, which Munkres hasn't destroyed!
		if (assigned_goal != -1 && opt_cost_mat_backup[req_idx][assigned_goal] == COST_INF) {
			std::cout << "\n[NETWORK HALT] Drone " << req_idx << " parked to prevent tether snap.";
			opt_goal_vec[req_idx] = -1; // Force the drone to go inactive
		}
	}
	// cout << "\nOG...";

	#ifdef DEBUG_OG
		debug_filename = con_plan_exec_pkg_path + OUTPUT_DIR + DEBUG_OG_FILENAME + to_string(call_id) + TXT_EXT;
		debug_file.open(debug_filename.c_str());

		for(uint rob_id = 0; rob_id < num_of_robs; rob_id++)
		{
			cout << "\nR_" << rob_id << "\t\tG_" << opt_goal_vec[rob_id];
			debug_file << opt_goal_vec[rob_id];

			if((rob_id + 1) != num_of_robs)
				debug_file << endl;
		}

		debug_file.close();
	#endif
	//================================================== 2. Optimal Goals : END
	//================================================== 3. Optimal Paths : START
	loc_mat opt_paths = compute_optimal_paths(ws_graph, robs_states, goals_locs, opt_goal_vec, S_r, paths_old, active_count);
	// cout << "\nOP...";

	#ifdef DEBUG_OP
		debug_filename = con_plan_exec_pkg_path + OUTPUT_DIR + DEBUG_OP_FILENAME + to_string(call_id) + CSV_EXT;
		debug_file.open(debug_filename.c_str());

		for(uint rob_id = 0; rob_id < rob_count; rob_id++)
		{
			loc_vec path_tmp = opt_paths[rob_id];
			uint path_len_tmp = path_tmp.size() - 1;
			cout << "\nPath_" << rob_id << " ";

			for(uint state_id = 0; state_id <= path_len_tmp; state_id++)
			{
				cout << "(" << path_tmp[state_id].x << ", " << path_tmp[state_id].y << ", " << path_tmp[state_id].theta << ") ";
				debug_file << "(" << path_tmp[state_id].x << " " << path_tmp[state_id].y << " " << path_tmp[state_id].theta << "),";
			}

			debug_file << endl;
		}
		
		debug_file.close();
	#endif
	//================================================== 3. Optimal Paths : END

	return opt_paths;
}


loc_mat GAP::get_col_free_paths(int ws_size_x, int ws_size_y, uint num_of_robs, uint call_id, bool_vec S_r, int_vec opt_goal_vec, loc_mat opt_paths, uint active_count, uint look_ahead_intrvl, uint &start_from_intrvl, uint &fsbl_comp_intrvl,float comm_range)
{
	uint rob_count = S_r.size();		// Total number of robots

	std::string con_plan_exec_pkg_path = ament_index_cpp::get_package_share_directory("con_plan_exec_pkg");
	ofstream debug_file;
	std::string debug_filename;

	uint iter_id = 0, killed_count, revived_count, revived_goal_count;
	loc_mat feasible_paths;
	bool_mat partial_order;
	bool flag_to_found;
	int_vec total_order, start_time_offsets;

	while(true)
	{
		//================================================== 4. Feasible Paths : START
		killed_count = 0;
		revived_count = 0;
		revived_goal_count = 0;
		feasible_paths = get_feasible_paths(opt_goal_vec, opt_paths, killed_count, revived_count, revived_goal_count, ws_size_x, ws_size_y, S_r, look_ahead_intrvl, start_from_intrvl, fsbl_comp_intrvl);
		// cout << "\nFP...";

		#ifdef DEBUG_FP
			debug_filename = con_plan_exec_pkg_path + OUTPUT_DIR + DEBUG_FP_FILENAME + std::to_string(call_id) + "_I" + std::to_string(fsbl_comp_intrvl) + "_i" + std::to_string(iter_id) + CSV_EXT;
			debug_file.open(debug_filename.c_str());

			for(uint rob_id = 0; rob_id < num_of_robs; rob_id++)
			{
				loc_vec path_tmp = feasible_paths[rob_id];
				uint path_tmp_len = path_tmp.size() - 1;
				cout << "\nPath_" << rob_id << " ";

				for(uint state_id = 0; state_id <= path_tmp_len; state_id++)
				{
					cout << "(" << path_tmp[state_id].x << ", " << path_tmp[state_id].y << ", " << path_tmp[state_id].theta << ") ";
					debug_file << "(" << path_tmp[state_id].x << " " << path_tmp[state_id].y << " " << path_tmp[state_id].theta << "),";
				}

				debug_file << endl;
			}

			debug_file.close();
		#endif

		debug_filename = con_plan_exec_pkg_path + OUTPUT_DIR + DEBUG_FP_STAT_FILENAME + TXT_EXT;
		debug_file.open(debug_filename.c_str(), ios::app);
		debug_file << call_id << "," << iter_id << "," << active_count << "," << killed_count << "," << revived_count << "," << revived_goal_count << endl;
		debug_file.close();
		//================================================== 4. Feasible Paths : END
		//================================================== 5. Partial Orders : START
		partial_order = compute_partial_orders(rob_count, feasible_paths, S_r);
		// cout << "\nPO...";

		#ifdef DEBUG_PO
			debug_filename = con_plan_exec_pkg_path + OUTPUT_DIR + DEBUG_PO_FILENAME + to_string(call_id) + "_i" + to_string(iter_id) + CSV_EXT;
			debug_file.open(debug_filename.c_str());

			for(uint rob_id = 0; rob_id < rob_count; rob_id++)
			{
				bool_vec po_row = partial_order[rob_id];
				cout << endl;

				for(uint rob_id2 = 0; rob_id2 < rob_count; rob_id2++)
				{
					if(po_row[rob_id2])
					{
						cout << "1 ";
						debug_file << "1";
					}
					else
					{
						cout << "0 ";
						debug_file << "0";
					}


					if((rob_id2 + 1) != rob_count)
						debug_file << ",";
				}

				debug_file << endl;
			}

			debug_file.close();
		#endif
		//================================================== 5. Partial Orders : END
		//================================================== 6. Total Order : START
		bool_mat po_residue = partial_order;		// Residue of PO
		total_order = compute_total_order(rob_count, partial_order, po_residue);

		flag_to_found = true;

		for(uint rob_id = 0; rob_id < rob_count; rob_id++)
			if(total_order[rob_id] == -1)
			{
				flag_to_found = false;
				break;
			}

		active_count = rob_count;

		if(!flag_to_found)
		{
			// cout << "\nInvalid TO";

			#ifdef DEBUG_TO
				debug_filename = con_plan_exec_pkg_path + OUTPUT_DIR + DEBUG_TO_FILENAME + to_string(call_id) + "_i" + to_string(iter_id) + CSV_EXT;
				debug_file.open(debug_filename.c_str());

				for(uint rob_id = 0; rob_id < rob_count; rob_id++)
				{
					bool_vec po_row = po_residue[rob_id];
					cout << endl;

					for(uint rob_id2 = 0; rob_id2 < rob_count; rob_id2++)
					{
						if(po_row[rob_id2])
						{
							cout << "1 ";
							debug_file << "1";
						}
						else
						{
							cout << "0 ";
							debug_file << "0";
						}

						if((rob_id2 + 1) != rob_count)
							debug_file << ",";
					}

					debug_file << endl;
				}

				debug_file.close();
			#endif

			break_dependency_cycles(opt_goal_vec, po_residue, S_r);
			adjust_dependent_paths(opt_goal_vec, feasible_paths, S_r, active_count);
		}
		else
		{
			// cout << "\nTO...";

			#ifdef DEBUG_TO
				debug_filename = con_plan_exec_pkg_path + OUTPUT_DIR + DEBUG_TO_FILENAME + to_string(call_id) + "_i" + to_string(iter_id) + TXT_EXT;
				debug_file.open(debug_filename.c_str());

				for(uint rob_id = 0; rob_id < rob_count; rob_id++)
				{
					cout << total_order[rob_id] << " ";
					debug_file << total_order[rob_id] << " ";
				}

				cout << endl;
				debug_file.close();
			#endif

			//================================================== 6. Start-time offsets : START
			bool flag_inactivated_robot = false;
			start_time_offsets = compute_start_time_offsets(total_order, feasible_paths, S_r, opt_goal_vec, flag_inactivated_robot);
			
			// --- NEW: COMMUNICATION HALT LOGIC (Mimicking Collision Halt) ---
            // --- NEW: COMMUNICATION HALT LOGIC (Mimicking Collision Halt) ---
            if (!flag_inactivated_robot) {
                loc_mat temp_traj = compute_optimal_trajectories(num_of_robs, start_time_offsets, feasible_paths, S_r, call_id);
                uint max_time = 0;
                for (uint i = 0; i < rob_count; i++) {
                    if (temp_traj[i].size() > max_time) max_time = temp_traj[i].size();
                }

                int break_time = -1;
                float previous_max_edge = FLT_MAX; // SMART RECOVERY: Track if network is healing

                for (uint t = 0; t < max_time; t++) {
                    loc_vec swarm_locs(rob_count);
                    for (uint i = 0; i < rob_count; i++) {
                        if (t < temp_traj[i].size()) swarm_locs[i] = temp_traj[i][t];
                        else swarm_locs[i] = temp_traj[i].back();
                    }

                    bool_vec in_tree(rob_count, false);
                    vector<float> min_dist(rob_count, FLT_MAX);
                    in_tree[0] = true;
                    
                    for (uint i = 1; i < rob_count; i++) {
                        float dx = swarm_locs[0].x - swarm_locs[i].x;
                        float dy = swarm_locs[0].y - swarm_locs[i].y;
                        min_dist[i] = sqrt(dx*dx + dy*dy);
                    }

                    bool network_safe = true;
                    float current_max_edge = 0;

                    for (uint step = 1; step < rob_count; step++) {
                        float best_dist = FLT_MAX;
                        int best_node = -1;
                        for (uint i = 0; i < rob_count; i++) {
                            if (!in_tree[i] && min_dist[i] < best_dist) {
                                best_dist = min_dist[i];
                                best_node = i;
                            }
                        }

                        if (best_dist > current_max_edge) current_max_edge = best_dist;

                        in_tree[best_node] = true;
                        for (uint i = 0; i < rob_count; i++) {
                            if (!in_tree[i]) {
                                float dx = swarm_locs[best_node].x - swarm_locs[i].x;
                                float dy = swarm_locs[best_node].y - swarm_locs[i].y;
                                float dist = sqrt(dx*dx + dy*dy);
                                if (dist < min_dist[i]) {
                                    min_dist[i] = dist;
                                }
                            }
                        }
                    }

                    // SMART RECOVERY: Only snap if it is breaking AND getting worse
                    if (current_max_edge > comm_range) {
                        if (current_max_edge < previous_max_edge) {
                            network_safe = true; 
                        } else {
                            network_safe = false; 
                            break_time = t;
                            break;
                        }
                    }
                    previous_max_edge = current_max_edge;
                    if (!network_safe) break;
                }

                if (break_time != -1) {
                    float com_x = 0, com_y = 0;
                    for (uint i = 0; i < rob_count; i++) {
                        struct loc l = (break_time < temp_traj[i].size()) ? temp_traj[i][break_time] : temp_traj[i].back();
                        com_x += l.x; com_y += l.y;
                    }
                    com_x /= rob_count; com_y /= rob_count;

                    int furthest_rob = -1;
                    float max_dist = -1;
                    for (uint i = 0; i < rob_count; i++) {
                        if (S_r[i]) { 
                            int req_idx = -1;
                            for(uint j = 0; j <= i; j++) if(S_r[j]) req_idx++;

                            // BUG FIX: ONLY evaluate drones that are NOT already parked
                            if (opt_goal_vec[req_idx] != -1) {
                                struct loc l = (break_time < temp_traj[i].size()) ? temp_traj[i][break_time] : temp_traj[i].back();
                                float d = sqrt(pow(l.x - com_x, 2) + pow(l.y - com_y, 2));
                                if (d > max_dist) {
                                    max_dist = d;
                                    furthest_rob = i;
                                }
                            }
                        }
                    }

                    if (furthest_rob != -1) {
                        int req_idx = -1;
                        for(uint j = 0; j <= furthest_rob; j++)
                            if(S_r[j]) req_idx++;

                        // --- NEW: TOTAL DEADLOCK PREVENTION ---
                        int currently_active = 0;
                        for (uint j = 0; j < opt_goal_vec.size(); j++) {
                            if (opt_goal_vec[j] != -1) currently_active++;
                        }

                        // If parking this drone paralyzes the entire swarm, refuse to park it!
                        if (currently_active <= 1 && opt_goal_vec[req_idx] != -1) {
                            
                            // --- NEW: COHESIVE RETREAT (TENSION RELIEF) ---
                            // Instead of stepping away, force the drone to step BACK toward the swarm!
                            struct loc current_loc = temp_traj[furthest_rob][0];
                            struct loc retreat_step = current_loc;
                            
                            float dx = com_x - current_loc.x;
                            float dy = com_y - current_loc.y;
                            
                            // Move 1 cell towards the Center of Mass
                            if (abs(dx) > 0.1 || abs(dy) > 0.1) { 
                                if (abs(dx) > abs(dy)) {
                                    retreat_step.x += (dx > 0) ? 1 : -1;
                                } else {
                                    retreat_step.y += (dy > 0) ? 1 : -1;
                                }
                            }
                            
                            // Overwrite the drone's path with the Retreat Step
                            if (temp_traj[furthest_rob].size() > 1) {
                                temp_traj[furthest_rob][1] = retreat_step;
                                temp_traj[furthest_rob].resize(2); // strictly 1 step
                            } else {
                                temp_traj[furthest_rob].push_back(retreat_step);
                            }
                            
                            std::cout << "\n[DEADLOCK AVERTED] Cohesive Retreat: R_" << furthest_rob << " stepping back to swarm.";
                            feasible_paths[furthest_rob] = temp_traj[furthest_rob];
                            // Notice we DO NOT set flag_inactivated_robot = true here.
                        } 
                        else if (opt_goal_vec[req_idx] != -1) {
                            // std::cout << "\n[NETWORK HALT] R_" << furthest_rob << " parked to prevent tether snap.";
                            opt_goal_vec[req_idx] = -1;
                            flag_inactivated_robot = true;
                        }
                        // --------------------------------------
                    }
                }
            }
            // ----------------------------------------------------------------------------------------------------------------------------

			if(flag_inactivated_robot)
				adjust_dependent_paths(opt_goal_vec, feasible_paths, S_r, active_count);
			else
			{
				// cout << "\nSTO...";

				#ifdef DEBUG_SO
					debug_filename = con_plan_exec_pkg_path + OUTPUT_DIR + DEBUG_SO_FILENAME + to_string(call_id) + "_i" + to_string(iter_id) + TXT_EXT;				
					debug_file.open(debug_filename.c_str());
					cout << endl;

					for(uint rob_id = 0; rob_id < rob_count; rob_id++)
					{
						uint sto = start_time_offsets[rob_id];

						if(sto)
							cout << "R_" << rob_id << "=" << sto << " ";
						else	
							cout << sto << " ";

						debug_file << sto;

						if((rob_id + 1) != rob_count)
							debug_file << " ";
					}

					debug_file.close();
				#endif

				break;
			}
			//================================================== 6. Start-time offsets : END
			//================================================== 6. Start-time offsets : END
		}

		opt_paths = feasible_paths;
		// cout << "\nDP...";

		#ifdef DEBUG_DP
			debug_filename = con_plan_exec_pkg_path + OUTPUT_DIR + DEBUG_DP_FILENAME + to_string(call_id) + "_i" + to_string(iter_id) + CSV_EXT;
			debug_file.open(debug_filename.c_str());

			for(uint rob_id = 0; rob_id < rob_count; rob_id++)
			{
				loc_vec path_tmp = opt_paths[rob_id];
				uint path_tmp_len = path_tmp.size() - 1;
				cout << "\nPath_" << rob_id << " ";

				for(uint state_id = 0; state_id <= path_tmp_len; state_id++)
				{
					cout << "(" << path_tmp[state_id].x << ", " << path_tmp[state_id].y << ", " << path_tmp[state_id].theta << ") ";
					debug_file << "(" << path_tmp[state_id].x << " " << path_tmp[state_id].y << " " << path_tmp[state_id].theta << "),";
				}

				debug_file << endl;
			}
		#endif
		//int pause;cout<<"\n...Pause...\n";cin>>pause;
		//================================================== 5. Total Order : END

		iter_id++;
	}

	//================================================== 7. Collision Averted Paths : START
	loc_mat trajectories = compute_optimal_trajectories(num_of_robs, start_time_offsets, feasible_paths, S_r, call_id);
	// cout << "\nCAP...\n";

	#ifdef DEBUG_FILE_CAP
		debug_filename = con_plan_exec_pkg_path + OUTPUT_DIR + DEBUG_CAP_FILENAME + to_string(call_id) + CSV_EXT;
		debug_file.open(debug_filename.c_str());

		for(uint rob_id = 0; rob_id < rob_count; rob_id++)
		{
			#ifdef DEBUG_DISPLAY_CAP
				if(S_r[rob_id])
					cout << "\nPR_" << rob_id;
				else
					cout << "\nNPR_" << rob_id;
			#endif

			loc_vec t = trajectories[rob_id];

			for(uint state_id = 0; state_id < t.size(); state_id++)
			{
				struct loc robot_loc = t[state_id];

				#ifdef DEBUG_DISPLAY_CAP
					cout << " (" << robot_loc.x << ", " << robot_loc.y << ")";
				#endif

				debug_file << "(" << robot_loc.x << " " << robot_loc.y << " " << robot_loc.theta << "),";
			}

			// #ifdef DEBUG_DISPLAY_CAP
			// 	cout << endl;
			// #endif
			
			debug_file << endl;
		}

		debug_file.close();
	#endif
	//================================================== 7. Collision Averted Paths : END

	#ifdef MONITOR_PATHS
		monitor_paths(ws_size_x, ws_size_y, rob_count, trajectories);
	#endif

	return trajectories;

}


