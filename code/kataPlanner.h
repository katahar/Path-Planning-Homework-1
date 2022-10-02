#ifndef _MEX
    #define _MEX
    #include <mex.h>
#endif
#ifndef _MATH
    #define _MATH
    #include <math.h>
#endif
#ifndef _KATANODE
    #define _KATANODE
    #include "planNode.h"
#endif

#include <list>
#include <stdio.h>
#include <vector>
#include <limits>
#include <queue>
#include <bits/stdc++.h>
#include <algorithm>    // std::max
#include <map>
#include <utility>      // std::pair, std::make_pair
#include <iostream>
#include <chrono>
#include <tuple>
#include <cmath>



//==========================================================================================
class kataPlanner
{
    protected:
        #define NUMDIRS 9
        double*	map;
        double* target_traj;
        int target_steps;
        int robotposeX;
        int robotposeY;
        int collision_thresh;
        int x_size;
        int y_size;
        int heuristic_weight = 1;
        int last_x, last_y; 
        // std::priority_queue<planNode*, std::vector<planNode*>, std::greater<std::vector<planNode*>::value_type> > open_list; 
        
        struct compareFvals{
            bool operator()(planNode* const& left, planNode* const& right)
            {
                if(left->get_f() != right->get_f())
                {
                    return left->get_f() > right->get_f();
                }
                else
                {
                    //tie-breaking if f is the same
                    return left->get_h() < right->get_h();
                }
            }     
        };
        
        std::priority_queue<planNode*, std::vector<planNode*>, compareFvals> open_list;      
        
        struct tuple_hash_function
        {
            size_t operator()(const std::tuple<int, int, int>&x) const
            {
                return std::get<0>(x) ^ std::get<1>(x) ^ std::get<2>(x);
            }
        };

        std::unordered_set<std::tuple<int, int, int>, tuple_hash_function> closed_list;

        struct tuple_hash_function2D
        {
            size_t operator()(const std::tuple<int, int>&x) const
            {
                return std::get<0>(x) ^ std::get<1>(x);
            }
        };

        std::unordered_set<std::tuple<int, int>, tuple_hash_function2D> closed_list2D;

        std::chrono::time_point<std::chrono::system_clock> startTime;
        int elapsed_time; 
        bool expanded_goal = false; 
        std::vector<int> path; 
        planNode* last_found_goal = new planNode();
        
        // 8-connected grid
        int dX[NUMDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
        int dY[NUMDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};


    public: 
        

        kataPlanner()
        {
            // mexPrintf("Function %s line %d \n", __FUNCTION__, __LINE__);
        }

        kataPlanner(double*	map_in, int x_size, int y_size, int target_steps, double* target_traj_in, int col_thresh, int robotposeX, int robotposeY)
        {
            this -> target_steps = target_steps;
            this -> target_traj = target_traj_in;
            this -> map =  map_in;
            this -> collision_thresh = col_thresh;
            this -> robotposeX = robotposeX;
            this -> robotposeY = robotposeY;
            this -> x_size = x_size;
            this -> y_size = y_size;
            last_x = target_traj[target_steps-1]-1; //TAKE INTO ACCOUNT THIS OFFSET FOR MATLAB INDEXING
            last_y = target_traj[target_steps-1 + target_steps]-1;
            start_timer();
        } 

        int get_map_ind(int x_ind, int y_ind)
        {
            return y_ind*x_size + x_ind;
        }

        int get_last_x()
        {
            return last_x;
        }
        
        int get_last_y()
        {
            return last_y;
        }

        planNode* get_next_from_open() //pulls the node with the next best f value. 
        {           
            planNode* ret_val = new planNode();
            ret_val = open_list.top();
            open_list.pop();
            // mexPrintf("Popped x: %d, y: %d, t:%d, g: %d, OL: %d\n", ret_val->get_dim(0), ret_val->get_dim(1), ret_val->get_dim(2), ret_val->get_g(), open_list.size());

            return ret_val;
        }

        bool open_is_empty()
        {
            return open_list.empty();
        }
    
        bool set_costs(planNode* neighbor, int cumulative_cost) //returns true if an obstacle
        {
            bool obs_flag = neighbor-> set_c(map[get_map_ind(neighbor->get_dim(0), neighbor->get_dim(1))], collision_thresh); 
            if(!(obs_flag)); //set_c returns true if obstacle
            {
                neighbor->set_g_cumulative(cumulative_cost); //node adds cost to the provided cumulative cost. 
                neighbor->set_h(scaled_slide_heuristic(neighbor)); //@TODO: add in heuristics. 
                return obs_flag; 
            }
        }

        int scaled_slide_heuristic(planNode* neighbor)
        {
            if(neighbor->get_dim(2) < target_steps)
            {
                // return euclidean_dist(neighbor->get_dim(0), neighbor->get_dim(1), target_traj[target_steps-neighbor->get_dim(2)],target_traj[target_steps+target_steps-neighbor->get_dim(2)]);
                return euclidean_dist(neighbor->get_dim(0), neighbor->get_dim(1), target_traj[target_steps+neighbor->get_dim(2)]-1,target_traj[target_steps+neighbor->get_dim(2)]-1);

            }
            else
            {
                return 100000000;
            }
        }

        int euclidean_dist(int x1, int y1, int x2, int y2)
        {
            return int(std::sqrt( std::pow(x1-x2,2) + std::pow(y1-y2,2)));
        }

        int end_heuristic(planNode* input)
        {
            return int(std::sqrt( std::pow(input->get_dim(0)-get_last_x(),2) + std::pow(input->get_dim(1)-get_last_y(),2)));
        }

        void start_timer()
        {
            startTime = std::chrono::system_clock::now();
        }

        double cumulative_time()
        {
            std::chrono::time_point<std::chrono::system_clock> curTime = std::chrono::system_clock::now();
            return std::chrono::duration_cast<std::chrono::milliseconds>(curTime - startTime).count()/1000;
        }

        bool goal_not_expanded()
        {
            return !expanded_goal;
        }

        void print_path()
        {
            mexPrintf("\nFinal target location\nx: %d y: %d\n\n", get_last_x(),get_last_y());
            for(int i = 0; i < path.size(); i = i+2)
            {
                mexPrintf("x: %d, y: %d \n", path[i], path[i+1]);
            }
        }
        
        int get_x_dir(int t_step)
        {
            if(2*t_step < path.size())
            {
                // mexPrintf("x: %d", path[2*t_step]);
                return path[2*t_step];
            }
            else
            {
                // mexPrintf("x: %d", path[path.size()-2]);
                return path[path.size()-2];
            }
        }

        int get_y_dir(int t_step)
        {
            if(2*t_step < path.size())
            {
                // mexPrintf("y: %d", path[(2*t_step) + 1]);
                return path[(2*t_step) + 1];
            }
            else
            {
                // mexPrintf("y: %d", path[path.size()-1]);
                return path[path.size()-1];
            }
        }

        //@TODO: split these into their individual child classes VV

        bool valid_coords(int x1, int y1)
        {
            if(x1 > -1 && y1 > -1 && x1 < x_size && y1 < y_size)
            {
                return true; 
            }
            return false; 
        }

        // bool valid_coords(int x1, int y1, int t1 )
        // {
        //     if(x1 > -1 && y1 > -1 && x1 < x_size && y1 < y_size)
        //     {
        //         return true; 
        //     }
        //     return false; 
        // }

        bool valid_coords(std::tuple<int, int, int> input )
        {
            if( (std::get<0>(input) == 175) && (std::get<1>(input) == 100) ||
                (std::get<0>(input) == 176) && (std::get<1>(input) == 101) ||
                (std::get<0>(input) == 174 && (std::get<1>(input) == 99)))
                {
                    mexPrintf("valid coordinating this current x: %d, y: %d, t:%d\n", std::get<0>(input), std::get<1>(input), std::get<2>(input));
                    // mexPrintf("FOUND x SECRETLY\n");

                }

            if( std::get<0>(input) > -1 && 
                std::get<1>(input) > -1 && 
                std::get<0>(input) < x_size && 
                std::get<1>(input) < y_size && 
                std::get<2>(input) <target_steps )
            {
                return true; 
            }
            return false; 
        }


        bool in_closed(std::tuple<int, int, int> input)
        {
            // mexPrintf("Function %s line %d \n", __FUNCTION__, __LINE__);
            if(closed_list.find(input) != closed_list.end())
            {
                // mexPrintf("Function %s line %d \n", __FUNCTION__, __LINE__);
                return true;
            }
            // mexPrintf("Function %s line %d \n", __FUNCTION__, __LINE__);
            return false; 
        }

        bool in_closed(std::tuple<int, int> input)
        {
            if(closed_list2D.find(input) != closed_list2D.end())
            {
                return true;
            }
            return false; 
        }


};

class kataPlanner2D : public kataPlanner
{
    public:
        #define axes 2 
        // double*	heuristic_map;
        std::unordered_map<std::tuple<int, int>,int,tuple_hash_function2D> open_g_track;
        std::unordered_map<std::tuple<int, int>,int,tuple_hash_function2D> heuristic_map;
        bool first_run = true;

        kataPlanner2D()
        :kataPlanner()
        {

        }
        
        kataPlanner2D(double* map_in, int x_size, int y_size, int target_steps, double* target_traj, int col_thresh, int robotposeX, int robotposeY)
        :kataPlanner(map_in, x_size, y_size, target_steps, target_traj, col_thresh, robotposeX, robotposeY)
        {
            this->robotposeX = robotposeX-1;
            this->robotposeY = robotposeY-1;
        }

        bool cur_is_goal(planNode* input) //introducing offset here. 
        {
            if(first_run)
            {
                // works for 6,5,4
                if(target_traj[input->get_dim(2)+2]-1 == input->get_dim(0) &&
                target_traj[target_steps+input->get_dim(2)+2]-1 ==  input->get_dim(1) )
                {
                    input->set_is_goal(true);
                    return true;
                }
            }
            else
            {
                //works for 3,2, 1
                for(int i = input->get_dim(2)+2; i < target_steps; ++i)
                {
                    if(input->get_dim(0)== target_traj[i]-1 && 
                        input->get_dim(1) == target_traj[target_steps+i]-1 )
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        void evaluate_neighbors(planNode* current)
        {
            for(int i = 0; i < NUMDIRS; ++i)
            {
                if(valid_coords(current->get_dim(0)+dX[i], current->get_dim(1)+dY[i]) && goal_not_expanded() )
                {
                    evaluate_neighbor(current, dX[i], dY[i]);
                }
            }               
            
            if(cur_is_goal(current)) 
            {
                mark_expanded(current); 
                return;
            }
            // print_open();
        }

        bool improved_g(planNode* new_candidate) // true if the new candidate has a better g value than existing (if any ) g for that location. 
        {
            auto search_iter = open_g_track.find(std::make_tuple(new_candidate->get_dim(0), new_candidate->get_dim(1)));
            if( search_iter != open_g_track.end()) //found candidate in open list
            {
                // mexPrintf("Found something: ");
                // mexPrintf("existing g: %d, new g: %d\n",search_iter->second, new_candidate->get_g());
                if(search_iter->second > new_candidate->get_g()) //new candidate has better g value
                {
                    // search_iter->second = new_candidate->get_g();
                    return true; 
                    // mexPrintf("\tNeed to update value\n");
                }
                else
                {
                    return false;
                    // mexPrintf("\tExisting value is better\n");

                }
            }
            else
            {
                // mexPrintf("No value in open list yet\n");
                return true; 
                //meaning that it is not in open. 
                // std::tuple<int, int> new_key= std::make_tuple(new_candidate->get_dim(0), new_candidate->get_dim(1));
                // open_g_track.insert({new_key,new_candidate->get_g()});
            }
        }

        void print_open()
        {
            std::priority_queue<planNode*, std::vector<planNode*>, compareFvals> print_open_list;      

            print_open_list = open_list;

            mexPrintf("Latest Open list by F:\n");
            while(!print_open_list.empty())
            {
                mexPrintf("%d\n",print_open_list.top()->get_f());
                print_open_list.pop();
            }     

        }

        void add_to_open(planNode* input)
        {
            std::tuple<int, int> new_key= std::tuple<int, int>(input->get_dim(0), input->get_dim(1));
            // std::make_tuple(input->get_dim(0), input->get_dim(1));
            open_g_track.erase(new_key);
            // if(open_g_track.erase(new_key))
            // {
            //     // mexPrintf("Erased existing key.\n");
            // }
            // else 
            // {
            //     // mexPrintf("nothing to erase.\n");

            // }
            // mexPrintf("G going into emplacement is %d\n", input->get_g());
            open_g_track.insert({new_key,input->get_g()});
            // mexPrintf("Retrieved g: %d\n", open_g_track.find(new_key)->second);
            open_list.push(input);
        }
        
        void evaluate_neighbor(planNode* current, int rel_x, int rel_y)
        {
            if(!in_closed(std::make_tuple(current->get_dim(0)+rel_x,current->get_dim(1)+rel_y))) //verify that the neighbor is not already in the closed list
            {
                planNode* neighbor = new planNode(current->get_dim(0) + rel_x, current->get_dim(1) + rel_y, current->get_dim(2) + 1);
                if(!(set_costs(neighbor, current->get_g())) && improved_g(neighbor))//returns true if neighbor has a better g value than the one currently in the open list. 
                {
                    neighbor -> set_prev(current);
                    add_to_open(neighbor);
                }
                else
                {
                    delete neighbor;
                }

            }
        }   

        void mark_expanded(planNode* goal_input)
        {
            last_found_goal = goal_input->get_ptr();
            expanded_goal = true;
            goal_input->set_is_goal(true);
            mexPrintf("Goal found! (%d, %d, %d)\n", goal_input->get_dim(0), goal_input->get_dim(1), goal_input->get_dim(2));
        }

        void generate_path()
        {
            planNode* start = new planNode(robotposeX, robotposeY,0);
            start->set_is_start();
            add_to_open(start);

            while(!open_is_empty() && goal_not_expanded())  
            {
                planNode* current = new planNode();
                current = get_next_from_open();
                if(!in_closed(std::make_tuple(current->get_dim(0),current->get_dim(1) ))) //current is in closed, so already evaluated. 
                {
                    evaluate_neighbors(current);
                    add_to_closed(current);
                }
                else
                {
                    delete current; //memory management
                }
            }

            if(open_is_empty() || goal_not_expanded())
            {
                mexPrintf("Original planner failed. Modifying success definition.\n");
                open_g_track.clear();
                closed_list2D.clear();
                first_run = false;
                while(!open_list.empty())
                {
                    open_list.pop();
                }

                planNode* start = new planNode(robotposeX, robotposeY,0);
                start->set_is_start();
                add_to_open(start);

                while(!open_is_empty() && goal_not_expanded())  
                {
                    planNode* current = new planNode();
                    current = get_next_from_open();
                    if(!in_closed(std::make_tuple(current->get_dim(0),current->get_dim(1) ))) //current is in closed, so already evaluated. 
                    {
                        evaluate_neighbors(current);
                        add_to_closed(current);
                    }
                    else
                    {
                        delete current; //memory management
                    }
                }

            }
            
            populate_path(last_found_goal);
           
            // mexPrintf("Ending time %f", cumulative_time());
        }

        void populate_path(planNode* goal)
        {
            // mexPrintf("Populating path."); // x: %d, y: %d\n", goal->get_dim(0),goal->get_dim(1));
            planNode* current = new planNode();
            current = goal;
            planNode* prev = new planNode();
            prev = goal->get_prev_ptr();
            int count = 0; 

            while(!(prev->get_is_start()))
            {
                path.insert(path.begin(), current->get_dim(1)+1); //puts at the beginning. 
                path.insert(path.begin(), current->get_dim(0)+1);
                // mexPrintf("%f \n", map[get_map_ind(current->get_dim(0), current->get_dim(1))] );
                current = prev;
                prev = current->get_prev_ptr();
            } 
            path.insert(path.begin(), current -> get_dim(1)+1); 
            path.insert(path.begin(), current -> get_dim(0)+1);
            path.insert(path.begin(), prev -> get_dim(1)+1); 
            path.insert(path.begin(), prev -> get_dim(0)+1);
            // mexPrintf("Population complete \n");

        }

        void add_to_closed(planNode* input)
        {
            std::tuple<int, int> temp_tuple = std::make_tuple(input->get_dim(0),input->get_dim(1));
            closed_list2D.insert(temp_tuple);
        }

        // ==================================== Heuristic Functions ====================================================
        void evaluate_neighbor_no_heuristic(planNode* current, int rel_x, int rel_y)
        {
            if(!in_closed(std::make_tuple(current->get_dim(0)+rel_x,current->get_dim(1)+rel_y))) //verify that the neighbor is not already in the closed list
            {
                planNode* neighbor = new planNode(current->get_dim(0) + rel_x, current->get_dim(1) + rel_y);
                if(!(set_costs_no_heuristic(neighbor, current->get_g())))//returns true if neighbor has a better g value than the one currently in the open list. 
                {
                    neighbor -> set_prev(current);
                    add_to_open(neighbor);
                }
                else
                {
                    delete neighbor;
                }

            }
        }   

        void evaluate_neighbors_no_heuristic(planNode* current)
        {
            for(int i = 0; i < NUMDIRS; ++i)
            {
                if(valid_coords(current->get_dim(0)+dX[i], current->get_dim(1)+dY[i]) )
                {
                    evaluate_neighbor_no_heuristic(current, dX[i], dY[i]);
                }
            }
        }

       bool set_costs_no_heuristic(planNode* neighbor, int cumulative_cost) //returns true if an obstacle
        {
            bool obs_flag = neighbor-> set_c(map[get_map_ind(neighbor->get_dim(0), neighbor->get_dim(1))], collision_thresh); 
            if(!(obs_flag)); //set_c returns true if obstacle
            {
                neighbor->set_g_cumulative(cumulative_cost); //node adds cost to the provided cumulative cost. 
                return obs_flag; 
            }
        }


        void add_traj_to_open()
        {
            for(int i = 0; i < target_steps; ++i)
            {
                planNode* traj_node = new planNode(target_traj[i]-1, target_traj[target_steps + i]-1);
                // mexPrintf("Adding from trajectory: (%d,%d)\n",traj_node ->get_dim(0), traj_node ->get_dim(1));
                traj_node -> set_c(map[get_map_ind(traj_node ->get_dim(0), traj_node ->get_dim(1))]);
                add_to_open(traj_node);
            }
        }

        void reverse_dijkstra()
        {
            while( !open_is_empty() && goal_not_expanded() )  
            {
                planNode* current = new planNode();
                current = get_next_from_open();
                std::tuple<int, int> new_key;
                new_key = std::make_tuple(current->get_dim(0),current->get_dim(1));
                if(!in_closed(new_key)) //current is in closed, so already evaluated. 
                {
                    evaluate_neighbors_no_heuristic(current);
                    add_to_closed(current);
                    heuristic_map.emplace(new_key, current->get_g());
                    // mexPrintf("%d \n", heuristic_map.find(new_key)->second);
                }
                else
                {
                    delete current; //memory management
                }
            }
        }

        void run_as_heuristic()
        {
            add_traj_to_open();
            mexPrintf("Open list size: %d\n", open_list.size());
            mexPrintf("Executing reverse diklstra...\n");
            reverse_dijkstra();
            mexPrintf("Complete...%d\n", heuristic_lookup(120,120));
            // print_heuristic();
        }

        

        int heuristic_lookup(int x, int y)
        {
            std::tuple<int, int> input_loc = std::make_tuple(x,y);
            return heuristic_lookup(input_loc);
        }

        int heuristic_lookup(std::tuple<int, int> input_loc)
        {
            if(heuristic_map.find(input_loc) != heuristic_map.end())
            {
                // mexPrintf("Heuristic: %d\n", heuristic_map.find(input_loc)->second);
                return heuristic_map.find(input_loc)->second;
            }
            else 
            {
                return -2;
            }
        }


        void print_heuristic()
        {
            for (int i = 0; i <x_size; ++i)
            {
                for(int j = 0; j < y_size; ++j)
                {
                    mexPrintf("%d ",heuristic_lookup(i,j));
                }
                mexPrintf("\n");
            }
        }

};


class kataPlanner3D : public kataPlanner
{
    public:
        #define axes 3 
        double*	heuristic_map;
        int last_goal_t_step = -1;
        std::unordered_map<std::tuple<int, int, int>,int,tuple_hash_function> open_g_track3D;
        kataPlanner2D heuristic_planner;


        kataPlanner3D()
        :kataPlanner()
        {

        }
        
        kataPlanner3D(double* map_in, int x_size, int y_size, int target_steps, double* target_traj, int col_thresh, int robotposeX, int robotposeY)
        :kataPlanner(map_in, x_size, y_size, target_steps, target_traj, col_thresh, robotposeX, robotposeY)
        {
            heuristic_planner = kataPlanner2D(map_in, x_size, y_size, target_steps, target_traj, col_thresh, robotposeX, robotposeY);
            mexPrintf("Starting heuristic. \n");
            heuristic_planner.run_as_heuristic();
            mexPrintf("Heristic calculation complete. \n");
            // heuristic_planner.print_heuristic();
        }

        void evaluate_neighbors(planNode* current)
        {
            // mexPrintf("Evaluating current x: %d, y: %d, t:%d, g: %d\n", current->get_dim(0), current->get_dim(1), current->get_dim(2), current->get_g());

            for(int i = 0; i < NUMDIRS; ++i)
            {
                std::tuple<int,int,int> neighbor_tuple= std::make_tuple(current->get_dim(0)+dX[i], current->get_dim(1)+dY[i], current->get_dim(2)+1);
                // if( (current->get_dim(0)+dX[i] == 174) ||
                //     (current->get_dim(0) == 174))
                // {
                //     mexPrintf("Evaluating current x: %d, y: %d, t:%d, g: %d\n", current->get_dim(0), current->get_dim(1), current->get_dim(2), current->get_g());
                //     // mexPrintf("FOUND x SECRETLY\n");

                // }
                if(valid_coords(neighbor_tuple) && goal_not_expanded() )
                {
                    evaluate_neighbor(current, neighbor_tuple);
               
                    if(cur_is_goal(current)) 
                    {
                        mark_expanded(current); 
                        return;
                    }
                }
            }
            // print_open();
        }

        bool improved_g(planNode* new_candidate) // true if the new candidate has a better g value than existing (if any ) g for that location. 
        {
            // mexPrintf("Function %s line %d \n", __FUNCTION__, __LINE__);
            auto existing_entry = open_g_track3D.find(new_candidate->get_tuple_3d());
            // mexPrintf("evaluating %d %d %d \n", std::get<0>(new_candidate->get_tuple_3d()), std::get<1>(new_candidate->get_tuple_3d()), std::get<2>(new_candidate->get_tuple_3d()));

            // mexPrintf("Function %s line %d \n", __FUNCTION__, __LINE__);
            // mexPrintf("Found %d %d %d \n", std::get<0>(existing_entry->first), std::get<1>(existing_entry->first), std::get<2>(existing_entry->first));
            if( existing_entry != open_g_track3D.end()) //found candidate in open list
            {
                // mexPrintf("Found something: ");
                //  mexPrintf("\texisting g: %d, new g: %d, x: %d, y: %d, t:%d\n",existing_entry->second, new_candidate->get_g(), new_candidate->get_dim(0), new_candidate->get_dim(1), new_candidate->get_dim(2));
                if(existing_entry->second <= new_candidate->get_g()) 
                {
                    return false;
                }
                else
                {
                    // mexPrintf("\tNeed to update value\n");
                    return true; 
                }
            }
            else
            {
                // mexPrintf("Failed to find in open list. \n");
                return true; 
                //meaning that it is not in open. 
            }
        }

        void print_open()
        {
            std::priority_queue<planNode*, std::vector<planNode*>, compareFvals> print_open_list;      

            print_open_list = open_list;

            mexPrintf("Latest Open list by F:\n");
            while(!print_open_list.empty())
            {
                mexPrintf("%d\n",print_open_list.top()->get_f());
                print_open_list.pop();
            }     

        }

        void add_to_open(planNode* input)
        {
            std::tuple<int, int, int> new_key= std::tuple<int,int, int>(input->get_dim(0), input->get_dim(1), input->get_dim(2));
            open_g_track3D.erase(new_key); //doesnt matter if there is no corresponding key in the map
            open_g_track3D.insert({new_key,input->get_g()});
            // mexPrintf("Retrieved g: %d\n", open_g_track.find(new_key)->second);
            open_list.push(input);
        }
        
        bool cur_is_goal(planNode* input) //introducing offset here. 
        {
            int elapsed = cumulative_time();
            if(elapsed+input->get_dim(2) < target_steps)
            {
                // mexPrintf("Target %d, Node Time %d, elapsed time %d, combined%d\n",target_steps, input->get_dim(2),elapsed,input->get_dim(2)+elapsed );

                // mexPrintf("Target steps %d\tEvaluating Node (%d, %d, %d) \t Elapsed: %d \t X ind: %d Y ind: %d \t Goal(%d %d) \n", target_steps, input->get_dim(0), input->get_dim(1), input->get_dim(2), elapsed, input->get_dim(2)+elapsed, target_steps+input->get_dim(2)+elapsed, int(target_traj[input->get_dim(2)+elapsed]), int(target_traj[target_steps+input->get_dim(2)+elapsed]) );
                
                if(int(target_traj[input->get_dim(2)+elapsed]-1) ==  input->get_dim(0) &&
                int(target_traj[target_steps + input->get_dim(2)+elapsed]-1) ==  input->get_dim(1) )
               {
                    input->set_is_goal(true);
                    mexPrintf("Elapsed time: %d sec\n", elapsed);
                    mexPrintf("Intercept x value: %d\n", input->get_dim(0) );
                    mexPrintf("Intercept y value: %d\n", input->get_dim(1) );
                    mexPrintf("Intercept t value: %d\n", elapsed+input->get_dim(2) );

                    return true;
               }
            }
            if(elapsed > target_steps)
            {
                // mexPrintf("Failed to find goal in time. I'll still run though :/ \n");
            }
            return false;

        }

        void generate_path()
        {
            planNode* start = new planNode(robotposeX, robotposeY, 0);
            start->set_is_start();
            start->set_g(0);
            add_to_open(start);

            while(!open_is_empty() && goal_not_expanded())  
            {
                planNode* current = new planNode();
                current = get_next_from_open();
                // mexPrintf("OL size%d  CL Siz:%d\n", open_list.size(), closed_list.size());
                // mexPrintf("Function %s line %d \n", __FUNCTION__, __LINE__);
                if(!in_closed(current->get_tuple_3d())) //current is in closed, so already evaluated. 
                {
                    // mexPrintf("Function %s line %d \n", __FUNCTION__, __LINE__);
                    evaluate_neighbors(current);
                    // mexPrintf("Function %s line %d \n", __FUNCTION__, __LINE__);
                    add_to_closed(current);
                }
                else
                {
                    delete current; //memory management
                }
            }

            if(open_is_empty())
            {
                mexPrintf("Terminated because Open list is empty. \n");
            }
            if(goal_not_expanded())
            {
                mexPrintf("goal was not expanded.  \n");
            }
            populate_path(last_found_goal);

        }

        void populate_path(planNode* goal)
        {
            mexPrintf("Populating path."); // x: %d, y: %d\n", goal->get_dim(0),goal->get_dim(1));
            planNode* current = new planNode();
            current = goal;
            planNode* prev = new planNode();
            prev = goal->get_prev_ptr();
            int count = 0; 

            while(!(prev->get_is_start()))
            {
                path.insert(path.begin(), current->get_dim(1)); //puts at the beginning. 
                path.insert(path.begin(), current->get_dim(0));
                // mexPrintf("%f \n", map[get_map_ind(current->get_dim(0), current->get_dim(1))] );
                current = prev;
                prev = current->get_prev_ptr();
            } 
            path.insert(path.begin(), current -> get_dim(1)); 
            path.insert(path.begin(), current -> get_dim(0));
            // path.insert(path.begin(), prev -> get_dim(1));  //remove this in 2d?
            // path.insert(path.begin(), prev -> get_dim(0));
            // print_path();

            // mexPrintf("Population complete \n");
        }
       
        void mark_expanded(planNode* goal_input)
        {
            last_found_goal = goal_input->get_ptr();
            expanded_goal = true;
            goal_input->set_is_goal(true);
            mexPrintf("Goal found! (%d, %d, %d)\n", goal_input->get_dim(0), goal_input->get_dim(1), goal_input->get_dim(2));
        }

        int get_x_dir(int t_step)
        {
            return path[2*t_step];
        }

        int get_y_dir(int t_step)
        {
            return path[(2*t_step) + 1];
        }

        // void evaluate_neighbor(planNode* current, int rel_x, int rel_y, int rel_t)
        // {
        //     if(!in_closed(std::make_tuple(current->get_dim(0)+rel_x,current->get_dim(1)+rel_y, current->get_dim(2)+rel_t))) //verify that the neighbor is not already in the closed list
        //     {
        //         planNode* neighbor = new planNode(current->get_dim(0) + rel_x, current->get_dim(1) + rel_y, current->get_dim(2)+rel_t);

        //         if((!(set_costs(neighbor, current->get_g()))) && improved_g(neighbor) ) //returns true if neighbor has a better g value than the one currently in the open list. 
        //         {
        //             // mexPrintf("Adding to open \n");
        //             neighbor -> set_prev(current);
        //             add_to_open(neighbor);
        //             // mexPrintf("\tEvaluating neighbor x: %d, y: %d, t:%d, g: %d\n", neighbor->get_dim(0), neighbor->get_dim(1), neighbor->get_dim(2), neighbor->get_g());

        //         }
        //         else
        //         {
        //             delete neighbor;
        //         }
        //     }
        // }
        void evaluate_neighbor(planNode* current, std::tuple<int, int, int> neigh_tuple)
        {
            if(!in_closed(neigh_tuple)) //verify that the neighbor is not already in the closed list
            {
                planNode* neighbor = new planNode(neigh_tuple);

                if((!(set_costs(neighbor, current->get_g()))) && improved_g(neighbor) ) //returns true if neighbor has a better g value than the one currently in the open list. 
                {
                    // mexPrintf("Adding to open \n");
                    neighbor -> set_prev(current);
                    add_to_open(neighbor);
                    // mexPrintf("\tEvaluating neighbor x: %d, y: %d, t:%d, g: %d\n", neighbor->get_dim(0), neighbor->get_dim(1), neighbor->get_dim(2), neighbor->get_g());

                }
                else
                {
                    delete neighbor;
                }
            }
        }

        void add_to_closed(planNode* input)
        {
            std::tuple<int, int, int> temp_tuple = input->get_tuple_3d();
            closed_list.insert(temp_tuple);
        }


        bool set_costs(planNode* neighbor, int cumulative_cost) //returns true if an obstacle
        {
            bool obs_flag = neighbor-> set_c(map[get_map_ind(neighbor->get_dim(0), neighbor->get_dim(1))], collision_thresh); 
            if(!(obs_flag)); //set_c returns true if obstacle
            {
                neighbor->set_g_cumulative(cumulative_cost); //node adds cost to the provided cumulative cost. 
                neighbor->set_h(20*heuristic_planner.heuristic_lookup(neighbor->get_dim(0),neighbor->get_dim(1)));
                return obs_flag; 
            }
        }

};