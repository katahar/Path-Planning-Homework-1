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
        std::priority_queue<planNode*, std::vector<planNode*>, std::greater<std::vector<planNode*>::value_type> > open_list;      
        
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
        planNode* last_found_goal;
        
        // 8-connected grid
        int dX[NUMDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
        int dY[NUMDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

        int last_x, last_y; 
        
    public: 

        kataPlanner()
        {
            // mexPrintf("Function %s start\n", __FUNCTION__);
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
            last_x = map[x_size*y_size-2];
            last_y = map[x_size*y_size-1];
 
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

        // bool in_closed(planNode* input)
        // {
        //     if(closed_list.find(std::make_tuple(input->get_dim(0),input->get_dim(1),input->get_dim(2))) != closed_list.end())
        //     {
        //         return true;
        //     }
        //     return false; 
        // }

        bool in_closed(std::tuple<int, int, int> input)
        {
            if(closed_list.find(input) != closed_list.end())
            {
                // mexPrintf("in closed");
                return true;
            }
            return false; 
        }

        bool in_closed(std::tuple<int, int> input)
        {
            if(closed_list2D.find(input) != closed_list2D.end())
            {
                // mexPrintf("in closed");
                return true;
            }
            return false; 
        }


        void add_to_open(planNode* input)
        {
            open_list.push(input);
        }

        planNode* get_next_from_open() //pulls the node with the next best f value. 
        {           
            planNode* ret_val = new planNode();
            ret_val = open_list.top();
            open_list.pop();
            return ret_val;
        }

        bool open_is_empty()
        {
            return open_list.empty();
        }
    

        void set_costs(planNode* neighbor, int cumulative_cost)
        {
            if(!(neighbor-> set_c(map[get_map_ind(neighbor->get_dim(0),neighbor->get_dim(1))], collision_thresh))); //true if obstacle
            {
                neighbor->set_g_cumulative(cumulative_cost); //node adds cost to the provided cumulative cost. 
                neighbor->set_h(end_heuristic(neighbor)); //@TODO: add in heuristics.  
            }
        }

        int end_heuristic(planNode* input)
        {
            return int(std::sqrt( std::pow(input->get_dim(0)-get_last_x(),2) + std::pow(input->get_dim(1)-get_last_y(),2)));
        }

        void start_timer()
        {
            startTime = std::chrono::system_clock::now();
        }

        int cumulative_time()
        {
            std::chrono::time_point<std::chrono::system_clock> curTime;
            return std::chrono::duration_cast<std::chrono::seconds>(curTime - startTime).count();
        }

        bool goal_not_expanded()
        {
            return !expanded_goal;
        }

        // bool valid_coords(std::vector<int> input)
        // {
        //     if(input[0] > -1 && input[1] > -1 && input[0] < x_size && input[1] < y_size)
        //     {
        //         return true; 
        //     }
        //     return false; 
        // }

        bool valid_coords(int x1, int y1)
        {
            if(x1 > -1 && y1 > -1 && x1 < x_size && y1 < y_size)
            {
                return true; 
            }
            return false; 
        }

        bool valid_coords(int x1, int y1, int t1 )
        {
            if(x1 > -1 && y1 > -1 && x1 < x_size && y1 < y_size)
            {
                return true; 
            }
            return false; 
        }

};


class kataPlanner3D : public kataPlanner
{
    public:
        #define axes 3 
        double*	heuristic_map;
        int last_goal_t_step = -1;


        kataPlanner3D()
        :kataPlanner()
        {

        }
        
        kataPlanner3D(double* map_in, int x_size, int y_size, int target_steps, double* target_traj, int col_thresh, int robotposeX, int robotposeY)
        :kataPlanner(map_in, x_size, y_size, target_steps, target_traj, col_thresh, robotposeX, robotposeY)
        {

        }

        void evaluate_neighbors(planNode* current)
        {
            for(int i = 0; i < NUMDIRS; ++i)
            {
                if(valid_coords(current->get_dim(0)+dX[i], current->get_dim(1)+dY[i], current->get_dim(2)+1))
                {
                    evaluate_neighbor(current, dX[i], dY[i], 1);
                }
            }
        }
       
        bool cur_is_goal(planNode* input) //introducing offset here. 
        {
            int elapsed = cumulative_time();
            if(target_traj[2*(input->get_dim(2))] ==  input->get_dim(0) &&
               target_traj[2*(input->get_dim(2))+1] ==  input->get_dim(1) )
               {
                    input->set_is_goal(true);
                    return true;
               }
            return false;
        }

        void generate_heuristic()
        {

        }

        void generate_path()
        {
            planNode* start = new planNode(robotposeX, robotposeY, 0);
            start->set_is_start();
            add_to_open(start);

            while(!open_is_empty() && goal_not_expanded())  //at this point, the first goal has been reached. But need to validate that it is at the right time
            {
                planNode* current = new planNode();
                current = get_next_from_open();
                evaluate_neighbors(current);
                add_to_closed(current);
            }
            populate_path(last_found_goal);

        }

        void populate_path(planNode* goal)
        {
            mexPrintf("Populating path.\n");
            planNode* current = goal; 
            planNode* prev = goal->get_prev_ptr();

            while(!(prev->get_is_start()))
            {
                path.insert(path.begin(), (current->get_dim(1) - prev->get_dim(1))); //puts at the beginning. 
                path.insert(path.begin(), (current->get_dim(0) - prev->get_dim(0)));
                current = prev;
                prev = current->get_prev_ptr();
            } 
        }
       
        void mark_expanded(planNode* goal_input)
        {
            last_goal_t_step = goal_input->get_dim(2);
            last_found_goal = goal_input;
            expanded_goal = true;
            goal_input->set_is_goal(true);
            goal_input->get_prev_ptr();
        }

        int get_x_dir(int t_step)
        {
            return path[2*t_step];
        }

        int get_y_dir(int t_step)
        {
            return path[(2*t_step) + 1];
        }

        void evaluate_neighbor(planNode* current, int rel_x, int rel_y, int rel_t)
        {
            if(!in_closed(std::make_tuple(current->get_dim(0),current->get_dim(1),current->get_dim(2)))) //verify that the node is not in the closed list
            {
                planNode* temp_node = new planNode(current->get_dim(0)+rel_x, current->get_dim(1)+rel_y, current->get_dim(2)+rel_t);
                set_costs(temp_node,current->get_g());
                temp_node -> set_prev(current);
                add_to_open(temp_node);
            }

            if(cur_is_goal(current))
            {
                mark_expanded(current); //could probably just check the closed list for goals.
                // mexPrintf("Line Number %s:%d\n", __FUNCTION__, __LINE__);
                //Verify that goals are actually populated. !!!!!
            }
        }

        void add_to_closed(planNode* input)
        {
            std::tuple<int, int, int> temp_tuple = std::make_tuple(input->get_dim(0),input->get_dim(1),input->get_dim(2));
            closed_list.insert(temp_tuple);
        }
};

class kataPlanner2D : public kataPlanner
{
    public:
        #define axes 2 
        // double*	heuristic_map;

        kataPlanner2D()
        :kataPlanner()
        {

        }
        
        kataPlanner2D(double* map_in, int x_size, int y_size, int target_steps, double* target_traj, int col_thresh, int robotposeX, int robotposeY)
        :kataPlanner(map_in, x_size, y_size, target_steps, target_traj, col_thresh, robotposeX, robotposeY)
        {

        }

        bool cur_is_goal(planNode* input) //introducing offset here. 
        {
            if(target_traj[target_steps-2] ==  input->get_dim(0) &&
               target_traj[target_steps-1] ==  input->get_dim(1) )
               {
                    input->set_is_goal(true);
                    return true;
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
        }

        void generate_heuristic()
        {

        }

        void evaluate_neighbor(planNode* current, int rel_x, int rel_y)
        {
            if(!in_closed(std::make_tuple(current->get_dim(0)+rel_x,current->get_dim(1)+rel_y))) //verify that the neighbor is not already in the closed list
            {
                planNode* temp_node = new planNode(current->get_dim(0)+rel_x, current->get_dim(1)+rel_y);
                set_costs(temp_node,current->get_g());
                temp_node -> set_prev(current);
                add_to_open(temp_node);
            }

            if(cur_is_goal(current)) //never achieves this!
            {
                mark_expanded(current); 
                return;
                //could probably just check the closed list for goals.
                // mexPrintf("Line Number %s:%d\n", __FUNCTION__, __LINE__);
                //Verify that goals are actually populated. !!!!!
            }
        }   
       
        void mark_expanded(planNode* goal_input)
        {
            last_found_goal = goal_input;
            expanded_goal = true;
            goal_input->set_is_goal(true);
            goal_input->get_prev_ptr();
            mexPrintf("Goal found!\n");
        }

        void generate_path()
        {
            planNode* start = new planNode(robotposeX, robotposeY);
            start->set_is_start();
            add_to_open(start);

            while(!open_is_empty() && goal_not_expanded())  
            {
                planNode* current = new planNode();
                current = get_next_from_open();
                if(!in_closed(std::make_tuple(current->get_dim(0),current->get_dim(1)))) //current is in closed, so already evaluated. 
                {
                    evaluate_neighbors(current);
                    add_to_closed(current);
                }
                else
                {
                    delete current; //memory management
                }
            }

            if(open_is_empty())
            {
                mexPrintf("empty. \n");
            }
            if(goal_not_expanded())
            {
                mexPrintf("goal not expanded. \n");
            }
            //at this point, the first goal has been reached. But need to validate that it is at the right time
            populate_path(last_found_goal);
        }

        void populate_path(planNode* goal)
        {
            mexPrintf("Populating path.\n");
            planNode* current = new planNode();
            current = goal;
            planNode* prev = new planNode();
            prev = goal->get_prev_ptr();
            int count = 0; 

            while(!(prev->get_is_start()))
            {
                path.insert(path.begin(), current->get_dim(1));//(-current->get_dim(1) + prev->get_dim(1))); //puts at the beginning. 
                path.insert(path.begin(), current->get_dim(0));//(-current->get_dim(0) + prev->get_dim(0)));
                current = prev;
                prev = current->get_prev_ptr();
            } 
            mexPrintf("Population complete \n");

            mexPrintf("Final target location\n x: %d y: %d",get_last_x(),get_last_y());
            for(int i = 0; i < path.size(); i = i+2)
            {
                mexPrintf("x: %d, y: %d \n", path[i], path[i+1]);
            }

        }

        int get_x_dir(int t_step)
        {
            if(t_step < path.size())
            {
                return path[2*t_step];
            }
            else
            {
                return 0;
            }
        }

        int get_y_dir(int t_step)
        {
            if(t_step < path.size())
            {
                return path[(2*t_step) + 1];
            }
            else
            {
                return 0;
            }
        }

        void add_to_closed(planNode* input)
        {
            std::tuple<int, int> temp_tuple = std::make_tuple(input->get_dim(0),input->get_dim(1));
            closed_list2D.insert(temp_tuple);
        }
};