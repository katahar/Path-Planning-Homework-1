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
        // std::priority_queue<planNode*, std::vector<planNode*>, std::greater<std::vector<planNode*>::value_type> > open_list; 
        
        struct compareFvals{
            bool operator()(planNode* const& left, planNode* const& right)
            {
                return left->get_f() > right->get_f();
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
            last_x = target_traj[target_steps-1];
            last_y = target_traj[target_steps-1 + target_steps];
 
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
            if(!(neighbor-> set_c(map[get_map_ind(neighbor->get_dim(0), neighbor->get_dim(1))], collision_thresh))); //set_c returns true if obstacle
            {
                neighbor->set_g_cumulative(cumulative_cost); //node adds cost to the provided cumulative cost. 
                neighbor->set_h(1*end_heuristic(neighbor)); //@TODO: add in heuristics.  
            }
        }

        int end_heuristic(planNode* input)
        {
            return int(std::sqrt( std::pow(input->get_dim(0)-get_last_x(),2) + std::pow(input->get_dim(1)-get_last_y(),2)));

            // return std::abs(input->get_dim(1)-get_last_y());
            return 0;
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
        std::unordered_map<std::tuple<int, int, int>,double,tuple_hash_function> open_g_track;


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

        void add_to_open(planNode* input)
        {
            std::tuple<int,int, int> new_key= std::make_tuple(input->get_dim(0), input->get_dim(1), input->get_dim(2));
            open_g_track.erase(new_key);
            open_g_track.insert(std::make_pair(new_key,input->get_g()));
            open_list.push(input);
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
        std::unordered_map<std::tuple<int, int>,double,tuple_hash_function2D> open_g_track;

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
            if(get_last_x() ==  input->get_dim(0) &&
               get_last_y() ==  input->get_dim(1) )
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

                if(cur_is_goal(current)) 
                {
                    mark_expanded(current); 
                    return;
                }
            }
            // print_open();
        }

        void generate_heuristic()
        {

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
            // open_g_track.erase(new_key);
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
            open_list.push(input);
        }
        
        void evaluate_neighbor(planNode* current, int rel_x, int rel_y)
        {
            if(!in_closed(std::make_tuple(current->get_dim(0)+rel_x,current->get_dim(1)+rel_y))) //verify that the neighbor is not already in the closed list
            {
                planNode* neighbor = new planNode(current->get_dim(0)+rel_x, current->get_dim(1)+rel_y);
                // mexPrintf("The cumulative g is %d\n", current->get_g());
                set_costs(neighbor, current->get_g());
                // mexPrintf("The new g with cumulative factored in is %d\n", neighbor->get_g());
                if(improved_g(neighbor))//returns true if neighbor has a better g value than the one currently in the open list. 
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
            mexPrintf("Goal found! (%d, %d)\n", goal_input->get_dim(0), goal_input->get_dim(1));
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
            // mexPrintf("Populating path."); // x: %d, y: %d\n", goal->get_dim(0),goal->get_dim(1));
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
            path.insert(path.begin(), current -> get_dim(1)); 
            path.insert(path.begin(), current -> get_dim(0));

            // mexPrintf("Population complete \n");

            // mexPrintf("\nFinal target location\nx: %d y: %d\n\n", get_last_x(),get_last_y());
            // for(int i = 0; i < path.size(); i = i+2)
            // {
            //     mexPrintf("x: %d, y: %d \n", path[i], path[i+1]);
            // }
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

        void add_to_closed(planNode* input)
        {
            std::tuple<int, int> temp_tuple = std::make_tuple(input->get_dim(0),input->get_dim(1));
            closed_list2D.insert(temp_tuple);
        }
};