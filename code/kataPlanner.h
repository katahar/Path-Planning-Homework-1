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
        std::unordered_set <planNode*> closed_list; 

        std::multimap<double,planNode*> open_list_f_sorted;
        std::map<int,planNode*> open_list_loc_sorted;

        
        int hash_base; 
        std::chrono::time_point<std::chrono::system_clock> startTime;
        int elapsed_time; 
        bool expanded_goal = false; 
        int last_goal_t_step = -1;
        std::vector<int> path; 
        planNode* last_found_goal;
        
        // 8-connected grid
        int dX[NUMDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1, 0};
        int dY[NUMDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1, 0};

        
    public: 
        kataPlanner()
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
        }

        kataPlanner(int x_size, int y_size, int target_steps, int col_thresh, int robotposeX, int robotposeY)
        {
            
            // mexPrintf("Function %s start\n", __FUNCTION__);
            // this -> target_steps = target_steps;
            // mexPrintf("Function %s start\n", __FUNCTION__);
            // this -> target_traj = target_traj_in;
            // mexPrintf("Function %s start\n", __FUNCTION__);
            // this -> map =  map_in;
            // mexPrintf("Function %s start\n", __FUNCTION__);
            // this -> collision_thresh = col_thresh;
            // mexPrintf("Function %s start\n", __FUNCTION__);
            // this -> robotposeX = robotposeX;
            // mexPrintf("Function %s start\n", __FUNCTION__);
            // this -> robotposeY = robotposeY;
            // mexPrintf("Function %s start\n", __FUNCTION__);
            // this -> x_size = x_size;
            // mexPrintf("Function %s start\n", __FUNCTION__);
            // this -> y_size = y_size; 
            // mexPrintf("Pre hash base");
            // this -> find_hash_base();
            // mexPrintf("Post Hash base");
            // start_timer();
            // mexPrintf("Function %s end\n", __FUNCTION__);

        } 

        int find_hash_base()
        {       
            mexPrintf("Function %s start\n", __FUNCTION__);

            int temp = std::max(x_size,y_size);
            hash_base = 1; 
             while(temp != 0) 
            {
                temp = temp / 10;
                hash_base*=10;
            }
            hash_base*=10;
        }

        int get_hash_key(planNode* input)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);

            std::vector<int> coordinates = input -> get_coords();
            int key; 
            for(int i = 0; i < coordinates.size(); i++)
            {
                key = key + (coordinates[i] * pow(hash_base,i));
            }
            return key;
        }

        // ~kataPlanner()
        // {
        //     mexPrintf("Function %s start\n", __FUNCTION__);

        //     if(map != nullptr)
        //     {
        //         delete [] map;
        //         map = nullptr;
        //     }
        //     // if(target_traj != nullptr)
        //     // {
        //     //     delete [] target_traj;
        //     //     target_traj = nullptr;
        //     // }
        //     if(last_found_goal != nullptr)
        //     {
        //         delete last_found_goal;
        //         last_found_goal = nullptr;
        //     }
        // }
        
        int get_map_ind(int x_ind, int y_ind)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            if(x_ind >=0 && y_ind >= 0)
            {
                int temp_return = y_ind*x_size + x_ind;
                if(!(temp_return > x_size*y_size))
                {
                    return temp_return;
                }
                
            }
             return 0;
        }

        bool in_closed(planNode* input)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            if(closed_list.find(input) != closed_list.end())
            {
                return true;
            }
            return false; 
        }

        bool in_open(planNode* input)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            if(open_list_loc_sorted.find(get_hash_key(input)) != open_list_loc_sorted.end())
            {
                return true;
            }
            return false;
        }

        void add_to_open(planNode* input)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            // this -> open_list.push_back(input);   
            // this -> open_list_checker.insert(std::pair<int,int>(this->get_hash_key(input),input));
            open_list_loc_sorted.insert(std::pair<int,planNode*>(this->get_hash_key(input),input));
            open_list_f_sorted.insert(std::pair<double,planNode*>(input->get_f(),input));
        }

        // planNode* get_from_open()
        // {
        //     planNode* ret_val = open_list.top();
        //     open_list.pop();
        //     // open_list_checker.erase(get_hash_key(ret_val));
        //     return ret_val;  
        // }

        planNode* get_from_open(planNode* input) //used to remove a specific node. Unsure of the final use of this function. 
        {           
            mexPrintf("Function %s start\n", __FUNCTION__);
            planNode* ret_val;
            double f_lookup = input->get_f();
            std::multimap<double,planNode*>::iterator temp = open_list_f_sorted.find(f_lookup);
            if(temp != open_list_f_sorted.end())
            {
                for(std::multimap<double, planNode*>::iterator itr = temp; itr->first == f_lookup; itr++)
                {
                    if(itr->second == input)
                    {
                        // std::cout << "Found the search iterator!" << std::endl;
                        ret_val = input; 
                        open_list_f_sorted.erase(itr);
                        open_list_loc_sorted.erase(get_hash_key(input));
                        return ret_val;       
                    }
                }
            }
            else
            {
                std::cout << "Value not found :(  " << std::endl;
                return nullptr;  
            }
        }

        planNode* get_next_from_open() //pulls the node with the next best f value. 
        {           
            mexPrintf("Function %s start\n", __FUNCTION__);
            if(!open_list_f_sorted.empty())
            {
                std::multimap<double,planNode*>::iterator first_itr = open_list_f_sorted.begin();
                planNode* ret_val = first_itr->second;
                open_list_f_sorted.erase(first_itr);
                open_list_loc_sorted.erase(get_hash_key(ret_val));
                return ret_val;
            }
            std::cout << "open list is empty!" << std::endl;
            return nullptr;
        }

        bool open_is_empty()
        {
             mexPrintf("Function %s start\n", __FUNCTION__);
           return open_list_loc_sorted.empty();
        }

        void add_to_closed(planNode* input)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            closed_list.insert(input);
        }

        std::vector<int> get_coords_from_rel(std::vector<int> base, std::vector<int> rel_direction)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            std::vector<int> ret_val; 
            for(int i = 0; i < base.size(); ++i)
            {
                ret_val.push_back(base[i] + rel_direction[i]);
            }
            return ret_val;
        }
        
        void mark_expanded(planNode* goal_input)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);

            last_goal_t_step = goal_input->get_dim(2);
            last_found_goal = goal_input;
            expanded_goal = true;
        }

        void set_costs(planNode* neighbor, double cumulative_cost)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            if(!(neighbor-> set_c(map[get_map_ind(neighbor->get_dim(0),neighbor->get_dim(1))], collision_thresh))); //true if obstacle
            {
                neighbor->set_g_cumulative(cumulative_cost); //node adds cost to the provided cumulative cost. 
                // neighbor->set_h() //@TODO: add in heuristics.  
            }
        }

        bool update_cost_existing(planNode* input, double cumulative_cost)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            //evaluate if this is a lower cost. 
            std::map<int, planNode*>::iterator iter = open_list_loc_sorted.find(get_hash_key(input));
            double current_g = iter->second->get_g();

            double new_g = cumulative_cost + iter->second->get_c(); //because C and H shouldn't change based on path
            if(current_g < new_g ) //already optimal
            {
                //do nothing. Dump the input. 
                delete input;
                input = nullptr;
                return false;
            }
            else //new calculated value is improved. Need to update. 
            {
                //removing from f_sorted open list
                double cur_f = iter->second->get_f();
                std::multimap<double,planNode*>::iterator del_iter = open_list_f_sorted.find(cur_f);
                for(std::multimap<double, planNode*>::iterator for_itr = del_iter; for_itr->first == cur_f; for_itr++)
                {
                    if(for_itr->second == input)
                    {
                        open_list_f_sorted.erase(for_itr); 
                    }
                }
                //----------

                //setting the new g value
                input->set_g_cumulative(cumulative_cost);

                //reinserting into f_sorted open list
                open_list_f_sorted.insert(std::pair<double,planNode*>(input->get_f(),input)); 
                return true;
            }
        }

        bool is_goal(planNode* input) //introducing offset here. 
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            int elapsed = cumulative_time();
            if(target_traj[2*(input->get_dim(2) + elapsed)] ==  input->get_dim(0) &&
               target_traj[2*(input->get_dim(2) + elapsed)+1] ==  input->get_dim(1) )
               {
                    input->set_is_goal(true);
                    return true;
               }
            return false;
        }

        void start_timer()
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            startTime = std::chrono::system_clock::now();
        }

        int cumulative_time()
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            std::chrono::time_point<std::chrono::system_clock> curTime;
            return std::chrono::duration_cast<std::chrono::seconds>(curTime - startTime).count();
        }

        bool goal_not_expanded()
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            return !expanded_goal;
        }

        bool valid_coords(std::vector<int> input)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            if(input[0] > -1 && input[1] > -1 && input[0] < x_size && input[1] < y_size)
            {
                return true; 
            }
            return false; 
        }

        void evaluate_neighbor(planNode* current, std::vector<int> rel_direction)
        {
             mexPrintf("Function %s start\n", __FUNCTION__);
           std::vector<int> new_coords = get_coords_from_rel(current->get_coords(), rel_direction);
            if(valid_coords(new_coords))
            {
                planNode* temp_node = planNode(new_coords).get_ptr();
                if(!in_closed(temp_node)) //verify that the node is not in the closed list
                {
                    if(!in_open(temp_node)) //not already in open, can be added as a new planNode
                    {
                        set_costs(temp_node,current->get_g());
                        temp_node -> set_prev(current);
                        add_to_open(temp_node);
                    }
                    else //meaning that it is already in open list, with a different g value.
                    {
                        temp_node -> set_prev(current);
                        update_cost_existing(temp_node,current->get_g());
                    }
                }

                if(current->is_goal())
                {
                    mark_expanded(current); //could probably just check the closed list for goals.
                    //Verify that goals are actually populated. !!!!!
                }
            }

        }

};


class kataPlanner3D : public kataPlanner
{
    public:
        #define axes 3 
        double*	heuristic_map;


        kataPlanner3D()
        {
            mexPrintf("AAAAAA");
             mexPrintf("Function %s start\n", __FUNCTION__);
           //do nothing.
        }
        
        //double* map_in double* target_traj_in,
         kataPlanner3D(int x_size, int y_size, int target_steps, int col_thresh, int robotposeX, int robotposeY)
        :kataPlanner(x_size, y_size, target_steps, col_thresh, robotposeX, robotposeY)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            //nothing different.
        }

        
        void evaluate_neighbors(planNode* current)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            for(int i = 0; i < NUMDIRS; ++i)
            {
                std::vector<int> relative_dim;
                relative_dim.push_back(dX[i]);
                relative_dim.push_back(dY[i]);
                relative_dim.push_back(current->get_dim(2) + 1);
                evaluate_neighbor(current,relative_dim);
            }
             
        }

        void generate_heuristic()
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            //@TODO: Fill in
            std::cout << "Running heuristic. " << std::endl;

        }

        void generate_path()
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            std::vector<int> start_coords; 
            start_coords.push_back(robotposeX);
            start_coords.push_back(robotposeY);
            start_coords.push_back(0);
            planNode* start = planNode(start_coords).get_ptr();
            add_to_open(start);

            while(!open_is_empty() && goal_not_expanded())  //at this point, the first goal has been reached. But need to validate that it is at the right time
            {
                planNode* current = get_next_from_open();
                evaluate_neighbors(current);
                add_to_closed(current);
            }
            populate_path(last_found_goal);
        }

        void populate_path(planNode* goal)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            planNode* current = goal; 
            planNode* prev = goal->get_prev_ptr();

            while(!(current->get_is_start()))
            {
                // path.push_back(current->get_dim(0) - prev->get_dim(0));
                // path.push_back(current->get_dim(1) - prev->get_dim(1));
                path.insert(path.begin(), (current->get_dim(1) - prev->get_dim(1))); //puts at the beginning. 
                path.insert(path.begin(), (current->get_dim(0) - prev->get_dim(0)));
                current = prev;
                prev = current->get_prev_ptr();
            } 
        }

        int get_x_dir(int t_step)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            return path[2*t_step];
        }

        int get_y_dir(int t_step)
        {
            mexPrintf("Function %s start\n", __FUNCTION__);
            return path[(2*t_step) + 1];
        }

};
