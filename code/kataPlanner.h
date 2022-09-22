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

        
        // int hash_base; 
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
        kataPlanner();
        kataPlanner(double* map_in, int x_size, int y_size, double* target_traj_in, int target_steps, int col_thresh, int robotposeX, int robotposeY)
        {
            this -> target_steps = target_steps;
            this -> target_traj = target_traj_in;
            this -> map =  map_in;
            this -> collision_thresh = col_thresh;
            this -> robotposeX = robotposeX;
            this -> robotposeY = robotposeY;
            this -> x_size = x_size;
            this -> y_size = y_size; 
            this -> find_hash_base()
            start_timer()
        } 

        int find_hash_base()
        {       
            int temp = std::max(x_size,y_size);
            int hash_base = 1; 
             while(temp != 0) 
            {
                temp = temp / 10;
                hash_base*=10;
            }
            hash_base*=10;
        }

        int get_hash_key(planNode* input)
        {
            std::vector<int> coordinates = input -> get_coords();
            int key; 
            for(int i = 0; i < coordinates.size(); i++)
            {
                key = key + (coordinates[i] * pow(hash_base,i));
            }
            return key;
        }

        ~kataPlanner()
        {
            if(map != nullptr)
            {
                delete [] map;
                map = nullptr;
            }
            if(target_traj != nullptr)
            {
                delete [] target_traj;
                target_traj = nullptr;
            }
            if(last_found_goal != nullptr)
            {
                delete last_found_goal;
                last_found_goal = nullptr;
            }
        }
        
        int get_map_ind(int x_ind, int y_ind)
        {
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
            if(closed_list.find(input) != closed_list.end())
            {
                return true;
            }
            return false; 
        }

        bool in_open(planNode* input)
        {
            if(open_list_loc_sorted.find(get_hash_key(input)) != open_list_checker.end())
            {
                return true;
            }
            return false;
        }

        void add_to_open(planNode* input)
        {
            // this -> open_list.push_back(input);   
            // this -> open_list_checker.insert(std::pair<int,int>(this->get_hash_key(input),input));
            open_list_loc_sorted.insert(std::pair<int,planNode*>(this->get_hash_key(input),input));
            open_list_f_sorted.insert(std::pair<int,planNode*>(input->get_f(),input));
        }

        // planNode* get_from_open()
        // {
        //     planNode* ret_val = open_list.top();
        //     open_list.pop();
        //     // open_list_checker.erase(get_hash_key(ret_val));
        //     return ret_val;  
        // }

        planNode* get_from_open(planNode* input)
        {           
            double f_lookup = input->get_f();
            open_list_f_sorted.
            
            open_list_checker.erase(get_hash_key(input));

            return ret_val;  
        }


        bool open_is_empty()
        {
            return open_list.empty();
        }

        void add_to_closed(planNode* input)
        {
            closed_list.insert(input);
        }

        std::vector<int> get_coords_from_rel(std::vector<int> base, std::vector<int> rel_direction)
        {
            std::vector<int> ret_val; 
            for(int i = 0; i < base.size(); ++i)
            {
                ret_val.push_back(base[i] + rel_direction[i]);
            }
            return ret_val;
        }
        
        void mark_expanded(planNode* goal_input)
        {
            last_goal_t_step = goal_input->get_dim(2);
            last_found_goal = goal_input;
            expanded_goal = true;
        }

        void update_cost(planNode* neighbor, double cumulative_cost)
        {
            if(!(neighbor-> set_c(map[get_map_ind(neighbor->get_x(),node->get_y())], collision_thresh))); //true if obstacle
            {
                neighbor->set_g_cumulative(cumulative_cost); //node adds cost to the provided cumulative cost. 
                // neighbor->set_h() //@TODO: add in heuristics.  
            }
        }

        bool is_goal(planNode* input) //introducing offset here. 
        {
            int elapsed = cumulative_time()
            if(target_traj[2*(input->get_dim(2) + elapsed)] ==  input->get_dim(0) &&
               target_traj[2*(input->get_dim(2) + elapsed)+1] ==  input->get_dim(1) )
            return input->is_goal();
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

        void evaluate_neighbor(planNode* current, std::vector<int> rel_direction)
        {
            planNode* temp_node = planNode(get_coords_from_rel(current->get_coords(), rel_direction));
            if(!in_closed(temp_node)) //verify that the node is not in the closed list
            {
                if(!in_open(temp_node))
                {
                    update_cost(temp_node,current->get_g());
                    temp_node -> set_prev(current);
                    add_to_open(temp_node);
                }
                else //meaning that it is already in open list, with a different g value.
                {
                    delete temp_node;
                    temp_node = 
                }
            }

            if(current->is_goal())
            {
                mark_expanded(current); //could probably just check the closed list for goals.
                //Verify that goals are actually populated. !!!!!
            }
        }




};


class kataPlanner3D : public kataPlanner
{
    public:
        #define axes 3 
        double*	heuristic_map;


        // kataPlanner3D();
        
        void evaluate_neighbors(planNode* current)
        {
            for(int i = 0; i < NUMDIRS; ++i)
            {
                evaluate_neighbor(current, std::vector<int> relative_dim{ dX[i], dY[i], current->get_dim[2] + 1 });
            }
             
        }

        void generate_heuristic()
        {
            //@TODO: Fill in
            std::cout << "Running heuristic. " << std::endl;

        }


        void generate_path()
        {
            planNode* start = planNode(std::vector<int> coords{robotposeX,robotposeY,0});
            add_to_open(start);

            while(!open_is_empty() && goal_not_expanded())  //at this point, the first goal has been reached. But need to validate that it is at the right time
            {
                planNode* current = get_from_open();
                evaluate_neighbors(current);
                add_to_closed(current);
            }
            populate_path(last_found_goal);
        }

        void populate_path(planNode* goal)
        {
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
            return path[2*t_step];
        }

        int get_y_dir(int t_step)
        {
            return path[(2*t_step) + 1];
        }


};
