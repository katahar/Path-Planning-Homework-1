#ifndef _MEX
    #define _MEX
    #include <mex.h>
#endif
#ifndef _MATH
    #define _MATH
    #include <math.h>
#endif

#include <list>
#include <stdio.h>
#include <vector>
#include <limits>



class planNode
{
    private:
        int x = 0;
        int y = 0;
        int t = 0;
        double f = 0.0;
        double g = std::numeric_limits<int>::max();
        double h = 0.0;
        double c = 0.0;
        int prev_t;
        bool is_goal = false; 
        bool is_obstacle = false; 
        double collision_thresh = 0.0;
        planNode* prevPos; 

    public:
        planNode()
        {

        }

        planNode(int x, int y, int t, double c, double collision_thresh)
        {
            set_x(x);
            set_y(y);
            set_t(t);
            set_c(c);
            set_collision_thresh(collision_thresh);
            set_c(c, collision_thresh);
        }
        ~planNode()
        {
            delete prevPos;
            prevPos = nullptr;
        }


        void update_f()
        {
            this->f = this->g + this->h;
        }
        void set_g(double g_in)
        {
            this->g = g_in;
            this->update_f();
        }
        void set_g_cumulative(double cumulative_cost) //assumes that cost for this node has already been assigned, will automatically add to given g
        {
            this->g = cumulative_cost + this->c;
            this->update_f();
        }        
        void set_h(double h_in)
        {
            this->h = h_in;
            this->update_f();
        }
        void set_prev(planNode* prev)
        {
            this -> prevPos = prev;
        }
        
        void set_x(int x)
        {
            this -> x = x;
        }
        void set_y(int y)
        {
            this -> y = y;
        }
        void set_t(int t)
        {
            this -> t = t;
        }
        void set_coord(int x, int y, int t)
        {
            set_x(x);
            set_y(y);
            set_t(t);
        }
        void set_c(double c)
        {
            this -> c = c;
        }
        bool set_c(double c, double collision_thresh)
        {
            set_c(c);
            if(c >= collision_thresh)
            {
                is_obstacle = true;
            }
            return is_obstacle;
        }
        void set_collision_thresh(double thresh)
        {
            this->collision_thresh = collision_thresh;
        }
        void set_is_goal(bool is_goal)
        {
            this -> is_goal = is_goal;
        }
        void set_prev(planNode* previous_node)
        {
            prevPos = previous_node;
        }

        double get_g()
        {
            return this->g;
        }
        double get_f()
        {
            return this->f;
        }
        double get_h()
        {
            return this->h;
        }
        double get_c()
        {
            return this->c;
        }
        int get_x()
        {
            return this->x;
        }
        int get_y()
        {
            return this->y;
        }
        int get_t()
        {
            return this->t;
        }
        
        planNode* get_ptr()
        {
            return this;
        }
        

};

//==========================================================================================
class planMap
{
     private:
        std::vector<std::vector<std::vector<planNode*>>> map_vec_3d;
        double*	map;
        double* target_traj;
        int target_steps;
        int robotposeX;
        int robotposeY;
        int collision_thresh;
        int x_size;
        int y_size;

        
    public: 
        planMap();
        planMap(double* map_in, int x_size, int y_size, double* target_traj_in, int target_steps, int col_thresh, int robotposeX, int robotposeY)
        {
            resize_map(x_size,y_size,target_steps);
            this->target_steps = target_traj_in;
            this->map =  map_in;
            this->collision_thresh = col_thresh;
            this->robotposeX = robotposeX;
            this->robotposeY = robotposeY;

        } 

        ~planMap()
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

        void resize_map(int x_dim, int y_dim, int t_dim)
        {
            this->target_steps = t_dim;
            this->x_size = x_dim;
            this->y_size = y_dim;
            map_vec_3d.resize(x_dim,std::vector<std::vector<planNode*> >(y_dim,std::vector<planNode*>(t_dim)));
        }


        void update_cost(planNode* node, double cumulative_cost)
        {
            if(node->get_g != std::numeric_limits<int>::max()) //meaning that it has not been evaluated yet
            {
                if(node-> set_c(map[get_map_ind(node->get_x,node->get_y)],collision_thresh)); //true if obstacle
                {
                    // do nothing, because do not want to evaluate obstacles
                }
                else
                {

                    node->set_g_cumulative(cumulative_cost); //node adds cost to the provided cumulative cost. 
                    // node->set_h() //@TODO: add in heuristics.  
                }
            }
            else //the node has already been assessed
            {
                // check if obstacle first!! TODO
                if(node->get_g() > (cumulative_cost + node->get_c()))
                {
                    node->set_g(cumulative_cost + node->get_c());
                    // TODO: add update flag so original node is added to this for backtracing purposes
                }
            }
            
        }

        void evaluate_neighbor(planNode* current, planNode* neighbor)
        {
            neighbor->set_coord(current->get_x(),current->get_y(),current->get_t()+1);
            update_cost(neighbor,current->get_g());
        }

        void evaluate_all_neighbors(planNode* current)
        {

        }


    
   

 

};

//==========================================================================================
class kataPlanner
{
private:
    /* data */
public:
    kataPlanner(/* args */);
    ~kataPlanner();
};