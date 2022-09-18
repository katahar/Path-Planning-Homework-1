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
    public:
        int x = 0;
        int y = 0;
        int t = 0;
        double f = 0.0;
        double g = std::numeric_limits<int>::max();
        double h = 0.0;
        double c = 0.0;
        int prev_x; 
        int prev_y;
        int prev_t;
        bool is_goal = false; 
        bool is_obstacle = false; 
        double collision_thresh = 0.0;

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

        void update_f()
        {
            this->f = this->g + this->h;
        }
        void set_g(double g_in)
        {
            this->g = g_in;
            this->update_f();
        }
        void set_h(double h_in)
        {
            this->h = h_in;
            this->update_f();
        }
        void set_prev(int x_in, int y_in, int t_in)
        {
            this -> prev_t = t_in;
            this -> prev_x = x_in;
            this -> prev_y = y_in;
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
        void set_c(double c, double collision_thresh)
        {
            set_c(c);
            if(c >= collision_thresh)
            {
                is_obstacle = true;
            }
        }
        void set_collision_thresh(double thresh)
        {
            this->collision_thresh = collision_thresh;
        }
        void set_is_goal(bool is_goal)
        {
            this -> is_goal = is_goal;
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
        

};

//==========================================================================================
class planMap
{
    public: 
        
    
    private:
        std::vector<std::vector<std::vector<planNode>>> map_vec;

        planMap(); 
 

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