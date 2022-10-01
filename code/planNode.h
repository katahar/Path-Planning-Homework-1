#include <stdio.h>
#include <vector>
#include <limits>
#include <tuple>


class planNode
{
    private:
        int f = 0;
        int g = 100000;
        int h = 0;
        int c = 0;
        bool flag_is_goal = false; 
        bool is_obstacle = false; 
        int collision_thresh = 0.0;
        planNode* prevPos; 
        int num_dims;
        bool is_start = false; 

        int x_coord = -17;
        int y_coord = -17;
        int t_coord = -17;
        std::tuple<int, int, int> tuple_3d;
        std::tuple<int, int> tuple_2d;

    public:

        planNode()
        {
            //nothing.
        }
        planNode(int x, int y, int t)
        {
            this -> x_coord = x;
            this -> y_coord = y;
            this -> t_coord = t;
            this -> num_dims = 3; 
            this -> tuple_3d = std::make_tuple(x_coord,y_coord,t_coord);
        }

        planNode(std::tuple<int,int,int> coord_tuple)
        {
            this -> num_dims = 3; 
            x_coord = std::get<0>(coord_tuple);
            y_coord = std::get<1>(coord_tuple);
            t_coord = std::get<2>(coord_tuple);
            this -> tuple_3d = std::make_tuple(x_coord,y_coord,t_coord);
        }

        planNode(int x, int y)
        {
            this -> x_coord = x;
            this -> y_coord = y;
            this -> num_dims = 2; 
            this -> tuple_2d = std::make_tuple(x,y);

        }

        

        std::tuple<int, int, int> get_tuple_3d()
        {
            return tuple_3d;
        }

        // planNode(std::vector<int> input_coords, double c, double collision_thresh)
        // {
        //     mexPrintf("~~~~~~Inside secondary planNode constructor\n");
        //     this -> coordinates.push_back(input_coords[0]);
        //     this -> coordinates.push_back(input_coords[1]);
        //     this -> coordinates.push_back(input_coords[2]);
        //     num_dims = coordinates.size();
        //     set_c(c);
        //     set_collision_thresh(collision_thresh);
        //     set_c(c, collision_thresh);
        // }      

        void update_f()
        {
            this -> f = this -> g + this -> h;
        }
        void set_g(int g_in)
        {
            this -> g = g_in;
            update_f();
        }
        void set_g_cumulative(int cumulative_cost) //assumes that cost for this node has already been assigned, will automatically add to given g
        {
            this -> g = cumulative_cost + c;
            update_f();
        }        
        void set_h(int h_in)
        {
            this -> h = h_in;
            update_f();
        }
        void set_prev(planNode* prev)
        {
            this -> prevPos = prev;
        }

        void set_c(int c_in)
        {
            this -> c = c_in;
            this -> set_g(c);
        }

        bool set_c(int c_in, int collision_thresh_in)
        {
            this -> c = c_in;
            this -> collision_thresh = collision_thresh_in;
            if(c_in >= collision_thresh)
            {
                this -> is_obstacle = true;
            }
            return this -> is_obstacle;
        }

        void set_collision_thresh(int thresh_in)
        {
            this -> collision_thresh = thresh_in;
        }

        void set_is_start()
        {
            this -> is_start = true;
        }

        void set_is_goal(bool input)
        {
            this -> flag_is_goal = input;
        }

        int get_g()
        {
            return this -> g;
        }
        int get_f()
        {
            return this -> f;
        }
        int get_h()
        {
            return this -> h;
        }
        int get_c()
        {
            return this -> c;
        }

        int get_dim(int axis)
        {
            switch(axis)
            {
                case 0: 
                    return  this -> x_coord;
                    break;
                case 1: 
                    return  this -> y_coord;
                    break;
                case 2: 
                    return  this -> t_coord;
                    break;
            }
        }
        bool get_is_start()
        {
            return this -> is_start; 
        }

        bool get_is_obstacle()
        {
            return this->is_obstacle;
        }
        
        planNode* get_prev_ptr()
        {
            return this -> prevPos;
        }
        planNode* get_ptr()
        {
            return this;
        }

        bool is_goal()
        {
            return this -> flag_is_goal;
        }

        friend bool operator<(const planNode& l, const planNode& r)
        {
            return l.f < r.f;
        }
        friend bool operator>(const planNode& l, const planNode& r)
        {
            return l.f > r.f;
        }
        friend bool operator==(const planNode& l, const planNode& r)
        {
            return (l.x_coord == r.x_coord) && (l.y_coord == r.y_coord) && (l.t_coord == r.t_coord);
        }
};