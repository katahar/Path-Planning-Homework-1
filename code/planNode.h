#include <stdio.h>
#include <vector>
#include <limits>

class planNode
{
    private:
        // std::vector<int> coordinates;
        int f = std::numeric_limits<double>::max();
        int g = std::numeric_limits<double>::max();
        int h = 0.0;
        int c = 0.0;
        // int prev_t;
        bool flag_is_goal = false; 
        bool is_obstacle = false; 
        double collision_thresh = 0.0;
        planNode* prevPos; 
        int num_dims;
        bool is_start = false; 

        int x_coord = -17;
        int y_coord = -17;
        int t_coord = -17;

    public:
        // planNode(std::vector<int> input_coords)
        // {
        //     mexPrintf("~~~~~~Inside planNode constructor\n");
        //     mexPrintf("~X coordinate: %d \n", input_coords[0]);
        //     mexPrintf("~y coordinate: %d \n", input_coords[1]);
        //     mexPrintf("~t coordinate: %d \n", input_coords[2]);
        //     this -> coordinates.push_back(input_coords[0]);
        //     this -> coordinates.push_back(input_coords[1]);
        //     this -> coordinates.push_back(input_coords[2]);
        //     this -> num_dims = this->coordinates.size();
        // }

        planNode(int x, int y, int t)
        {

            // this -> coordinates.push_back(x);
            // this -> coordinates.push_back(y);
            // this -> coordinates.push_back(t);
            // this -> num_dims = this->coordinates.size();

            // this -> x_coord = x;
            // this -> y_coord = y;
            // this -> t_coord = t;
            // this -> num_dims = 3; 
            x_coord = x;
            y_coord = y;
            t_coord = t;
            // mexPrintf("~~~~~~Inside planNode constructor 3D\n");
            // mexPrintf("~X coordinate: %d \n", x_coord);
            // mexPrintf("~y coordinate: %d \n", y_coord);
            mexPrintf("~t coordinate: %d \n", t_coord);

            num_dims = 3; 
        }

        planNode(int x, int y)
        {
            mexPrintf("~~~~~~Inside planNode constructor 2D\n");
            mexPrintf("~X coordinate: %d \n", x);
            mexPrintf("~y coordinate: %d \n", y);
            // this -> coordinates.push_back(x);
            // this -> coordinates.push_back(y);
            // this -> num_dims = this->coordinates.size();
            
            // this -> x_coord = x;
            // this -> y_coord = y;
            // this -> num_dims = 2;

            x_coord = x;
            y_coord = y;
            num_dims = 2; 
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

        // double round(double input)
        // {
        //     return double(int(input*100)/100);
        // }

        void update_f()
        {
            // this->f = this->g + this->h;
            f = g + h;
        }
        void set_g(int g_in)
        {
            // this->g = round(g_in);
            // this->update_f();
            g = g_in;
            update_f();
        }
        void set_g_cumulative(int cumulative_cost) //assumes that cost for this node has already been assigned, will automatically add to given g
        {
            // this->g = round(cumulative_cost) + this->c;
            // this->update_f();
            g = cumulative_cost + c;
            update_f();
        }        
        void set_h(int h_in)
        {
            // this->h = round(h_in);
            // this->update_f();
            h = h_in;
            update_f();
        }
        void set_prev(planNode* prev)
        {
            // this -> prevPos = prev;
            prevPos = prev;
        }
        
        // void set_coord(std::vector<int> coordinate_inputs, int num_dimensions)
        // {
        //     for(int i = 0; i < num_dimensions; ++i)
        //     {
        //         this->coordinates[i] = coordinate_inputs[i];
        //     }
        // }

        void set_c(int c_in)
        {
            // this -> c = c_in;
            c = c_in;
        }
        bool set_c(int c_in, int collision_thresh)
        {
            set_c(c_in);
            if(c_in >= collision_thresh)
            {
                is_obstacle = true;
            }
            return is_obstacle;
        }

        void set_collision_thresh(int thresh_in)
        {
            // this->collision_thresh = thresh_in;
            collision_thresh = thresh_in;
        }

        void set_is_goal(bool input)
        {
            // this -> flag_is_goal = input;
            flag_is_goal = input;
        }

        int get_g()
        {
            // return this->g;
            return g;
        }
        int get_f()
        {
            // return this->f;
            return f;
        }
        int get_h()
        {
            // return this->h;
            return h;
        }
        int get_c()
        {
            // return this->c;
            return c;
        }
        int get_dim(int axis)
        {
            // return this->coordinates[axis];
            // switch(axis)
            // {
            //     case 0: 
            //         return this -> x_coord;
            //         break;
            //     case 1: 
            //         return this -> y_coord;
            //         break;
            //     case 2: 
            //         return this -> t_coord;
            //         break;
            // }
            switch(axis)
            {
                case 0: 
                    return  x_coord;
                    break;
                case 1: 
                    return  y_coord;
                    break;
                case 2: 
                    return  t_coord;
                    break;
            }
        }
        bool get_is_start()
        {
            return is_start; 
        }
        

        // std::vector<int> get_coords()
        // {
        //     mexPrintf("--ABout to return coords\n");
        //     mexPrintf("--X coordinate: %d \n", coordinates[0]);
        //     mexPrintf("--y coordinate: %d \n", coordinates[1]);
        //     mexPrintf("--t coordinate: %d \n", coordinates[2]);
        //     return this->coordinates;
        // }     

        planNode* get_prev_ptr()
        {
            return prevPos;
        }
        planNode* get_ptr()
        {
            return this;
        }

        bool is_goal()
        {
            return flag_is_goal;
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
            // return l.coordinates == r.coordinates;
            return (l.x_coord == r.x_coord) && (l.y_coord == r.y_coord) && (l.t_coord == r.t_coord);
        }
};