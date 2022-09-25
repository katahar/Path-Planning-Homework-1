#include <stdio.h>
#include <vector>
#include <limits>

class planNode
{
    private:
        std::vector<int> coordinates;
        double f = std::numeric_limits<double>::max();
        double g = std::numeric_limits<double>::max();
        double h = 0.0;
        double c = 0.0;
        // int prev_t;
        bool flag_is_goal = false; 
        bool is_obstacle = false; 
        double collision_thresh = 0.0;
        planNode* prevPos; 
        int num_dims;
        bool is_start = false; 

    public:
        planNode(std::vector<int> input_coords)
        {
            mexPrintf("~~~~~~Inside planNode constructor\n");
            mexPrintf("~X coordinate: %i \n", input_coords[0]);
            mexPrintf("~y coordinate: %i \n", input_coords[1]);
            mexPrintf("~t coordinate: %i \n", input_coords[2]);
            this -> coordinates.push_back(input_coords[0]);
            this -> coordinates.push_back(input_coords[1]);
            this -> coordinates.push_back(input_coords[2]);
            this -> num_dims = this->coordinates.size();
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

        double round(double input)
        {
            return double(int(input*100)/100);
        }

        void update_f()
        {
            this->f = this->g + this->h;
        }
        void set_g(double g_in)
        {
            this->g = round(g_in);
            this->update_f();
        }
        void set_g_cumulative(double cumulative_cost) //assumes that cost for this node has already been assigned, will automatically add to given g
        {
            this->g = round(cumulative_cost) + this->c;
            this->update_f();
        }        
        void set_h(double h_in)
        {
            this->h = round(h_in);
            this->update_f();
        }
        void set_prev(planNode* prev)
        {
            this -> prevPos = prev;
        }
        
        // void set_coord(std::vector<int> coordinate_inputs, int num_dimensions)
        // {
        //     for(int i = 0; i < num_dimensions; ++i)
        //     {
        //         this->coordinates[i] = coordinate_inputs[i];
        //     }
        // }

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
        void set_is_goal(bool input)
        {
            this -> flag_is_goal = input;
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
        int get_dim(int axis)
        {
            return this->coordinates[axis];
        }
        bool get_is_start()
        {
            return is_start; 
        }
        

        std::vector<int> get_coords()
        {
            mexPrintf("--ABout to return coords\n");
            mexPrintf("--X coordinate: %i \n", coordinates[0]);
            mexPrintf("--y coordinate: %i \n", coordinates[1]);
            mexPrintf("--t coordinate: %i \n", coordinates[2]);
            return this->coordinates;
        }     

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
            return l.coordinates == r.coordinates;
        }
};