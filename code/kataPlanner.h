#include <list>
#include <stdio.h>

class planNode
{
    public:
        int x = 0;
        int y = 0;
        int t = 0;
        double f = 0.0;
        double g = 0.0;
        double h = 0.0;
        double c = 0.0;
        int prev_x; 
        int prev_y;
        bool is_goal = false; 
        bool is_obstacle = false; 
        double collision_thresh = 0.0;

        planNode(int x, int y, int t, double c, double collision_thresh)
        {
            this->x = x;
            this->y = y;
            this->t = t;
            this->c = c;
            this->collision_thresh = collision_thresh;
            if(c >= collision_thresh)
            {
                is_obstacle = true;
            }
        }
};


class planMap
{
    public: 
        planNode* map; 

        // planMap(int x_size, int y_size, int t_size)
        // {
        //     map = new planNode[x_size][y_size][t_size]
        // }
};


class kataPlanner
{
private:
    /* data */
public:
    kataPlanner(/* args */);
    ~kataPlanner();
};

kataPlanner::kataPlanner(/* args */)
{
}

kataPlanner::~kataPlanner()
{
}
