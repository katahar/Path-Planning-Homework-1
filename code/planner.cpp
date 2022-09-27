/*=================================================================
 *
 * planner.sc
 *
 *=================================================================*/
#ifndef _MATH
    #define _MATH
    #include <math.h>
#endif
#ifndef _MEX
    #define _MEX
    #include <mex.h>
#endif
#ifndef _KATAPLAN
    #define _KATAPLAN
    #include "kataPlanner.h"
#endif
#include <iostream>


/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))
#define GETMAPINDEXZEROED(X, Y, XSIZE, YSIZE) ((Y)*XSIZE + (X))


#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

bool first_run_complete = false; 
// kataPlanner3D planner_3d;
kataPlanner2D planner_2d;

int time_count = 0; 

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    //NOTE: Robot and target positions must be corrected due to difference in indexing
    int robotposeXcorrected = robotposeX - 1;
    int robotposeYcorrected = robotposeY - 1;
    int targetposeXcorrected = targetposeX - 1;
    int targetposeYcorrected = targetposeY - 1;

    if(!first_run_complete)
    {
        // mexPrintf("About to run constructor\n");

        // mexPrintf("x_size: %i \n", x_size);
        // mexPrintf("y_size: %i \n", y_size);
        // mexPrintf("target_steps: %i \n", target_steps);
        // mexPrintf("collision_thresh: %i \n", collision_thresh);
        // mexPrintf("robotposeXcorrected: %i \n", robotposeXcorrected);
        // mexPrintf("robotposeYcorrected: %i \n", robotposeYcorrected);
        
        

        // planner_3d = kataPlanner3D(map, x_size, y_size, target_steps, target_traj, collision_thresh, robotposeXcorrected, robotposeYcorrected);
        // mexPrintf("Constructotr done\n");

        planner_2d = kataPlanner2D(map, x_size, y_size, target_steps, target_traj, collision_thresh, robotposeXcorrected, robotposeYcorrected);
        mexPrintf("2d Constructotr done\n");

        planner_2d.generate_path();
        first_run_complete = true;
        mexPrintf("First run complete. \n");

    }
    else
    {
        // mexPrintf("in subsequent runs\n");
        // action_ptr[0] = planner_3d.get_x_dir(time_count);
        // action_ptr[1] = planner_3d.get_y_dir(time_count);
    }

    //===================================================
    
    // int int_val = 9;
    // double dub_val = 9.1;
    // mexPrintf("%i \n", int_val);
    // mexPrintf("%g \n", dub_val);

    // mexPrintf("Line Number %s->%s:%d\n", __FILE__, __FUNCTION__, __LINE__);

    // mexPrintf("%g \n", map[GETMAPINDEXZEROED(i,j,x_size,y_size)]);
    // mexPrintf("x_size: %i, y_size: %i\n", x_size, y_size); 



    //===================================================

    // // 8-connected grid
    // int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    // int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // // for now greedily move towards the final target position,
    // // but this is where you can put your planner

    // int goalposeX = (int) target_traj[target_steps-1];
    // int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // // printf("goal: %d %d;\n", goalposeX, goalposeY);

    // int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    // double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    // double disttotarget;
    // for(int dir = 0; dir < NUMOFDIRS; dir++)
    // {
    //     int newx = robotposeX + dX[dir];
    //     int newy = robotposeY + dY[dir];

    //     if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
    //     {
    //         if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
    //         {
    //             disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
    //             if(disttotarget < olddisttotarget)
    //             {
    //                 olddisttotarget = disttotarget;
    //                 bestX = dX[dir];
    //                 bestY = dY[dir];
    //             }
    //         }
    //     }
    // }
    // robotposeX = robotposeX + bestX;
    // robotposeY = robotposeY + bestY;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }

    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;
}