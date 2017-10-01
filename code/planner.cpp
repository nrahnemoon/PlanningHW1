/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "planner.h"
#include "Node.h"
#include "WorldInfo.h"
#include <queue>          // std::priority_queue

using namespace std;

int roundNumber = 0;

bool applyaction(int *map, int x_size, int y_size, float robotposeX, float robotposeY, float robotposeTheta,
                 float *newx, float *newy, float *newtheta, PrimArray mprim, int dir, int prim)
{
    int i;
    for (i = 0; i < NUMOFINTERSTATES; i++) {
        *newx = robotposeX + mprim[dir][prim][i][0];
        *newy = robotposeY + mprim[dir][prim][i][1];
        *newtheta = mprim[dir][prim][i][2];
        
        int gridposx = (int)(*newx / RES + 0.5);
        int gridposy = (int)(*newy / RES + 0.5);

        /* check validity */
        if (gridposx < 1 || gridposx > x_size || gridposy < 1 || gridposy > y_size){
            return false;
        }

        if ((int)map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0){
            return false;
        }
    }

    return true;
}

int getPrimitiveDirectionforRobotPose(float angle)
{
    /* returns the direction index with respect to the PrimArray */
    /* normalize bw 0 to 2pi */
    angle = fmod(angle, (2 * M_PI));
    if (angle < 0.0) {
        angle += 2 * M_PI;
    }
    int dir = (int)(angle / (2 * M_PI / NUMOFDIRS) + 0.5);
    if (dir == 8) {
        dir = 0;
    }
    return dir;
}

static void planner(
    bool* map,
    int x_size,
    int y_size,
    float robotposeX,
    float robotposeY,
    float robotposeTheta,
    float goalposeX,
    float goalposeY,
    PrimArray mprim,
    int *prim_id)
{
    printf("Round Number = %d\n", roundNumber);
    roundNumber = roundNumber + 1;
    
    WorldInfo worldInfo = WorldInfo(robotposeX, robotposeY, goalposeX, goalposeY,
            (x_size * RES), (y_size * RES), map, ((PrimArrayPtr) mprim));
    Node startNode = Node(robotposeX, robotposeX, robotposeTheta, -1, &worldInfo, NULL);

    priority_queue<Node*> priorityQueue;
    priorityQueue.push(&startNode);

    Node* currNode;

    while (true) {

        currNode = priorityQueue.top();
        priorityQueue.pop();
        
        if (currNode->isGoal())
            break;

        currNode->discoverNeighbors();

        for (int i = 0; i < currNode->getNumNeighbors(); i++) {
            Node* neighbor = currNode->getNeighbor(i);
            priorityQueue.push(neighbor);
        }

        currNode->close();
    }
    
    // currNode is the goal at this point
    while(currNode->getParent()->getParent() != NULL) {
        currNode = currNode->getParent();
    }
    // currNode is the node immediately after the start node at this point
    *prim_id = currNode->getIncomingPrimitive();

    printf("action %d\n", *prim_id);
    return;

    /*printf("robot: %d %d; ", robotposeX, robotposeY);*/
    /*printf("goal: %d %d;", goalposeX, goalposeY);*/
    
	/*for now greedily move towards the target, */
	/*but this is where you can put your planner */
	/*double mindisttotarget = 1000000;
    int dir;
    int prim;
	dir = getPrimitiveDirectionforRobotPose(robotposeTheta);
    
    for (prim = 0; prim < NUMOFPRIMS; prim++) {
        float newx, newy, newtheta;
        bool ret;
        ret = applyaction(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, &newx, &newy, &newtheta, mprim, dir, prim); */
            /* skip action that leads to collision */
        /* if (ret) {
            double disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
            if(disttotarget < mindisttotarget){
                mindisttotarget = disttotarget;
                
                *prim_id = prim;
            }            
        }

    }
    printf("action %d\n", *prim_id);
    return; */
}

/*prhs contains input parameters (3): 
1st is matrix with all the obstacles
2nd is a row vector <x,y> for the robot pose
3rd is a row vector <x,y> for the target pose
plhs should contain output parameters (1): 
1st is a row vector <dx,dy> which corresponds to the action that the robot should make*/

void parseMotionPrimitives(PrimArray mprim)
{
    FILE * fp;
    fp = fopen ("unicycle_8angles.mprim", "r+");
    char skip_c[100];
    int skip_f;
    float resolution;
    int num_angles;
    int num_mprims;
    fscanf(fp, "%s %f", skip_c, &resolution);
    fscanf(fp, "%s %d", skip_c, &num_angles);
    fscanf(fp, "%s %d", skip_c, &num_mprims);

    int i, j, k;
    for (i = 0; i < NUMOFDIRS; ++i) {
        for (j = 0; j < NUMOFPRIMS; ++j) {
            fscanf(fp, "%s %d", skip_c, &skip_f);
            for (k = 0; k < NUMOFINTERSTATES; ++k) {
                fscanf(fp, "%f %f %f", &mprim[i][j][k][0], &mprim[i][j][k][1], &mprim[i][j][k][2]);
            }

        }
    }
}

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )
     
{

    /* Read motion primtives */
    PrimArray motion_primitives;
    parseMotionPrimitives(motion_primitives);

    /* Check for proper number of arguments */    
    if (nrhs != 3) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
    } else if (nlhs != 1) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    bool* map = ((bool*)((int*)mxGetPr(MAP_IN)));
    
    /* get the dimensions of the robotpose and the robotpose itself*/     
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 3.");         
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    float robotposeX = (float)robotposeV[0];
    float robotposeY = (float)robotposeV[1];
    float robotposeTheta = (float)robotposeV[2];
    
    /* get the dimensions of the goalpose and the goalpose itself*/     
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 3.");         
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    float goalposeX = (float)goalposeV[0];
    float goalposeY = (float)goalposeV[1];
        
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( 1, 1, mxINT8_CLASS, mxREAL); 
    int* action_ptr = (int*) mxGetData(ACTION_OUT);

    /* Do the actual planning in a subroutine */
    planner(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY, motion_primitives, &action_ptr[0]);

    return;
    
}





