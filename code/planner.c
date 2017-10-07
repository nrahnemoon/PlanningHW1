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
#include <future>
#include <chrono>
#include <thread>

using namespace std;

int roundNumber = 0;

WorldInfo* worldInfo = NULL;
Node* goalNodeWithBestPath = NULL;
bool aStarRunning = false;
long currCloseID = 1;
long mapMemoryLocation = -1;

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

static void callComputeDijkstraHeuristics() {
    mexPrintf("New Dijkstra Loop!\n");
    worldInfo->computeDijkstraHeuristics();
}

static void doAStar() {
    aStarRunning = true;
    Node* startNode;
    mexPrintf("New A Star Loop!\n");
    if (worldInfo->nodeExists(worldInfo->getDiscreteStartX(), worldInfo->getDiscreteStartY(), worldInfo->getDiscreteStartOrientation(), -1)) {
        startNode = worldInfo->getNode(worldInfo->getDiscreteStartX(), worldInfo->getDiscreteStartY(), worldInfo->getDiscreteStartOrientation(), -1);
    } else {
        startNode = new Node(worldInfo->getStartX(), worldInfo->getStartY(), worldInfo->getStartOrientation(), -1, worldInfo, NULL);
    }

    //mexPrintf("[A Star] Start X = %f, Start Y = %f\n", worldInfo->getStartX(), worldInfo->getStartY());

    priority_queue<Node*, std::vector<Node*>, CompareNode> priorityQueue;
    priorityQueue.push(startNode);

    Node* currNode;

    while (true) {
        if (priorityQueue.size() == 0) {
            mexPrintf("The goal is not reachable!!!\n");
            return;
        }
        if (worldInfo->goalReached()) {
            mexPrintf("The goal has been reached!  Anymore work is useless.\n");
            return;                                                                                                                         
        }
        currNode = priorityQueue.top();
        priorityQueue.pop();

        if (currNode->isClosed()) {
            //mexPrintf("Curr Node is closed!");
            continue;
        }
        if (currNode->isCloseEnoughToGoal())
            break;

        if (!currNode->getNeighborsDiscovered())
            currNode->discoverNeighbors();
        
        // mexPrintf("currNode location = (%f, %f, %f)\n", currNode->getX(), currNode->getY(), currNode->getOrientation());
        for (int i = 0; i < currNode->getNumNeighbors(); i++) {
            Node* neighbor = currNode->getNeighbor(i);
            // mexPrintf("neighbor location = (%f, %f, %f)\n", neighbor->getX(), neighbor->getY(), neighbor->getOrientation());
            priorityQueue.push(neighbor);
        }
        // mexPrintf("Closing current node.\n");
        currNode->close();
    }
    goalNodeWithBestPath = currNode;
    aStarRunning = false;
    mexPrintf("A Star Loop Completed!");
}

static void planner(
    double* map,
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
    mexPrintf("Map size = (%d, %d)", x_size, y_size);
    if (worldInfo == NULL) {
        mexPrintf("Creating worldinfo!\n");
        worldInfo = new WorldInfo(robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY,
                (x_size * RES), (y_size * RES), map, ((PrimArrayPtr) mprim));
    } else {
        mexPrintf("Updating worldInfo!\n");
        worldInfo->update(robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY,
                (x_size * RES), (y_size * RES), map, ((PrimArrayPtr) mprim));
    }
    
    if (worldInfo->goalReached()) {
        delete worldInfo;
        return;
    }

    callComputeDijkstraHeuristics();

    if (!aStarRunning) {
        doAStar();
    }

    // doAStar should have set goalNodeWithBestPath
    Node* currNode = goalNodeWithBestPath;
    
    int i = 0;
    //mexPrintf("Goal = (%d, %d)\n", worldInfo->getDiscreteGoalX(), worldInfo->getDiscreteGoalY());
    // currNode is the goal at this point
    while(currNode->getParent()->getParent() != NULL) {
        //mexPrintf("BackPath[%d] = (%d, %d, %d, %d)\n", i, currNode->getDiscreteX(), currNode->getDiscreteY(), currNode->getDiscreteOrientation(), currNode->getIncomingPrimitive());
        currNode = currNode->getParent();
        i++;
    }
    if (currNode->getParent()->getDiscreteX() != worldInfo->getDiscreteStartX() ||
            currNode->getParent()->getDiscreteY() != worldInfo->getDiscreteStartY()) {
        goalNodeWithBestPath = NULL;
        return;
    }
    /*mexPrintf("BackPath[%d] = (%d, %d, %d, %d)\n", i, currNode->getDiscreteX(), currNode->getDiscreteY(), currNode->getDiscreteOrientation(), currNode->getIncomingPrimitive());    
    mexPrintf("BackPath[%d] = (%d, %d, %d, %d)\n", (i+1), currNode->getParent()->getDiscreteX(), currNode->getParent()->getDiscreteY(), currNode->getParent()->getDiscreteOrientation(), currNode->getParent()->getIncomingPrimitive());
    mexPrintf("Start = (%d, %d, %d)\n", worldInfo->getDiscreteStartX(), worldInfo->getDiscreteStartY(), worldInfo->getDiscreteStartOrientation());

    mexPrintf("BackPath[%d] = (%d, %d, %d, %d)\n", i, currNode->getDiscreteX(), currNode->getDiscreteY(), currNode->getDiscreteOrientation(), currNode->getIncomingPrimitive());
    mexPrintf("Curr Node's Primitive = %d\n", currNode->getIncomingPrimitive());
    mexPrintf("Curr Node Is In Colllision = %d\n", worldInfo->isInCollision(currNode->getDiscreteX(), currNode->getDiscreteY()));
    mexPrintf("Here's what the docs say: %d\n", ((int) map[GETMAPINDEX(currNode->getDiscreteX(), currNode->getDiscreteY(), worldInfo->getDiscreteMapWidth(), worldInfo->getDiscreteMapHeight())]));
    for (int x = -20; x <= 20; x++) {
        for (int y = -20; y <= 20; y++) {
            mexPrintf("Curr Node(%d,%d) Is In Colllision = %d\n", x, y, worldInfo->isInCollision(currNode->getDiscreteX()+x, currNode->getDiscreteY()+y));
        }
    }*/
    *prim_id = currNode->getIncomingPrimitive();

    currNode->setParent(NULL); // currNode is now the start node

    return;
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
    fclose(fp);
}

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )
     
{
    mexPrintf("In mex function!");
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
    mexPrintf("About to load the map\n");   
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    mexPrintf("loaded the map\n");
    //mexPrintf("map memory location = %lu\n", map);
    if (mapMemoryLocation != -1 && mapMemoryLocation != ((long) map)) {
        delete worldInfo;
        delete goalNodeWithBestPath;
        mexPrintf("Current map is different from previous -- reloading map!\n");
    } else {
        mexPrintf("Current map is new or the same as the previous -- reusing map!\n");
    }

    mapMemoryLocation = ((long) map);

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





