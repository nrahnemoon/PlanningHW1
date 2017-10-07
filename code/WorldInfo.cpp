#include "WorldInfo.h"
#include <math.h>
#include <queue>
#include <iostream>
#include "mex.h"

using namespace std;

WorldInfo::WorldInfo(float startX, float startY, float startOrientation, float endX, float endY, float mapWidth, float mapHeight, double* obstacleMap, PrimArrayPtr primitives)
    :mStartX(startX), mStartY(startY), mStartOrientation(startOrientation), mEndX(endX), mEndY(endY), mMapWidth(mapWidth), mMapHeight(mapHeight), mObstacleMap(obstacleMap), mPrimitives(primitives) {

    mDiscreteStartX = discretize(mStartX);
    mDiscreteStartY = discretize(mStartY);
    mDiscreteEndX = discretize(mEndX);
    mDiscreteEndY = discretize(mEndY);

    mDiscreteMapWidth = discretize(mMapWidth);
    mDiscreteMapHeight = discretize(mMapHeight);
    mDiscreteMapSize = mDiscreteMapWidth * mDiscreteMapHeight;
    mCloseThreshold = 0;
    /*mexPrintf("Discrete map size = (%d, %d)\n", mDiscreteMapWidth, mDiscreteMapHeight);

    for (int y = 0; y < mDiscreteMapHeight; y++) {
        for (int x = 0; x < mDiscreteMapWidth; x++) {
            if (isInCollision(x,y))
                mexPrintf("X");
            else
                mexPrintf(" ");
        }
        mexPrintf("\n");
    }*/
    
    //computeDumbEuclideanHeuristics();
    //computeDijkstraHeuristics();
}

WorldInfo::~WorldInfo() {
    delete mObstacleMap;
    for (map<int, Node*>::const_iterator nodeGridIterator = mNodeGrid.begin(); nodeGridIterator != mNodeGrid.end(); ++nodeGridIterator) {
        delete nodeGridIterator->second;
    }
}

float WorldInfo::getStartX() { return mStartX; }

float WorldInfo::getStartY() { return mStartY; }

float WorldInfo::getStartOrientation() { return mStartOrientation; }

float WorldInfo::getGoalX() { return mEndX; }

float WorldInfo::getGoalY() { return mEndY; }

int WorldInfo::getDiscreteGoalX() { return mDiscreteEndX; }

int WorldInfo::getDiscreteGoalY() { return mDiscreteEndY; }

int WorldInfo::getDiscreteStartX() { return mDiscreteStartX; }

int WorldInfo::getDiscreteStartY() { return mDiscreteStartY; }

int WorldInfo::getDiscreteMapWidth() { return mDiscreteMapWidth; }

int WorldInfo::getDiscreteMapHeight() { return mDiscreteMapHeight; }

int WorldInfo::getDiscreteStartOrientation() { return discretizeAngle(mStartOrientation); }

float WorldInfo::getMapWidth() { return mMapWidth; }

float WorldInfo::getMapHeight()  { return mMapHeight; }

bool WorldInfo::isStart(int discreteX, int discreteY) { return (mDiscreteStartX == discreteX && mDiscreteStartY == discreteY); }

float WorldInfo::getPrimitive(int discreteOrientation, int primitiveNum, int selector) {
    // mexPrintf("orientation, primitiveNum, selector, primitive = %d, %d, %d, %f\n", orientation, primitiveNum, selector, mPrimitives[orientation][primitiveNum][NUMOFINTERSTATES - 1][selector]);
    return mPrimitives[discreteOrientation][primitiveNum][NUMOFINTERSTATES - 1][selector];
}

int WorldInfo::discretize(float pos) {
    return (int)(pos / RES + 0.5); // RES comes from planner.h
}

int WorldInfo::discretizeAngle(float angle) {
    /* returns the direction index with respect to the PrimArray */
    /* normalize bw 0 to 2pi */
    angle = fmod(angle, (2 * M_PI));
    while (angle < 0.0) {
        angle += 2 * M_PI;
    }
    int dir = (int)(angle / (2 * M_PI / NUMOFDIRS) + 0.5);
    if (dir == 8) {
        dir = 0;
    }
    return dir;
}

bool WorldInfo::isInCollision(int discreteX, int discreteY) {
    int mapIndex = GETMAPINDEX(discreteX, discreteY, mDiscreteMapWidth, mDiscreteMapHeight);
    mexPrintf("Checking if (%d, %d) with mapIndex %d is in collision\n", discreteX, discreteY, mapIndex);
    return (((int) mObstacleMap[mapIndex]) != 0.0);
}

bool WorldInfo::nodeExists(int discreteX, int discreteY, int discreteOrientation, int incomingPrimitive) {
    int nodeID = getID(discreteX, discreteY, discreteOrientation, incomingPrimitive);
    return (mNodeGrid.find(nodeID) != mNodeGrid.end());
}

Node* WorldInfo::getNode(int discreteX, int discreteY, int discreteOrientation, int incomingPrimitive) {
    if (!nodeExists(discreteX, discreteY, discreteOrientation, incomingPrimitive))
        return NULL;

    int nodeID = getID(discreteX, discreteY, discreteOrientation, incomingPrimitive);
    return mNodeGrid[nodeID];
}

int WorldInfo::getMapIndex(int discreteX, int discreteY) {
    return (discreteY * mDiscreteMapWidth + discreteX);
}

int WorldInfo::getID(int discreteX, int discreteY, int discreteOrientation, int incomingPrimitive) {
    return (((getMapIndex(discreteX, discreteY) * NUMOFDIRS) + discreteOrientation) * NUMOFPRIMS) + incomingPrimitive;
}

int WorldInfo::getDumbEuclideanToStart(int discreteX, int discreteY) {
    return (pow(discreteX - mDiscreteStartX, 2) + pow(discreteY - mDiscreteStartY, 2));
}

int WorldInfo::getDumbEuclideanToEnd(int discreteX, int discreteY) {
    return (pow(discreteX - mDiscreteEndX, 2) + pow(discreteY - mDiscreteEndY, 2));
}

void WorldInfo::computeDumbEuclideanHeuristics() {
    int currMapIndex;
    for(int discreteX = 0; discreteX < mDiscreteMapWidth; discreteX++) {
        for(int discreteY = 0; discreteY < mDiscreteMapHeight; discreteY++) {
            currMapIndex = getMapIndex(discreteX, discreteY);
            if (!mObstacleMap[currMapIndex]) {
                mHeuristics[currMapIndex] = getDumbEuclideanToEnd(discreteX, discreteY);
                //mexPrintf("Euclidean Heuristic for (%d, %d) = %d\n", discreteX, discreteY, mHeuristics[currMapIndex]);
            }
        }
    }
}

class DijkstraNode {
    public :
        int mDiscreteX;
        int mDiscreteY;
        int mSortValue;
        
        DijkstraNode(int discreteX, int discreteY, int sortValue)
            :mDiscreteX(discreteX), mDiscreteY(discreteY), mSortValue(sortValue) {}
};

class CompareDijkstraNode {
    public:
        bool operator() (DijkstraNode* thisNode, DijkstraNode* thatNode) {
            return thisNode->mSortValue > thatNode->mSortValue;
        }
};

void WorldInfo::computeDijkstraHeuristics() {

    priority_queue<DijkstraNode*, vector<DijkstraNode*>, CompareDijkstraNode> dijkstraQueue; // Returns minimum item
    dijkstraQueue.push(new DijkstraNode(mDiscreteEndX, mDiscreteEndY, getDumbEuclideanToStart(mDiscreteEndX, mDiscreteEndY)));

    mHeuristics[getMapIndex(mDiscreteEndX, mDiscreteEndY)] = 0; // Assumes goal isn't on an obstacle...

    int neighborX[] = {1, -1, 0, 0};
    int neighborY[] = {0 , 0, 1, -1};
    int currMapIndex;
    int neighborDiscreteX, neighborDiscreteY, neighborMapIndex;

    bool countdown = false;
    int countdownNum = 100; // Go 100 nodes beyond start for backwards Dijkstra

    DijkstraNode* currDijkstraNode;
    
    while(true) {

        if (dijkstraQueue.size() == 0)
            break;

        currDijkstraNode = dijkstraQueue.top();
        dijkstraQueue.pop();

        currMapIndex = getMapIndex(currDijkstraNode->mDiscreteX, currDijkstraNode->mDiscreteY);
        //mexPrintf("\nCurr Dij Node = (%d, %d)\n", currDijkstraNode->mDiscreteX, currDijkstraNode->mDiscreteY);

        if (countdown) {
            countdownNum--;
            if (countdownNum == 0)
                return;
        } else if (isStart(currDijkstraNode->mDiscreteX, currDijkstraNode->mDiscreteY)) {
            countdown = true;
        }

        for (int i = 0; i < 4; i++) {
            neighborDiscreteX = currDijkstraNode->mDiscreteX + neighborX[i];
            neighborDiscreteY = currDijkstraNode->mDiscreteY + neighborY[i];
            //mexPrintf("Neighbor = (%d, %d)\n", neighborDiscreteX, neighborDiscreteY);
            if (!outOfBounds(neighborDiscreteX, neighborDiscreteY)) {
                neighborMapIndex = getMapIndex(neighborDiscreteX, neighborDiscreteY);
                if (!mObstacleMap[neighborMapIndex]) {
                    if (mHeuristics[neighborMapIndex] != mHeuristics[currMapIndex] + 1) {
                        mHeuristics[neighborMapIndex] = mHeuristics[currMapIndex] + 1;
                        //mexPrintf("Dijkstra Heuristic for (%d, %d) = %d\n", neighborDiscreteX, neighborDiscreteY, mHeuristics[neighborMapIndex]);
                        dijkstraQueue.push(new DijkstraNode(neighborDiscreteX, neighborDiscreteY, getDumbEuclideanToStart(neighborDiscreteX, neighborDiscreteY)));
                    }
                }
            }
        }
        delete currDijkstraNode;
    }
}

int WorldInfo::getHeuristicAt(int discreteX, int discreteY) {
    int pos = getMapIndex(discreteX, discreteY);
    if (mHeuristics.find(pos) == mHeuristics.end())
        return getDumbEuclideanToEnd(discreteX, discreteY);
    else
        return mHeuristics[pos];
}

float WorldInfo::getDistanceToGoal(float x, float y) {
    float deltaX = fabs(mEndX - x);
    float deltaY = fabs(mEndY - y);
    return euclideanDistance(deltaX, deltaY);
}

float WorldInfo::euclideanDistance(float deltaX, float deltaY) {
    return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}

bool WorldInfo::outOfBounds(int discreteX, int discreteY) {
    return (discreteX > mDiscreteMapWidth || discreteX < 1 || discreteY > mDiscreteMapHeight || discreteY < 1);
}

void WorldInfo::addToNodeGrid(Node* node) {
    int nodeID = getID(node->getDiscreteX(), node->getDiscreteY(), node->getDiscreteOrientation(), node->getIncomingPrimitive());
    mNodeGrid[nodeID] = node;
}

void WorldInfo::update(float startX, float startY, float startOrientation, float endX, float endY, float mapWidth, float mapHeight, double* obstacleMap, PrimArrayPtr primitives) {
    mStartX = startX;
    mStartY = startY;
    mStartOrientation = startOrientation;
    mEndX = endX;
    mEndY = endY;
    mDiscreteStartX = discretize(mStartX);
    mDiscreteStartY = discretize(mStartY);
    mDiscreteEndX = discretize(mEndX);
    mDiscreteEndY = discretize(mEndY);
    
    mMapWidth = mapWidth;
    mMapHeight = mapHeight;
    mObstacleMap = obstacleMap;
    
    mDiscreteMapWidth = discretize(mMapWidth);
    mDiscreteMapHeight = discretize(mMapHeight);
    mDiscreteMapSize = mDiscreteMapWidth * mDiscreteMapHeight;
    mPrimitives = primitives;
    
    resetCloseThreshold();
}

int WorldInfo::getCloseThreshold() {
    return mCloseThreshold;
}

void WorldInfo::resetCloseThreshold() {
    mCloseThreshold++;
}

bool WorldInfo::isCloseEnoughToGoal(float x, float y) {
    // Bug in Matlab code says the target has been reached when the robot is 0.5 away from the goal
    // Let this be 0.4 for safety...
    return (euclideanDistance(fabs(x- mEndX), fabs(y - mEndY)) <= 0.4);
}

bool WorldInfo::goalReached() {
    // Bug in Matlab code says the target has been reached when the robot is 0.5 away from the goal
    return isCloseEnoughToGoal(mStartX, mStartY);
}
