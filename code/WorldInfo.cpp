#include "WorldInfo.h"
#include <math.h>
#include <queue>
#include <iostream>

using namespace std;

WorldInfo::WorldInfo(float startX, float startY, float endX, float endY, float mapWidth, float mapHeight, bool* map, PrimArrayPtr primitives)
    :mStartX(startX), mStartY(startY), mEndX(endX), mEndY(endY), mMapWidth(mapWidth), mMapHeight(mapHeight), mMap(map), mPrimitives(primitives) {
	cout << "Here!" << endl;
    computeDijkstraCost();
}

float WorldInfo::getStartX() { return mStartX; }

float WorldInfo::getStartY() { return mStartY; }

float WorldInfo::getGoalX() { return mEndX; }

float WorldInfo::getGoalY() { return mEndY; }

float WorldInfo::getMapWidth() { return mMapWidth; }

float WorldInfo::getMapHeight()  { return mMapHeight; }

float WorldInfo::getPrimitive(int orientation, int primitiveNum, int selector) {
    return mPrimitives[orientation][primitiveNum][NUMOFINTERSTATES - 1][selector];
}

int WorldInfo::discretize(float pos) {
    return ceil(pos/RES + 0.5); // RES comes from planner.h
}

bool WorldInfo::isInCollision(float x, float y) {
    int mapIndex = GETMAPINDEX(discretize(x), discretize(y), discretize(mMapWidth), discretize(mMapWidth));
    return ((bool) mMap[mapIndex]);
}

bool WorldInfo::nodeExists(float x, float y, float orientation) {
    int nodeID = getID(x, y, orientation);
    return (mNodeGrid.find(nodeID) != mNodeGrid.end());
}

Node* WorldInfo::getNode(float x, float y, float orientation) {
    if (!nodeExists(x, y, orientation))
        return NULL;

    int nodeID = getID(x, y, orientation);
    return mNodeGrid[nodeID];
}

int WorldInfo::getMapIndex(float x, float y) {
    return GETMAPINDEX(discretize(x), discretize(y), discretize(mMapWidth), discretize(mMapWidth));
}

int WorldInfo::getID(float x, float y, float orientation) {
    return (getMapIndex(x, y) * NUMOFDIRS) + orientation;
}

void WorldInfo::computeDijkstraCost() {
    cout << "Here1" << endl;
    queue<int> dijkstraQueue;
    int goalPos = getMapIndex(mEndX, mEndY);
    dijkstraQueue.push(goalPos);

    int mapWidth = discretize(mMapWidth);
    int mapHeight = discretize(mMapHeight);
    mDijkstraCost[goalPos] = 0; // Assume goal isn't on an obstacle...

    while(!dijkstraQueue.empty()) {
        int currPos = dijkstraQueue.front();
        dijkstraQueue.pop();
        int neighbors [4];
        int neighborPos = 0;

        if ((currPos + 1) % mapWidth != 0)
            neighbors[neighborPos++] = (currPos + 1);
        if (currPos % mapWidth != 0)
            neighbors[neighborPos++] = (currPos - 1);
        if (currPos / mapWidth != (mapHeight - 1))
            neighbors[neighborPos++] = (currPos + mapWidth);
        if (currPos / mapWidth != 0)
            neighbors[neighborPos++] = (currPos - mapWidth);

        for (int i = 0; i <= neighborPos; i++) {
            if (mDijkstraCost.find(neighbors[i]) == mDijkstraCost.end()) {
                if (!mMap[neighbors[i]]) {
                    mDijkstraCost[neighbors[i]] = mDijkstraCost[currPos] + 1;
                    dijkstraQueue.push(neighbors[i]);
                }
            }
        }
        cout << "Dijkstra cost size = " << mDijkstraCost.size() << endl;
    }
    
    int currX = discretize(mEndX);
    int currY = discretize(mEndY);
    int mapXMax = discretize(mMapWidth);
    int mapYMax = discretize(mMapHeight);
}

int WorldInfo::getDijkstraCostAt(float x, float y) {
    int pos = getMapIndex(x, y);
    if (mDijkstraCost.find(pos) == mDijkstraCost.end())
        return mMapWidth * mMapHeight;
    else
        return mDijkstraCost[pos];
}

float WorldInfo::getDistanceToGoal(float x, float y) {
    float deltaX = fabs(mEndX - x);
    float deltaY = fabs(mEndY - y);
    return euclideanDistance(deltaX, deltaY);
}

float WorldInfo::euclideanDistance(float deltaX, float deltaY) {
    return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}
