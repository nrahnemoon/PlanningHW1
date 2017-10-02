#ifndef WORLD_INFO_H
#define WORLD_INFO_H

#include "Node.h"
#include "planner.h"
#include <map>
using namespace std;

class Node;

class WorldInfo {
    
    private:

        float mStartX;
        float mStartY;
        float mEndX;
        float mEndY;
        float mMapWidth;
        float mMapHeight;
        PrimArrayPtr mPrimitives;
        bool* mMap;
        
        // This takes into account orientation, so max size is
        // mMapHeight * mMapWidth * NUMOFDIRS
        map<int, Node*> mNodeGrid;
        
        // This only takes into account position, so max size is
        // mMapHeight * mMapWidth
        map<int, int> mDijkstraCost;
        
        void computeDijkstraCost();

    public:

        WorldInfo(float startX, float startY, float endX, float endY, float mapWidth, float mapHeight, bool* map, const PrimArrayPtr primitives);

        float getStartX();
        float getStartY();
        float getGoalX();
        float getGoalY();
        float getMapWidth();
        float getMapHeight();
        float getPrimitive(int orientation, int primitiveNum, int selector);
        
        void addToNodeGrid(Node* node);
        int discretize(float pos);
        bool isInCollision(float x, float y);
        bool nodeExists(float x, float y, float orientation);
        Node* getNode(float x, float y, float orientation);
        int getMapIndex(float x, float y);
        int getID(float x, float y, float orientation);
        int getDijkstraCostAt(float x, float y);
        float euclideanDistance(float deltaX, float deltaY);
        float getDistanceToGoal(float x, float y);
        bool outOfBounds(float x, float y);
};

#endif
