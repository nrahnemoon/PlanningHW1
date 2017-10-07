#ifndef WORLD_INFO_H
#define WORLD_INFO_H

#include "Node.h"
#include "planner.h"
#include <map>
#include <ctime>

using namespace std;

class Node;

class WorldInfo {
    
    private:

        float mStartX;
        float mStartY;
        float mStartOrientation;
        float mEndX;
        float mEndY;
        int mDiscreteStartX;
        int mDiscreteStartY;
        int mDiscreteEndX;
        int mDiscreteEndY;
        float mMapWidth;
        float mMapHeight;
        int mDiscreteMapWidth;
        int mDiscreteMapHeight;
        int mDiscreteMapSize;
        PrimArrayPtr mPrimitives;
        double* mObstacleMap;
        int mCloseThreshold;
        
        // This takes into account orientation, so max size is
        // mMapHeight * mMapWidth * NUMOFDIRS * NUMOFPRIMS
        map<int, Node*> mNodeGrid;
        
        // This only takes into account position, so max size is
        // mMapHeight * mMapWidth
        map<int, int> mHeuristics;

        void computeDumbEuclideanHeuristics();
        bool isStart(int discreteX, int discreteY);

    public:

        WorldInfo(float startX, float startY, float startOrientation, float endX, float endY, float mapWidth, float mapHeight, double* obstacleMap, const PrimArrayPtr primitives);
        ~WorldInfo();

        float getStartX();
        float getStartY();
        float getStartOrientation();
        float getGoalX();
        float getGoalY();
        float getMapWidth();
        float getMapHeight();
        float getPrimitive(int discreteOrientation, int primitiveNum, int selector);
        
        void addToNodeGrid(Node* node);
        int discretize(float pos);
        int discretizeAngle(float angle);
        bool isInCollision(int discreteX, int discreteY);
        bool nodeExists(int discreteX, int discreteY, int discreteOrientation, int incomingPrimitive);
        Node* getNode(int discreteX, int discreteY, int discreteOrientation, int incomingPrimitive);
        int getMapIndex(int discreteX, int discreteY);
        int getID(int discreteX, int discreteY, int discreteOrientation, int incomingPrimitive);
        int getHeuristicAt(int discreteX, int discreteY);
        float euclideanDistance(float deltaX, float deltaY);
        float getDistanceToGoal(float x, float y);
        bool outOfBounds(int discreteX, int discreteY);
        int getDiscreteGoalX();
        int getDiscreteGoalY();
        int getDiscreteStartX();
        int getDiscreteStartY();
        int getDiscreteMapWidth();
        int getDiscreteMapHeight();
        int getDiscreteStartOrientation();
        void computeDijkstraHeuristics();
        int getDumbEuclideanToEnd(int discreteX, int discreteY);
        int getDumbEuclideanToStart(int discreteX, int discreteY); // Euclidean without square-root
        void update(float startX, float startY, float startOrientation, float endX, float endY, float mapWidth, float mapHeight, double* obstacleMap, PrimArrayPtr primitives);
        int getCloseThreshold();
        void resetCloseThreshold();
        bool isCloseEnoughToGoal(float x, float y);
        bool goalReached();
};

#endif
