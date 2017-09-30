#include "WorldInfo.h"
#include <math.h>

class WorldInfo {

    public:
        WorldInfo(float startX, float startY, float goalX, float goalY, float mapWidth, float mapHeight, int* map, PrimArray primitives)
            :mStartX(startX), mStartY(startY), mGoalX(goalX), mGoalY(goalY), mMapWidth(mapWidth), mMapHeight(mapHeight), mMap(map), mPrimitives(primitives) {}
        
        float getStartX() { return mStartX; }
        float getStartY() { return mStartY; }
        float getGoalX() { return mGoalX; }
        float getGoalY() { return mGoalY; }
        float getMapWidth() { return mMapWidth; }
        float getMapHeight()  { return mMapHeight; }
        PrimArray getPrimitives() { return mPrimitives; }

        int discretize(float pos) {
            return ceil(pos/RES); // RES comes from planner.h
        }

        bool isInCollision(float x, float y) {
            int mapIndex = GETMAPINDEX(discretize(x), discretize(y), discretize(mMapWidth), discretize(mMapWidth));
            return ((bool) mMap[mapIndex]);
        }

        bool nodeExists(float x, float y) {
            int mapIndex = GETMAPINDEX(discretize(x), discretize(y), discretize(mMapWidth), discretize(mMapWidth));
            return (mNodeGrid.find(mapIndex) != mNodeGrid.end());
        }
        
        Node* getNode(float x, float y) {
            if (!nodeExists(x, y))
                return -1;
            
            int mapIndex = GETMAPINDEX(discretize(x), discretize(y), discretize(mMapWidth), discretize(mMapWidth));
            return mNodeGrid[mapIndex];
        }
}
