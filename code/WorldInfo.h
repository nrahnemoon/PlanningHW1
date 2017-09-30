#include "planner.h"
#include <map>
using namespace std;

class WorldInfo {
    
    private:

        float mStartX;
        float mStartY;
        float mEndX;
        float mEndY;
        float mMapWidth;
        float mMapHeight;
        PrimArray mPrimitives;
        bool* mMap;
        map<int, Node*> mNodeGrid;

    public:

        WorldInfo(float startX, float startY, float goalX, float goalY, float mapWidth, float mapHeight, bool* map, PrimArray primitives);

        float getStartX();
        float getStartY();
        float getGoalX();
        float getGoalY();
        float getMapWidth();
        float getMapHeight();
        PrimArray getPrimitives();
        
        bool isInCollision(float x, float y);
        bool nodeExists(float x, float y);
        Node* getNode(float x, float y);
}
