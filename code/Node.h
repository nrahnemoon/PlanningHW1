#ifndef NODE_H
#define NODE_H

#include "planner.h"
#include "WorldInfo.h"
#include <vector>
using namespace std;

class WorldInfo;

class Node {

    private:

        vector<Node*> mNeighbors;
        bool mClosed;
        float mX, mY, mOrientation;
        float mG, mH;
        WorldInfo* mWorldInfo;
        Node* mParent;
        int mIncomingPrimitive; // Set to -1 for the start node

        float getDistanceToParent();
        float getDistanceToGoal();

    public:

        Node(float x, float y, float orientation, int incomingPrimitive, WorldInfo* worldInfo, Node* parent);

        bool operator<(Node thatNode) const;
        void discoverNeighbors();

        float getX();
        float getY();
        float getOrientation();
        float getG();
        bool isClosed();
        float getCost() const;
        void close();
        Node* getNeighbor(int index);
        int getNumNeighbors();
        Node* getParent();
        bool isGoal();
        int getID();
        int getIncomingPrimitive();
};

#endif
