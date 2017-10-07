#ifndef NODE_H
#define NODE_H

#include "planner.h"
#include "WorldInfo.h"
#include <vector>
#include <ctime>
using namespace std;

class WorldInfo;

class Node {

    private:

        Node* mNeighbors[NUMOFPRIMS];
        int mNumNeighbors;
        bool mNeighborsDiscovered;
        int mCloseID;
        float mX, mY, mOrientation;
        int mDiscreteX, mDiscreteY, mDiscreteOrientation;
        float mF, mG, mH;
        WorldInfo* mWorldInfo;
        Node* mParent;
        int mIncomingPrimitive; // Set to -1 for the start node

        float getDistanceToParent();
        float getDistanceToGoal();
        void updateG();
        void updateH();

    public:

        Node(float x, float y, float orientation, int incomingPrimitive, WorldInfo* worldInfo, Node* parent);

        bool operator<(Node thatNode);
        void discoverNeighbors();

        float getX();
        float getY();
        float getOrientation();
        int getDiscreteX();
        int getDiscreteY();
        int getDiscreteOrientation();
        float getF();
        float getG();
        float getH();
        void close();
        bool isClosed();
        float getCost();
        Node* getNeighbor(int index);
        int getNumNeighbors();
        Node* getParent();
        void setParent(Node* parent);
        bool isGoal();
        int getID();
        int getIncomingPrimitive();
        void setIncomingPrimitive(int incomingPrimitive);
        bool isCloseEnoughToGoal();
        bool getNeighborsDiscovered();
};

class CompareNode {

    public:
    
        bool operator() (Node* thisNode, Node* thatNode);
};

#endif
