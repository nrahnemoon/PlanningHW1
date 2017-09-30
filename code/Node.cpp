#include "Node.h"
#include <math.h>

class Node {

    private:

    float euclideanDistance(float deltaX, float deltaY) {
        return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
    }

    float getDistanceToParent() {
        float deltaX = abs(mParent->getX() - mX);
        float deltaY = abs(mParent->getY() - mY);
        return euclideanDistance(deltaX, deltaY);
    }

    float getDistanceToGoal() {
        float deltaX = abs(mWorldInfo->getGoalX() - mX);
        float deltaY = abs(mWorldInfo->getGoalY() - mY);
        return euclideanDistance(deltaX, deltaY);
    }

    bool operator<(Node thatNode) const {
        return getCost() > thatNode.getCost();
    }

    public:

    Node(float x, float y, float orientation, WorldInfo* worldInfo, Node* parent)
        :mX(x), mY(y), mOrientaiton(orientation), mWorldInfo(worldInfo), mParent(parent), mClosed(false) {
        mG = mParent->getG() + getDistanceToParent();
        mH = getDistanceToGoal();
    }

    void discoverNeighbors() {

        for (int i = 0; i < NUMOFPRIMS; i++) {

            float newX = mX + mPrimitives[mOrientation][i][NUMOFINTERSTATES - 1][0];
            float newY = mY + mPrimitives[mOrientation][i][NUMOFINTERSTATES - 1][1];
            float newOrientation = mPrimitives[mOrientation][i][NUMOFINTERSTATES - 1][2];
            
            if(!mWorldInfo->isInCollision(newX, newY)) {
                if (nodeExists(newX, newY)) {
                    Node* testNode = mWorldInfo->getNode(newX, newY);
                    if (!testNode->isClosed()) {
                        mNeighbors.insert(testNode);
                        continue;
                    }
                }
                Node newNode = Node(newX, newY, newOrientation, mWorldInfo, this);
                mNeighbors.insert(&newNode);
            }
        }
    }

    float getX() { return mX; }
    float getY() { return mY; }
    float getOrientation() { return mOrientation; }
    float getG() { return mG; }
    bool isClosed() { return mClosed; }
    float getCost() { return (mG + mH); }

    Node* getNeighbor(int index) {
        return mNeighbors[index];
    }

    int getNumNeighbors() {
        return mNeighbors.size();
    }
}
