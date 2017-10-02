#include "Node.h"
#include <math.h>

Node::Node(float x, float y, float orientation, int incomingPrimitive, WorldInfo* worldInfo, Node* parent)
    :mX(x), mY(y), mOrientation(orientation), mIncomingPrimitive(incomingPrimitive), mWorldInfo(worldInfo), mParent(parent), mClosed(false), mNumNeighbors(0) {
    if (mParent == NULL) {
        mG = 0;
    } else {
        mG = mParent->getG() + getDistanceToParent();
    }
    // Using Dijkstra heuristic
    // mH = mWorldInfo->getDijkstraCostAt(mX, mY);
    // Uncomment for Euclidean heuristic
    mH = getDistanceToGoal();
    mWorldInfo->addToNodeGrid(this);
}

float Node::getDistanceToParent() {
    float deltaX = fabs(mParent->getX() - mX);
    float deltaY = fabs(mParent->getY() - mY);
    return mWorldInfo->euclideanDistance(deltaX, deltaY);
}

float Node::getDistanceToGoal() {
    float deltaX = fabs(mWorldInfo->getGoalX() - mX);
    float deltaY = fabs(mWorldInfo->getGoalY() - mY);
    return mWorldInfo->euclideanDistance(deltaX, deltaY);
}

bool Node::operator<(Node thatNode) const {
    return this->getCost() > thatNode.getCost();
}

void Node::discoverNeighbors() {

    for (int primitiveNum = 0; primitiveNum < NUMOFPRIMS; primitiveNum++) {
        float newX = mX + mWorldInfo->getPrimitive(mOrientation, primitiveNum, 0);
        float newY = mY + mWorldInfo->getPrimitive(mOrientation, primitiveNum, 1);
        float newOrientation = mWorldInfo->getPrimitive(mOrientation, primitiveNum, 2);

        if(!mWorldInfo->isInCollision(newX, newY) && !mWorldInfo->outOfBounds(newX, newY)) {
            if (mWorldInfo->nodeExists(newX, newY, newOrientation)) {
                Node* testNode = mWorldInfo->getNode(newX, newY, newOrientation);
                if (testNode && !testNode->isClosed()) {
                    mNeighbors[mNumNeighbors] = testNode;
                    mNumNeighbors++;
                }
                continue;
            }
            mNeighbors[mNumNeighbors] = new Node(newX, newY, newOrientation, primitiveNum, mWorldInfo, this);
            // mexPrintf("newNode contains = %f, %f, %f", mNeighbors[mNumNeighbors]->getX(), mNeighbors[mNumNeighbors]->getY(), mNeighbors[mNumNeighbors]->getOrientation());
            mNumNeighbors++;
        }
    }
}

float Node::getX() { return mX; }

float Node::getY() { return mY; }

float Node::getOrientation() { return mOrientation; }

float Node::getG() { return mG; }

bool Node::isClosed() { return mClosed; }

float Node::getCost() const { return (mG + mH); }

void Node::close() { mClosed = true; }

bool Node::isGoal() { return (mX == mWorldInfo->getGoalX() && mY == mWorldInfo->getGoalY()); }

Node* Node::getNeighbor(int index) { return mNeighbors[index]; }

int Node::getNumNeighbors() { return mNumNeighbors; }

Node* Node::getParent() { return mParent; }

int Node::getID() {
    return (mWorldInfo->getMapIndex(mX, mY) * NUMOFDIRS) + mOrientation;
}

int Node::getIncomingPrimitive() {
    return mIncomingPrimitive;
}
