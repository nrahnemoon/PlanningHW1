#include "Node.h"
#include <math.h>

Node::Node(float x, float y, float orientation, int incomingPrimitive, WorldInfo* worldInfo, Node* parent)
    :mX(x), mY(y), mOrientation(orientation), mIncomingPrimitive(incomingPrimitive), mWorldInfo(worldInfo), mParent(parent), mNumNeighbors(0), mNeighborsDiscovered(false) {
    
    mDiscreteX = mWorldInfo->discretize(mX);
    mDiscreteY = mWorldInfo->discretize(mY);
    mDiscreteOrientation = mWorldInfo->discretizeAngle(mOrientation);

    updateG();
    updateH();

    mWorldInfo->addToNodeGrid(this);
    mCloseID = mWorldInfo->getCloseThreshold();
}

bool Node::getNeighborsDiscovered() {
    return mNeighborsDiscovered;
}

float Node::getDistanceToParent() {
    int deltaX = mParent->getDiscreteX() - mDiscreteX;
    int deltaY = mParent->getDiscreteY() - mDiscreteY;
    return mWorldInfo->euclideanDistance(((float) deltaX), ((float) deltaY));
}

float Node::getDistanceToGoal() {
    int deltaX = mWorldInfo->getDiscreteGoalX() - mDiscreteX;
    int deltaY = mWorldInfo->getDiscreteGoalY() - mDiscreteY;
    return mWorldInfo->euclideanDistance(((float) deltaX), ((float) deltaY));
}

void Node::discoverNeighbors() {
    // mexPrintf("Neighbors for (%d, %d, %d)\n\n", mDiscreteX, mDiscreteY, mDiscreteOrientation);

    for (int primitiveNum = 0; primitiveNum < NUMOFPRIMS; primitiveNum++) {
        float newX = mX + mWorldInfo->getPrimitive(mDiscreteOrientation, primitiveNum, 0);
        float newY = mY + mWorldInfo->getPrimitive(mDiscreteOrientation, primitiveNum, 1);
        float newOrientation = mWorldInfo->getPrimitive(mDiscreteOrientation, primitiveNum, 2);
        //mexPrintf("New Neighbor[%d] = (%f, %f, %f)\n", primitiveNum, newX, newY, newOrientation);

        int discreteNewX = mWorldInfo->discretize(newX);
        int discreteNewY = mWorldInfo->discretize(newY);
        int discreteNewOrientation = mWorldInfo->discretizeAngle(newOrientation);

        //mexPrintf("New Neighbor[%d] = (%d, %d, %d)\n", primitiveNum, discreteNewX, discreteNewY, discreteNewOrientation);
        //mexPrintf("New Neighbor 1\n");
        if(!mWorldInfo->isInCollision(discreteNewX, discreteNewY) && !mWorldInfo->outOfBounds(discreteNewX, discreteNewY)) {
            //mexPrintf("New Neighbor 2\n");
            if (mWorldInfo->nodeExists(discreteNewX, discreteNewY, discreteNewOrientation, primitiveNum)) {
                Node* testNode = mWorldInfo->getNode(discreteNewX, discreteNewY, discreteNewOrientation, primitiveNum);
                //mexPrintf("testNode->isClosed() = %d\n", testNode->isClosed());
                //mexPrintf("mWorldInfo->getClosingThreshold() = %d\n", mWorldInfo->getCloseThreshold());
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
    mNeighborsDiscovered = true;
    // mexPrintf("\n\n");
}

float Node::getX() { return mX; }

float Node::getY() { return mY; }

float Node::getOrientation() { return mOrientation; }

int Node::getDiscreteX() { return mDiscreteX; }

int Node::getDiscreteY() { return mDiscreteY; }

int Node::getDiscreteOrientation() { return mDiscreteOrientation; }

void Node::updateG() {
    if (mParent == NULL) {
        mG = 0;
    } else {
        mG = mParent->getG() + getDistanceToParent();
    }
}

void Node::updateH() {
    // Using combo Euclidean + Dijkstra heuristic
    mH = mWorldInfo->getHeuristicAt(mDiscreteX, mDiscreteY);
    
    // Uncomment for simple Euclidean heuristic
    // mH = getDistanceToGoal();
}

float Node::getG() {
    return mG;
}

float Node::getH() {
    return mH;
}

bool Node::isClosed() {
    return mCloseID > mWorldInfo->getCloseThreshold();
}

float Node::getCost() {
    updateG();
    updateH();
    return (mG + mH);
}

void Node::close() { mCloseID = mWorldInfo->getCloseThreshold() + 1; }

bool Node::isGoal() { return (mDiscreteX == mWorldInfo->getDiscreteGoalX() && mDiscreteY == mWorldInfo->getDiscreteGoalY()); }

Node* Node::getNeighbor(int index) { return mNeighbors[index]; }

int Node::getNumNeighbors() { return mNumNeighbors; }

Node* Node::getParent() { return mParent; }

void Node::setParent(Node* parent) {
    mParent = parent;
}

int Node::getID() {
    return (mWorldInfo->getMapIndex(mX, mY) * NUMOFDIRS) + mOrientation;
}

int Node::getIncomingPrimitive() {
    return mIncomingPrimitive;
}

void Node::setIncomingPrimitive(int incomingPrimitive) {
    mIncomingPrimitive = incomingPrimitive;
}

bool Node::isCloseEnoughToGoal() {
    return mWorldInfo->isCloseEnoughToGoal(mX, mY);
}

bool CompareNode::operator() (Node* thisNode, Node* thatNode) {
    return thisNode->getCost() > thatNode->getCost();
}
