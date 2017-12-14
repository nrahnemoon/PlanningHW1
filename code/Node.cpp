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

void Node::resetNeighborsDiscovered() {
    mNeighborsDiscovered = false;
}

bool Node::getNeighborsDiscovered() {
    return mNeighborsDiscovered;
}

float Node::getDistanceToParent() {
    int deltaX = mParent->getDiscreteX() - mDiscreteX;
    int deltaY = mParent->getDiscreteY() - mDiscreteY;
    return pow(deltaX, 2) + pow(deltaY, 2);
}

float Node::getDistanceToGoal() {
    return mWorldInfo->getDumbEuclideanToEnd(mDiscreteX, mDiscreteY);
}

void Node::discoverNeighbors() {
    // mexPrintf("Neighbors for (%d, %d, %d)\n\n", mDiscreteX, mDiscreteY, mDiscreteOrientation);
    mNumNeighbors = 0;
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
        if(!mWorldInfo->outOfBounds(discreteNewX, discreteNewY) && !mWorldInfo->isPathInCollision(this, primitiveNum)) {
            //mexPrintf("New Neighbor 2\n");
            if (mWorldInfo->nodeExists(discreteNewX, discreteNewY, discreteNewOrientation, primitiveNum)) {
                Node* testNode = mWorldInfo->getNode(discreteNewX, discreteNewY, discreteNewOrientation, primitiveNum);
                
                if (testNode && testNode->getCloseID() == mWorldInfo->getCloseThreshold() - 1) {
                    testNode->setParent(this);
                    testNode->resetCloseID();
                    testNode->getCost();
                    testNode->resetNeighborsDiscovered();
                    //mexPrintf("Old neighbor exists at %d, %d.  Setting this node to parent.\n", discreteNewX, discreteNewY);
                    mNeighbors[mNumNeighbors] = testNode;
                    mNumNeighbors++;
                    continue;
                }
                if (testNode && !testNode->isClosed()) {
                    Node* testNodeParent = testNode->getParent();
                    float testNodeCost = testNode->getCost();
                    testNode->setParent(this);
                    if (testNode->getCost() >= testNodeCost) {
                        testNode->setParent(testNodeParent);
                        testNode->getCost();
                        mNeighbors[mNumNeighbors] = testNode;
                        mNumNeighbors++;
                        //mexPrintf("Cheaper neighbor exists at %d, %d\n", discreteNewX, discreteNewY);
                        continue;
                    }
                    mNeighbors[mNumNeighbors] = testNode;
                    mNumNeighbors++;
                    testNode->resetCloseID();
                    testNode->resetNeighborsDiscovered();
                    //mexPrintf("Costlier neighbor exists at %d, %d.  Setting this node to parent.\n", discreteNewX, discreteNewY);
                }
                continue;
            }
            Node* newNode = new Node(newX, newY, newOrientation, primitiveNum, mWorldInfo, this);
            mNeighbors[mNumNeighbors] = newNode;
            //mexPrintf("New node = %f, %f, %f.\n", mNeighbors[mNumNeighbors]->getX(), mNeighbors[mNumNeighbors]->getY(), mNeighbors[mNumNeighbors]->getOrientation());
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
        mG = mParent->getG() + (50/getDistanceToParent());
        // 50/getDistanceToParent() biases towards longer steps -> less moves
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

int Node::getCloseID() {
    return mCloseID;
}

void Node::resetCloseID() {
    mCloseID = mWorldInfo->getCloseThreshold();
}

bool Node::isClosed() {
    return mCloseID > mWorldInfo->getCloseThreshold();
}

float Node::getCost() {
    updateG();
    updateH();
    return (mG + 0.1 * mH);
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
