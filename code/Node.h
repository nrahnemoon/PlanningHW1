#include "planner.h"
#include "WorldInfo.h"
#include <vector>
using namespace std;

class Node {

    private:

        vector<Node*> mNeighbors;
        bool mClosed;
        float mX, mY, mOrientation;
        float mG, mH;
        WorldInfo* mWorldInfo;
        Node* mParent;

        float euclideanDistance(float deltaX, float deltaY);
        float getDistanceToParent();

    public:

        Node(float x, float y, float orientation, WorldInfo* worldInfo, Node* parent);

        bool operator<(Node thatNode) const;
        void discoverNeighbors();

        float getX();
        float getY();
        float getOrientation();
        float getG();
        bool isClosed();
        float getCost();
        Node* getNeighbor(int index);
        int getNumNeighbors();
}
