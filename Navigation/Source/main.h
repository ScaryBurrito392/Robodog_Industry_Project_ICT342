//
// Created by jraf2 on 8/09/2025.
//

#ifndef DLITE_PATHFINDER_MAIN_H
#define DLITE_PATHFINDER_MAIN_H

#endif //DLITE_PATHFINDER_MAIN_H

#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <vector>
# include <utility>
#include <hash_map>
#include <unordered_map>
// #include <pybind11/pybind11.h>
// namespace py = pybind11;

namespace GoDog {
    constexpr double STRAIGHT_TRAVERSAL_COST = 1;
    constexpr double DIAGONAL_TRAVERSAL_COST = 1.41421;

    /*true if key a smaller than key b*/
    bool compareKeys(double aK1, double aK2, double bK1, double bK2);

    /*
     * Returns true if the difference between values is less than the epsilon
     */
    bool epsilonEquality(double val1,double val2, double epsilon);


    /*
     * This class contains necessary information required for a D* Lite traversal algorith
     * These are intended to serve as squares which form a grid on which the algorithm operates
     */
    class GridSquare {
    private:
        double g;
        double rhs;
        int obstacle;
        bool sourceObstacle;

    public:
        GridSquare();

        GridSquare& operator=(const GridSquare& other);

        void setG(double g);

        [[nodiscard]] double getG() const;

        void setRHS(double rhs);

        [[nodiscard]] double getRHS() const;

        /*
         *This class stores how many times it is set as an obstacle.
         * This is because obstacles on the grid can have padding to account for the width of the entity
         * To facilitate this, squares where an obstacle really is are set as source obstacles, and a certain radius around
         * them, all squares are made obstacles.
         * When a source obstacle is made no longer a source, all squares within the radius are set as not obstacles
         * For this reason, every time a square is set as an obstacle, the obstacle int is incremented, and decremented
         * when set as not an obstacle.
         * This way, when queried whether it is an obstacle, it will only return false when it is not in radius of a source
         * obstacle, so its value is at 0
         * The set obstacle methods below return whether the square changed whether it is an obstacle or not, as this
         * allows for easily detecting changed squares
         */

        bool setObstacle(bool setObstacle);

        [[nodiscard]] bool isObstacle() const;

        [[nodiscard]] bool isSourceObstacle() const;

        bool setSourceObstacle(bool setSourceObstacle);
    };


    /*
     * This class acts as a wrapper object for a GridSquare.
     * It holds features that are lost when the GridSquare is removed from the grid, such as its coordinates.
     * It also holds data necessary for use in the D* Lite traversal algorithm such as keys and a version used within
     * the priority queue
     */
    class GridSquareNode {
    private:
        int x, y;  //x and y of square
        GridSquare *square;
        double k1{}, k2{};
        int version{};
    public:
        GridSquareNode(GridSquare *square, int x, int y);

        [[nodiscard]] GridSquare *getContents() const;

        GridSquareNode();


        void setG(double g) const;

        [[nodiscard]] double getG() const;

        [[nodiscard]] int getVersion() const;

        void setVersion(int version);

        void setRHS(double rhs) const;

        [[nodiscard]] double getRHS() const;

        bool setObstacle(bool obstacle) const;

        [[nodiscard]] bool isObstacle() const;

        [[nodiscard]] bool isSourceObstacle() const;

        bool setSourceObstacle(bool sourceObstacle) const;

        [[nodiscard]] int getX() const;

        void setX(int x);

        [[nodiscard]] int getY() const;

        void setY(int y);


        void updateKeys(double k1, double k2);

        [[nodiscard]] double getK1() const;

        [[nodiscard]] double getK2() const;

        std::pair<int, int> getCoords();

        //this will return true even if comparing itself
        /**
         * Determines whether another node is diagonal of this node
         * @param other the other node
         * @return whether the nodes are diagonal
         */
        bool isDiagonalOf(GridSquareNode* other) const;

        /**
         * gets the relative coordinates to another node from this node
         * @param other the other node
         * @return the relative coordinates to the other node
         */
        std::pair<int, int> getRelativeCoordinatesTo(const GridSquareNode* other) const;

        /**
         * Checks if a node is at the specified relative coordinates compared to this node
         * @param other the node to be checked whether it is at the relative coordinates
         * @param relativeCoords the relative coordinates
         * @return whether the other node is at the relative coordinates compared to this node
         */
        bool isAtRelativeCoordinatesFrom(const GridSquareNode* other, std::pair<int, int> relativeCoords) const;
    };



    /*
     * This class manages the grid which the traversal algorithm operates on
     */
    class NavigationGrid {
    private:
        const double MAP_SIZE_SCALE = 1.5;
        const int MAP_BUFFER = 20;
        int obstacleRadius;
        int obstacleLength;
        int size{};
        int entityX;
        int entityY;
        int endX;
        int endY;
        int offset;
        double km{}; //used for square key calculation
        std::vector<std::pair<int, int>> obstacleShape{}; //the shape of an obstacle given the provided padding radius
        std::vector<std::vector<GridSquare*>> grid{};



        /*can use vector, not set because there are no repeats because squares only added if they change from obstacle to
 *not obstacle*/
        /**
         * Sets a specific square's sourceObstacle to isObstacle and adds all changed squares to changedSquares and returns
         * @param x the square's x position
         * @param y the square's y position
         * @param isObstacle what the square's source obstacle status is set to
         * @param changedSquares the changed squares
         * @return a vector containing the changed squares
         ***/
        std::vector<std::pair<int, int>> setObstacle(int x, int y, bool isObstacle, std::vector<std::pair<int, int>>& changedSquares);

        /**
         * Determines the start position of the entity and the end goal position based on therelative position
         * between the entity and the goal
         * @param endRelativeX the x position of the end relative to the start
         * @param endRelativeY the y position of the end relative to the start
         */
        void initializeGridAndPositions(int endRelativeX, int endRelativeY);

        /**
        * Calculates the shape of a singular obstacle, given the provided obstacle radius
        ***/
        void getObstacleShape();

        /**
         * Sets the size of the grid
         * @param size the new size of the grid
         */
        void setSize(int size);

        /**
         * Calculates the size of the map after the given size is multiplied by the scale factor
         * @param size the current theoretical size of the map
         * @return the scaled size of the map
         */
        [[nodiscard]] int scaleSize(int size) const;

        /**
         * Calculates the size of the map after the current size is multiplied by the scale factor
         * @return the scaled size of the map
         */
        int scaleSize();


        /**
         * Checks whether attempted access is within the current size of the map. If not, the map expands.
         * @param x the x position of the attempted access
         * @param y the y position of the attempted access
         */
        void checkSize(int x, int y);

        /**
         * Checks whether attempted access is within the current size of the map. If not, the map expands.
         * @param furthest_accessed the highest coordinated value from the attempted access
         */
        void checkSize(int furthest_accessed);


        /**
         * Determines whether a corner neighbour is accessible from the middle square. True only if the squares at
         * middleX, cornerY and cornerX, middleY are not obstacles
         * @param middleX the x position of the middle square
         * @param middleY the y position of the middle square
         * @param cornerX the x position of the corner square
         * @param cornerY the y position of the corner square
         * @return whether the corner is reachable from the middle square
         */
        bool cornerVerifier(int middleX, int middleY, int cornerX, int cornerY);


        /**
         * expands the grid to the new size. Note that it does not create GridSquares for each space in the grid, as these
         * are lazily initialized in GetSquare
         * @param newSize the new size of the grid
         */
        void expandGrid(int newSize);

    public:
        NavigationGrid (int endRelativeX, int endRelativeY, int obstacleRadius = 1);

        ~NavigationGrid();

        /**
         * This method takes a vector of coordinates of squares whose source obstacle status will be set to isObstacle
         * and returns all squares whose obstacle status has changed
         * @param obstacles the coordinates of squares whose sourceObstacle will be set to is Obstacle
         * @param isObstacle  whether given squares are becoming or unbecoming source obstacles
         * @return a vector of coordinates (x, y int pairs) of squares that changed from !isObstacle to isObstacle
         *
         */
        std::vector<std::pair<int, int>> setObstacles(const std::vector<std::pair<int, int>> &obstacles, bool isObstacle);



        /**
         * returns the size of the grid
         * @return the size of the grid
         */
        [[nodiscard]] int getSize() const;

        /**
         * Prints out the entire grid
         */
        void coutGrid();

        /**
         * Updates the keys of a node using data from the grid
         * @param node the node whose keys are updated
         */
        void updateKeys(GridSquareNode* node) const;


        /**
         * Creates a node for the square at the given coordinates
         * @param x the x position of the node square
         * @param y the y position of the node square
         * @return a unique_ptr to the created node
         */
        std::unique_ptr<GridSquareNode> getNode(int x, int y);

        /**
         * Creates a node for the goal square
         * @return a unique_ptr to the created end node
         */
        std::unique_ptr<GridSquareNode> getEndNode();

        /**
         * Gives a reference to the end square
         * @return a reference to the end square
         */
        GridSquare& getEndSquare();

        /**
         * checks whether the end goal is at the given coordinates
         * @param coords the coordinates to be checked against the end goal coordinates
         * @return whether the given coordinates match the end coordinates
         */
        [[nodiscard]] bool isEnd(std::pair<int, int> coords) const;

        /**
         * checks whether the end goal is at the same position as the node
         * @param node the node whose position is compared to the end goal
         * @return whether the node is at the coordinates of the end goal
         */
        bool isEnd(GridSquareNode *node) const;

        /**
         * Gives a reference to the square where the entity currently is
         * @return a reference to the square where the entity currently is
         */
        GridSquare& getEntitySquare();


        /**
         * Gives a node for the square where the entity currently is
         * @return a unique_ptr to the created node
         */
        std::unique_ptr<GridSquareNode> getEntityNode();

        /**
         * Checks whether the keys of a node are smalled than the keys of the square which the entity is at
         * @param node the node to be compared
         * @return whether the key of the node is smalled that the square of the entity
         */
        bool lessThanEntitySquare(const GridSquareNode *node);


        /**
         * Gives an int pair of the coordinates of the entity
         * @return an int pair of the coordinates of the entity
         */
        std::pair<int, int> getEntityPosition();

        std::pair<int, int> getEndPosition();

        /**
         * Sets the position of the entity
         * @param x the new x position of the entity
         * @param y the new y position of the entity
         */
        void setEntityPosition(int x, int y);

        /**
         * Gives a reference to the square at the given coordinates. Lazily initializes squares
         * @param x the x position of the square
         * @param y the y position of the square
         * @return a reference to the square
         */
        GridSquare& getSquare(int x, int y);

        /**
         * Gives a reference to the square at the given coordinates. Lazily initializes squares
         * @param coords the pair of coordinates of the square
         * @return a reference to the square
         */
        GridSquare& getSquare(std::pair<int, int> coords);

        /**
         * Returns nodes of the squares neighboring the given coordinates. It only returns them if they are not obstacles,
         * and only returns diagonals if the squares at middleX, cornerY and cornerX, middleY are not obstacles
         * @param x the x position of the square
         * @param y the y position of the square
         * @return the coordinates of the valid neighbours of the square
         */
        std::vector<std::pair<std::unique_ptr<GridSquareNode>, double>> getNeighbours(int x, int y);
    };

    /**
     * A struct used to create a hash for a pair
     */
    struct pair_hash {
        /**
         * creates a hash for the given pair
         * @param p the pair to be hashed
         * @return the created hash
         */
        std::size_t operator()(const std::pair<int,int>& p) const;
    };

    /**
     * A struct used to compare GridSquareNodes
     */
    struct NodeCompare {
        /**
         * compares keys returning true if the key of a is larger than the key of b
         * @param a a unique_ptr to GridSquareNode a to have its keys compared
         * @param b a unique_ptr to GridSquareNode b to have its keys compared
         * @return true if node a has larger keys than node b
         */
        bool operator()(const std::unique_ptr<GridSquareNode>& a, const std::unique_ptr<GridSquareNode>& b) const;
    };


    /**
     * A class that manages the execution of the D* Lite algorithm
     */
    class DLiteManager {
    private:
        std::priority_queue<std::unique_ptr<GridSquareNode>, std::vector<std::unique_ptr<GridSquareNode>>, NodeCompare> pq;
        std::unordered_map<std::pair<int, int>, int, pair_hash> manifest;
        NavigationGrid grid;

        /**
         * puts a node pointer into the priority queue, setting it as the current node representing its square
         * @param node the node to be enqueued
         */
        void enqueue(std::unique_ptr<GridSquareNode> node);

        /**
         * Increments the current up to date node version for a specific square. Only the node for a specific square which
         * is the latest version is used. This means that incrementing essentially discards all current nodes representing
         * the same square as the given node in the priority queue.
         * The returned result can be used as the version value for a new entry into the priority queue
         * @param node the node whose square's latest version will be incremented
         * @return the incremented latest version for the node's square
         */
        int incrementNodeVersion(GridSquareNode* node);


        /**
         * Takes and pops the top element from the priority queue. If it is not the latest version, a new node is taken
         * until a valid node is found
         * @return the top most valid node in the priority queue
         */
        std::unique_ptr<GridSquareNode> dequeue();

        /**
         * Updates the vertex. If the node doesn't have equal G and RHS values, it is put into the priority queue, else,
         * all current nodes for the same square in the priority queue are discarded
         * @param node the node to be updated
         */
        void updateVertex(std::unique_ptr<GridSquareNode> node);

        /**
         * Empties the queue and clears the version manifest
         */
        void purgeQueueAndManifest();


        /**
         * Updates the rhs of a node's neighbours, with it becoming the minimum of its current value or the g value of the
         * given node + the cost to get to the given neighbour. The neighbour vertex is then updated.
         * @param node the node whose neighbours are to be updated
         */
        void updatePredecessors(std::unique_ptr<GridSquareNode> node);

        /**
         * recalculates the rhs values of a node's neighbours, using the minimum values from the neighbour's neighbours
         * @param node the node whose neighbours are recalculated
         * @param oldG node's previous g value (current is infinity)
         */
        void recalculatePredecessors(std::unique_ptr<GridSquareNode> node, double oldG);

        /**
         * Calculate's a nodes rhs value by checking its neighbours
         * @param node the node for which the rhs is calculated
         * @return the node's new rhs
         */
        double getMinRHS(GridSquareNode* node);


        /**
         * finds the neighbour which has the lowest g cost + cost to travel to from node
         * @param node the node for which the min neighbour is found
         * @param preferredRelativeCoords the relative coordinates which the neighbour is preferably at compared to node (
         * for tie breaks)
         * @return the preferred neighbour with the lowest g + cost to travel to from node
         */
        std::unique_ptr<GridSquareNode> getMinNeighbour(GridSquareNode* node, const std::pair<int, int> &preferredRelativeCoords = std::make_pair(0, 0));


        /**
         * Gets the min RHS from node's neighbours, and a vector of nodes with the equally lowest g + cost to travel to from
         * node
         * These are calculated together because they are essentially both byproducts of calculating the other
         * @param node the node whose neighbours are used
         * @return a pair containing the vector of min neighbours, and the min RHS
         */
        std::pair<std::vector<std::unique_ptr<GridSquareNode>>, double> getMinNeighbourAndRHS(GridSquareNode* node);

        /**
         * Find the next square in the quickest path
         * @param node the node of the current position
         * @param preferredRelativeCoords the preferred relative coordinates of the next square relative to node
         * @return a unique_ptr to the next square in the quickest path
         */
        std::unique_ptr<GridSquareNode> getNextSquare(GridSquareNode* node, const std::pair<int, int> &preferredRelativeCoords = std::make_pair(0, 0));


    public:

        DLiteManager(int endRelativeX, int endRelativeY, int obstacleRadius);


        bool isEnd(std::pair<int, int> coords);

        /**
         * Updates the grid to reflect the new entity positions and changes in obstacles. calculates the new shortest path
         * given the changes
         * @param entityX the new x position of the entity
         * @param entityY the new y position of the entity
         * @param newObstacles a vector of new obstacle coordinates
         * @param removedObstacles a vector of removed obstacle coordinates
         */
        void updateMap(int entityX, int entityY, std::vector<std::pair<int, int>> &newObstacles, std::vector<std::pair<int, int>> &removedObstacles);

        /**
         * updates the position of the entity within the grid
         * @param entityX the new entity x position
         * @param entityY the new entity y position
         */
        void updateEntityPosition(int entityX, int entityY);


        /**
         * Calculates the shortest path from entity to goal given the obstacles on the grid. Does not return the path
         */
        void calculateShortestPath();


        /**
         * Gets the next straight segment in the path
         * @param x the start x
         * @param y the start y
         * @param startAndFinishOnly whether only the start and finish positions of the segment are returned, or the whole segment
         * @param maxStraightLength the maximum length of the straight segment
         * @return the next straight segment in the path
         */
        std::vector<std::pair<int, int>> getNextStraight(int x, int y, bool startAndFinishOnly = false, double maxStraightLength = std::numeric_limits<double>::infinity());

        /**
         * Gets the current position of the entity
         * @return the position of the entity
         */
        std::pair<int, int> getEntityPosition();

        std::pair<int, int> getEndPosition();

        /**
         * Gets the whole path, from the entity to the goal
         * @return the whole path
         */
        std::vector<std::pair<int, int>> getWholePath();

        /**
         * Gets the whole path, from the given coordinates to the goal
         * @param x the x position of the start
         * @param y the y position of the start
         * @return the whole path from the given coordinates to the goal
         */
        std::vector<std::pair<int, int>> getWholePath(int x, int y);
    };


    class MovementVector {
    private:
        double angle = 0.0;
        double magnitude = 0.0;

        static double toRadians(double degrees);

        static double toDegrees(double radians);

        /**
         * converts any angle in degrees to one between - and positive 180
         * @param angle
         * @return
         */
        static double snipAngleTo180(double angle);

    public:
        MovementVector();

        MovementVector(double angle, double magnitude);
        /**
         * generates an angle and a magnitude based on a start and finish point
         * @param startCoordinate the start x and y values
         * @param finishCoordinate the finish x and y values
         * @param angleAdjustment the angle adjustment required to get from the entity's angle to the map's upward angle
         */
        MovementVector(std::pair<int, int> startCoordinate, std::pair<int, int> finishCoordinate, double angleAdjustment = 0);


        [[nodiscard]] double getAngle() const;

        [[nodiscard]] double getMagnitude() const;
    };

    /**
     * This class handles providing the entity with movements based on the path calculated by the pathfinding algorithm
     */
    class EntityMover {
    private:
        DLiteManager pathFinder;
        double maxStraightLength{};


    public:
        EntityMover(int endRelativeX, int endRelativeY, int obstacleRadius, double maxStraightLength = -1);

        std::pair<int, int> getEntityPosition();

        /**
         * Updates the map with the newly provided obstacles and calculates the shortest path
         * @param entityX
         * @param entityY
         * @param addedObstacles
         * @param removedObstacles
         */
        void updateMap(int entityX, int entityY, std::vector<std::pair<int, int>> &addedObstacles, std::vector<std::pair<int, int>> &removedObstacles);

        /**
         * gets the next movement for the entity
         * @param entityX the x position of the entity
         * @param entityY the y position of the entity
         * @param angleAdjustment the angle adjustment required to get from the entity's angle to the map's upward angle
         * @return a MovementVector of the next movement the entity should take
         */
        MovementVector getNextMovement(int entityX, int entityY, double angleAdjustment = 0);
    };
}
