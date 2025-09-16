#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <vector>
# include <utility>
#include <hash_map>
#include <unordered_map>
#include "main.h"
// #include <pybind11/pybind11.h>
// namespace py = pybind11;

namespace GoDog {

    /*true if key a smaller than key b*/
    bool compareKeys(const double aK1, const double aK2, const double bK1, const double bK2) {
        if (epsilonEquality(aK1, bK1, 0.01)) {
            return aK2 < bK2;
        } return aK1 < bK1;
    }

    /*
     * Returns true if the difference between values is less than the epsilon
     */
    bool epsilonEquality(const double val1, const double val2, const double epsilon) {
        return std::abs(val1 - val2) < epsilon;
    }


    /*
     * This class contains necessary information required for a D* Lite traversal algorith
     * These are intended to serve as squares which form a grid on which the algorithm operates
     */



    GridSquare::GridSquare() {
        g = std::numeric_limits<double>::infinity();
        rhs = std::numeric_limits<double>::infinity();
        obstacle = 0;
        sourceObstacle = false;
    }

    GridSquare& GridSquare::operator=(const GridSquare& other) {
        if (this != &other) {
            this->g = other.g;
            this->rhs = other.rhs;
            this->obstacle = other.obstacle;
        }
        return *this;
    }

    void GridSquare::setG(double g) {
        this->g = g;
    }

    [[nodiscard]] double GridSquare::getG() const {
        return g;
    }

    void GridSquare::setRHS(double rhs) {
        this->rhs = rhs;
    }

    [[nodiscard]] double GridSquare::getRHS() const {
        return rhs;
    }

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

    bool GridSquare::setObstacle(const bool setObstacle) {
        const bool oldObstacle = isObstacle();
        if (setObstacle) {
            obstacle ++;
        } else {
            obstacle = std::max(0, obstacle - 1);
        }
        return oldObstacle != isObstacle();  //true if state changed
    }

    [[nodiscard]] bool GridSquare::isObstacle() const {
        return obstacle > 0;
    }

    [[nodiscard]] bool GridSquare::isSourceObstacle() const {
        return sourceObstacle;
    }

    bool GridSquare::setSourceObstacle(const bool setSourceObstacle) {
        if (setSourceObstacle == sourceObstacle) {return false;}
        this->sourceObstacle = setSourceObstacle;
        return setObstacle(setSourceObstacle);
    }



/*
 * This class acts as a wrapper object for a GridSquare.
 * It holds features that are lost when the GridSquare is removed from the grid, such as its coordinates.
 * It also holds data necessary for use in the D* Lite traversal algorithm such as keys and a version used within
 * the priority queue
 */

    GridSquareNode::GridSquareNode(GridSquare *square, const int x, const int y) : x(x), y(y), square(square) {}

    [[nodiscard]] GridSquare* GridSquareNode::getContents() const {
        return square;
    }

    GridSquareNode::GridSquareNode() {
        x = 0;
        y = 0;
        square = nullptr;
    }


    void GridSquareNode::setG(const double g) const {
        square->setG(g);
    }
    [[nodiscard]] double GridSquareNode::getG() const {
        return square->getG();
    }

    [[nodiscard]] int GridSquareNode::getVersion() const {
        return version;
    }

    void GridSquareNode::setVersion(int version) {
        this->version = version;
    }

    void GridSquareNode::setRHS(const double rhs) const {
        square->setRHS(rhs);
    }

    [[nodiscard]] double GridSquareNode::getRHS() const {
        return square->getRHS();
    }

    bool GridSquareNode::setObstacle(const bool obstacle) const {
        return square->setObstacle(obstacle);
    }

    [[nodiscard]] bool GridSquareNode::isObstacle() const {
        return square->isObstacle();
    }

    [[nodiscard]] bool GridSquareNode::isSourceObstacle() const {
        return square->isSourceObstacle();
    }

    bool GridSquareNode::setSourceObstacle(const bool sourceObstacle) const {
        return square->setSourceObstacle(sourceObstacle);
    }

    [[nodiscard]] int GridSquareNode::getX() const {
        return x;
    }
    void GridSquareNode::setX(int x) {
        this->x = x;
    }
    [[nodiscard]] int GridSquareNode::getY() const {
        return y;
    }
    void GridSquareNode::setY(int y) {
        this->y = y;
    }


    void GridSquareNode::updateKeys(double k1, double k2) {
        this->k1 = k1;
        this->k2 = k2;
    }

    [[nodiscard]] double GridSquareNode::getK1() const {  //formulate for calculating key to compare priority in pq
        return k1;
    }

    [[nodiscard]] double GridSquareNode::getK2() const {
        return k2;
    }

    std::pair<int, int> GridSquareNode::getCoords() {
        return {x, y};
    }

    //this will return true even if comparing itself
    /**
     * Determines whether another node is diagonal of this node
     * @param other the other node
     * @return whether the nodes are diagonal
     */
    bool GridSquareNode::isDiagonalOf(GridSquareNode* other) const {
        int offsetX = this->getX() - other->getX();
        int offsetY = this->getY() - other->getY();
        return abs(offsetX) == abs(offsetY);
    }

    /**
     * gets the relative coordinates to another node from this node
     * @param other the other node
     * @return the relative coordinates to the other node
     */
    std::pair<int, int> GridSquareNode::getRelativeCoordinatesTo(const GridSquareNode* other) const {
        return {other->getX() - getX(),other->getY() - getY()};
    }

    /**
     * Checks if a node is at the specified relative coordinates compared to this node
     * @param other the node to be checked whether it is at the relative coordinates
     * @param relativeCoords the relative coordinates
     * @return whether the other node is at the relative coordinates compared to this node
     */
    bool GridSquareNode::isAtRelativeCoordinatesFrom(const GridSquareNode* other, std::pair<int, int> relativeCoords) const {
        return other->getX() + relativeCoords.first == getX() && other->getY() + relativeCoords.second == getY();
    }



/*
 * This class manages the grid which the traversal algorithm operates on
 */





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
    std::vector<std::pair<int, int>> NavigationGrid::setObstacle(int x, int y, bool isObstacle, std::vector<std::pair<int, int>>& changedSquares) {
        int startX = x - obstacleRadius;
        int startY = y - obstacleRadius;
        if (getSquare(x, y).isSourceObstacle() == isObstacle) {
            return changedSquares;
        }
        if (getSquare(x, y).setSourceObstacle(isObstacle)) {
            changedSquares.emplace_back(x, y);
        }
        else {
        }
        for (int i{}; i < obstacleShape.size(); i++) {
            if (getSquare(obstacleShape[i].first + startX, obstacleShape[i].second + startY).setObstacle(isObstacle)) { //this is updating the obstacle status
                changedSquares.emplace_back(obstacleShape[i].first + startX, obstacleShape[i].second + startY);
            }
        }
        return changedSquares;
    }

    /**
     * Determines the start position of the entity and the end goal position based on the starting difference
     * between the entity and the goal
     * @param endRelativeX the x difference between the starting position of the entity and the goal
     * @param endRelativeY the y difference between the starting position of the entity and the goal
     */
    void NavigationGrid::initializeGridAndPositions(int endRelativeX, int endRelativeY) {
        grid = std::vector<std::vector<GridSquare*>>(getSize(), std::vector<GridSquare*>(size));
        int xOdd = (endRelativeX) % 2;
        if (endRelativeX < 0) {
            xOdd *= -1;
        }//this will be 1 if the diff is odd, cause can't be split nicely, so must add remainder to end pos
        int yOdd = (endRelativeY) % 2;
        if (endRelativeY < 0) {
            yOdd *= -1;
        }
        int xDev = endRelativeX / 2;
        int yDev = endRelativeY / 2;
        entityX = -1 * xDev;
        entityY = -1 * yDev;
        endX = xDev + xOdd;
        endY = yDev + yOdd;
        offset = size / 2;

    }

    /**
    * Calculates the shape of a singular obstacle, given the provided obstacle radius
    ***/
    void NavigationGrid::getObstacleShape() {
        obstacleShape.reserve(obstacleLength * obstacleLength);
        const int squaredRadius = obstacleRadius * obstacleRadius;
        for (int i{}; i < obstacleLength; ++i) {
            for (int j{}; j < obstacleLength; ++j) {
                if (i == obstacleRadius && j == obstacleRadius) {continue;}
                const int xDif = obstacleRadius - i;
                const int yDif = obstacleRadius - j;
                if (xDif * xDif + yDif * yDif < squaredRadius) {
                    obstacleShape.emplace_back(i, j);
                }
            }
        }
        obstacleShape.shrink_to_fit();
    }

    /**
     * Sets the size of the grid
     * @param size the new size of the grid
     */
    void NavigationGrid::setSize(int size) {
        this->size = size;
    }

    /**
     * Calculates the size of the map after the given size is multiplied by the scale factor
     * @param size the current theoretical size of the map
     * @return the scaled size of the map
     */
    [[nodiscard]] int NavigationGrid::scaleSize(int size) const {
        int newSize = static_cast<int>(size * MAP_SIZE_SCALE);
        if ((newSize % 2) == 0) {
            newSize += 1;
        }
        return newSize;
    }

    /**
     * Calculates the size of the map after the current size is multiplied by the scale factor
     * @return the scaled size of the map
     */
    int NavigationGrid::scaleSize() {
        return scaleSize(getSize());
    }


    /**
     * Checks whether attempted access is within the current size of the map. If not, the map expands.
     * @param x the x position of the attempted access
     * @param y the y position of the attempted access
     */
    void NavigationGrid::checkSize(int x, int y) {
        checkSize(std::max(abs(x), abs(y)));
    }

    /**
     * Checks whether attempted access is within the current size of the map. If not, the map expands.
     * @param furthest_accessed the highest coordinated value from the attempted access
     */
    void NavigationGrid::checkSize(int furthest_accessed) { //possibly slightly inefficient
        furthest_accessed = abs(furthest_accessed);
        int gridSize = getSize();
        while (furthest_accessed + MAP_BUFFER > gridSize / 2) {
            gridSize = scaleSize(gridSize);
        }
        if (gridSize > getSize()) {
            expandGrid(gridSize);
        }
    }


    /**
     * Checks that a new obstacle is
     * @param obstaclePosition
     * @return
     */
    bool NavigationGrid::safeObstacle(const std::pair<int, int> &obstaclePosition) const {
    int relativeEntX = obstaclePosition.first - entityX;
    int relativeEntY = obstaclePosition.second - entityY;
    int relativeEndX = obstaclePosition.first - endX;
    int relativeEndY = obstaclePosition.second - endY;
    double entDistance = relativeEntX * relativeEntX + relativeEntY * relativeEntY;
    double endDistance = relativeEndX * relativeEndX + relativeEndY * relativeEndY;
    int acceptableDistance = obstacleRadius * obstacleRadius + 1;
    return (entDistance > acceptableDistance && endDistance > acceptableDistance);
}


    /**
     * Determines whether a corner neighbour is accessible from the middle square. True only if the squares at
     * middleX, cornerY and cornerX, middleY are not obstacles
     * @param middleX the x position of the middle square
     * @param middleY the y position of the middle square
     * @param cornerX the x position of the corner square
     * @param cornerY the y position of the corner square
     * @return whether the corner is reachable from the middle square
     */
    bool NavigationGrid::cornerVerifier(const int middleX, const int middleY, const int cornerX, const int cornerY) {
        return !getSquare(middleX, cornerY).isObstacle() && !getSquare(cornerX, middleY).isObstacle();
    }


    /**
     * expands the grid to the new size. Note that it does not create GridSquares for each space in the grid, as these
     * are lazily initialized in GetSquare
     * @param newSize the new size of the grid
     */
    void NavigationGrid::expandGrid(int newSize) {
        if (newSize <= getSize()) {return;}
        int expansionPerSide = (newSize - getSize()) / 2;
        for (int i = 0; i < getSize(); i++) {
            std::vector<GridSquare*> frontRows(expansionPerSide);
            std::vector<GridSquare*> backRows(expansionPerSide);
            grid[i].reserve(getSize() + expansionPerSide * 2);
            grid[i].insert(grid[i].begin(), frontRows.begin(), frontRows.end());
            grid[i].insert(grid[i].end(), backRows.begin(), backRows.end());
        }
        std::vector<std::vector<GridSquare*>> frontColumns(expansionPerSide, std::vector<GridSquare*>(newSize));
        std::vector<std::vector<GridSquare*>> backColumns(expansionPerSide, std::vector<GridSquare*>(newSize));
        grid.reserve(getSize() + expansionPerSide * 2);
        grid.insert(grid.begin(), frontColumns.begin(), frontColumns.end());
        grid.insert(grid.end(), backColumns.begin(), backColumns.end());
        setSize(newSize);
        offset = getSize() / 2;
    }

    NavigationGrid::NavigationGrid (int endRelativeX, int endRelativeY, int obstacleRadius) {
        size = std::max(std::max(endRelativeX, endRelativeY), 10);
        size = scaleSize();
        this->obstacleRadius = obstacleRadius + 1; // radius always seems to be one less than it should be
        this->obstacleLength = this->obstacleRadius * 2 + 1;
        initializeGridAndPositions(endRelativeX, endRelativeY);
        getObstacleShape();
    }

    NavigationGrid::~NavigationGrid() {
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                delete grid[i][j];
            }
        }
    }

    /**
     * This method takes a vector of coordinates of squares whose source obstacle status will be set to isObstacle
     * and returns all squares whose obstacle status has changed
     * @param obstacles the coordinates of squares whose sourceObstacle will be set to is Obstacle
     * @param isObstacle  whether given squares are becoming or unbecoming source obstacles
     * @return a vector of coordinates (x, y int pairs) of squares that changed from !isObstacle to isObstacle
     *
     */
    std::vector<std::pair<int, int>> NavigationGrid::setObstacles(const std::vector<std::pair<int, int>> &obstacles, bool isObstacle) {
        std::vector<std::pair<int, int>> changedSquares{};
        for (int i{}; i < obstacles.size(); i++) {
            if (safeObstacle(obstacles[i])) {
                setObstacle(obstacles[i].first, obstacles[i].second, isObstacle, changedSquares);
            } else {
                std::cout << "unsafe obstacle ignored" << std::endl;
            }
        }
        return changedSquares;
    }




    /**
     * returns the size of the grid
     * @return the size of the grid
     */
    [[nodiscard]] int NavigationGrid::getSize() const {
        return size;
    }

    /**
     * Prints out the entire grid
     */
    void NavigationGrid::coutGrid() const {
        for (int i{getSize() - 1}; i >= 0; i--) {
            for (int j{}; j < getSize(); j++) {
                GridSquare* temp = grid[j][i];
                if (j - offset == entityX && i - offset == entityY) {
                    std::cout << "X";
                } else if (j - offset == endX && i - offset == endY) {
                    std::cout << "!";
                }
                else if (temp == nullptr) {
                    std::cout << 0;
                } else {
                    std::cout << grid[j][i]->isObstacle();
                }
            }
            std::cout << std::endl;
        }
    }


    /**
     * Updates the keys of a node using data from the grid
     * @param node the node whose keys are updated
     */
    void NavigationGrid::updateKeys(GridSquareNode* node) const {
        const double k2 = std::min(node->getG(), node->getRHS());
        const double k1 = k2 + sqrt((node->getX() - entityX)*(node->getX() - entityX) + (node->getY() - entityY) * (node->getY() - entityY)) + km;
        node->updateKeys(k1, k2);

    }

    /**
     * Creates a node for the square at the given coordinates
     * @param x the x position of the node square
     * @param y the y position of the node square
     * @return a unique_ptr to the created node
     */
    std::unique_ptr<GridSquareNode> NavigationGrid::getNode(const int x, const int y) {
        return std::make_unique<GridSquareNode>(&getSquare(x, y), x, y);
    }

    /**
     * Creates a node for the goal square
     * @return a unique_ptr to the created end node
     */
    std::unique_ptr<GridSquareNode> NavigationGrid::getEndNode() {
        return getNode(endX, endY);
    }

    /**
     * Gives a reference to the end square
     * @return a reference to the end square
     */
    GridSquare& NavigationGrid::getEndSquare() {
        return getSquare(endX, endY);
    }

    /**
     * checks whether the end goal is at the given coordinates
     * @param coords the coordinates to be checked against the end goal coordinates
     * @return whether the given coordinates match the end coordinates
     */
    [[nodiscard]] bool NavigationGrid::isEnd(std::pair<int, int> coords) const {
        return coords.first == endX && coords.second == endY;
    }

    /**
     * checks whether the end goal is at the same position as the node
     * @param node the node whose position is compared to the end goal
     * @return whether the node is at the coordinates of the end goal
     */
    bool NavigationGrid::isEnd(GridSquareNode *node) const {
        return node->getX() == endX && node->getY() == endY;
    }

    /**
     * Gives a reference to the square where the entity currently is
     * @return a reference to the square where the entity currently is
     */
    GridSquare& NavigationGrid::getEntitySquare() {
        return getSquare(entityX, entityY);
    }


    /**
     * Gives a node for the square where the entity currently is
     * @return a unique_ptr to the created node
     */
    std::unique_ptr<GridSquareNode> NavigationGrid::getEntityNode() {
        return getNode(entityX, entityY);
    }

    /**
     * Checks whether the keys of a node are smalled than the keys of the square which the entity is at
     * @param node the node to be compared
     * @return whether the key of the node is smalled that the square of the entity
     */
    bool NavigationGrid::lessThanEntitySquare(const GridSquareNode *node) {
        const double entityK2 = std::min(getEntitySquare().getG(), getEntitySquare().getRHS());
        const double entityK1 = entityK2 + km;
        if (entityK1 + entityK2 == std::numeric_limits<double>::infinity() && node->getK1() + node->getK2() == std::numeric_limits<double>::infinity()) {
            return true;
        }
        // std::cout << "entity keys:" << entityK1 << " " << entityK2 << std::endl;
        // std::cout << "node keys:" << node->getK1() << " " << node->getK2() << std::endl;
        // std::cout << compareKeys(node->getK1(), node->getK2(), entityK1, entityK2) << std::endl;
        return compareKeys(node->getK1(), node->getK2(), entityK1, entityK2);
    }


    /**
     * Gives an int pair of the coordinates of the entity
     * @return an int pair of the coordinates of the entity
     */
    std::pair<int, int> NavigationGrid::getEntityPosition() {
        return {entityX, entityY};
    }

    std::pair<int, int> NavigationGrid::getEndPosition() {
        return {endX, endY};
    }

    /**
     * Sets the position of the entity
     * @param x the new x position of the entity
     * @param y the new y position of the entity
     */
    void NavigationGrid::setEntityPosition(int x, int y) {
        km += sqrt(pow(x - entityX, 2) + pow(y - entityY, 2)); //distance between new and position added
        entityX = x;
        entityY = y;
    }

    /**
     * Gives a reference to the square at the given coordinates. Lazily initializes squares
     * @param x the x position of the square
     * @param y the y position of the square
     * @return a reference to the square
     */
    GridSquare& NavigationGrid::getSquare(int x, int y) {
        checkSize(x, y);
        const int xIndex = x + offset;
        const int yIndex = y + offset;
        if (grid[xIndex][yIndex] == nullptr) { //lazy initialization
            grid[xIndex][yIndex] = new GridSquare();
        }
        return *grid[xIndex][yIndex];
    }

    /**
     * Gives a reference to the square at the given coordinates. Lazily initializes squares
     * @param coords the pair of coordinates of the square
     * @return a reference to the square
     */
    GridSquare& NavigationGrid::getSquare(std::pair<int, int> coords) {
        return getSquare(coords.first, coords.second);
    }

    /**
     * Returns nodes of the squares neighboring the given coordinates. It only returns them if they are not obstacles,
     * and only returns diagonals if the squares at middleX, cornerY and cornerX, middleY are not obstacles
     * @param x the x position of the square
     * @param y the y position of the square
     * @return the coordinates of the valid neighbours of the square
     */
    std::vector<std::pair<std::unique_ptr<GridSquareNode>, double>> NavigationGrid::getNeighbours(const int x, const int y) {
        std::vector<std::pair<std::unique_ptr<GridSquareNode>, double>> neighbours;
        neighbours.reserve(8);
        for (int i{x - 1}; i <= x + 1; ++i) {
            for (int j{y - 1}; j <= y + 1; ++j) {
                if (getSquare(i, j).isObstacle() || (i == x && j == y)) {
                    continue;
                }
                if (i != x && j != y) {
                    if (cornerVerifier(x, y, i, j)) {
                        neighbours.emplace_back(getNode(i, j), DIAGONAL_TRAVERSAL_COST);
                    }
                } else {
                    neighbours.emplace_back(getNode(i, j), STRAIGHT_TRAVERSAL_COST);
                }
            }
        }
        return neighbours;
    }


    /**
     * creates a hash for the given pair
     * @param p the pair to be hashed
     * @return the created hash
     */
    std::size_t pair_hash::operator()(const std::pair<int,int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }

/**
 * A struct used to compare GridSquareNodes
 */
    /**
     * compares keys returning true if the key of a is larger than the key of b
     * @param a a unique_ptr to GridSquareNode a to have its keys compared
     * @param b a unique_ptr to GridSquareNode b to have its keys compared
     * @return true if node a has larger keys than node b
     */
    bool NodeCompare::operator()(const std::unique_ptr<GridSquareNode>& a, const std::unique_ptr<GridSquareNode>& b) const {
        return !compareKeys(a->getK1(), a->getK2(), b->getK1(), b->getK2());
    }


/**
 * A class that manages the execution of the D* Lite algorithm
 */


    /**
     * puts a node pointer into the priority queue, setting it as the current node representing its square
     * @param node the node to be enqueued
     */
    void DLiteManager::enqueue(std::unique_ptr<GridSquareNode> node) {
        node->setVersion(incrementNodeVersion(node.get()));
        grid.updateKeys(node.get());
        pq.push(std::move(node));  //not sure if node needs to be constant
    }

    /**
     * Increments the current up to date node version for a specific square. Only the node for a specific square which
     * is the latest version is used. This means that incrementing essentially discards all current nodes representing
     * the same square as the given node in the priority queue.
     * The returned result can be used as the version value for a new entry into the priority queue
     * @param node the node whose square's latest version will be incremented
     * @return the incremented latest version for the node's square
     */
    int DLiteManager::incrementNodeVersion(GridSquareNode* node) { // also works to essentially remove a node from pq, as it will be discarded when taken from top
        const std::pair<int, int> coordinates = node->getCoords();
        if (manifest.contains(coordinates)) {
            manifest[coordinates]++;
        } else {
            manifest[coordinates] = 0;
        }
        return manifest[coordinates];
    }


    /**
     * Takes and pops the top element from the priority queue. If it is not the latest version, a new node is taken
     * until a valid node is found
     * @return the top most valid node in the priority queue
     */
    std::unique_ptr<GridSquareNode> DLiteManager::dequeue() {
        while (!pq.empty()) {
            auto node = std::move(const_cast<std::unique_ptr<GridSquareNode>&>(pq.top()));
            pq.pop();
            if (node->getVersion() == manifest[node->getCoords()]) {
                return node;
            }
        }
        return {nullptr};
    }

    /**
     * Updates the vertex. If the node doesn't have equal G and RHS values, it is put into the priority queue, else,
     * all current nodes for the same square in the priority queue are discarded
     * @param node the node to be updated
     */
    void DLiteManager::updateVertex(std::unique_ptr<GridSquareNode> node) {
        if (!epsilonEquality(node->getG(), node->getRHS(), 0.001)) {
            enqueue(std::move(node));
        } else {
            incrementNodeVersion(node.get());
        }
    }

    /**
     * Empties the queue and clears the version manifest
     */
    void DLiteManager::purgeQueueAndManifest() {
        while (!pq.empty()) {
            pq.pop();
        }
        manifest.clear();
    }


    /**
     * Updates the rhs of a node's neighbours, with it becoming the minimum of its current value or the g value of the
     * given node + the cost to get to the given neighbour. The neighbour vertex is then updated.
     * @param node the node whose neighbours are to be updated
     */
    void DLiteManager::updatePredecessors(std::unique_ptr<GridSquareNode> node) {
        auto neighbours = grid.getNeighbours(node->getX(), node->getY());
        for (int i{}; i < neighbours.size(); i++) {
            if (neighbours[i].first->getContents() == &grid.getEndSquare()) {
                continue;
            }
            neighbours[i].first->setRHS(std::min(neighbours[i].first->getRHS(), neighbours[i].second + node->getG()));
            updateVertex(std::move(neighbours[i].first));
        }

    }

    /**
     * recalculates the rhs values of a node's neighbours, using the minimum values from the neighbour's neighbours
     * @param node the node whose neighbours are recalculated
     * @param oldG node's previous g value (current is infinity)
     */
    void DLiteManager::recalculatePredecessors(std::unique_ptr<GridSquareNode> node, double oldG) {
        auto neighbours = grid.getNeighbours(node->getX(), node->getY());
        neighbours.emplace_back(std::move(node), 0);
        for (int i{}; i < neighbours.size(); i++) {
            if (neighbours[i].first->getContents() == &grid.getEndSquare()) {continue;}
            if (epsilonEquality(neighbours[i].first->getRHS(), neighbours[i].second + oldG, 0.01)) {
                neighbours[i].first->setRHS(getMinRHS(neighbours[i].first.get()));
            }
            updateVertex(std::move(neighbours[i].first));
        }
    }

    /**
     * Calculate's a nodes rhs value by checking its neighbours
     * @param node the node for which the rhs is calculated
     * @return the node's new rhs
     */
    double DLiteManager::getMinRHS(GridSquareNode* node) {
        auto valuePair= getMinNeighbourAndRHS(node);
        return valuePair.second;
    }


    /**
     * finds the neighbour which has the lowest g cost + cost to travel to from node
     * @param node the node for which the min neighbour is found
     * @param preferredRelativeCoords the relative coordinates which the neighbour is preferably at compared to node (
     * for tie breaks)
     * @return the preferred neighbour with the lowest g + cost to travel to from node
     */
    std::unique_ptr<GridSquareNode> DLiteManager::getMinNeighbour(GridSquareNode* node, const std::pair<int, int> &preferredRelativeCoords) {
        auto valuePair = getMinNeighbourAndRHS(node);
        if (valuePair.first.empty()) {
            return nullptr;
        }
        unsigned long long cheapestIndex = valuePair.first.size() - 1;
        for (int i{}; i < valuePair.first.size() - 1; i++) {
            if (valuePair.first[i]->isAtRelativeCoordinatesFrom(node, preferredRelativeCoords)) {
                cheapestIndex = i;
                break;
            }
        }
        return std::move(valuePair.first[cheapestIndex]);
    }


    /**
     * Gets the min RHS from node's neighbours, and a vector of nodes with the equally lowest g + cost to travel to from
     * node
     * These are calculated together because they are essentially both byproducts of calculating the other
     * @param node the node whose neighbours are used
     * @return a pair containing the vector of min neighbours, and the min RHS
     */
    std::pair<std::vector<std::unique_ptr<GridSquareNode>>, double> DLiteManager::getMinNeighbourAndRHS(GridSquareNode* node) {
        std::vector<std::pair<std::unique_ptr<GridSquareNode>, double>> neighbours = grid.getNeighbours(node->getX(), node->getY());
        double minRHS{std::numeric_limits<double>::infinity()};
        std::vector<std::unique_ptr<GridSquareNode>> cheapest{};
        for (int i{}; i < neighbours.size(); i++) {
            double neighbourValue = neighbours[i].first->getG() + neighbours[i].second;
            if (minRHS > neighbourValue) {
                minRHS = neighbourValue;
                cheapest.clear();
                cheapest.push_back(std::move(neighbours[i].first));
            } else if (epsilonEquality(minRHS, neighbourValue, 0.01)) {
                cheapest.push_back(std::move(neighbours[i].first));
            }
        }
        return {std::move(cheapest), minRHS};
    }

    /**
     * Find the next square in the quickest path
     * @param node the node of the current position
     * @param preferredRelativeCoords the preferred relative coordinates of the next square relative to node
     * @return a unique_ptr to the next square in the quickest path
     */
    std::unique_ptr<GridSquareNode> DLiteManager::getNextSquare(GridSquareNode* node, const std::pair<int, int> &preferredRelativeCoords) {
        auto cheapNeighbour = getMinNeighbour(node, preferredRelativeCoords);
        if (cheapNeighbour == nullptr) {
            throw std::runtime_error("No Path Exists");
        }
        return cheapNeighbour;
    }



    DLiteManager::DLiteManager(int endRelativeX, int endRelativeY, int obstacleRadius) : grid(endRelativeX, endRelativeY, obstacleRadius) {
        std::unique_ptr<GridSquareNode> endNode = grid.getEndNode();
        endNode->setRHS(0);
        enqueue(std::move(endNode));
    }


    bool DLiteManager::isEnd(std::pair<int, int> coords) {
        return grid.isEnd(coords);
    }

    /**
     * Updates the grid to reflect the new entity positions and changes in obstacles. calculates the new shortest path
     * given the changes
     * @param entityX the new x position of the entity
     * @param entityY the new y position of the entity
     * @param newObstacles a vector of new obstacle coordinates
     * @param removedObstacles a vector of removed obstacle coordinates
     */
    void DLiteManager::updateMap(int entityX, int entityY, std::vector<std::pair<int, int>> &newObstacles, std::vector<std::pair<int, int>> &removedObstacles) {
        grid.setEntityPosition(entityX, entityY);
        const auto blockedSquares = grid.setObstacles(newObstacles, true);
        const auto clearedSquares = grid.setObstacles(removedObstacles, false);
        const std::vector<std::pair<int, int>> *squares[] = {&blockedSquares, &clearedSquares};
        for (int i{}; i < 2; i++) {
            for (int j{}; j < squares[i]->size(); j++) {
                if (grid.isEnd((*squares[i])[j])) {continue;}
                auto changedNode = grid.getNode((*squares[i])[j].first, (*squares[i])[j].second);
                changedNode->setRHS(getMinRHS(changedNode.get()));
                updateVertex(std::move(changedNode));
            }
        }
        calculateShortestPath();
    }

    /**
     * updates the position of the entity within the grid, and recalculates the route if the current position is not
     * part of the path
     * @param entityX the new entity x position
     * @param entityY the new entity y position
     */
    void DLiteManager::updateEntityPosition(const int entityX, const int entityY) {
        grid.setEntityPosition(entityX, entityY);
        std::unique_ptr<GridSquareNode> newEntityNode = grid.getNode(entityX, entityY);
        if (newEntityNode->getG() == std::numeric_limits<double>::infinity()) {
            newEntityNode->setRHS(getMinRHS(newEntityNode.get()));
            const auto node = grid.getEndNode();
            node->setG(std::numeric_limits<double>::infinity());
            enqueue(std::move(grid.getEndNode()));
            calculateShortestPath();
        }
    }


    /**
     * Calculates the shortest path from entity to goal given the obstacles on the grid. Does not return the path
     */
    void DLiteManager::calculateShortestPath() {
        while (!pq.empty() && (grid.lessThanEntitySquare(pq.top().get()) || grid.getEntitySquare().getRHS() > grid.getEntitySquare().getG())) {
            std::unique_ptr<GridSquareNode> node = dequeue();
            double oldK1 = node->getK1();
            double oldK2 = node->getK2();
            grid.updateKeys(node.get());
            //old key is smaller
            if (compareKeys(oldK1, oldK2, node->getK1(), node->getK2())) {
                enqueue(std::move(node));
            }
            else if (node->getG() > node->getRHS()) {
                node->setG(node->getRHS());
                updatePredecessors(std::move(node));
            }
            else if (node->getG() < node->getRHS()) {
                const double oldG = node->getG();
                node->setG(std::numeric_limits<double>::infinity());
                recalculatePredecessors(std::move(node), oldG);
            }
        }
        purgeQueueAndManifest();

    }


    /**
     * Gets the next straight segment in the path
     * @param x the start x
     * @param y the start y
     * @param startAndFinishOnly whether only the start and finish positions of the segment are returned, or the whole segment
     * @param maxStraightLength the maximum length of the straight segment
     * @return the next straight segment in the path
     */
    std::vector<std::pair<int, int>> DLiteManager::getNextStraight(int x, int y, const bool startAndFinishOnly, const double maxStraightLength) {
        auto node = grid.getNode(x, y);
        std::vector<std::pair<int, int>> path{};
        path.emplace_back(x, y);
        auto firstSquare = getNextSquare(node.get());
        std::pair<int, int> relativeCoordinates = node->getRelativeCoordinatesTo(firstSquare.get());
        double traversalCost{};
        if (node->isDiagonalOf(firstSquare.get())) {
            traversalCost = DIAGONAL_TRAVERSAL_COST;
        } else {
            traversalCost = STRAIGHT_TRAVERSAL_COST;
        }
        double traversalTotal = traversalCost;
        node = std::move(firstSquare);
        while (traversalTotal < maxStraightLength) {
            if (grid.isEnd(node.get())) {
                break;
            }
            auto nextSquare = getNextSquare(node.get(), relativeCoordinates);
            if (!nextSquare->isAtRelativeCoordinatesFrom(node.get(), relativeCoordinates)) {
                break;
            }
            if (!startAndFinishOnly) {
                path.push_back(node->getCoords());
            }
            node = std::move(nextSquare);
            traversalTotal += traversalCost;
        }
        path.push_back(node->getCoords());
        return path;
    }

    /**
     * Gets the current position of the entity
     * @return the position of the entity
     */
    std::pair<int, int> DLiteManager::getEntityPosition() {
        return grid.getEntityPosition();
    }

    std::pair<int, int> DLiteManager::getEndPosition() {
        return grid.getEndPosition();
    }

    /**
     * Gets the whole path, from the entity to the goal
     * @return the whole path
     */
    std::vector<std::pair<int, int>> DLiteManager::getWholePath() {
        auto entityCoords = grid.getEntityPosition();
        auto result = getWholePath(entityCoords.first, entityCoords.second);
        return result;
    }

    /**
     * Gets the whole path, from the given coordinates to the goal
     * @param x the x position of the start
     * @param y the y position of the start
     * @return the whole path from the given coordinates to the goal
     */
    std::vector<std::pair<int, int>> DLiteManager::getWholePath(int x, int y) {
        std::unique_ptr<GridSquareNode> node = grid.getNode(x, y);
        std::vector<std::pair<int, int>> path{};
        std::pair<int, int> relativeCoordinates = std::make_pair(0, 0);
        while (!grid.isEnd(node.get())) {
            path.push_back(node->getCoords());
            auto nextSquare = getNextSquare(node.get(), relativeCoordinates);
            relativeCoordinates = node->getRelativeCoordinatesTo(nextSquare.get());
            node = std::move(nextSquare);
        }
        path.push_back(node->getCoords());
        return path;
    }

    void DLiteManager::coutGrid() {
        grid.coutGrid();
    }




    double MovementVector::toRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }

    double MovementVector::toDegrees(double radians) {
        return radians * 180.0 / M_PI;
    }

    /**
     * converts any angle in degrees to one between - and positive 180
     * @param angle
     * @return
     */
    double MovementVector::snipAngleTo180(double angle) {
        if (angle >= -180 && angle <= 180) {
            return angle;
        }
        int multiplier{};
        if (angle < 0) {
            multiplier = -1;
            angle = -angle;
        }
        const int intAngle = static_cast<int>(angle);
        const double decimals = angle - intAngle;
        angle = intAngle % 360 + decimals;
        if (angle > 180) {
            angle = -1 * (180 - (angle - 180));
        }
        return angle * multiplier;
    }

    MovementVector::MovementVector() {
        angle = 0.0;
        magnitude = 0.0;
    }

    MovementVector::MovementVector(const double angle, const double magnitude) {
        this->angle = angle;
        this->magnitude = magnitude;
    }
    /**
     * generates an angle and a magnitude based on a start and finish point
     * @param startCoordinate the start x and y values
     * @param finishCoordinate the finish x and y values
     * @param angleAdjustment the angle adjustment required to get from the entity's angle to the map's upward angle
     */
    MovementVector::MovementVector(std::pair<int, int> startCoordinate, std::pair<int, int> finishCoordinate, double angleAdjustment) {
        double relativeFinishX = finishCoordinate.first - startCoordinate.first;
        double relativeFinishY = finishCoordinate.second - startCoordinate.second;
        if (relativeFinishX == 0) {
            if (relativeFinishY >= 0) {
                angle = 0;
            } else {
                angle = 180;
            }
            magnitude = std::abs(relativeFinishY);
            return;
        }
        if (relativeFinishY == 0) {
            if (relativeFinishX >= 0) {
                angle = -90;
            } else {
                angle = 90;
            }
            magnitude = std::abs(relativeFinishX);
            return;
        }
        int multiplier{1};
        if (relativeFinishX > 0) {
            multiplier = -1;
        }
        bool yIsNegative = relativeFinishY < 0;
        relativeFinishX = std::abs(relativeFinishX);
        relativeFinishY = std::abs(relativeFinishY);


        double calculatedAngle = multiplier * toDegrees(std::atan(relativeFinishX/relativeFinishY));
        if (yIsNegative) {
            calculatedAngle = multiplier * 180 - calculatedAngle;
        }
        magnitude = sqrt(relativeFinishX * relativeFinishX + relativeFinishY * relativeFinishY);
        angle = snipAngleTo180(calculatedAngle + angleAdjustment);
    }


    [[nodiscard]] double MovementVector::getAngle() const {
        return angle;
    }

    [[nodiscard]] double MovementVector::getMagnitude() const {
        return magnitude;
    }

    /**
     * This class handles providing the entity with movements based on the path calculated by the pathfinding algorithm
     */


    EntityMover::EntityMover(int endRelativeX, int endRelativeY, int obstacleRadius, double maxStraightLength): pathFinder(endRelativeX, endRelativeY, obstacleRadius) {
        if (maxStraightLength < 2) {
            maxStraightLength = std::numeric_limits<double>::infinity();
        }
        this->maxStraightLength = maxStraightLength;
        pathFinder.calculateShortestPath();
    }

    std::pair<int, int> EntityMover::getEntityPosition() {
        return pathFinder.getEntityPosition();
    }

    /**
     * Updates the map with the newly provided obstacles and calculates the shortest path
     * @param entityX
     * @param entityY
     * @param addedObstacles
     * @param removedObstacles
     */
    void EntityMover::updateMap(int entityX, int entityY, std::vector<std::pair<int, int>> &addedObstacles, std::vector<std::pair<int, int>> &removedObstacles) {
        pathFinder.updateMap(entityX, entityY, addedObstacles, removedObstacles);
    }

    /**
     * gets the next movement for the entity
     * @param entityX the x position of the entity
     * @param entityY the y position of the entity
     * @param angleAdjustment the angle adjustment required to get from the entity's angle to the map's upward angle
     * @return a MovementVector of the next movement the entity should take
     */
    MovementVector EntityMover::getNextMovement(int entityX, int entityY, double angleAdjustment) {
        pathFinder.updateEntityPosition(entityX, entityY);
        std::vector<std::pair<int, int>> path = pathFinder.getNextStraight(entityX, entityY, true, maxStraightLength);
        return {path[0], path[1], angleAdjustment};
    }

    void EntityMover::coutGrid() {
        pathFinder.coutGrid();
    }

}

// int main () {
//     GoDog::EntityMover mov(6, 6, 5);
//     std::pair<int, int> position = mov.getEntityPosition();
//     GoDog::MovementVector movement = mov.getNextMovement(position.first, position.second);
//     std::cout << movement.getMagnitude() << movement.getAngle() << std::endl;
//     std::vector<std::pair<int, int>> added_obstacles{std::make_pair<int, int>(20, 20), std::make_pair<int, int>(0, 1)};
//     std::vector<std::pair<int, int>> removed_obstacles{};
//     mov.updateMap(position.first, position.second, added_obstacles, removed_obstacles);
//     mov.coutGrid();
//     std::cout << position.first << " " << position.second << std::endl;
//     movement = mov.getNextMovement(position.first, position.second);
//     std::cout << movement.getMagnitude() << movement.getAngle() << std::endl;
//     movement = mov.getNextMovement(30, -3);
//     mov.coutGrid();
//     std::cout << movement.getMagnitude() << movement.getAngle() << std::endl;
// }