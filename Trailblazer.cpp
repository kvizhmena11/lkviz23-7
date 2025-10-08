/******************************************************************************
 * File: Trailblazer.cpp
 *
 * Implementation of the graph algorithms that comprise the Trailblazer
 * assignment.
 */

#include "Trailblazer.h"
#include "TrailblazerGraphics.h"
#include "TrailblazerTypes.h"
#include "TrailblazerPQueue.h"
#include "random.h"

using namespace std;

void launchStartNodes(Loc start, TrailblazerPQueue<Loc>& dataQueue,
    Grid<pathData>& info, Grid<double>& world, Loc end,
    double heuristic(Loc, Loc, Grid<double>&));

void neighbourHelper(Loc current, TrailblazerPQueue<Loc>& dataQueue,
    Loc neighbor, Grid<double>& world,
    Grid<pathData>& info,
    double costFn(Loc, Loc, Grid<double>&),
    Loc end,
    double heuristic(Loc, Loc, Grid<double>&));

Set<Edge> calculateEdges(int numRows, int numCols);
void calculateFinalEdges(TrailblazerPQueue<Edge>& MazeDataQueue, Vector<int>& comp,
    Set<Edge>& edgesOfMaze, int numCols, int totalCells);

/* Function: shortestPath
 *
 * Finds the shortest path between the locations given by start and end in the
 * specified world. The cost of moving from one edge to the next is specified
 * by the given cost function. The resulting path is then returned as a
 * Vector<Loc> containing the locations to visit in the order in which they
 * would be visited. If no path is found, this function should report an
 * error.
 *
 * Updated to support both Dijkstra's algorithm (when heuristic returns 0)
 * and A* algorithm (when heuristic provides estimates).
 */

 // just a constant for nonExistent location which i described as nonExist.
const Loc nonExist = { -1,-1 };

Vector<Loc>
shortestPath(Loc start,
    Loc end,
    Grid<double>& world,
    double costFn(Loc from, Loc to, Grid<double>& world),
    double heuristic(Loc start, Loc end, Grid<double>& world)) {

    int rows = world.numRows();
    int cols = world.numCols();
    // i just initialized route information for every little cell.
    Grid<pathData> info(rows, cols);

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            info[row][col].color = GRAY;
            info[row][col].ancestor = nonExist;
            info[row][col].lengthOfpath = numeric_limits<double>::infinity();
        }
    }

    // this is our dataQueue where we add starting nodes in the grid.
    TrailblazerPQueue<Loc> dataQueue;
    launchStartNodes(start, dataQueue, info, world, end, heuristic);

    // and here is the main while loop where the A* algorithm is coded.
    while (true) {
        if (dataQueue.isEmpty()) {
            break;
        }
        Loc currLoc = dataQueue.dequeueMin();
        pathData& currData = info[currLoc.row][currLoc.col];

        currData.color = GREEN;
        colorCell(world, currLoc, GREEN);

        if (currLoc.col == end.col && currLoc.row == end.row) {
            break;
        }

        // as we know one little point has 8 neighbors so we
        // check them all with this method.
        int diffR[] = { -1, -1, -1,  0, 0, 1, 1, 1 };
        int diffC[] = { -1,  0,  1, -1, 1,-1, 0, 1 };

        for (int k = 0; k < 8; k++) {
            Loc directionNeighbour = { currLoc.row + diffR[k], currLoc.col + diffC[k] };
            neighbourHelper(currLoc, dataQueue, directionNeighbour, world, info, costFn, end, heuristic);
        }
    }

    // and finally we construct the route from beginning to end.
    Vector<Loc> route;
    Loc currRoute = end;
    while (!(currRoute == start)) {
        route.insert(0, currRoute);
        Loc ancestor = info[currRoute.row][currRoute.col].ancestor;
        if (ancestor.row == -1 && ancestor.col == -1) {
            error("i could not find any path between these ranges.");
        }
        currRoute = ancestor;
    }
    route.insert(0, start);
    return route;
}

// with this method we can generate a random maze.
Set<Edge> createMaze(int numRows, int numCols) {
    // so first i calculated all of the edges in the grid.
    Set<Edge> finalEdges = calculateEdges(numRows, numCols);
    Set<Edge> edgesOfMaze;
    TrailblazerPQueue<Edge> MazeDataQueue;

    // and then i added these edges to MazeDataQueue with randomReal(0,1) weights.
    for (Edge currEdge : finalEdges) {
        double cost = randomReal(0, 1);
        MazeDataQueue.enqueue(currEdge, cost);
    }

    // after i initialized every little cells comp.
    Vector<int> comp(numCols * numRows);
    int k = 0;
    while (k < comp.size()) {
        comp[k] = k;
        k++;
    }

    // and lastly with this helper method i applied the algorithm to generate a maze.
    calculateFinalEdges(MazeDataQueue, comp, edgesOfMaze,
        numCols, numRows * numCols);
    return edgesOfMaze;
}

// as i mentioned earlier we initialize start nodes and then add to dataQueue.
void launchStartNodes(Loc start, TrailblazerPQueue<Loc>& dataQueue,
    Grid<pathData>& info, Grid<double>& world, Loc end,
    double heuristic(Loc, Loc, Grid<double>&)) {

    // here im using heuristic method.
    double heuristicValuable = heuristic(start, end, world);
    dataQueue.enqueue(start, 0 + heuristicValuable);
    info[start.row][start.col].lengthOfpath = 0;
    info[start.row][start.col].color = YELLOW;
    colorCell(world, start, YELLOW);
}

// this helper method helps me to process neighbor points in A* algorithm.
void neighbourHelper(Loc current, TrailblazerPQueue<Loc>& dataQueue,
    Loc neighbor, Grid<double>& world,
    Grid<pathData>& info,
    double costFn(Loc, Loc, Grid<double>&),
    Loc end,
    double heuristic(Loc, Loc, Grid<double>&)) {

    // if the algorithm goes out of bounds i tell the code to return.
    if (!world.inBounds(neighbor.row, neighbor.col)) return;

    auto& neighborData = info[neighbor.row][neighbor.col];
    auto& currentData = info[current.row][current.col];

    double costOfEdge = costFn(current, neighbor, world);
    double totalCost = costOfEdge + currentData.lengthOfpath;

    // again im using heuristic method.
    double heuristicValuable = heuristic(neighbor, end, world);
    double priorityValue = totalCost + heuristicValuable;

    bool Visited = neighborData.color != GRAY;
    bool hasBetterPath = (YELLOW == neighborData.color && totalCost < neighborData.lengthOfpath);

    // if there is a better route i tell the code to update it.
    if (hasBetterPath) {
        neighborData.ancestor = current;
        neighborData.lengthOfpath = totalCost;
        dataQueue.decreaseKey(neighbor, priorityValue);
    }
    // if not then we go back to that node.
    else if (!Visited) {
        neighborData.color = YELLOW;
        neighborData.ancestor = current;
        neighborData.lengthOfpath = totalCost;
        dataQueue.enqueue(neighbor, priorityValue);

        colorCell(world, neighbor, YELLOW);
    }
}

// with this code i calculated all of the edges.
Set<Edge> calculateEdges(int numRows, int numCols) {
    Set<Edge> total;
    for (int i = 0; i < numRows; i++) {
        for (int j = 0; j < numCols; j++) {
            if (i + 1 < numRows) {
                total.add(makeEdge(makeLoc(i, j), makeLoc(i + 1, j)));
            }
            if (j + 1 < numCols) {
                total.add(makeEdge(makeLoc(i, j), makeLoc(i, j + 1)));
            }
        }
    }
    return total;
}

//i wrote a code to merge edges and then calculate selected edge.
// we only merge those components which are different.
void calculateFinalEdges(TrailblazerPQueue<Edge>& MazeDataQueue, Vector<int>& comp,
    Set<Edge>& mazeEdges, int numCols, int totalCells) {
    while (!MazeDataQueue.isEmpty() && totalCells - 1 > mazeEdges.size()) {
        Edge currEdge = MazeDataQueue.dequeueMin();

        int endindx = currEdge.end.col + currEdge.end.row * numCols;
        int startIndx = currEdge.start.col + currEdge.start.row * numCols;

        int endComp = comp[endindx];
        int beginComp = comp[startIndx];

        bool isMergable = (beginComp != endComp);
        if (isMergable) {
            for (int k = 0; k < comp.size(); k++) {
                if (endComp == comp[k]) {
                    comp[k] = beginComp;
                }
            }
            mazeEdges.add(currEdge);
        }
    }
}