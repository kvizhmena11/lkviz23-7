/******************************************************************************
 * File: Trailblazer.h
 *
 * Exports functions that use Dijkstra's algorithm, A* search, and Kruskal's
 * algorithm as specified in the assignment handout.
 */
#ifndef Trailblazer_Included
#define Trailblazer_Included
#include "TrailblazerTypes.h"
#include "set.h"
#include "grid.h"
#include "TrailblazerPQueue.h"

 /* Function: shortestPath
  *
  * Finds the shortest path between the locations given by start and end in the
  * specified world. The cost of moving from one edge to the next is specified
  * by the given cost function. The resulting path is then returned as a
  * Vector<Loc> containing the locations to visit in the order in which they
  * would be visited. If no path is found, this function should report an
  * error.
  *
  * Updated to include heuristic parameter for A* algorithm support.
  */
Vector<Loc>
shortestPath(Loc start,
    Loc end,
    Grid<double>& world,
    double costFn(Loc from, Loc to, Grid<double>& world),
    double heuristic(Loc start, Loc end, Grid<double>& world));

/* Function: createMaze
 *
 * Creates a maze of the specified dimensions using a randomized version of
 * Kruskal's algorithm, then returns a set of all of the edges in the maze.
 *
 * As specified in the assignment handout, the edges you should return here
 * represent the connections between locations in the graph that are passable.
 * Our provided starter code will then use these edges to build up a Grid
 * representation of the maze.
 */
Set<Edge> createMaze(int numRows, int numCols);

struct pathData {
    Loc ancestor;
    Color color;
    double lengthOfpath;
};

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

#endif