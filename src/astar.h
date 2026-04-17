#ifndef ASTAR_H
#define ASTAR_H

#include <vector>

using namespace std;

namespace astar {

struct Position {
    int row;
    int col;
};

struct PathResult {
    bool found;
    int totalCost;
    vector<Position> path;
};

class Solver {
public:
    Solver(const vector<vector<int>>& grid);
    PathResult findPath(const Position& start, const Position& goal) const;

private:
    int numRows;
    int numCols;
    vector<vector<int>> grid;

    bool isInBounds(const Position& pos) const;
    bool isPassable(const Position& pos) const;
    int terrainCost(const Position& pos) const;
    int heuristic(const Position& a, const Position& b) const;
};

} // namespace astar

#endif // ASTAR_H

