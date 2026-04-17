#include "astar.h"

#include <algorithm>
#include <limits>
#include <queue>

using namespace std;

namespace astar {

namespace {
    constexpr int INF = numeric_limits<int>::max() / 2;

    struct Node {
        Position pos;
        int g;
        int f;
    };

    struct Compare {
        bool operator()(const Node& a, const Node& b) const {
            if (a.f != b.f) {
                return a.f > b.f;
            }
            return a.g < b.g;
        }
    };
}

Solver::Solver(const vector<vector<int>>& grid)
    : numRows(static_cast<int>(grid.size())),
      numCols(numRows ? static_cast<int>(grid[0].size()) : 0),
      grid(grid) {}

bool Solver::isInBounds(const Position& pos) const {
    return pos.row >= 0 && pos.row < numRows && pos.col >= 0 && pos.col < numCols;
}

bool Solver::isPassable(const Position& pos) const {
    return isInBounds(pos) && grid[pos.row][pos.col] >= 0;
}

int Solver::terrainCost(const Position& pos) const {
    return grid[pos.row][pos.col];
}

int Solver::heuristic(const Position& a, const Position& b) const {
    return abs(a.row - b.row) + abs(a.col - b.col);
}

PathResult Solver::findPath(const Position& start, const Position& goal) const {
    PathResult result{false, 0, {}};
    if (!isPassable(start) || !isPassable(goal)) {
        return result;
    }

    vector<vector<int>> gScore(numRows, vector<int>(numCols, INF));
    vector<vector<bool>> closed(numRows, vector<bool>(numCols, false));
    vector<vector<Position>> parent(numRows, vector<Position>(numCols, Position{-1, -1}));

    priority_queue<Node, vector<Node>, Compare> open;
    gScore[start.row][start.col] = 0;
    open.push(Node{start, 0, heuristic(start, goal)});

    const vector<Position> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        if (closed[current.pos.row][current.pos.col]) {
            continue;
        }
        closed[current.pos.row][current.pos.col] = true;

        if (current.pos.row == goal.row && current.pos.col == goal.col) {
            result.found = true;
            result.totalCost = gScore[goal.row][goal.col];
            Position p = goal;
            while (!(p.row == start.row && p.col == start.col)) {
                result.path.push_back(p);
                p = parent[p.row][p.col];
            }
            result.path.push_back(start);
            reverse(result.path.begin(), result.path.end());
            return result;
        }

        for (const Position& direction : directions) {
            Position neighbor{current.pos.row + direction.row, current.pos.col + direction.col};
            if (!isPassable(neighbor) || closed[neighbor.row][neighbor.col]) {
                continue;
            }

            int tentativeG = gScore[current.pos.row][current.pos.col] + terrainCost(neighbor);
            if (tentativeG < gScore[neighbor.row][neighbor.col]) {
                gScore[neighbor.row][neighbor.col] = tentativeG;
                parent[neighbor.row][neighbor.col] = current.pos;
                int fScore = tentativeG + heuristic(neighbor, goal);
                open.push(Node{neighbor, tentativeG, fScore});
            }
        }
    }

    return result;
}

} // namespace astar
