#include "astar.h"

#include <algorithm>
#include <limits>
#include <queue>

using namespace std;

namespace astar {

    // namespace anonimo. É visivel apenas dentro desse arquivo
namespace {
    constexpr int INF = numeric_limits<int>::max() / 2;

    struct Node {
        Position pos;
        int g; //g(n)
        int f; //prioridade
    };

    // Define a prioridade da fila
    struct Compare {
        bool operator()(const Node& a, const Node& b) const {
            if (a.f != b.f) {
                return a.f > b.f;
            }
            // escolhe o maior g(n)
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

// retorna o custo de entrada do nó
int Solver::terrainCost(const Position& pos) const {
    return grid[pos.row][pos.col];
}

// Distancia manhattan h(n) = |x - xfinal| + |y - yfinal|
int Solver::heuristic(const Position& a, const Position& b) const {
    return abs(a.row - b.row) + abs(a.col - b.col);
}

PathResult Solver::findPath(const Position& start, const Position& goal) const {
    PathResult result{false, 0, {}}; // inicializa a struct
    // verifica se inicio e fim são validos
    if (!isPassable(start) || !isPassable(goal)) {
        return result;
    }

    // custo minimo conhecido até cada célula
    vector<vector<int>> gScore(numRows, vector<int>(numCols, INF));
    // Nós ja processados
    vector<vector<bool>> closed(numRows, vector<bool>(numCols, false));
    // Guarda de que célula você veio
    vector<vector<Position>> parent(numRows, vector<Position>(numCols, Position{-1, -1}));
    // Guarda o menor f(prioridade)
    priority_queue<Node, vector<Node>, Compare> open;

    // faz começar no nó inicial
    gScore[start.row][start.col] = 0;
    open.push(Node{start, 0, heuristic(start, goal)});

    const vector<Position> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    while (!open.empty()) {
        // pega o melhor nó f(maior prioridade)
        Node current = open.top();
        open.pop(); 

        // ignora se o nó ja foi visitado
        if (closed[current.pos.row][current.pos.col]) {
            continue;
        }
        closed[current.pos.row][current.pos.col] = true;

        // verifica se chegou no objetivo.
        if (current.pos.row == goal.row && current.pos.col == goal.col) {
            result.found = true;
            result.totalCost = gScore[goal.row][goal.col];
            Position p = goal;
            // reconstrução do caminho
            while (!(p.row == start.row && p.col == start.col)) {
                result.path.push_back(p);
                p = parent[p.row][p.col];
            }
            result.path.push_back(start); //coloca o start no vetor
            reverse(result.path.begin(), result.path.end()); //inverte o vetor
            return result;
        }

        // espanção dos vizinhos
        for (const Position& direction : directions) {
            // cia vizinho
            Position vizinho{
                current.pos.row + direction.row,
                current.pos.col + direction.col
            };

            if (!isPassable(vizinho) || closed[vizinho.row][vizinho.col]) {
                continue;
            }

            // calculo de custo g(n)
            int tentativeG = gScore[current.pos.row][current.pos.col] + terrainCost(vizinho);

            // verifica se achou um caminho melhor até o vizinho
            if (tentativeG < gScore[vizinho.row][vizinho.col]) {
                // salva o melhor custo e de onde veio
                gScore[vizinho.row][vizinho.col] = tentativeG;
                parent[vizinho.row][vizinho.col] = current.pos;
                // calculo do f(n) = g(n) + h(n)
                int fScore = tentativeG + heuristic(vizinho, goal);

                open.push(Node{vizinho, tentativeG, fScore}); // insere na fila
            }
        }
    }

    return result;
}

}