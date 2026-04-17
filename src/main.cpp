#include <iostream>
#include <vector>
#include <random>

#include "astar.h"

using namespace std;

vector<vector<int>> createGrid() {
    vector<vector<int>> grid(25, vector<int>(25, 1));

    // Obstacles (impassable terrain)
    for (int col = 0; col < 25; ++col) {
        if (col == 6 || col == 18) {
            continue; // keep two gaps for path
        }
        grid[12][col] = -1;
    }

    grid[5][5] = -1;
    grid[5][6] = -1;
    grid[7][2] = -1;
    grid[8][2] = -1;
    grid[9][2] = -1;
    grid[14][10] = -1;
    grid[15][10] = -1;
    grid[16][10] = -1;
    grid[17][10] = -1;
    grid[18][10] = -1;

    // Terrain cost variation
    for (int row = 0; row < 6; ++row) {
        grid[row][14] = 2; // grass
    }

    for (int row = 13; row < 22; ++row) {
        for (int col = 3; col < 10; ++col) {
            if (grid[row][col] != -1) {
                grid[row][col] = 5; // mountain
            }
        }
    }

    for (int col = 20; col < 25; ++col) {
        grid[10][col] = 2; // grass corridor
    }

    // Ensure the start and goal are passable
    grid[0][0] = 1;
    grid[24][24] = 1;

    return grid;
}

vector<vector<int>> createRandomGrid() {
    vector<vector<int>> grid(25, vector<int>(25, 1));

    // Random number generation
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> costDist(1, 5); // costs 1-5
    uniform_real_distribution<> obstacleDist(0.0, 1.0); // for obstacles

    for (int row = 0; row < 25; ++row) {
        for (int col = 0; col < 25; ++col) {
            // 15% chance of obstacle
            if (obstacleDist(gen) < 0.15) {
                grid[row][col] = -1;
            } else {
                grid[row][col] = costDist(gen);
            }
        }
    }

    // Ensure the start and goal are passable
    grid[0][0] = 1;
    grid[24][24] = 1;

    // Ensure path exists by clearing some areas around start and goal
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (r + 0 < 25 && c + 0 < 25) grid[r + 0][c + 0] = 1;
            if (24 - r >= 0 && 24 - c >= 0) grid[24 - r][24 - c] = 1;
        }
    }

    return grid;
}

void printGrid(const vector<vector<int>>& grid, const vector<astar::Position>& path) {
    vector<vector<char>> display(grid.size(), vector<char>(grid[0].size(), '.'));

    for (int row = 0; row < static_cast<int>(grid.size()); ++row) {
        for (int col = 0; col < static_cast<int>(grid[row].size()); ++col) {
            if (grid[row][col] < 0) {
                display[row][col] = '#';
            } else if (grid[row][col] == 2) {
                display[row][col] = 'g';
            } else if (grid[row][col] == 5) {
                display[row][col] = 'M';
            } else {
                display[row][col] = '.';
            }
        }
    }

    for (const auto& pos : path) {
        if ((pos.row == 0 && pos.col == 0) || (pos.row == 24 && pos.col == 24)) {
            continue;
        }
        display[pos.row][pos.col] = 'P';
    }

    display[0][0] = 'S';
    display[24][24] = 'G';

    for (int row = 0; row < static_cast<int>(display.size()); ++row) {
        for (int col = 0; col < static_cast<int>(display[row].size()); ++col) {
            cout << display[row][col] << ' ';
        }
        cout << '\n';
    }
}

int main() {
    cout << "Escolha o tipo de mapa:" << '\n';
    cout << "1 - Mapa deterministico (fixo)" << '\n';
    cout << "2 - Mapa randomico (aleatorio)" << '\n';
    cout << "Digite 1 ou 2: ";
    
    int choice;
    cin >> choice;
    
    vector<vector<int>> grid;
    if (choice == 1) {
        grid = createGrid();
        cout << "Usando mapa deterministico." << '\n';
    } else if (choice == 2) {
        grid = createRandomGrid();
        cout << "Usando mapa randomico." << '\n';
    } else {
        cout << "Opcao invalida. Usando mapa deterministico." << '\n';
        grid = createGrid();
    }
    
    astar::Solver solver(grid);
    astar::Position start{0, 0};
    astar::Position goal{24, 24};

    astar::PathResult result = solver.findPath(start, goal);
    if (!result.found) {
        cout << "Caminho não encontrado." << '\n';
        return 1;
    }

    cout << "Caminho encontrado!" << '\n';
    cout << "Custo total: " << result.totalCost << '\n';
    cout << "Numero de passos: " << result.path.size() - 1 << '\n';
    cout << "Mapa (S=start, G=chegada, P=caminho, #=obstaculo, g=grama, M=montanha):" << '\n';
    printGrid(grid, result.path);

    return 0;
}
