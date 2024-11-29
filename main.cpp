/*
* SonarQube, open source software quality management tool.
 * Copyright (C) 2008-2013 SonarSource
 * http://github.com/SonarOpenCommunity/sonar-cxx
 *
 * SonarQube is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * SonarQube is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Instituto Tecnológico de Monterrey
 # Evidencia Integradora 2
 * Clase: Análisis y diseño de algoritmos avanzados (Gpo 604)
 * Profesor: Felipe Castillo Rendón
 * Fecha: 29 Noviembre 2024 
 *
 * Este código fue desarrollado como parte de una actividad académica.
 * Integrantes del equipo 3:
 * - Mauricio Lozano Zarate (A00833216)
 * - Aleksandra Stupiec (A00835071)
 * - Enrique Macías López (A01641402)
 * 
 * Este código está protegido por derechos de autor y solo debe ser usado
 * con fines educativos en el contexto del curso mencionado.
 */

#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <cmath>
#include <limits>
#include <set>
#include <numeric>

using namespace std;

// rep para Kruskal
struct Edge {
    int u, v, weight;
    bool operator<(const Edge &other) const {
        return weight < other.weight;
    }
};

// Kruskal
int find(vector<int> &parent, int i) {
    if (parent[i] == i)
        return i;
    return parent[i] = find(parent, parent[i]);
}

void union_sets(vector<int> &parent, vector<int> &rank, int u, int v) {
    int pu = find(parent, u), pv = find(parent, v);
    if (rank[pu] > rank[pv])
        parent[pv] = pu;
    else if (rank[pu] < rank[pv])
        parent[pu] = pv;
    else {
        parent[pv] = pu;
        rank[pu]++;
    }
}

vector<Edge> kruskal(int n, vector<vector<int>> &graph) {
    vector<Edge> edges, result;
    vector<int> parent(n), rank(n, 0);

    for (int i = 0; i < n; i++)
        parent[i] = i;

    // lista de aristas
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (graph[i][j] > 0) {
                edges.push_back({i, j, graph[i][j]});
            }
        }
    }

    sort(edges.begin(), edges.end());

    for (const auto &edge : edges) {
        if (find(parent, edge.u) != find(parent, edge.v)) {
            union_sets(parent, rank, edge.u, edge.v);
            result.push_back(edge);
        }
    }

    return result;
}

// Traveling Salesman
int tsp(int n, vector<vector<int>> &graph, vector<int> &path) {
    vector<int> nodes(n - 1);
    iota(nodes.begin(), nodes.end(), 1);

    int minCost = numeric_limits<int>::max();
    vector<int> bestPath;

    do {
        int cost = 0, prev = 0;
        for (int node : nodes) {
            cost += graph[prev][node];
            prev = node;
        }
        cost += graph[prev][0];
        if (cost < minCost) {
            minCost = cost;
            bestPath = nodes;
        }
    } while (next_permutation(nodes.begin(), nodes.end()));

    path = bestPath;
    path.insert(path.begin(), 0);
    path.push_back(0);
    return minCost;
}

// Ford-Fulkerson
bool bfs(vector<vector<int>> &residual, int source, int sink, vector<int> &parent) {
    int n = residual.size();
    vector<bool> visited(n, false);
    queue<int> q;

    q.push(source);
    visited[source] = true;
    parent[source] = -1;

    while (!q.empty()) {
        int u = q.front();
        q.pop();

        for (int v = 0; v < n; v++) {
            if (!visited[v] && residual[u][v] > 0) {
                q.push(v);
                parent[v] = u;
                visited[v] = true;
                if (v == sink) return true;
            }
        }
    }
    return false;
}

int ford_fulkerson(vector<vector<int>> &capacity, int source, int sink) {
    int n = capacity.size();
    vector<vector<int>> residual = capacity;
    vector<int> parent(n);
    int maxFlow = 0;

    while (bfs(residual, source, sink, parent)) {
        int pathFlow = numeric_limits<int>::max();
        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            pathFlow = min(pathFlow, residual[u][v]);
        }

        for (int v = sink; v != source; v = parent[v]) {
            int u = parent[v];
            residual[u][v] -= pathFlow;
            residual[v][u] += pathFlow;
        }

        maxFlow += pathFlow;
    }

    return maxFlow;
}

// entral más cercana
double distance(pair<int, int> a, pair<int, int> b) {
    return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
}

int nearest_central(pair<int, int> house, vector<pair<int, int>> &centrals) {
    int bestCentral = -1;
    double minDist = numeric_limits<double>::max();

    for (int i = 0; i < centrals.size(); i++) {
        double d = distance(house, centrals[i]);
        if (d < minDist) {
            minDist = d;
            bestCentral = i;
        }
    }

    return bestCentral;
}

int main() {
    int n;
    cout << "Ingrese el numero de las colonias: " << endl;
    cin >> n;

    vector<vector<int>> graph(n, vector<int>(n));
    vector<vector<int>> capacity(n, vector<int>(n));
    vector<pair<int, int>> locations(n);
    vector<pair<int, int>> centrals;

    cout << "Ingrese los valores del grafo, que representan las distancias entre las colonias de la ciudad: " << endl;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            cin >> graph[i][j];

    cout << "Ingrese los valores del que representan las capacidades maximas de flujo de datos entre colonia i y colonia j: " << endl;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            cin >> capacity[i][j];
    cout << "Ingrese pares ordenados (A,B), representando la ubicacion en un plano" << endl;
    for (int i = 0; i < n; i++)
        cin >> locations[i].first >> locations[i].second;

    int centralCount;
    cout << "Ingrese el numero de los centrales: " << endl;
    cin >> centralCount;

    cout << "Ingrese el valor de la ubicacion de los centrales (x, y): " << endl;
    for (int i = 0; i < centralCount; i++) {
        pair<int, int> central;
        cin >> central.first >> central.second;
        centrals.push_back(central);
    }

    // Kruskal
    auto edges = kruskal(n, graph);
    cout << "1.\n";
    for (const auto &edge : edges)
        cout << "(" << char('A' + edge.u) << ", " << char('A' + edge.v) << ")\n";

    // Traveling Salesman Problem
    vector<int> path;
    int minCost = tsp(n, graph, path);
    cout << "2.\n";
    for (int node : path)
        cout << char('A' + node) << " ";
    cout << endl;

    // Ford-Fulkerson
    int maxFlow = ford_fulkerson(capacity, 0, n - 1);
    cout << "3.\n" << maxFlow << endl;

    // Central más cercana
    cout << "4.\n";
    for (const auto &house : locations) {
        int nearest = nearest_central(house, centrals);
        cout << "(" << centrals[nearest].first << ", " << centrals[nearest].second << ")\n";
    }

    return 0;
}
