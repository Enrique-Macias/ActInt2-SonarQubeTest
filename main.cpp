/*
* SonarQube, open source software quality management tool.
 * Copyright (C) 2008-2013 SonarSource
 * http://github.com/SonarOpenCommunity/sonar-cxx
 *
 * SonarQube es software libre; puedes redistribuirlo y/o
 * modificarlo bajo los términos de la Licencia Pública General Reducida de GNU
 * como se publica por la Free Software Foundation; ya sea
 * la versión 3 de la Licencia, o (a tu elección) cualquier versión posterior.
 *
 * SonarQube se distribuye con la esperanza de que sea útil,
 * pero SIN NINGUNA GARANTÍA; incluso sin la garantía implícita de
 * COMERCIABILIDAD o IDONEIDAD PARA UN PROPÓSITO PARTICULAR. Consulte la
 * Licencia Pública General Reducida de GNU para más detalles.
 *
 * Instituto Tecnológico de Monterrey
 # Evidencia Integradora 2
 * Clase: Análisis y diseño de algoritmos avanzados (Gpo 604)
 * Profesor: Felipe Castillo Rendón
 * Fecha: 1 Diciembre 2024
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

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

using namespace std;

// Definimos las colonias como letras para facilitar la identificación
char get_colony_name(int index) { return 'A' + index; }

// Estructura para representar una arista en el grafo
struct Edge {
  int origen;
  int destino;
  int peso;
  bool operator<(const Edge &e) const { return peso < e.peso; }
};

// Estructura para la búsqueda de conjuntos disjuntos (Union-find)
struct DisjointSets {
  vector<int> parent, rank;
  DisjointSets(int n) {
    parent.resize(n);
    rank.resize(n);
    for (int i = 0; i < n; i++)
      parent[i] = i;
  }
  int find(int u) {
    if (u != parent[u])
      parent[u] = find(parent[u]);
    return parent[u];
  }
  void merge(int x, int y) {
    x = find(x), y = find(y);
    if (rank[x] > rank[y])
      parent[y] = x;
    else
      parent[x] = y;
    if (rank[x] == rank[y])
      rank[y]++;
  }
};

// Función para implementar el algoritmo de Kruskal y encontrar el árbol mínimo
// de expansión
vector<Edge> kruskal_mst(int n, const vector<vector<int>> &grafo) {
  vector<Edge> edges;
  // Convertimos la matriz de adyacencia en una lista de aristas
  for (int i = 0; i < n; i++) {
    for (int j = i + 1; j < n; j++) {
      if (grafo[i][j] != 0) {
        edges.push_back({i, j, grafo[i][j]});
      }
    }
  }
  // Ordenamos las aristas por peso
  sort(edges.begin(), edges.end());
  DisjointSets ds(n);
  vector<Edge> result;
  for (auto &edge : edges) {
    int u_set = ds.find(edge.origen);
    int v_set = ds.find(edge.destino);
    if (u_set != v_set) {
      result.push_back(edge);
      ds.merge(u_set, v_set);
    }
  }
  return result;
}

// Función para implementar el algoritmo TSP (Utilizamos fuerza bruta ya que N
// es pequeño)
int tsp(int n, const vector<vector<int>> &grafo, vector<int> &ruta_optima) {
  vector<int> vertices;
  for (int i = 1; i < n; i++)
    vertices.push_back(i);
  int min_path = numeric_limits<int>::max();
  do {
    int current_weight = 0;
    int k = 0;
    for (int i = 0; i < vertices.size(); i++) {
      current_weight += grafo[k][vertices[i]];
      k = vertices[i];
    }
    current_weight += grafo[k][0];
    if (current_weight < min_path) {
      min_path = current_weight;
      ruta_optima = vertices;
    }
  } while (next_permutation(vertices.begin(), vertices.end()));
  return min_path;
}

// Función para encontrar el flujo máximo utilizando el algoritmo de
// Ford-Fulkerson
bool bfs(const vector<vector<int>> &rGraph, int s, int t, vector<int> &parent) {
  int n = rGraph.size();
  vector<bool> visited(n, false);
  queue<int> q;
  q.push(s);
  visited[s] = true;
  parent[s] = -1;
  while (!q.empty()) {
    int u = q.front();
    q.pop();
    for (int v = 0; v < n; v++) {
      if (!visited[v] && rGraph[u][v] > 0) {
        if (v == t) {
          parent[v] = u;
          return true;
        }
        q.push(v);
        parent[v] = u;
        visited[v] = true;
      }
    }
  }
  return false;
}

int ford_fulkerson(vector<vector<int>> &graph, int s, int t) {
  int u, v;
  int n = graph.size();
  vector<vector<int>> rGraph = graph;
  vector<int> parent(n);
  int max_flow = 0;
  while (bfs(rGraph, s, t, parent)) {
    int path_flow = numeric_limits<int>::max();
    for (v = t; v != s; v = parent[v]) {
      u = parent[v];
      path_flow = min(path_flow, rGraph[u][v]);
    }
    for (v = t; v != s; v = parent[v]) {
      u = parent[v];
      rGraph[u][v] -= path_flow;
      rGraph[v][u] += path_flow;
    }
    max_flow += path_flow;
  }
  return max_flow;
}

// Función para realizar la búsqueda lineal y encontrar la central más cercana
pair<int, int> buscar_central_mas_cercana(const vector<pair<int, int>> &centrales,
                                       pair<int, int> nuevaUbicacion) {
  double distanciaMinima = numeric_limits<double>::max();
  pair<int, int> centralMasCercana;
  for (const auto &central : centrales) {
    double distancia = sqrt(pow(central.first - nuevaUbicacion.first, 2) +
                            pow(central.second - nuevaUbicacion.second, 2));
    if (distancia < distanciaMinima) {
      distanciaMinima = distancia;
      centralMasCercana = central;
    }
  }
  return centralMasCercana;
}

int main() {
  int N;
  cin >> N;
  // Lectura de la matriz de distancias
  vector<vector<int>> grafo(N, vector<int>(N));
  for (int i = 0; i < N; i++)
    for (int j = 0; j < N; j++)
      cin >> grafo[i][j];
  // Lectura de la matriz de capacidades
  vector<vector<int>> capacidades(N, vector<int>(N));
  for (int i = 0; i < N; i++)
    for (int j = 0; j < N; j++)
      cin >> capacidades[i][j];
  // Lectura de las ubicaciones de las centrales y la nueva contratación
  vector<pair<int, int>> centrales;
  pair<int, int> nuevaUbicacion;
  string line;
  getline(cin, line); // Limpiar el buffer

  // Vamos a leer N+1 líneas de coordenadas
  int numCoordenadas = N + 1;
  for (int i = 0; i < numCoordenadas; i++) {
    getline(cin, line);
    if (line.empty()) {
      i--; // Ignorar líneas vacías
      continue;
    }
    int x, y;
    sscanf(line.c_str(), "(%d,%d)", &x, &y);
    if (i < numCoordenadas - 1) {
      centrales.push_back({x, y});
    } else {
      nuevaUbicacion = {x, y};
    }
  }

  // Punto 1: Kruskal's Algorithm
  vector<Edge> mst = kruskal_mst(N, grafo);
  cout << "1.\n";
  for (auto &edge : mst) {
    cout << "(" << get_colony_name(edge.origen) << ", "
         << get_colony_name(edge.destino) << ")\n";
  }
  // Punto 2: Traveling Salesman Problem
  vector<int> ruta_optima;
  tsp(N, grafo, ruta_optima);
  cout << "2.\n";
  cout << get_colony_name(0) << " ";
  for (auto &v : ruta_optima)
    cout << get_colony_name(v) << " ";
  cout << get_colony_name(0) << "\n";
  // Punto 3: Ford-Fulkerson Algorithm
  int max_flow = ford_fulkerson(capacidades, 0, N - 1);
  cout << "3.\n" << max_flow << "\n";
  // Punto 4: Búsqueda lineal
  pair<int, int> centralCercana =
      buscar_central_mas_cercana(centrales, nuevaUbicacion);
  cout << "4.\n";
  cout << "(" << centralCercana.first << ", " << centralCercana.second << ")\n";
  return 0;
}
