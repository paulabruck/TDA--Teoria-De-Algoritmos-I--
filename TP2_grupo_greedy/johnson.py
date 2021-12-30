from queue import PriorityQueue
import sys

class Graph:

  def __init__(self, num_of_vertices):
    self.v = num_of_vertices
    self.edges = {i:{j:float('inf') for j in range(num_of_vertices)} for i in range(num_of_vertices)}
    self.h = [float('inf') for i in range(num_of_vertices)] #h(u)

  def expand_graph(self, num_resize):
    aux_edges = self.edges
    aux_h = self.h
    aux_v = self.v
      
    self.edges = [[float('inf') for i in range(self.v + num_resize)] for j in range(self.v + num_resize)]
    self.h = [float('inf') for i in range(self.v + num_resize)]
    self.v += num_resize
      

    for i in range(aux_v):
      for j in range(aux_v):
        self.edges[i][j] = aux_edges[i][j]

      self.h[i] = aux_h[i]


  def add_edge(self, u, v, weight):
    self.edges[u][v] = weight


  # Complejidad 0((n+m)log(n)) En el loop ppal se hace un pop de la priority queue por cada 
  # vertice > O(nlogn). Ademas para cada arista se hace un put en la priority queue a lo sumo >
  # O(mlogn), luego O(nlogn) + O(mlogn) = O((n+m)logn). 
  def dijkstra(self, start_vertex):
    D = {v:float('inf') for v in range(self.v)} #diccionario con (num_vertice, valor_camino_min)
    D[start_vertex] = 0

    visited = []

    pq = PriorityQueue()
    pq.put((0, start_vertex))

    while not pq.empty(): # O(n) > para cada vertice
        (dist, current_vertex) = pq.get() # O(log n) > pop de priority queue 
        visited.append(current_vertex) # O(1) > push en lista

        for neighbor in range(self.v): # es para cada sucesor -> hay m en total 
            if self.edges[current_vertex][neighbor] != -1:
                distance = self.edges[current_vertex][neighbor] # O(1) > acceso a arreglo de 2 dim
                if neighbor not in visited:
                    old_cost = D[neighbor] # O(1) > acceso a diccionario
                    new_cost = D[current_vertex] + distance
                    if new_cost < old_cost:
                        pq.put((new_cost, neighbor)) # O(log n) > push de priority queue 
                        D[neighbor] = new_cost

    return D

def find_predecessors(graph, v):
  p = []
  for u in range(graph.v):
    if graph.edges[u][v] != float("Inf"):
      p.append(u)

  return p

def find_neighbours(graph, v):
  n = []
  for u in range(graph.v):
    if graph.edges[v][u] != float("Inf"):
      n.append(u)

  return n

def bellman_ford(graph, source):
  distance = {}
  for node in range(graph.v):
      distance[node] = float('inf')
  distance[source] = 0

  for _ in range(graph.v-1):
      for node in range(graph.v):
          n = find_neighbours(graph, node)
          for neighbour in n:
              if distance[neighbour] > distance[node] + graph.edges[node][neighbour]:
                  distance[neighbour] = distance[node] + graph.edges[node][neighbour]
                  
              if graph.h[neighbour] > distance[node] + graph.edges[node][neighbour]:
                graph.h[neighbour] = distance[node] + graph.edges[node][neighbour]

  #Check for negative weight cycles
  for node in range(graph.v):
      n = find_neighbours(graph, node)
      for neighbour in n:
          assert distance[neighbour] <= distance[node] + graph.edges[node][neighbour], "Negative weight cycle."

  return distance

def new_g(graph):
  ng = Graph(graph.v+1)

  for u in range(graph.v):
    for v in range(graph.v):
      if graph.edges[u][v] != float("Inf"):
        ng.add_edge(u,v,graph.edges[u][v])


  for v in range(graph.v):
    ng.add_edge(graph.v, v, 0)

  return ng


def update_g(graph_extended):
  graph = Graph(graph_extended.v-1)

  for u in range(graph.v):
    for v in range(graph.v):
      if(graph_extended.edges[u][v] != float("Inf")):
        graph.add_edge(u,v,graph_extended.edges[u][v])

    graph.h[u] = graph_extended.h[u]

  return graph

def johnson(g):
  ng = new_g(g)
  
  s = ng.v

  for v in range(ng.v):
    bellman_ford(ng,v)

  for u in range(s):
    for v in range(s):
      ng.edges[u][v] = ng.edges[u][v] + ng.h[u] - ng.h[v]

  g = update_g(ng)

  D = [[0 for i in range(g.v)] for j in range(g.v)]

  for u in range(g.v):
    aux = g.dijkstra(u)
    for v in aux:
      D[u][v] = aux[v] + g.h[v] - g.h[u]

  return D

def location(graph):
  D = johnson(graph)
  min = float("Inf")
  register = 0

  for u in range(graph.v):
    cost = 0
    for v in range(graph.v):
      cost += D[u][v]

    if min > cost:
      min = cost
      register = u

  
  return register, cost #(ciudad donde ubicar la fábrica, costo)

def add_deposits(line, deposits_dict, graph):
  deposits = line.split(",")
  weight = int(deposits[2].split('\n')[0])
  if deposits[0] not in deposits_dict:
    deposits_dict[deposits[0]] = graph.v
    graph.expand_graph(1)

  if deposits[1] not in deposits_dict:
    deposits_dict[deposits[1]] = graph.v
    graph.expand_graph(1)

  graph.add_edge(deposits_dict[deposits[0]], deposits_dict[deposits[1]], weight)

def location_with_johnson(path):
  f = open(path, "r")
  line = 1
  deposits_dict = {}
  graph = Graph(0)
  while(line):
    line = f.readline()
    if line:
      add_deposits(line, deposits_dict, graph)
  
  f.close()

  deposit, cost = location(graph)

  name_deposit = [k for k in deposits_dict if deposits_dict[k]==deposit][0]

  return name_deposit, cost


deposit,cost = location_with_johnson(sys.argv[1])
print('Deposit number: ', deposit)
print('Cost: ', cost)