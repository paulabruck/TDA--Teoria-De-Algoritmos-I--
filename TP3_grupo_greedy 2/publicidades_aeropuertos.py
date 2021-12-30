import sys

class Node:
  def __init__(self, key, edges=None):
    self.key = key
    if edges: self.edges = edges
    else: self.edges = []

  #Se agrega un eje hacia adelante con la capacidad
  #brindada, y uno hacia atrás con capacidad 0
  def add_edge(self, node_destiny, capacity):
    edge = Edge(self, node_destiny, capacity, False)
    self.edges.append(edge)

    backward_edge = Edge(node_destiny, self, 0, True)
    node_destiny.edges.append(backward_edge)
    
  #Obtiene todos los nodos con los que tiene conexión el nodo actual
  def connected_nodes(self):
    nodes = []
    for edge in self.edges:
      if(not edge.is_saturated):
        nodes.append(edge.destiny)

    return nodes

  def has_relation_with(self, node, include_cero=False):
    for edge in self.edges:
      if((not edge.is_saturated) and (edge.destiny.key == node.key)):
        return True

      if (edge.destiny.key == node.key) and (edge.capacity > 0):
        return True

    return False

  def obtain_edge(self, key_node):
    for edge in self.edges:
      if edge.destiny.key == key_node:
        return edge

    return None

class Edge:
  def __init__(self, origin, destiny, capacity, is_backward):
    self.origin = origin
    self.destiny = destiny
    self.capacity = capacity
    self.is_saturated = False
    self.is_backward = is_backward
    self.publicity = False

  def add_publicity(self):
    self.publicity = True

  def aument_foward_edge(self, bottleneck):
    self.capacity -= bottleneck
    if(self.capacity <= 0):
      self.capacity = 0
      self.is_saturated = True

  def aument_backward_edge(self, bottleneck):
    self.capacity += bottleneck

  def obtain_backward_edge(self):
    return self.destiny.obtain_edge(self.origin.key)

  def obtain_publicity_edge(self):
    if(not self.is_backward):
      new_edge = Edge(self.origin, self.destiny, 1, False)
    else:
      new_edge = Edge(self.origin, self.destiny, 0, True)
    
    return new_edge

class AumentPath:
  def __init__(self, graph, nodes_keys):
    self.edges = graph.generate_aument_path(nodes_keys)

  def bottleneck(self):
    min = self.edges[0].capacity
    for edge in self.edges:
      if((edge.capacity < min) and (not edge.is_saturated)):
        min = edge.capacity

    return min

  def aument(self):
    bottleneck = self.bottleneck()
    for edge in self.edges:
      edge.aument_foward_edge(bottleneck)
      backward_edge = edge.obtain_backward_edge()
      backward_edge.aument_backward_edge(bottleneck)

    return bottleneck

class Graph:
  def __init__(self, source, sink, nodes):
    self.source = source
    self.sink = sink
    self.nodes = nodes
    self.len = len(nodes)

  def generate_aument_path(self, nodes_keys):
    i = 0
    edges = []
    for (key_origin, key_destiny) in nodes_keys:
      if i == 0:
        edges.append(self.source.obtain_edge(key_destiny))
      else:
        node_origin = edges[i-1].destiny
        edges.append(node_origin.obtain_edge(key_destiny))

      i+=1

    return edges

def bfs(graph):
  source = graph.source
  sink = graph.sink
  
  queue = [source]
  paths = {source.key:[]}
  if source.key == sink.key:
    return paths[source.key]

  #Complejidad O(V^2) -> Siempre que el diccionario se acceda en O(1)
  while queue: 
      u = queue.pop(0)
      for v in graph.nodes:
        if(u.has_relation_with(v)) and (v.key not in paths):
            paths[v.key] = paths[u.key]+[(u.key,v.key)]
            if v.key == sink.key:
                return paths[v.key]
            queue.append(v)
  return None

def edmond_karps(graph):
  current_flow = 0
  edges = bfs(graph)

  #Complejidad O(E*V^2) -> Complejidad de bfs es O(V^2) y se itera por todo E
  while edges:
    aument_path = AumentPath(graph, edges)
    current_flow += aument_path.aument()
    
    edges = bfs(graph)

  return current_flow

#usamos un bfs modificado para obtener el corte mínimo
def obtain_min_cut(graph):
  source = graph.source
  queue = [source]
  paths = {}
  edges = [source.key]

  #Complejidad O(V^2 ) -> Siempre que el diccionario se acceda en O(1)
  while queue: 
      u = queue.pop(0)
      nodes = u.connected_nodes()
      for node in nodes:
        key = '({},{})'.format(u.key,node.key)
        edge = u.obtain_edge(node.key)
        if((key not in paths) and (edge.capacity > 0)):
          paths[key] = edge
          queue.append(node)
          if node.key not in edges: edges.append(node.key)

  return paths, edges

#grafo con todos 1s
def generate_graph_publicity(graph):
  new_nodes = {}
  for node in graph.nodes:
    new_nodes[node.key] = Node(node.key)

  #Complejidad O(E*V)
  for node in graph.nodes:
    for edge in node.edges:
      destiny_key = edge.destiny.key
      if not edge.is_backward:
        new_nodes[node.key].add_edge(new_nodes[destiny_key], 1)

  return Graph(new_nodes[graph.source.key], new_nodes[graph.sink.key], new_nodes.values())

#corremos ambas iteraciones de Edmond Karps, con el grafo ya creado 
def solve_problem(graph):
  new_graph1 = generate_graph_publicity(graph)
  new_graph2 = generate_graph_publicity(graph)
  flow = edmond_karps(graph)

  edmond_karps(new_graph1)
  min_cut,edges = obtain_min_cut(new_graph1)

  #Complejidad O(E^2 * V)
  for node in new_graph2.nodes:
    for edge in node.edges:
      key = "({},{})".format(node.key,edge.destiny.key)
      if (key not in min_cut) and (node.key in edges) and (edge.destiny.key not in edges) and (not edge.is_backward):
        print("Agregar publicidad en: {}-{}".format(node.key,edge.destiny.key))
        edge.add_publicity()

  print("Cantidad máxima de pasajeros de {} a {}: {}".format(graph.source.key, graph.sink.key, flow))

def airport_advertising(path):
  f = open(path, "r")
  source_key = f.readline().split('\n')[0]
  sink_key = f.readline().split('\n')[0]
 
  line = 1
  if (not source_key) or (not sink_key): return

  airports = {source_key: Node(source_key), sink_key: Node(sink_key)}
  while(line):
    line = f.readline()
    if line:
      origin_key, destiny_key, cap = line.split(",")
      capacity = int(cap.split('\n')[0])
      if origin_key not in airports: airports[origin_key] = Node(origin_key)
      if destiny_key not in airports: airports[destiny_key] = Node(destiny_key)
      airports[origin_key].add_edge(airports[destiny_key], capacity)

  f.close()

  graph = Graph(airports[source_key], airports[sink_key], airports.values())
  
  solve_problem(graph)


airport_advertising(sys.argv[1])