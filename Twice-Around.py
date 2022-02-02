from collections import defaultdict 

#Classe Grafo
class GraphFleury: 

	def __init__(self,vertices): 
		self.V= vertices #No. de vertices 
		self.graph = defaultdict(list) # default dictionary para armazenar o grafo
		self.Time = 0

	# Adiciona aresta (não direcionada)
	def addEdge(self,u,v): 
		self.graph[u].append(v) 
		self.graph[v].append(u)
    
 def dfs(u, graph, visited_edge, path=[]):
    path = path + [u]
    for v in graph[u]:
        if visited_edge[u][v] == False:
            visited_edge[u][v], visited_edge[v][u] = True, True
            path = dfs(v, graph, visited_edge, path)
    return path

# for checking in graph has euler path or circuit
def check_circuit_or_path(graph, max_node):
    odd_degree_nodes = 0
    odd_node = -1
    for i in range(max_node):
        if i not in graph.keys():
            continue
        if len(graph[i]) % 2 == 1:
            odd_degree_nodes += 1
            odd_node = i
    if odd_degree_nodes == 0:
        return 1, odd_node
    if odd_degree_nodes == 2:
        return 2, odd_node
    return 3, odd_node


def check_euler(graph, max_node):
    visited_edge = [[False for _ in range(max_node + 1)] for _ in range(max_node + 1)]
    check, odd_node = check_circuit_or_path(graph, max_node)
    if check == 3:
        print("graph is not Eulerian")
        print("no path")
        return
    start_node = 1
    if check == 2:
        start_node = odd_node
        print("graph has a Euler path")
    if check == 1:
        print("graph has a Euler cycle")
    path = dfs(start_node, graph, visited_edge)
    print(path)
    
def criargraphFleury(g):

  gf = GraphFleury(g.V*2)
  for u,v,w in g.graph:
    gf.addEdge(u,v)
  check_euler(gf.graph,gf.V)

 import heapq

class Graph:

    def __init__(self,vertice):
        self.V = vertice
        self.graph = []

    def addEdge(self, u,v,w):
      self.graph.append([u,v,w])

    def PrimMst(self, raiz):
        n = self.V
        m = len(self.graph)
        H = []
        n_out = [[]*n for i in range(n)]
        for j in range(m):
            a, b, c = self.graph[j]
            n_out[a].append((b, c))
            n_out[b].append((a, c))

        for (x, c) in n_out[raiz]:
            heapq.heappush(H, (c,raiz,x))

        n_edges = 0
        custo_tot = 0
        marcados = [raiz]
        arv_ger_min = []

        while n_edges < n - 1:
            while True:
                (c, a, b) = heapq.heappop(H)
                if b not in marcados:
                    break
            marcados.append(b)
            custo_tot += c
            arv_ger_min.append([a,b,c])
            n_edges += 1
            for (x, c) in n_out[b]:
                if x not in marcados:
                    heapq.heappush(H, (c, b, x))
        print("MST")
        self.graph = arv_ger_min.copy()
        return arv_ger_min
    

def Twice_Around(g):
  T = g.PrimMst(0)
  print(g.graph)
  g.graph=g.graph+T
  print(g.graph)
  L = criargraphFleury(g)
  
 if __name__ == '__main__':
    g = Graph(6)
v1=0
v2=1
v3=2
v4=3
v5=4
v6=5
g.addEdge(v1,v2,1)
g.addEdge(v1,v3,4)
g.addEdge(v1,v4,9)
g.addEdge(v1,v5,8)
g.addEdge(v1,v6,2)

g.addEdge(v2,v3,5)
g.addEdge(v2,v4,5)
g.addEdge(v2,v5,7)
g.addEdge(v2,v6,6)

g.addEdge(v3,v4,10)
g.addEdge(v3,v5,7)
g.addEdge(v3,v6,4)

g.addEdge(v4,v5,1)
g.addEdge(v4,v6,7)

g.addEdge(v5,v6,3)
Twice_Around(g)
