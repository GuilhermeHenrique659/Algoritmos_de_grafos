class Graph:
  def __init__(self, vertices):
      self.V = vertices
      self.graph = []

  def addEdge(self,u,v,w):
      self.graph.append([u,v,w])
      self.graph.append([v,u,w])
      
  def minDistance(self, lambda_dist, T):
    min = float('inf')

    for v in range(self.V):
      if lambda_dist[v] < min and T[v]==False:
        min = lambda_dist[v]
        min_index = v

    return min_index

  def printSolution(self, lambda_dist):
    print('Vertice \ta partir da origin')
    for node in range(self.V):
      print(node,"\t",lambda_dist[node])

  def dijkstra(self, src):
    lambda_dist = [float('inf')] * self.V
    lambda_dist[src] = 0
    T = [False] * self.V
    
    for count in range(self.V):
      u = self.minDistance(lambda_dist,T)
      for v in range(self.V):
        for item in self.graph:
          if(item[0]==u) and (item[1]==v):
            if item[2] > 0 and T[v] == False and lambda_dist[v] > lambda_dist[u] + item[2]:
              lambda_dist[v] = lambda_dist[u] + item[2]
        T[u] = True
    print('\nSolução pelo algoritmo de dijkstra\n')
    self.printSolution(lambda_dist)


  def bellman_ford(self, src):
        lambda_dist = [float("Inf")] * self.V
        lambda_dist[src] = 0
        for _ in range(self.V - 1):
            for u, v, w in self.graph:
                if lambda_dist[u] != float("Inf") and lambda_dist[u] + w < lambda_dist[v]:
                    lambda_dist[v] = lambda_dist[u] + w
        for u, v, w in self.graph:
            if lambda_dist[u] != float("Inf") and lambda_dist[u] + w < lambda_dist[v]:
                return False
        print('\nSolução pelo algoritmo de bellman-ford\n')
        self.printSolution(lambda_dist)
        return True


if __name__ == '__main__':
  g = Graph(6)
s = 0
a = 1
b = 2
c = 3
d = 4
t = 5

g.addEdge(s,a,18)
g.addEdge(s,c,15)

g.addEdge(a,b,9)
g.addEdge(a,c,6)

g.addEdge(b, c, 14)
g.addEdge(b, d, 10)
g.addEdge(b, t, 28)

g.addEdge(c, d, 7)

g.addEdge(d, t, 36)

g.dijkstra(s)
g.bellman_ford(s)
