from collections import deque

class graph:

    def __init__(self,vertice):
        self.V = vertice
        self.graph = []
        self.pi = []
        self.d = []
        self.cor = []

    def addEdge(self,u,v):
        self.graph.append([u,v])
        self.graph.append([v,u])

    def BFS(self,s):
        self.cor =  ['White'] * (self.V)
        self.pi =  ['-1'] * (self.V)
        self.d =  ['9999'] * (self.V)

        self.cor[s] = 'Gray'
        self.d[s] = 0
        self.pi[s] = -1
        Q = deque([])
        Q.append(s)
        while len(Q) != 0:
            print(Q)
            u = Q.popleft()
            for v in self.graph:
                if u == v[0]:
                    if self.cor[v[1]] == 'White':
                        self.cor[v[1]] = 'Gray'
                        self.d[v[1]] = self.d[u]+1
                        self.pi[v[1]] = u
                        Q.append(v[1])
            self.cor[u] = 'Brack'

        print('predecessores: ', self.pi)
        print('distancia: ', self.d)
        print('vetor visitado ', self.cor)

if __name__ == '__main__':
      s = 0
    a = 1
    f = 2
    d = 3
    b = 4
    e = 5
    c = 6
    t = 7
    g = graph(8)
    g.addEdge(s,a)
    g.addEdge(s,f)
    g.addEdge(a,d)
    g.addEdge(a,b)
    g.addEdge(a,e)
    g.addEdge(f,d)
    g.addEdge(f,e)
    g.addEdge(b,c)
    g.addEdge(d,c)
    g.addEdge(d,e)
    g.addEdge(b,t)
    g.addEdge(e,t)
    g.addEdge(c,t)

    g.BFS(s)
