import heapq

class Graph:

    def __init__(self,vertice):
        self.V = vertice
        self.graph = []

    def addEdge(self, u,v,w):
      self.graph.append([u,v,w])

    def find(self, parent, i):
      if parent[i] == -1:
        return i
      elif parent[i] != -1:
        return self.find(parent,parent[i])

    def union(self,parent,x,y):
        x_set = self.find(parent, x)
        y_set = self.find(parent, y)
        parent[x_set] = y_set
        print(parent)

    def KruskalMST(self):
        result = []

        i=0
        e=0

        self.graph = sorted(self.graph, key=lambda item: item[2])
   #     print('Grafo com pesos ordenados:',self.graph)

        parent = []

        for node in range(self.V):
            parent.append(-1)

        while e < self.V -1:
            u,v,w = self.graph[i]
            i = i + 1

            x = self.find(parent, u)
            y = self.find(parent, v)
            if x != y:
                e = e + 1
                result.append([u,v,w])
                self.union(parent,x,y)

            else:
                print()

#           print('Folling are the edges in the constructed MST')
#           for u,v,weight in result:
#              print('%d -- %d == %d' % (u,v,weight))
        print(result)
        

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
            arv_ger_min.append((a, b))
            n_edges += 1
            for (x, c) in n_out[b]:
                if x not in marcados:
                    heapq.heappush(H, (c, b, x))

        print(custo_tot)
        print(arv_ger_min)

if __name__ == '__main__':
    g = Graph(5)
    g2 = Graph(9)
    #A
    g.addEdge(0,1,15)
    g.addEdge(0,2,12)
    g.addEdge(0,3,13)
    g.addEdge(0,4,20)

    #B
    g.addEdge(1,2,16)
    g.addEdge(1,3,16)
    g.addEdge(1,4,5)

    #C
    g.addEdge(2,3,1)
    g.addEdge(2,4,18)

    #D
    g.addEdge(3,4,17)
    print("Krukal")
    g.KruskalMST()
    print("Primm")
    g.PrimMst(3)


    #a
    g2.addEdge(0, 1, 4)
    g2.addEdge(0, 7, 9)

    #b
    g2.addEdge(1,2,8)
    g2.addEdge(1,7,11)

    #c
    g2.addEdge(2,8,2)
    g2.addEdge(2,3,7)
    g2.addEdge(2,5,4)

    #d
    g2.addEdge(3,4,9)
    g2.addEdge(3,5,14)

    #f
    g2.addEdge(5,4,10)
    g2.addEdge(5,6,2)

    #g
    g2.addEdge(6,7,1)
    g2.addEdge(6,8,6)
    #h
    g2.addEdge(7,8,7)
    print('arvore 2')
    g2.PrimMst(0)
    g2.KruskalMST()
