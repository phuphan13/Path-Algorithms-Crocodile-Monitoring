from typing import Sized
import csv

import math
import numpy as np
size=100
class CrocMonitor:
    locationList =[]
    import csv
    def __init__(self, size):
        
        self.locationList = []
        self.matrix = [[0 for x in range(size)]for y in range(size)]
        self.points=[]
        
        #added code begin - Phu
        self.d=[] #d[u]: shortest path from Start to vertex v
        self.Trace=[] #Trace[u] = v, i.e. v is the previous vertex of u in the shortest path
        self.Free=[] #Free[u]: u is free, i.e. not visited
        self.Start = 0 
        self.Finish = 0
        self.paths =[]
        #added code end - Phu
        
        self.readData()
        self.storeDistance()

    def readData(self):
        with open('Locations.csv') as f:
            csv_reader = csv.reader(f)
            index = 0
            next(csv_reader)
            for line in csv_reader:
                
                pointName=line[0]
                x=line[1]
                y=line[2]
                number=line[3]
                edge=line[4]
                
                water=False
                if line[5] == "W":
                    water=True

                self.locationList .append( [pointName,  x, y, number, edge, water] ) # etc
                
                if not pointName in self.points:
                   
                    self.points.append(pointName)
                index += 1
        
        f.close()

    def storeDistance(self):
    
        for index in range(0, len(self.locationList)-1):
   
            if self.locationList[index][4]!="":
                startpoint = self.locationList[index][0]
                endpoint = self.locationList[index][4]

                #added code begin - Phu
                if endpoint != "":
                
                    xa = int(self.locationList[index][1])
                    ya = int(self.locationList[index][2])

                    for j in range(0, len(self.locationList)-1):
                
                        if self.locationList[j][0] == endpoint:
                            xb = int(self.locationList[j][1])
                            yb = int(self.locationList[j][2])
                            break
            
                    distance = math.sqrt((xa-xb)**2 + (ya-yb)**2)
            
                    #position in the adjacent matrix
                    indexa = self.points.index(startpoint)
                    indexb = self.points.index(endpoint)
                        
                    #add the weighting of the edge
                    self.matrix[indexa][indexb] = distance
                    self.matrix[indexb][indexa] = distance           
            
                #added code end - Phu
                
                """ Just comment out the code from Dr. Kutay
                for indexa in range (0, len(self.points)-1):
                    if self.points[indexa] == startpoint:
                        indexPointa=indexa
                
                        for indexb in range(0, len(self.points)-1):
                            if self.points[indexb] == endpoint:
                                indexPointb = indexb
                              
                                distance = self.computeDistance(startpoint, endpoint)
                           #store distance along path    
                                break
                        break
                """
      

    def computePathDistance (self,path):
       
        #provide the distance between two points a and b, as the end points on a path. Assume not adjacent
        distance=0
        return distance
  

    def findPath(self,a,b):
        path=[]
        #returns shortest path a to b
        return path
        

    def computeDistance (self, a, b):
       
        # provide the distance between two points a and b on a path. Assume adjacent
        distance=0
        return distance

    def computeCosting(self, a, b):
    # unit costs for scanning all points on all paths between two locations and give exhaustive path for rangers to follow, returned as an list
        path=[]
        costing=0
        return costing,path
    
    def improveDistance (self, a, b):
    #return point blocked as a value on map (eg A1) and scaled increase in distance between points
        point="A1"
        scaledImprovement=0
        return point, scaledImprovement

    def countCroc(self, beach, x):
    #count the number of crocs likely in a x mile radius of a beach. Return an array [location, number]
        number=0
        return number
            

    def locateOptimalBlockage(self,a,b):
    # return the point blocked eg A1 and the increase in protection provided using some weighting
        point="A1"
        protection=1
        return point, protection

    def minTime(self,a,b):
    #return list of points trevelled and the time required
        path=[]
        return path


    #added code begin - Phu
    
    def InitGraph(self, a, b):
        
        n = len(self.points)
        MAX = 1000000
        self.d = [MAX]*(n)
        self.Trace = [-1]*(n)
        self.Free = [True]*(n) 
        self.paths = []
        
        Start = self.points.index(str(a))
        Finish = self.points.index(str(b))
        
        self.Start = Start
        self.Finish = Finish
        self.d[Start] = 0 #shortest path from Start to Start is 0
            
        return
    
    def ExhaustiveSearch(self, a, b):
        
        self.InitGraph(a, b)
        self.Try(self.Start)
        
        return
    
    def Try(self, a): #DFS recursively backtracking
              
        self.Free[a] = False
        for u in range(len(self.points)):
            if self.matrix[a][u] != 0 and self.Free[u] == True:
                self.Trace[u] = a
                self.Free[u] = True
                if u == self.Finish: #when the Finish vertex is reached
                    path =[]
                    #trace the full path
                    while self.Start != u:
                            path.append(self.points[u])
                            u = self.Trace[u]    
                    path.append(self.points[self.Start])
                    path.reverse()
                    #track the path into array
                    self.paths.append(path) 
                    return
                else:
                    self.Try(u)
                    self.Free[u] = True 
            
        return        
            
    def ShortestPath(self, a, b): #dijkstra algorithm, complexity is O(n^2)
        
        self.InitGraph(a,b)
        MAX = 1000000
        path = []        
        
        while True:
            min = MAX+1
            u = 0 
            
            #finding the vertex which having shortest path
            for i in range(len(self.points)):
                if self.Free[i] and min > self.d[i]:
                    min = self.d[i]
                    u = i
        
            #if the Finish vertex is reached
            if u == self.Finish:    
                break
            
            self.Free[u] = False # vertex u is marked as visited (i.e. not free)
            
            for v in range(len(self.points)):
                if self.matrix[u][v] != 0: 
                    #if the shortest path from Start to vertex v greater than 
                    #the shortest path from Start to u and distance from u to v
                    #then update the distance from Start to v 
                    if self.d[v] > self.d[u] + self.matrix[u][v]:
                       self.d[v] = self.d[u] + self.matrix[u][v]
                       self.Trace[v] = u
                        
        #trace the shortest path
        if self.d[self.Finish] == MAX: #no path from Start to End
            self.d[self.Finish] = -1
        else:
            u = self.Finish
            while self.Start != u:
                path.append(self.points[u])
                u = self.Trace[u] 
            path.append(self.points[self.Start])       
            path.reverse()    
                   
        return path, self.d[self.Finish]    
   
    #added code end - Phu
    
    
    
    
if __name__ == '__main__':
   
    cm=CrocMonitor(size) 
    #print (cm.locationList)
    
    
    
    print(cm.locateOptimalBlockage("15","18"))
    #return 17 as other points have alternatives to bypass 
    cm.computeCosting("15","18")
    # exhaustive path is  [15,16, 17,16, 18] so return length of this as unit cost - note data changes in Locations.csv

    cm.locateOptimalBlockage("15", "18")
    #returns 16 as other routes have alternative paths
    #may use other data to decide optimum path, but explain in requirements for this method


    #added code begin - Phu
    
    while True:
        s = input("Press 1 for Shortest Path, or Press 2 for Search exhaustive paths ")
        if s not in ("1","2"): break
        Start = input("Start: ")
        Finish = input("Finish: ")
        if s == "1": 
            path, length = cm.ShortestPath(Start, Finish)
            print(path)
            print("Length: %.2f" %length)
        else: 
            cm.ExhaustiveSearch(Start,Finish)
            for p in cm.paths:
                print(p)
                
    #added code end - Phu