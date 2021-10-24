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
        
        ###added code
        self.d = [] #d[u]: shortest path from Start to vertex v
        self.Trace = [] #Trace[u] = v, i.e. v is the previous vertex of u in the shortest path
        self.Free = [] #Free[u]: u is free, i.e. not visited
        self.Start = 0 
        self.Finish = 0
        self.paths = []
        ###
        
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
       
            ###added code 
            
            startpoint = self.locationList[index][0]
            endpoint = self.locationList[index][4]
            
            if startpoint != "" and endpoint != "":
                distance = self.computeDistance(startpoint, endpoint)
            
                #position in the adjacent matrix
                indexa = self.points.index(startpoint)
                indexb = self.points.index(endpoint)
                        
                #add the weighting of the edge
                self.matrix[indexa][indexb] = distance
                self.matrix[indexb][indexa] = distance           
            
            ###   
    

    def computePathDistance (self,path):
       
        #provide the distance between two points a and b, as the end points on a path. Assume not adjacent
        distance=0
        
        ###added code
        #for i in range(len(path)-1):
        #    distance = distance + self.computeDistance(path[i],path[i+1])    
        
        for i in range(len(path)-1):
            distance = distance + self.matrix[self.points.index(path[i])][self.points.index(path[i+1])]  
        ###
        
        return distance      
    

    def computeDistance (self, a, b):
        
        # provide the distance between two points a and b on a path. Assume adjacent
        distance=0
        
        ###added code
        l = np.array(self.locationList)[:,0]        
        indexa = np.where(l==a)[0][0]
        indexb = np.where(l==b)[0][0]
        
        xa = int(self.locationList[indexa][1])
        ya = int(self.locationList[indexa][2])
        
        xb = int(self.locationList[indexb][1])
        yb = int(self.locationList[indexb][2])
        
        distance = math.sqrt((xa-xb)**2 + (ya-yb)**2)
        ###
        
        return distance


    def findPath(self,a,b):
        #returns shortest path a to b
       
        ###added code 
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
        ###       

    
    def computeCosting(self, a, b):
    # unit costs for scanning all points on all paths between two locations and give exhaustive path for rangers to follow, returned as an list
        path=[]
        costing=0
        
        ###added code
        #all points on path between two locations
        speed = 40 #40km/4
        distance = 0
        pointList,expath = self.findScope(a, b)
        
        print("All points for rangers to inspect: ", pointList)
        print("Exhaustive search path: ", expath)
        for i in range(len(expath)-1):
            d = self.matrix[self.points.index(expath[i])][self.points.index(expath[i+1])]  
            distance = distance + d
            print(expath[i], "->",expath[i+1],": %.2f"%d)
        
        costing = distance/speed 
        
        print("Distance (km): %.2f"%distance,"\nSpeed (km/h): %.2f"%speed,"\nCost (units of hour): %.2f"%costing)
        ###
        
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
        
        #added code
        time = 0
        path = []
        l = np.array(self.locationList)[:,:]        
        path,shortest = self.findPath(a, b) #finding the shortest path
        
        for i in range(len(path)-1,0,-1):
            #check the route between two locations is water or land 
            j = np.where( (l[:,0]==path[i-1]) & (l[:,4]==path[i]))[0]
            
            if len(j) == 0:
                j = np.where( (l[:,0]==path[i]) & (l[:,4]==path[i-1]))[0]
            
            if self.locationList[j[0]][5] == True:
                speed = 16.0
            else: speed = 6.0
            
            #find the index of two locations
            f = self.points.index(path[i-1])
            t = self.points.index(path[i])
            
            #calculate distance of two locations
            distance = self.matrix[f][t]
            time = time + distance/speed
            
        return path, time
        ###
        
    def findScope(self, a, b):
        #provide start and end point of search, collect points to consider in search
        pointList=[a,b]
        expath = [a,b]
          
        #find location of a and b in points list
        for index in range(0, len(self.points)):
            if self.points[index ]== a:
                indexa=index
            if self.points[index] == b:
                indexb = index 
        
        #Find all paths a to b - Select direct routes only, no cycles or backtracking
        
        ###added code
        self.FindAllPaths(a, b)
        ###
        
        #Find shortest route from path options 
        
        ###added code
        shortest = float("inf")
        print(f"\nAll possible paths from {a} to {b}: ")
        for p in self.paths:
            d = self.computePathDistance(p)
            if shortest > d: 
                shortest = d
                path = p
            print(p)
            
        print("\nShortest path: ", path," - Distance: %.2f"%shortest)
        ###

        # Add side points to inspect
        #include all nodes that are linked to (neighbour of) any internal point on path (ie point crocodiles can enter)  
        #       between a and b - this may add backtracking
        
        ###added code
        # add neighbours of each location except the starting and ending location
        for i in range(1, len(path)-1):
            
            #add path[i] into the pointlist then search for neighbours
            pointList.insert(len(pointList)-1,path[i])
            expath.insert(len(expath)-1,path[i])
            
            #check its index and neighbours
            j = self.points.index(path[i])
           
            for k in range(len(self.points)):
                #if points[k] is a neighbour then add it up to the pointlist
                if self.matrix[j][k] != 0:
                    if (self.points[k] not in pointList) and (self.points[k] not in path):
                        pointList.insert(len(pointList)-1,self.points[k])
                        expath.insert(len(expath)-1,self.points[k])
                        expath.insert(len(expath)-1,path[i]) #backtrack
        ###
               
        #Example findScope ("15","18")
            #paths are [15,16,18] and [15,16,17,19,20]
            #shortest path [15,16,18]
            #add neighbours [15,16,17,18]
        
        #This is the exhaustive list of points rangers need to inspect
        return pointList, expath

    ###added code
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
    
    def FindAllPaths(self, a, b):
        
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
    
    
def Question1():
    
    print("\nQuestion 1 - Exhaustive search: ")
    location1 = input("Location 1: ")
    location2 = input("Location 2: ")
    
    cm.computeCosting(location1, location2)
    
    return

def Question2():
    
    location1 = input("Location 1: ")
    location2 = input("Location 2: ")
    
    #pending as need to clarify with Dr. Cat
    cm.improveDistance(location1, location2)
    
    return

def Question3():
    
    print("\nQuestion 3 - MinTime: ")
    location1 = input("Location 1: ")
    location2 = input("Location 2: ")
    path, time = cm.minTime(location1, location2)
    print("Points travelled through on the path: ",path)
    print("Time value (unit of hour): %.2f" %time)
    
    print("\nQuestion 3 - Extension:")
    beach = input("Beach: ")
    radius = input("Radius: ")
    #Mathews is working on this
    cm.countCroc(beach,radius)
    
    #pending as need to discuss further
    cm.locateOptimalBlockage(location1, location2)
    
    return   
    ###




if __name__ == '__main__':
   
    cm=CrocMonitor(size) 
    
    """
    Dr. Cat Kutay's examples
    #print (cm.locationList)
    #Changed examples
    cm.computeCosting("15","18")
  
    # exhaustive path is  [15,16, 17,16, 18] so return the length of this as unit cost - note data changes in Locations.csv
    #algorithm to find scope of spanning tree is provided as findScope()
    cm.improveDistance("15","18")
    #output will be 16  Ratio is "original distance on [15,16,18]:0"
    cm.locateOptimalBlockage("15", "18")
    #returns 16 as other routes have alternative paths
    #may use other data to decide optimum path, but explain in requirements for this method
    cm.minTime("15", "18") 
    #returns [15,16,18] and time to travel that path
    """
    
    Choice = ""
    Options ={"1":Question1,"2":Question2,"3":Question3} 
    while True:
        Choice = input("\nEnter 1 for Question 1, 2 for Question 2, 3 for Question 3, or any key to exit... ")
        if Choice not in Options:
            break
        Options[Choice]()#Call the respective function
    print("Program End!!!")        

    