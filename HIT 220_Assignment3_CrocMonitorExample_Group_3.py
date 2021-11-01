"""
SEMESTER 2/2021 

HIT 200 - ALGORITHMS AND COMPLEXITY

LAB ASSESSMENT 3 

Group 3 members:

Student Name               Student ID
1. Dinh Gia Bao Hoang      S346284
2. Mathews Job             S341408  
3. Tai Phu Phan            S342489 

"""

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
        
        """
        self.d: 1D matrix to store the shortest path to current vertex
        self.Trace: 1D matrix to store the previous vertex to the current vertext in the shortest path
                    for example: Trace[u] = v, means v is the previous vertex of u in the shortest path
        self.Free: 1D matrix to check whether vertex is visited or not
                    for example: Free[u] = True, means vertex u is visited.
        self.Start: the starting vertex
        self.Finish: the finishing vertex
        self.paths: all paths from Start to Finish 
        
        """
        self.d = [] 
        self.Trace = []
        self.Free = [] 
        self.Start = 0 
        self.Finish = 0
        self.paths = []
        
        
        self.readData()
        self.storeDistance()

    """
    Function: readData(self)
        Purpose: read location file and update location list and the point array.
        Input: None
        Output: None
        Example: readData()
        
    """
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

    
    """
    Function: storeDistance(self)
        Purpose: store distance of all adjacient vertices
        Input: None
        Output: None
        Example: storeDistance()
        
        This function will traverse locationList row by row, pick up star point and end point
        calculate distance between the two and store it in the matrix. 
        
        This is an undirected graph so matrix[a][b] = matrix[b][a]    
        
    """
    def storeDistance(self):
    
        for index in range(0, len(self.locationList)-1):
       
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
    
    """
    Function: computePathDistance(self, path)
        Purpose: compute distance of all the connected vertices in a path 
        Input: a path including all connected vertices
        Output: distance of a given path
        Example: computePathDistance(self, ["15","16","18"])
        
    """
    def computePathDistance (self,path):
       
        #provide the distance between two points a and b, as the end points on a path. Assume not adjacent
        distance=0
        
        for i in range(len(path)-1):
            distance = distance + self.matrix[self.points.index(path[i])][self.points.index(path[i+1])]  
        
        return distance      
    
    
    """
    Function: computeDistance(self, a, b)
        Purpose: compute distance of two adjacent location
        Input: location a and b
        Output: distance of adjacent location
        Example: computeDistance("15","18")
        
        This function will find the index of location a and b in the location list,
        get the respective x and y, then calculate the distance using the pythago 
        theorem between two points
    
    """
    def computeDistance (self, a, b):
        
        # provide the distance between two points a and b on a path. Assume adjacent
        distance=0
        
        #convert locationList into numpy list for searching purpose
        l = np.array(self.locationList)[:,0]        
        
        #index of a and b in the location list
        indexa = np.where(l==a)[0][0]
        indexb = np.where(l==b)[0][0]
        
        xa = int(self.locationList[indexa][1])
        ya = int(self.locationList[indexa][2])
        xb = int(self.locationList[indexb][1])
        yb = int(self.locationList[indexb][2])
        
        #calculate distance between a and b
        distance = math.sqrt((xa-xb)**2 + (ya-yb)**2)
        
        return distance

    """
    Function: findPath(self, a, b)
        Purpose: finding the shortest path from a to be using Dijkstra algorithm
        Input: location a and b
        Output: shortest path and distance
        Example: findPath("15","18") -> ["15","16","18"], 8.94
    
        This function finds the shortest path from a to be and based on the Dijkstra algorithm 
    
    
    """
    def findPath(self,a,b):
        #returns shortest path a to b
       
        #init the Graph
        self.InitGraph(a,b)
        #assume the infinity number
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
            
            # vertex u is marked as visited (i.e. not free)
            self.Free[u] = False 
            
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


    """
    Function: computCosting(self, a, b)
        Purpose: find all points on path between a and b, then adding neighbours of internal points on path 
        Input: location of a and b
        Output: exhausitve path and costing
        Example: computeCosting("15","18") -> ['15','16','17','16','18'], 0.45
    
    """
    def computeCosting(self, a, b):
    # unit costs for scanning all points on all paths between two locations and give exhaustive path for rangers to follow, returned as an list
        path=[]
        costing=0
        speed = 40 
        distance = 0
        
        #all points on path between two locations, and the exhaustive search (i.e. neighbours of internal)
        pointList,path = self.findScope(a, b)
        
        print("All points for rangers to inspect: ", pointList)
        print("Exhaustive search path: ", path)
        for i in range(len(path)-1):
            #get the distance between two adjacient vertices through location matrix
            d = self.matrix[self.points.index(path[i])][self.points.index(path[i+1])]  

            #add-up the distance of the path
            distance = distance + d
            
            #format the output string
            if (i-1) >0 and path[i+1] == path[i-1]:
                str = path[i+1] + " <- " + path[i] + ": %.2f"%d
            else:
                str = path[i] + " -> " + path[i+1] + ": %.2f"%d       
            
            print(str)
        
        #calcualte costing given assumed speed.
        costing = distance/speed 
        
        print("Distance (km): %.2f"%distance,"\nSpeed (km/h): %.2f"%speed,"\nCost (units of hour): %.2f"%costing)
        
        return costing,path
        
    """
    Function: improveDistance(self, a, b)
        Purpose: find the shortest path from a to be, suggest the possible points to be blocked 
        Input: location and and b
        Output: array of possible points to be blocked
        Example: improveDistance("15","18") -> ['Point: 16', 'Distance: 2.24', 'Ratio: 0.25']
    
    """
    def improveDistance (self, a, b):
    #return point blocked as a value on map (eg A1) and scaled increase in distance between points
        points=[]
        scaledImprovement=0
        d = 0
        
        #find the shortest path between a and b
        path, shortest = self.findPath(a, b)
        print(f"\nThe shortest path: {path}")
        print("Total distance: %.2f"%shortest)
        
        #check for all internal points within the shortest path 
        #if there is any neighbour then that point will be tracked for ranger to consider
        for i in range(1,len(path)-1):
            k = self.points.index(path[i])
            z = self.points.index(path[i-1])
            #calculate accumulated distance
            d += self.matrix[k][z]
            #check wether any point in the shortest path has neihgbour
            for j in range(len(self.points)):
                #if found any neighbour from point j then track the place for blockage
                if self.matrix[k][j] != 0 and self.points[j] not in path:
                    points.append(["Point: "+path[i],"Distance: %.2f"%d,"Ratio: %.2f"%(d/shortest)])
                          
        return points #,scaledImprovement


    """
    Function: countCroc(self, beach, x)
        Purpose: find a location within the radius of the beach and number of crocs
        Input: beach and radius
        Output: list of location near the beach and the number of crocs
        Example: countCroc("B1",10) -> [['22',1.0]
                                        ['B2',0.0]]
             
    """
    
    def countCroc(self, beach, x):
       #count the number of crocs likely in a x mile radius of a beach. Return an array [location, number]
        
       #list of points within the radius of the beach
       list = []
       
       #convert locationList to numpy list for searching
       l = np.array(self.locationList)[:,0]        
       
       nearest_point = ""
       nearest_distance = float("inf")
       
       for i in range(len(self.points)):
             
            d = self.computeDistance(beach, self.points[i])
            
            if d > 0 and d <= x:
                
                j = np.where(l==self.points[i])[0][0] 
                                
                if self.locationList[j][3] != "":
                    crocs = float(self.locationList[j][3])
                else: crocs = 0.
                        
                list.append([self.points[i],crocs])
                
                #check nearest point 
                if nearest_distance > d:
                    nearest_distance = d
                    nearest_point = [self.points[i],crocs]
       
       #last element is the nearest
       if nearest_point != "":
           list.append(nearest_point)
                
       return list
    
    """
    Function locatOptimalBlockage(self, a, b)
        Purpose:
        Input: location a and b
        Output:
        Example:
        
         
    """
    def locateOptimalBlockage(self,beach,radius, list):
    # return the point blocked eg A1 and the increase in protection provided using some weighting
        point="A1"
        protection=1
        
        if len(list) == 0:
            print(f"No points within the radius of {radius} of {beach}")
            return
        
        print(f"\nAll locations and number of crocs are within the radius {radius} of {beach}")
        for i in range(len(list)-1):print(list[i])
        
        #nearest point to the beach (the last item in list)
        point = list[len(list)-1][0]         
        print(f"Nearest point to the beach {beach} is: {point}")
        
        for j in range(len(list)-1):
            if list[j][0] != point:
                self.FindAllPaths(list[j][0],point)
                #print(self.paths)

        return point, protection


    """
    Function: MinTime(self, a, b)
    Purpose: points travelled through the shorest path from a to b 
    Input: location a and b
    Output: array of points travelled and time value:
    Example: MinTime("15", "18")
             -> points: ["15","16","18"]
                time value (hours): 0.56
                
                "15" -> "16" (water), distance: 2.36, speed: 16 (km/h), time value: 2.36/16 = 0.14
                "16" -> "18" (water), distance: 6.71, speed: 16 (km/h), time value: 6.71/16 = 0.42
                Total time value (hours): 0.56
    
    """
    def minTime(self,a,b):
    #return list of points trevelled and the time required
        
        time = 0
        path = []
        trace = []
        water = ""
        
        #converst locationList to numpy list for search purpose
        l = np.array(self.locationList)[:,:]        
        
        #finding the shortest path
        path,shortest = self.findPath(a, b) 
        
        for i in range(len(path)-1,0,-1):
            #check the index of path[i-1] to path[i] in the locationList 
            #l[:,0] is the first column of locationList: sight name
            #l[:,4] is the fifth column of locationList: neighbour
            j = np.where( (l[:,0]==path[i-1]) & (l[:,4]==path[i]))[0]
            
            #if not then search again in the listlocation
            #just change the path[i] to the first column and path[i-1] in the fifth column
            if len(j) == 0:
                j = np.where( (l[:,0]==path[i]) & (l[:,4]==path[i-1]))[0]
            
            #check the the given location of a and be is water or land, column 5 in listLocation
            if self.locationList[j[0]][5] == True:
                speed = 16.0
                water = "Water"
            else: 
                speed = 6.0
                water = "Land"
            #find the index of two locations
            f = self.points.index(path[i-1])
            t = self.points.index(path[i])
            
            #calculate the unit of time given distance of two adjacent locations
            distance = self.matrix[f][t]
            time = time + distance/speed
            
            trace.insert(0,[path[i-1] + "->" +path[i],water,
                            "Distance: %.2f"%distance,
                            "Speed: %.1f"%speed,
                            "Time: %.2f"% (distance/speed)])
        
        return path, trace, shortest,time
    

    """
    Function: findScope(a, b)
        Purpose:
        Input: location a and b
        Output: pointList and exhaustive path 
        Example: findScope("15","18") 
                  -> poinstList = ["15","16","17","18"]
                     exhaustive path = ["15","16","17","16","18"]
        
        This function will find the shortest path from a to b, add the neighbours for internal points
        between the path except a and b, the exhaustive path is built on these points
        
    
    """        
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
        self.FindAllPaths(a, b)
        print(f"\nAll possible paths from {a} to {b}: ")
        for p in self.paths:
            print(p)
        
        #Find shortest route from path options 
        path, shortest = self.findPath(a, b)
        print("\nShortest path: ", path," - Distance: %.2f"%shortest)
        
        # Add side points to inspect
        #include all nodes that are linked to (neighbour of) any internal point on path (ie point crocodiles can enter)  
        #       between a and b - this may add backtracking
        
        #Add neighbours of each location except the starting and ending location
        for i in range(1, len(path)-1):
            
            #add path[i] into the pointlist then search for neighbours
            pointList.insert(len(pointList)-1,path[i])
            expath.insert(len(expath)-1,path[i])
            
            #check its index and neighbours
            j = self.points.index(path[i])
           
            for k in range(len(self.points)):
                #if points[k] is a nearest neighbour then add it up to the pointlist
                if self.matrix[j][k] != 0:
                    if (self.points[k] not in pointList) and (self.points[k] not in path):
                        #add the neighbour into point list
                        pointList.insert(len(pointList)-1,self.points[k])
                        #add the neighbour into exhaustive path, example i to k and k back to i
                        expath.insert(len(expath)-1,self.points[k])
                        expath.insert(len(expath)-1,path[i]) #backtrack
               
        #Example findScope ("15","18")
            #paths are [15,16,18] and [15,16,17,19,20]
            #shortest path [15,16,18]
            #add neighbours [15,16,17,18]
        
        #This is the exhaustive list of points rangers need to inspect
        return pointList, expath

    
    """
    Function: InitGraph(self, a, b)
        Purpose: initialize all the parameters for specific instance of the graph.
        Input: name location a and b
        Output: none
        Example: InitGraph("15", "18")
        
        This function will inititalize for the parameters below:
        self.[d] set to infinite for all item assuming the shortest path to each vertex is infinity
        self.[Trace] set to -1 for all vertices
        self.[Free] set to True for all vertices 
        self.paths set to null list
        self.Start will be the index of location a
        self.Finish will be the index of location b
        
        set the d[Start] = 0 means shortest path from Start to Start is 0
    
    """
    def InitGraph(self, a, b):
        
        n = len(self.points)
        MAX = 1000000
        self.d = [MAX]*(n)
        self.Trace = [-1]*(n)
        self.Free = [True]*(n) 
        self.paths = []
        
        #set the staring and finishing location
        self.Start = self.points.index(str(a))
        self.Finish = self.points.index(str(b))
        
        #shortest path from Start to Start is 0
        self.d[self.Start] = 0 
                
    """
    Function: FindAllPaths(self, a, b)
        Purpose: find all possible paths from a to b
        Input: location a and b 
        Output: None
        Example: FindAllPaths ("15","18")

        This funciton will call another function InitGraph given first and second location to initialize all the 
        parameters for the graph
        
        Then it will call Try function to start traversing from a, and find all possible paths from
        a to b. All the parameters will be updated once the Try function completes.
    
    """
    def FindAllPaths(self, a, b):
        
        self.InitGraph(a, b)
        self.Try(self.Start)
    
    
    """
    Function: Try(self, a)
        Purpose: traverse the locations on map
        Input: location a 
        Output: None
        Example: Try("15")
        
        This function is a recursive function given the first vertex then it will traverse all 
        possible adjacent vertices until it reaches the target vertex then full path will be tracked
        or having no neighbours
        
    """
    def Try(self, a):
              
        self.Free[a] = False
        #check for all other vertices
        for u in range(len(self.points)):
            #if adjacent and is not visited.
            if self.matrix[a][u] != 0 and self.Free[u] == True:
                #trace the path
                self.Trace[u] = a
                self.Free[u] = True
                #if the vertex u is the target
                if u == self.Finish: 
                    path =[]
                    #trace the full path
                    while self.Start != u:
                            path.append(self.points[u])
                            u = self.Trace[u]    
                    path.append(self.points[self.Start])
                    path.reverse()
                    #track the path into array of paths
                    self.paths.append(path) 
                    return
                else:
                    #try with vertex u if not adjacent. 
                    self.Try(u)
                    self.Free[u] = True    
        return        
    

"""
Function: Question1()

"""    
def Question1():
    
    print("\nQuestion 1 - Exhaustive search: ")
    location1 = input("Location 1: ")
    location2 = input("Location 2: ")
    
    cm.computeCosting(location1, location2)
    input("\nPress Enter to continue...")
    
"""
Funtion: Question2()

"""
def Question2():
    
    print("\nQuestion 2 - Improve Distance: ")
    location1 = input("Location 1: ")
    location2 = input("Location 2: ")
    
    #pending as need to clarify with Dr. Cat
    points = cm.improveDistance(location1, location2)
    
    if len(points) == 0:
        print("There is no any neighbour for any location within the shortest path. Rangers consider to place the blockage on any point in the path or elsewhere...")
    else:
        print("Possible locations for rangers to consider to place a blockage: ")
        for p in points:print(p)
    
    input("\nPress Enter to continue...")
    
"""
Function: Question3()

"""  
def Question3():
    
    print("\nQuestion 3 - MinTime: ")
    location1 = input("Location 1: ")
    location2 = input("Location 2: ")
    path, trace, distance,time = cm.minTime(location1, location2)
    print("\nPoints travelled through the shortest path: ",path)
    for t in trace:
        print(t)
    print("Total distance (km): %.2f"%distance)
    print("Time value (units of hour): %.2f" %time)
    
    print("\nQuestion 3 - Extension")
    print("Assumption: ")
    print("Input the beach and radius. For example B1 and 10")
    beach = input("Beach: ")
    radius = int(input("Radius: "))
    list = cm.countCroc(beach,radius)
     
    cm.locateOptimalBlockage(beach, radius,list)
    
    input("\nPress Enter to continue...")
    
    
#Driver code      
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
        print("\nPress following key for the questions.")
        print("1 for Question 1 - Exhaustive Path and Compute Costing\n2 for Question 2 - Finding points to be blocked\n3 for Question 3 - Couting Crocs and Optimum Blockage\nOr any key to exit...")
        Choice = input("Your input: ")
        if Choice not in Options:
            break
        #call the respective function
        Options[Choice]()
    print("Program End!!!")        

    