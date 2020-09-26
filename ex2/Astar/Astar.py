from Map import Map_Obj
import heapq

#Make a class for search-nodes
class searchNode:

    #initalizes a node 
    def __init__(self, state, parent=None, kids=[]): 
        self.state = state #object describing a state of the search processs
        self.parent = parent #pointer to best parent
        self.kids = kids #list of all sucessor nodes

        self.f = 0 #estimated total cost
        self.g = 0 #cost of getting to this node from the root
        self.h = 0 #estimated cost to get to the goal

    def __lt__(self, other):
        return self.f < other.f #returns true if current node has smaller f

#calculates manhattan distance between two nodes
def manhattan_distance(start, goal):
    return goal[0]-start[0] + goal[1]-start[1] #horizontal dist + vertical dist to goal


#generate all possible child nodes of given node
def generate_successors(parent,data):

    #find maximum x and y
    maxX, maxY = len(data), len(data[0])
    
    #current position of parent node 
    x,y = parent.state[0], parent.state[1]

    adjacents=[] #list of possible child nodes
    if y != 0:
        north = [x,y-1]
        northNode = searchNode(north)
        adjacents.append(northNode)
    if y != maxY:
        south = [x,y+1]
        southNode = searchNode(south)
        adjacents.append(southNode)
    if x != 0:
        west = [x-1,y]
        westNode = searchNode(west)
        adjacents.append(westNode)
    if x != maxX:
        east = [x+1,y]
        eastNode = searchNode(east)
        adjacents.append(eastNode)

    #check obstacles
    kids = [] #node of children accesible from parent
    for node in adjacents:
        x,y = node.state[0], node.state[1]
        if data[x,y] != -1:
            kids.append(node)

    return kids

#calculate cost of one movement between parent and child = cost on child position
def arc_cost(p,c, data):
    return data[c.state[0], c.state[1]]
     

#attaches a child node to the best parent node (so far) and updates costs for child
def attach_and_eval(c, p, data, goal):
     c.parent =p #assign parent node to child
     c.g = p.g + arc_cost(p,c,data) #update g value for child
     c.h = manhattan_distance(c.state, goal)
     c.f = c.g + c.h

#recurses through children and possibly many other descendants
def propagate_path_improvement(p, data):
    for c in p.kids:
        if p.g+arc_cost(p,c,data) < c.g:
            c.f = c.g+c.h
            propagate_path_improvement(c,data)

#finds optimal path by iterating back for each best parent
def back_iterate(node, data_str, start):
    x = node.parent.state[0]
    y = node.parent.state[1]
    data_str[x,y] = 0 #indicate path on map
    if node.parent.state != start:
        back_iterate(node.parent, data_str, start)
    return data_str


#implement the best-first-search
def best_first_search(task):

    #set open list and closed list empty
    openList = []
    closedList = []

    #read Map
    samf = Map_Obj(task)
    start, goal, endGoal, path = samf.fill_critical_positions(task)
    data, data_str = samf.read_map(path)

    #generate initial node 
    initialNode = searchNode(start) #assign state as current position of node
    initialNode.h = manhattan_distance(start, goal)
    initialNode.f = initialNode.g + initialNode.h

    #list of nodes visited
    visitedNodes = [] #updated each time a node is added to OpenList

    #push inital node to openList
    heapq.heappush(openList,initialNode)
    visitedNodes.append(initialNode)

    node = initialNode
    #Agenda Loop begins
    while node.state != goal:

        #check if empty list = failure
        if not openList: 
                return 'no solution'

        node = heapq.heappop(openList) #pop node from openList
        
        closedList.append(node) #push note to closedList

      
        
        #check if success
        if node.state == goal:
            data_str = back_iterate(node, data_str, start)
            samf.show_map(data_str)
    
            print('total cost: ', node.f)
            return "success!"

        #generate childnodes/successors
        succ = generate_successors(node,data)

        for s in succ:

            #if s already exists:
            for v in visitedNodes: #search for visited nodes
                if s.state == v.state:
                    s = v #assign preExisting node to s
            
            #add s to kids list of current node
            node.kids.append(s) 

            #if s is a new node
            if (s not in openList ) and (s not in closedList):
                attach_and_eval(s,node, data, goal) #update parent-child realtionship
                heapq.heappush(openList, s) #insert s in openList
                visitedNodes.append(s) #s is now visited


            #if we find a cheaper path to s
            elif node.g + arc_cost(node,s,data) < s.g:
                attach_and_eval(s, node, data, goal)
                if s in closedList:
                    propagate_path_improvement(s, data)

    

if __name__== "__main__":
    
    # task1
    best_first_search(1)
    # task 2
    best_first_search(2)
    # task 3 w/cost
    best_first_search(3)
    # task 4
    best_first_search(4)
    

    
    
    

    



