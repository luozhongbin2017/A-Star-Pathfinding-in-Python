####################################
#Title: A* Pathfinding Code in Python3
#Author: Liam Speakman
#Notes: Enjoy the code! :D Let me know if there is anything I could improve on as I am still learning Python!
#Copyright (C) 2018 Liam Speakman <lspeakman001@gmail.com>
#You are free to copy, redistribute, and modify this work with no permission from the author
####################################
import math

class Node:
    def __init__(self, x, y, type,blocked):
        self.x=x
        self.y=y
        self.type=type
        self.blocked = blocked
    gcost = 0
    fcost = 0
    hcost = 0
    parent = None

class Astar:
    openList = []
    closedList = []
    path = []
    nodeMap = []

    def __init__(self):
        # create nodeMap
        for i in range(len(map)):
            self.nodeMap.append([])
            for j in range(len(map[0])):
                self.nodeMap[i].append([])
                self.nodeMap[i][j] = Node(i, j, "█" if map[i][j] == 1 else "░", map[i][j] == 1)
    def check_cost(self,checkedNode,endNode,gpast):
        checkedNode.gcost = gpast
        checkedNode.hcost = math.sqrt((endNode.x-checkedNode.x)**2+(endNode.y-checkedNode.y)**2)
        checkedNode.fcost = checkedNode.gcost+checkedNode.fcost

    def checkNeighbor(self,node,endNode):
        if not self.is_in_closed_list(node) and node.blocked == False:
            self.check_cost(node, endNode, self.current.gcost)
            if not self.is_in_open_list(node):
                self.openList.append(node)
            if node.parent is None:
                node.parent = self.current

    def check_if_valid_node(self,x,y):
        if x >= 0 and x<10:
            if y >= 0 and y < 10:
                return True
        return False
    def reconstruct_path(self):
        temp_node = self.current
        self.path.append(temp_node)
        while temp_node.parent != None:
            self.path.append(temp_node.parent)
            temp_node = temp_node.parent
        return self.path
    def calc_path(self,startNode,endNode):
        self.openList = []
        self.closedList = []
        self.openList.append(startNode)
        while len(self.openList) > 0:
            self.look_for_lowest_cost()
            if self.current == endNode:
                self.path = self.reconstruct_path()
                return self.path
            if self.check_if_valid_node(self.current.x,self.current.y+1):
                self.checkNeighbor(self.nodeMap[self.current.x][self.current.y + 1],endNode)
            if self.check_if_valid_node(self.current.x,self.current.y-1):
                self.checkNeighbor(self.nodeMap[self.current.x][self.current.y - 1],endNode)
            if self.check_if_valid_node(self.current.x+1,self.current.y):
                self.checkNeighbor(self.nodeMap[self.current.x+1][self.current.y],endNode)
            if self.check_if_valid_node(self.current.x-1,self.current.y):
                self.checkNeighbor(self.nodeMap[self.current.x-1][self.current.y],endNode)

            self.openList.remove(self.current)
            self.closedList.append(self.current)
        return self.path
    def look_for_lowest_cost(self):
        self.current = self.openList[0]
        for i in range(len(self.openList)):
            if self.current.fcost >= self.openList[i].fcost:
                self.current = self.openList[i]
    def is_in_closed_list(self,node):
        for i in range(len(self.closedList)):
            if node == self.closedList[i]:
                return True
        return False
    def is_in_open_list(self,node):
        for i in range(len(self.openList)):
            if node == self.openList[i]:
                return True
        return False

def printPath(width,height,nodeMap):
    for i in range(width):
        for j in range(height):
            print(nodeMap[i][j].type, end='')
        print()


map = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
       [1, 0, 1, 0, 0, 0, 0, 0, 0, 1],
       [1, 0, 1, 0, 0, 1, 0, 1, 0, 1],
       [1, 0, 1, 0, 0, 1, 0, 1, 0, 1],
       [1, 0, 1, 0, 0, 1, 0, 0, 0, 1],
       [1, 0, 1, 1, 0, 1, 1, 1, 0, 1],
       [1, 0, 0, 0, 0, 0, 0, 1, 0, 1],
       [1, 0, 1, 1, 1, 1, 0, 1, 0, 1],
       [1, 0, 0, 0, 1, 0, 0, 1, 0, 1],
       [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]
astar = Astar()
printPath(len(astar.nodeMap), len(astar.nodeMap[0]), astar.nodeMap)
#create start and end nodes
startNode = astar.nodeMap[1][1]
startNode.type = 3
endNode = astar.nodeMap[8][8]
endNode.type = 4
#calculate path
path = astar.calc_path(startNode, endNode)
print()
for i in range(len(path)):
    path[i].type = "▓"
printPath(len(astar.nodeMap), len(astar.nodeMap[0]), astar.nodeMap)


