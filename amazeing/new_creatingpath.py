
import numpy as np
import heapq

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    

def astar(array, start, goal):    #returns a list of poses which is a path of lowest cost
                                  #takes in mapdata as array
                                  #start and goal in terms of grid coordinates
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}   #the distances between the current pose and the neighbour
    fscore = {start:heuristic(start, goal)}   # distance between neighbour and goal
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]   #after the for loop this will see the nearest neigbour and then go from there
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]   #if goal reached reconstruct path
            data = data + [start]
            data.reverse()
            return data
        close_set.add(current)     
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:                   #checking if neighbour in map bounds x
                if 0 <= neighbor[1] < array.shape[1]:                # checking if neighbour in map bounds y
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data
        
    return False

def costmap(data,width,height,resolution):
    data = np.array(data).reshape(height,width)
    print(data)
    wall = np.where(data == 100)
    print(wall)
    for i in range(-2,2+1):
        for j in range(-2,2+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    print(data)
    return data