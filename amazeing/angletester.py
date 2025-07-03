import math
import heapq
import numpy as np

print(math.atan2(51-52,21-24))

bestpath = [(24, 47), (24, 48), (24, 49), (24, 50), (24, 51), (25, 52), (24, 53), (23, 54), (23, 55),
          (23, 56), (23, 57), (23, 58), (23, 59), (23, 60), (23, 61), (23, 62), (23, 63), (23, 64), (23, 65), (23, 66), 
          (23, 67), (23, 68), (23, 69), (23, 70), (23, 71), (23, 72), (24, 73), (25, 74), (26, 75), (27, 76), (28, 77), 
          (29, 78), (30, 79), (31, 80), (32, 81), (33, 82), (34, 83), (35, 84), (36, 85)]
bestpath.reverse()

data = [[0, 0, 0, 0, 0, 0, 0, 100, 100, 0, 0, 0],
        [0, 100, 0, 100, 0, 0, 0, 0, 100, 0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0,0]]

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
costmap(data,12,5,1)


neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

neighbors.reverse()
print(neighbors+[[(1,1),(2,1),(3,1)]])

# lst = []
# print(lst)

# print(bestpath)
#         # goal to turn from current orientation at (24,47) to (24, 51)
# # gridcoord = bestpath.pop(4)
# # print(gridcoord)
# # print(bestpath)

# # b = 'b'
# # a = dict()
# # a[b] = 1
# # a['c'] = 2

# # print(a.items())

# # for i in a.keys():
# #     print(a[i])

