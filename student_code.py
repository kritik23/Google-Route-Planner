import heapq
import math
from collections import namedtuple

Cost_r = namedtuple('Cost', ['total', 'journey', 'to_goal'])
Path_r = namedtuple('Path', ['cost', 'intersections', 'previous', 'frontier']) 

"""
Returns Euclidean distance
"""
def euclidean_distance(origin: [float, float], destination: [float, float]) -> float:
    return math.sqrt((origin[0] - destination[0])**2 + (origin[1] - destination[1])**2)

"""
Estimates the distance between the current path frontier point and the goal. Accomplished the A* optimization
and returns estimated Euclidean distance
"""
def estimated_distance(frontier_updated: [float, float], goal: [float, float]) -> float:
    return euclidean_distance(origin=frontier_updated, destination=goal)


"""
Returns new point with path costs and intersections
path = current traversed path
new_frontier = coordinates of next point to add to the path
"""
def update_path(map: object, path: Path_r, new_frontier: int, goal: int) -> Path_r:
    #euclidean distance of path already been traversed
    path_traversed = euclidean_distance(origin=map.intersections[path.frontier], destination=map.intersections[new_frontier])
    #cost of new path
    cost_newPath = path.cost.journey + path_traversed
    #cost of new Goal if frontier changes
    cost_newGoal = estimated_distance(frontier_updated=map.intersections[new_frontier], goal=map.intersections[goal])
    totalCost_newPath = cost_newPath + cost_newGoal

    path_newIntersections = path.intersections + [new_frontier]

    newPath = Path_r(Cost_r(totalCost_newPath, cost_newPath, cost_newGoal),
                    path_newIntersections, path.frontier, new_frontier)

    return newPath

"""
returns the shortest path, based on A* algorithm
"""
def shortest_path(map: object, start: int, goal: int) -> list:
    paths = []
    path_goal_min = None
    path_goal_min_val = float('inf')
    if start == goal:       # Check if already on the goal
        return [start]  

    #defined intial start location    
    goal_initial_distance = estimated_distance(frontier_updated=map.intersections[start], goal=map.intersections[goal])
    #initializing path with start and 0
    path = Path_r(Cost_r(goal_initial_distance, 0, goal_initial_distance), [start], start, start)

    heapq.heappush(paths, path) 

    while len(paths) >= 1:
        path_nearestFrontier = heapq.heappop(paths)
        for neighborPath in map.roads[path_nearestFrontier.frontier]:

            if neighborPath == path_nearestFrontier.previous:  # Avoid going back
                continue
            else: 
                newPath = update_path(map=map, path=path_nearestFrontier, new_frontier=neighborPath, goal=goal)
                if neighborPath != goal:                        # check if not reached goal
                    if path_goal_min is None:                    # if not found the goal with path
                        heapq.heappush(paths, newPath)           # Goal not found, still exploring         
                    else:  
                        if newPath.cost.total < path_goal_min_val:  # Costly and not reached the goal
                            heapq.heappush(paths, newPath)          # Cheaper path, still exploring
                        else:  
                            pass 
                else:
                    if newPath.cost.total < path_goal_min_val:      # select newPath with previous path
                        path_goal_min_val = newPath.cost.total
                        path_goal_min = newPath.intersections
                    else:                                           # Reached destination, with higher cost -> disregard
                        pass
                    

    if path_goal_min is None:
        return -1
    else:
        return path_goal_min