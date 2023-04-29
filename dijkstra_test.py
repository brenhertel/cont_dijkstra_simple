
import numpy as np
import matplotlib.pyplot as plt

from shapely import Polygon, Point, LineString
from shapely.plotting import plot_polygon, plot_line, plot_points

from queue import PriorityQueue
from dataclasses import dataclass, field
from typing import Any


## STUFF FOR PRIORITY QUEUE

@dataclass(order=True)
class PrioritizedItem:
    priority: float
    item: Any=field(compare=False)

class node(object):
    def __init__(self, state, parent, key):
        self.state = state
        self.parent = parent
        self.key = key
        
    def is_equal(self, compare_state):
        return np.array_equal(self.state, compare_state)
        

class pqueue(PriorityQueue):
    def __init__(self, maxsize=0):            
        super().__init__(maxsize)
        #self.key = key

    def put(self, x):
        super().put(PrioritizedItem(x.key, x))

    def get(self):
        return super().get().item


def generate_children(current, vertices, obstacles):
    child_list = []
    for v in vertices:
        is_visible = True
        for obs in obstacles:
            if LineString([current, v]).crosses(obs):
                is_visible = False
        if is_visible:
            child_list.append(v)
    return child_list

def generate_path(end_node):
    path = [end_node]
    #print(path)
    while path[-1].parent is not None:
        path.append(path[-1].parent)
        #print(path)
    #print(list(reversed(path)))
    return list(reversed(path))
    
def run_dijkstra(start, end, vertices, obstacles):
    in_queue = pqueue()
    in_queue.put(node(start, None, 0.0))
    visited = []
    while not in_queue.empty():
        current = in_queue.get()
        already_visited = False
        for v in visited:
            if v.is_equal(current):
                already_visited = True
        if not already_visited:
                
            if current.is_equal(end):
                #found goal
                print('found goal at depth ', current.key)
                return generate_path(current), visited
            
            children = generate_children(current.state, vertices, obstacles) #generates all visible children
            for child in children:
                already_visited = False
                for v in visited:
                    if v.is_equal(child):
                        already_visited = True
                if not already_visited:
                    in_queue.put(node(child, current, current.key + current.state.distance(child)))
                    
            visited.append(current)
    #unable to find goal
    return [], visited
    
    

def main():
    start = Point(0.0, 0.5)
    end = Point(1.0, 0.5)
    
    triangle_pts = [Point(0.4, 0.6), Point(0.6, 0.6), Point(0.5, 0.3)]
    
    triangle1 = Polygon([(0.4, 0.6), (0.6, 0.6), (0.5, 0.3)])
    #triangle1 = Polygon(triangle_pts)
    
    vertices = triangle_pts + [end]
    
    print(vertices)
    print([triangle1])
    
    path, visited = run_dijkstra(start, end, vertices, [triangle1])
    
    print(path)
    print(visited)
    point_path = []
    for node in path:
        point_path.append(node.state)
    print(point_path)
    line_path = LineString(point_path)
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plot_points(start, ax=ax, color='k')
    plot_points(end, ax=ax, color='k')
    
    plot_line(line_path, ax=ax, add_points=True, color='b', alpha=0.7)
    
    plot_polygon(triangle1, ax=ax, add_points=True, color='g', alpha=0.5)
    
    plt.xlim([-0.1, 1.1])
    plt.ylim([-0.1, 1.1])
    plt.show()

if __name__ == '__main__':
    main()