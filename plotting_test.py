
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
        #print('Comparing', self.state, compare_state, np.array_equal(self.state, compare_state))
        return np.array_equal(self.state, compare_state)
        

class pqueue(PriorityQueue):
    def __init__(self, maxsize=0):            
        super().__init__(maxsize)
        #self.key = key

    def put(self, x):
        super().put(PrioritizedItem(x.key, x))

    def get(self):
        return super().get().item


def plot_current(explored, wavefronts, visible_lines, vertices, obstacles):
    waves = []
    #print(explored)
    #print(wavefronts)
    for i in range(len(explored)):
        if wavefronts[i] > 0:
            waves.append(explored[i].buffer(wavefronts[i]))
    #print(waves)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    #plot_points(start, ax=ax, color='k')
    #plot_points(end, ax=ax, color='k')
    plot_points(vertices, ax=ax, color='k')
    for w in waves:
        plot_polygon(w, ax=ax, color='b', add_points=False, linewidth=1, alpha=0.3)
    for obs in obstacles:
        plot_polygon(obs, ax=ax, add_points=True, color='g', alpha=0.5)
    for line in visible_lines:
        plot_line(line, ax=ax, color='r')
    
    plt.xlim([-0.1, 1.1])
    plt.ylim([-0.1, 1.1])
    #plt.show()
    plt.savefig("dijkstra2_" + str(len(explored)) + ".png", bbox_inches='tight', dpi=300)
    plt.close('all')

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
            if v.is_equal(current.state):
                already_visited = True
        if not already_visited:
            explored = [v.state for v in visited]
            wavefronts = [current.key - v.key for v in visited]
            visible_lines = [LineString([v.parent.state, v.state]) for v in visited[1:]]
            if current.parent is not None:
                visible_lines = visible_lines + [LineString([current.parent.state, current.state])]
            plot_current(explored, wavefronts, visible_lines, [start] + vertices, obstacles)
                
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
    
    triangle_pts = [Point(0.4, 0.65), Point(0.6, 0.65), Point(0.5, 0.4)]
    
    triangle1 = Polygon([(0.4, 0.65), (0.6, 0.65), (0.5, 0.4)])
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
    #plt.show()
    plt.savefig("dijkstra2_final.png", bbox_inches='tight', dpi=300)
    plt.close('all')

if __name__ == '__main__':
    main()