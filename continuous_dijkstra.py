
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
            #print(LineString([current, v]))
            #print(LineString([current, v]).contains(obs))
            #print(obs.contains(LineString([current, v])))
            #print(LineString([current, v]).crosses(obs))
            #print(obs.crosses(LineString([current, v])))
            if LineString([current, v]).crosses(obs) or obs.contains(LineString([current, v])):
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
            #plot_current(explored, wavefronts, visible_lines, [start] + vertices, obstacles)
                
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
    
def generate_polygon_vertices(center, radius, num_vertices):
    vertex_angles = [np.random.uniform(0.25, 1.0) * 2 * np.pi for n in range(num_vertices)]
    vertex_rads = [np.random.uniform(0.25, 1.0) * radius for n in range(num_vertices)]
    angles_sorted = np.sort(vertex_angles)
    point_list = []
    vertex_list = []
    for i in range(num_vertices):
        px = round(center[0] + vertex_rads[i] * np.cos(angles_sorted[i]), 2)
        py = round(center[1] + vertex_rads[i] * np.sin(angles_sorted[i]), 2)
        point_list.append(Point(px, py))
        vertex_list.append((px, py))
    return point_list, vertex_list
    
def main():
    start = Point(0.0, 0.0)
    end = Point(1.0, 1.0)
    
    #triangle_pts = [Point(0.4, 0.65), Point(0.6, 0.65), Point(0.5, 0.4)]
    
    #triangle1 = Polygon([(0.4, 0.65), (0.6, 0.65), (0.5, 0.4)])
    #triangle1 = Polygon(triangle_pts)
    obs1_points, obs1_verts = generate_polygon_vertices((0.25, 0.25), 0.25, 5)
    obs1 = Polygon(obs1_verts)
    obs2_points, obs2_verts = generate_polygon_vertices((0.6, 0.4), 0.2, 4)
    obs2 = Polygon(obs2_verts)
    obs3_points, obs3_verts = generate_polygon_vertices((0.75, 0.75), 0.25, 5)
    obs3 = Polygon(obs3_verts)
    obs4_points, obs4_verts = generate_polygon_vertices((0.4, 0.6), 0.2, 4)
    obs4 = Polygon(obs4_verts)
    obs5_points, obs5_verts = generate_polygon_vertices((0.5, 0.5), 0.1, 3)
    obs5 = Polygon(obs5_verts)
    
    vertices = obs1_points + obs2_points + obs3_points + obs4_points + obs5_points + [end]
    
    obstacles = [obs1, obs2, obs3, obs4, obs5]
    print(vertices)
    print(obstacles)
    
    
    path, visited = run_dijkstra(start, end, vertices, obstacles)
    
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
    
    for obs in obstacles:
        plot_polygon(obs, ax=ax, add_points=True, color='g', alpha=0.5)
    plot_line(line_path, ax=ax, add_points=True, color='b', alpha=0.7)
    
    plt.xlim([-0.1, 1.1])
    plt.ylim([-0.1, 1.1])
    plt.show()
    #plt.savefig("dijkstra2_final.png", bbox_inches='tight', dpi=300)
    #plt.close('all')

if __name__ == '__main__':
    main()