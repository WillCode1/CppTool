from Motion_planning.graph_search.tool import *


def breadth_first_search_3(graph, start, goal):
    frontier = Queue()
    frontier.put(start)
    came_from = {}
    came_from[start] = None     # 当前点：来源点，映射

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for neighbor in graph.neighbors(current):
            if neighbor not in came_from:
                frontier.put(neighbor)
                came_from[neighbor] = current

    return came_from


DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in
                  [21, 22, 51, 52, 81, 82, 93, 94, 111, 112, 123, 124, 133, 134, 141, 142, 153, 154, 163, 164, 171, 172,
                   173, 174, 175, 183, 184, 193, 194, 201, 202, 203, 204, 205, 213, 214, 223, 224, 243, 244, 253, 254,
                   273, 274, 283, 284, 303, 304, 313, 314, 333, 334, 343, 344, 373, 374, 403, 404, 433, 434]]

g = SquareGrid(30, 15)
g.walls = DIAGRAM1_WALLS

parents = breadth_first_search_3(g, (8, 7), (17, 2))
draw_grid(g, width=2, point_to=parents, start=(8, 7), goal=(17, 2))
draw_grid(g, width=2, path=reconstruct_path(parents, start=(8, 7), goal=(17, 2)))
