from Motion_planning.graph_search.tool import *


def dijkstra_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None     # 当前点：来源点，映射
    cost_so_far[start] = 0      # 当前点：到起点的cost，映射

    while not frontier.empty():
        current = frontier.get()  # 优先选取低cost的路径

        if current == goal:
            break

        for neighbor in graph.neighbors(current):
            # 当前neighbor cost + 自起点累计cost
            new_cost = cost_so_far[current] + graph.cost(current, neighbor)
            # 没有走过的点 或者 当前走法比之前的cost更小
            if (neighbor not in came_from) or (new_cost < cost_so_far[neighbor]):
                came_from[neighbor] = current
                cost_so_far[neighbor] = new_cost
                frontier.put(neighbor, new_cost)

    return came_from, cost_so_far


diagram4 = GridWithWeights(10, 10)
diagram4.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
diagram4.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                       (4, 3), (4, 4), (4, 5), (4, 6),
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6),
                                       (5, 7), (5, 8), (6, 2), (6, 3),
                                       (6, 4), (6, 5), (6, 6), (6, 7),
                                       (7, 3), (7, 4), (7, 5)]}

came_from, cost_so_far = dijkstra_search(diagram4, (1, 4), (7, 8))
draw_grid(diagram4, width=3, point_to=came_from, start=(1, 4), goal=(7, 8))
print()
draw_grid(diagram4, width=3, number=cost_so_far, start=(1, 4), goal=(7, 8))
print()
draw_grid(diagram4, width=3, path=reconstruct_path(came_from, start=(1, 4), goal=(7, 8)))
