from Motion_planning.graph_search.tool import *


# 网格形式的图，如果用距离来代表损失则有以下这些启发函数可以使用：
# · 如果图形中只允许朝上下左右四个方向移动，则可以使用曼哈顿距离（Manhattan distance）。
# · 如果图形中允许朝八个方向移动，则可以使用对角距离。切比雪夫距离
# · 如果图形中允许朝任何方向移动，则可以使用欧几里得距离（Euclidean distance）。
def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


# 优先级兼顾成本和距离
def a_star_search(graph, start, goal):
    open_set = PriorityQueue()
    open_set.put(start, 0)
    closed_set = {}
    cost_so_far = {}
    closed_set[start] = None  # 当前点：来源点，映射
    cost_so_far[start] = 0  # 当前点：到起点的cost，映射

    while not open_set.empty():
        current = open_set.get()

        if current == goal:
            break

        for neighbor in graph.neighbors(current):
            # 当前neighbor cost + 自起点累计cost
            new_cost = cost_so_far[current] + graph.cost(current, neighbor)
            # 没有走过的点 或者 当前走法比之前的cost更小
            if (neighbor not in closed_set) or (new_cost < cost_so_far[neighbor]):
                closed_set[neighbor] = current
                cost_so_far[neighbor] = new_cost
                # Greedy Best-First Search use heuristic(goal, neighbor)
                open_set.put(neighbor, new_cost + heuristic(goal, neighbor))

    return closed_set, cost_so_far


start, goal = (1, 4), (7, 8)
diagram4 = GridWithWeights(10, 10)
diagram4.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
diagram4.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                       (4, 3), (4, 4), (4, 5), (4, 6),
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6),
                                       (5, 7), (5, 8), (6, 2), (6, 3),
                                       (6, 4), (6, 5), (6, 6), (6, 7),
                                       (7, 3), (7, 4), (7, 5)]}
came_from, cost_so_far = a_star_search(diagram4, start, goal)
draw_grid(diagram4, width=3, point_to=came_from, start=start, goal=goal)
print()
draw_grid(diagram4, width=3, number=cost_so_far, start=start, goal=goal)
print()
draw_grid(diagram4, width=3, path=reconstruct_path(came_from, start=(1, 4), goal=(7, 8)))
