from random import shuffle, randrange


def small_maze():
    obstacles = [(0, 7), (0, 6), (1, 0),
                 (1, 1), (1, 2), (1, 3), (1, 4), (1, 6), (2, 6), (3, 1), (3, 2), (3, 3), (3, 4),
                 (3, 6), (4, 4), (4, 6), (5, 0), (5, 2), (5, 4), (6, 0), (6, 2), (6, 4), (6, 5),
                 (6, 6), (7, 0), (7, 2), (7, 6), (8, 0), (8, 2), (8, 4), (8, 6), (9, 0), (9, 1),
                 (9, 4), (9, 5), (9, 6), (8, 1)]
    # size, start, end, obstacles
    return (10, 8), (0, 0), (9, 2), set(obstacles)


def large_maze():
    obstacles = []

    for i in range(25, 27):
        for j in range(12):
            obstacles.append((i, j))

    for i in range(35, 36):
        for j in range(12):
            obstacles.append((i, j))

    for i in range(33, 38):
        for j in range(12, 30):
            obstacles.append((i, j))

    return (50, 50), (0, 0), (49, 0), set(obstacles)


def impossible_maze():
    obstacles = []

    for i in range(25, 27):
        for j in range(17):
            obstacles.append((i, j))

    for i in range(35, 36):
        for j in range(19):
            obstacles.append((i, j))

    for i in range(33, 38):
        for j in range(12, 30):
            obstacles.append((i, j))

    for i in range(42, 44):
        for j in range(50):
            obstacles.append((i, j))

    return (50, 50), (0, 0), (49, 0), set(obstacles)
