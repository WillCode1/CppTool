# Implementation of a grid class for the occupancy grid

from Motion_planning.DstarLite import graphics
from Motion_planning.DstarLite.point import Point
import time
import math

# Parameter to control how far the robot can see (default is 1)
# 可视半径
VISIBILITY = 1

# Integer representing infinity (for edge costs etc.)
INT_MAX = 10000

# Size of graphics window for grid
WINDOW_WIDTH = 600


class Map:

    # Initialize a grid of given height and width and set the start and
    # goal points and the obstacles on the grid.
    def __init__(self, width, height, obstacles, goal, start):
        self.width = width
        self.height = height
        self.win = None
        self.robot_elem = None
        self.robot_pos = None
        self.goal = goal

        # Required for fast draw
        self.to_draw_cells = []

        # Initialize the obstacles on the grid
        # 0：不可见，1：可见
        self.obstacles = dict((Point(o[0], o[1]), 0) for o in obstacles)

        # Intialize the edges for the grid to be euclidean distance between
        # the cells
        self.edges = [[{} for i in range(height)] for j in range(width)]

        for i in range(width):
            for j in range(height):
                surrounding_cells = []
                for o1 in [-1, 0, 1]:
                    for o2 in [-1, 0, 1]:
                        if o1 == 0 and o2 == 0:
                            continue

                        if i + o1 < 0 or j + o2 < 0:
                            continue

                        if i + o1 >= width or j + o2 >= height:
                            continue

                        surrounding_cells.append(Point(i + o1, j + o2))

                for c in surrounding_cells:
                    self.edges[i][j][c] = math.sqrt((c.y - j) ** 2 + (c.x - i) ** 2)

        # Initialize the robot to be at the start position
        self.move_to(start, True)

    # Find the edge cost between two cells.
    def get_distance(self, fromp, to):
        return self.edges[fromp.x][fromp.y].get(to, INT_MAX)

    # Find the successors or predecessors of any cell. Return cells that are
    # obstacles if show_obstacles is set to true.
    def children(self, point, show_obstacles=False):
        children = []

        for point, dist in self.edges[point.x][point.y].items():
            if show_obstacles or dist < INT_MAX:
                children.append(point)

        return children

    # Find the cells that are within the robot's vision recursively
    def get_visible_cells(self, position, depth):

        # Return empty list if robot cannot see beyond current depth
        if depth >= VISIBILITY:
            return []

        # Add all children cells of current cell since they are visible
        visible_cells = set(self.children(position))

        # Find children of children cells recursively i.e. explore next level
        # of visibility.
        neighbors = self.children(position)

        for cell in neighbors:
            child_neighbors = self.get_visible_cells(cell, depth + 1)
            for child_cell in child_neighbors:
                if child_cell not in visible_cells and child_cell != position:
                    visible_cells.add(child_cell)

        return visible_cells

    # Move the robot to a new position and update the edge costs based on
    # any newly discovered obstacles if add_edge is set to the true.
    def move_to(self, position, add_edge=False):

        # Move robot to new position
        self.robot_pos = position

        # Find cells that are visible to the robot at the new position
        visible_cells = self.get_visible_cells(position, 0)

        # Find any new obstacles that have now been discovered
        updated = []

        for obs, seen in self.obstacles.items():

            # Check if obstacle is visible and if it has been seen before
            if obs not in visible_cells or seen != 0:
                continue

            # Mark new obstacle as seen and add it to list of cells to be
            # drawn on the grid
            self.obstacles[obs] = 1
            self.to_draw_cells.append(obs)

            # Find all edges around new obstacle since edge costs for these
            # edges have now changed
            neighbors = self.children(obs)

            for n in neighbors:
                updated.append((n, obs))
                updated.append((obs, n))

                # Update the edge costs if add_edge is set to true
                if add_edge:
                    self.edges[n.x][n.y][obs] = INT_MAX
                    self.edges[obs.x][obs.y][n] = INT_MAX

        # Return list of edges whose costs have changed
        return updated

    # Draw the grid to visualize the robot and its current path
    def draw(self):

        # Define the size and scale of the window
        window_height = WINDOW_WIDTH * (self.height / self.width)
        width_scale = WINDOW_WIDTH / self.width
        height_scale = window_height / self.height

        if self.win is None:
            # Initialize the window for the graphics, since it hasn't been
            # created yet.
            self.win = graphics.GraphWin('SimSpace', WINDOW_WIDTH, window_height)

            # Draw the robot.
            pt = self.robot_pos.to_graphics_rect(width_scale, height_scale)
            pt.draw(self.win)
            pt.setFill('red')

            pt2 = self.goal.to_graphics_rect(width_scale, height_scale)
            pt2.draw(self.win)
            pt2.setFill('grey')
            pt2.setOutline('grey')

            self.robot_elem = pt

        else:
            # Create a blue trail for the robot.
            self.robot_elem.setFill('blue')

            # Draw the new robot position.
            self.robot_elem = self.robot_pos.to_graphics_rect(width_scale, height_scale)
            self.robot_elem.draw(self.win)
            self.robot_elem.setFill('red')

        # Draw all the new obstacles we need to draw, and indicate that those
        # obstacles have been drawn, by removing them from the array.
        for o in self.to_draw_cells:
            r = o.to_graphics_rect(width_scale, height_scale)
            r.draw(self.win)
            r.setFill('black')

        self.to_draw_cells = []

        # Sleep between frames.
        time.sleep(0.1)
