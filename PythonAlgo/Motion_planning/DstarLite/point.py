from Motion_planning.DstarLite import graphics


# Implementation of a point class representing cells in the occupancy grid

class Point:
    # Initialize a point with x and y coordinates
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # Convert point to a drawable object
    def to_graphics_rect(self, width_scale, height_scale):
        top_left = graphics.Point(self.x * width_scale, self.y * height_scale)
        top_right = graphics.Point((self.x + 1) * width_scale, (self.y + 1) * height_scale)
        return graphics.Rectangle(top_left, top_right)

    # Generate string representation of a point
    def __str__(self):
        return "({}, {})".format(self.x, self.y)

    # Generate printable version of a point
    def __repr__(self):
        return str(self)

    # Generate the hash of a point for fast lookup
    def __hash__(self):
        return hash((self.x, self.y))

    # Check if two point objects are equal
    def __eq__(self, other):
        return isinstance(other, Point) and self.x == other.x and self.y == other.y

    # Check if a point object is less than another
    def __lt__(self, other):
        return False
