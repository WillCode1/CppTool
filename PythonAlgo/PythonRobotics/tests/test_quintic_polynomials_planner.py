import conftest  # Add root path to sys.path
from PathPlanning.QuinticPolynomialsPlanner import quintic_polynomials_planner as m


def test1():
    m.show_animation = False
    m.main()


if __name__ == '__main__':
    conftest.run_this_test(__file__)
