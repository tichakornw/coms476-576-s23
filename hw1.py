import json, sys, os, argparse
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors
from matplotlib.ticker import AutoMinorLocator
from discrete_search import (
    StateSpace,
    ActionSpace,
    StateTransition,
    fsearch,
    ALG_BFS,
    ALG_DFS,
    ALG_ASTAR,
)


class Grid2DStates(StateSpace):
    def __init__(self, Xmin, Xmax, Ymin, Ymax, O):
        """
        Xmin, Xmax, Ymin, Ymax: integers that defines the boundary of the world
        O:    a list of tuples that represent the grid points
              that are occupied by an obstacle.
              A tuple (i, j) in O means that grid point (i,j) is occupied.
        """
        self.Xmin = Xmin
        self.Xmax = Xmax
        self.Ymin = Ymin
        self.Ymax = Ymax
        self.O = O

    def __contains__(self, x):
        # TODO: Implement this function
        raise NotImplementedError

    def get_distance_lower_bound(self, x1, x2):
        # TODO: Implement this function
        raise NotImplementedError

    def draw(self, ax, grid_on=True, tick_step=[1, 1]):
        G = np.zeros((self.Ymax - self.Ymin + 1, self.Xmax - self.Xmin + 1))
        for obs in self.O:
            # When representing the grid as a 2-dimensional list,
            # its indices are such that G[i][j] corresponds to cell (j, i).
            row_idx = obs[1] - self.Ymin
            col_idx = obs[0] - self.Xmin
            assert row_idx >= 0 and col_idx >= 0
            G[row_idx][col_idx] = 1

        # Define grid colors
        cmap = colors.ListedColormap(["white", "slategrey"])
        bounds = [-0.5, 0.5, 1.5]
        norm = colors.BoundaryNorm(bounds, cmap.N)

        # Draw grid
        ax.imshow(
            G,
            cmap=cmap,
            norm=norm,
            origin="lower",
            extent=(self.Xmin - 0.5, self.Xmax + 0.5, self.Ymin - 0.5, self.Ymax + 0.5),
        )

        # Draw grid lines and labels
        if grid_on:
            ax.grid(which="minor", axis="both", linestyle="-", color="k", linewidth=2)

        ax.set_xticks(np.arange(self.Xmin, self.Xmax + 1, tick_step[0]))
        ax.set_yticks(np.arange(self.Ymin, self.Ymax + 1, tick_step[1]))

        minor_locator = AutoMinorLocator(2)
        ax.tick_params(which="minor", width=0)
        ax.tick_params(which="minor", length=0)
        ax.xaxis.set_minor_locator(minor_locator)
        ax.yaxis.set_minor_locator(minor_locator)

        ax.set_xlim(self.Xmin - 0.5, self.Xmax + 0.5)
        ax.set_ylim(self.Ymin - 0.5, self.Ymax + 0.5)


class GridStateTransition(StateTransition):
    def __call__(self, x, u):
        # TODO: Implement this function
        raise NotImplementedError


class Grid2DActions(ActionSpace):
    all_actions = [(0, 1), (0, -1), (-1, 0), (1, 0)]

    def __init__(self, X, f):
        self.X = X
        self.f = f

    def __call__(self, x):
        # TODO: Implement this function
        raise NotImplementedError


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="Run forward search")
    parser.add_argument(
        "desc",
        metavar="problem_description_path",
        type=str,
        help="path to the problem description file containing the grid configuration, the initial cell, and the goal region",
    )
    parser.add_argument(
        "--alg",
        choices=[ALG_BFS, ALG_DFS, ALG_ASTAR],
        required=False,
        default=ALG_BFS,
        dest="alg",
        help="search algorithm, default to bfs",
    )
    parser.add_argument(
        "--out",
        metavar="output_path",
        type=str,
        required=False,
        default="",
        dest="out",
        help="path to the output file",
    )

    args = parser.parse_args(sys.argv[1:])
    if not args.out:
        args.out = (
            os.path.splitext(os.path.basename(args.desc))[0] + "_" + args.alg + ".json"
        )

    print("Problem description: ", args.desc)
    print("Search algorithm:    ", args.alg)
    print("Output:              ", args.out)

    return args


def parse_desc(desc):
    """Parse problem description json file to get the problem description"""
    with open(desc) as desc:
        data = json.load(desc)

    M = data["M"]
    N = data["N"]
    O = [tuple(x) for x in data["O"]]
    xI = tuple(data["xI"])
    XG = [tuple(x) for x in data["XG"]]
    return (M, N, O, xI, XG)


def draw_path(ax, path):
    """Draw the path on the axis ax"""
    if not path:
        return

    data = np.asarray(path)
    ax.plot(data[:, 0], data[:, 1], "r-", linewidth=5)


def mark_cell(ax, cells):
    """Mark cells on the axis ax"""
    if not cells:
        return

    data = np.asarray(cells)
    ax.plot(data[:, 0], data[:, 1], "bx", markersize=5)


if __name__ == "__main__":
    args = parse_args()
    (M, N, O, xI, XG) = parse_desc(args.desc)
    X = Grid2DStates(0, M, 0, N, O)
    f = GridStateTransition()
    U = Grid2DActions(X, f)

    result = fsearch(X, U, f, xI, XG, args.alg)

    with open(args.out, "w") as outfile:
        json.dump(result, outfile)

    fig, ax = plt.subplots()
    X.draw(ax)
    draw_path(ax, result["path"])
    mark_cell(ax, result["visited"])
    plt.show()
