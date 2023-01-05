import math


def draw(ax, cspace, obstacles, qI, qG, G, path, title=""):
    """Plot the C-space, obstacles, qI, qG, and graph on the axis ax

    @type ax: axes.Axes, created, e.g., fig, ax = plt.subplots()
    @type cspace: a list [(xmin, xmax), (ymin, ymax)] indicating that the C-space
        is given by [xmin, xmax] \times [ymin, ymax].
    @type obstacles: a list [obs_1, ..., obs_m] of obstacles, where obs_i is a list of coordinates
        on the boundary of the i^{th} obstacle.
    @type qI: a tuple (x, y), indicating the initial configuration.
    @type qG: a tuple (x, y), indicating the goal configuration
    @type path: a list of tuples specifying the sequence of configurations visited along the path
    @type title: a string, indicating the title of the plot
    """

    draw_cspace(ax, cspace, obstacles)
    G.draw(ax)
    if qI is not None:
        if len(qI) == 2:
            ax.plot(qI[0], qI[1], "bx", markersize=10)
        elif len(qI) == 3:
            ax.plot(
                qI[0],
                qI[1],
                marker=(3, 0, qI[2] * 180 / math.pi - 90),
                markersize=15,
                linestyle="None",
                markerfacecolor="blue",
                markeredgecolor="blue",
            )
    if qG is not None:
        if len(qI) == 2:
            ax.plot(qG[0], qG[1], "bo", markersize=10)
        elif len(qG) == 3:
            ax.plot(
                qG[0],
                qG[1],
                marker=(3, 0, qG[2] * 180 / math.pi - 90),
                markersize=15,
                linestyle="None",
                markerfacecolor="red",
                markeredgecolor="red",
            )
    if len(path) > 0:
        ax.plot(
            [state[0] for state in path],
            [state[1] for state in path],
            "b-",
            linewidth=5,
        )
    if len(title) > 0:
        ax.set_title(title, fontsize=20)


def draw_cspace(ax, cspace, obstacles, tick_step=[1, 1]):
    """Draw the C-space and C-space obstacles on the axis ax

    @type cspace: a list [(xmin, xmax), (ymin, ymax)] indicating that the C-space
        is given by [xmin, xmax] \times [ymin, ymax].
    @type obstacles: a list [obs_1, ..., obs_m] of obstacles, where obs_i is a list of coordinates
        on the boundary of the i^{th} obstacle.
    """
    for obs in obstacles:
        ax.plot([v[0] for v in obs], [v[1] for v in obs], "r-", linewidth=3)

    ax.set_xticks(
        range(math.ceil(cspace[0][0]), math.floor(cspace[0][1]) + 1, tick_step[0])
    )
    ax.set_yticks(
        range(math.ceil(cspace[1][0]), math.floor(cspace[1][1]) + 1, tick_step[1])
    )
    ax.set(xlim=cspace[0], ylim=cspace[1])
    ax.set_aspect("equal", adjustable="box")
    ax.tick_params(axis="x", labelsize=20)
    ax.tick_params(axis="y", labelsize=20)
