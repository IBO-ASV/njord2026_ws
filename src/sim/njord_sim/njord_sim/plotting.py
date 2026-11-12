"""Headless, exportable run plots using the same saved truth samples."""


def plot_run(scenario, samples, directory):
    if not samples:
        return
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    from .scenario import dock_segments

    fig, (axis, speed) = plt.subplots(1, 2, figsize=(11, 5))
    route = scenario.route.waypoints
    axis.plot(
        [w.x for w in route],
        [w.y for w in route],
        "--o",
        label="Ideal GPS route",
        markersize=3,
    )
    axis.plot(
        [row[1] for row in samples],
        [row[2] for row in samples],
        label="Simulated truth",
    )
    for w in route:
        axis.annotate(w.competition_id or w.waypoint_id, (w.x, w.y), fontsize=7)
    for x, y, mark in scenario.buoys:
        axis.add_patch(
            plt.Circle(
                (x, y),
                0.175,
                color="red"
                if mark == "RED"
                else "green"
                if mark == "GREEN"
                else "black",
            )
        )
        axis.annotate(mark, (x, y), fontsize=8)
    for dock in scenario.docks:
        for a, b in dock_segments(dock):
            axis.plot([a[0], b[0]], [a[1], b[1]], color="gray", linewidth=3)
    axis.set_aspect("equal")
    axis.grid()
    axis.legend()
    axis.set_xlabel("East [m]")
    axis.set_ylabel("North [m]")
    speed.plot([row[0] for row in samples], [row[4] for row in samples])
    speed.grid()
    speed.set_xlabel("Elapsed [s]")
    speed.set_ylabel("Speed [m/s]")
    fig.suptitle(scenario.task.task_id + " — SIMULATION")
    fig.tight_layout()
    fig.savefig(directory / "trajectory.png", dpi=140)
    plt.close(fig)
