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


def main():
    """Render saved geometry and truth without starting any ROS processes."""
    import argparse
    import csv
    import json
    from pathlib import Path
    from types import SimpleNamespace

    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    directory = args.directory.resolve()
    saved = json.loads((directory / "config/scenario.json").read_text())
    scenario = SimpleNamespace(
        task=SimpleNamespace(**saved["task"]),
        route=SimpleNamespace(
            waypoints=[SimpleNamespace(**w) for w in saved["route"]["waypoints"]]
        ),
        buoys=saved["buoys"],
        docks=saved["docks"],
    )
    with (directory / "trajectory.csv").open() as source:
        rows = csv.reader(source)
        next(rows)
        samples = [[float(value) for value in row] for row in rows]
    plot_run(scenario, samples, directory)


if __name__ == "__main__":
    main()
