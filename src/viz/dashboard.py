"""Real-time mission dashboard (matplotlib, step-driven from the main simulation loop)."""

from __future__ import annotations

import math
from collections import deque
from typing import TYPE_CHECKING, Any

import numpy as np

from src.core.drone import Drone
from src.core.environment import FREE, HAZARD, OBSTACLE, Environment
from src.core.mission_manager import MissionManager
from src.decision.fsm import AutonomyFSM
from src.planners.base_planner import BasePlanner

if TYPE_CHECKING:
    from src.faults.fault_injector import FaultInjector


def _drone_triangle_xy(cx: float, cy: float, heading: float, radius: float = 0.42) -> np.ndarray:
    """Vertices of an equilateral triangle; tip points along ``heading`` (radians, CCW from +x)."""
    phis = (
        heading,
        heading + 2.0 * math.pi / 3.0,
        heading - 2.0 * math.pi / 3.0,
    )
    return np.array(
        [[cx + radius * math.cos(p), cy + radius * math.sin(p)] for p in phis],
        dtype=np.float64,
    )


def _planner_display_name(planner: BasePlanner) -> str:
    raw = planner.get_name()
    if raw == "d_star_lite":
        return "D* Lite"
    if raw == "a_star":
        return "A*"
    return raw.replace("_", " ").title()


class Dashboard:
    """
    Two-panel figure: map + telemetry. Driven by :meth:`render` each timestep
    (not FuncAnimation with embedded simulation).
    """

    def __init__(
        self,
        visualization_cfg: dict[str, Any],
        environment: Environment,
        algorithm_cfg: dict[str, Any],
        drone_cfg: dict[str, Any],
        mission_waypoints: list[tuple[int, int]],
    ) -> None:
        self._viz = visualization_cfg
        self._enabled = bool(visualization_cfg.get("dashboard_enabled", True))
        self._fps = float(visualization_cfg.get("fps", 10))
        self._trail_len = int(visualization_cfg.get("trail_length", 50))
        self._show_sensor = bool(visualization_cfg.get("show_sensor_range", True))
        self._show_path = bool(visualization_cfg.get("show_planned_path", True))

        self._w = environment.width
        self._h = environment.height
        self._warn_th = float(algorithm_cfg["sensor_degradation_threshold"])
        self._crit_th = float(algorithm_cfg["critical_fault_threshold"])
        self._max_noise = float(drone_cfg["max_sensor_noise"])
        self._bat_cap = float(drone_cfg["battery_capacity"])
        self._waypoints = list(mission_waypoints)

        self._closed = False
        self._trail: deque[tuple[float, float]] = deque(maxlen=self._trail_len)

        self._img_rgb: np.ndarray | None = None
        self._grid_snapshot: np.ndarray | None = None
        self._initial_grid: np.ndarray | None = environment.get_grid().copy()

        self._fig = None
        self._ax_map = None
        self._im = None
        self._line_path = None
        self._line_trail = None
        self._drone_poly = None
        self._sensor_patch = None
        self._wp_scatter = None
        self._wp_annotations: list[Any] = []
        self._target_ring = None

        self._txt_fsm = None
        self._rect_sens_bg = None
        self._rect_sens_fg = None
        self._line_sens_warn = None
        self._line_sens_crit = None
        self._rect_bat_bg = None
        self._rect_bat_fg = None
        self._txt_planner = None
        self._txt_mission = None
        self._txt_faults = None
        self._txt_step = None

        if not self._enabled:
            return

        import matplotlib.pyplot as plt
        from matplotlib import gridspec
        from matplotlib.patches import Circle, Polygon, Rectangle

        plt.ion()
        self._plt = plt
        self._Rectangle = Rectangle
        self._Circle = Circle

        self._fig = plt.figure(figsize=(14, 8), num="Resilient Navigator")
        gs = gridspec.GridSpec(1, 2, figure=self._fig, width_ratios=[3, 1], wspace=0.25)

        self._ax_map = self._fig.add_subplot(gs[0, 0])
        self._ax_panel = self._fig.add_subplot(gs[0, 1])
        self._ax_panel.axis("off")

        # RGB base image
        self._img_rgb = np.ones((self._h, self._w, 3), dtype=np.float32) * 0.96
        self._im = self._ax_map.imshow(
            self._img_rgb,
            origin="lower",
            extent=(0, self._w, 0, self._h),
            interpolation="nearest",
            aspect="equal",
            zorder=0,
        )

        self._ax_map.set_xlim(0, self._w)
        self._ax_map.set_ylim(0, self._h)
        self._ax_map.set_xlabel("x")
        self._ax_map.set_ylabel("y")
        self._ax_map.set_title("Map")

        (self._line_path,) = self._ax_map.plot([], [], "c--", linewidth=1.5, zorder=3, label="Planned path")
        (self._line_trail,) = self._ax_map.plot([], [], "-", color="green", alpha=0.45, linewidth=2, zorder=2)

        wx = [w[0] + 0.5 for w in self._waypoints]
        wy = [w[1] + 0.5 for w in self._waypoints]
        self._wp_scatter = self._ax_map.scatter(
            wx,
            wy,
            s=80,
            c="#3498db",
            marker="D",
            edgecolors="navy",
            linewidths=0.8,
            zorder=4,
        )
        for i, (px, py) in enumerate(self._waypoints):
            t = self._ax_map.annotate(
                str(i + 1),
                (px + 0.5, py + 0.5),
                fontsize=8,
                ha="center",
                va="center",
                color="white",
                fontweight="bold",
                zorder=5,
            )
            self._wp_annotations.append(t)

        self._target_ring = Circle(
            (0, 0),
            1.4,
            fill=False,
            edgecolor="gold",
            linewidth=2.5,
            zorder=6,
            visible=False,
        )
        self._ax_map.add_patch(self._target_ring)

        self._sensor_patch = Circle(
            (0, 0),
            10,
            fill=True,
            facecolor="lime",
            edgecolor="green",
            alpha=0.12,
            zorder=1,
            visible=False,
        )
        self._ax_map.add_patch(self._sensor_patch)

        self._drone_poly = Polygon(
            _drone_triangle_xy(0.5, 0.5, 0.0),
            closed=True,
            facecolor="limegreen",
            edgecolor="darkgreen",
            linewidth=1.2,
            zorder=7,
        )
        self._ax_map.add_patch(self._drone_poly)

        self._planning_dot = Circle(
            (-1.0, -1.0),
            0.22,
            facecolor="#2e7d32",
            edgecolor="#1b5e20",
            linewidth=0.8,
            alpha=0.45,
            zorder=6,
            visible=False,
        )
        self._ax_map.add_patch(self._planning_dot)

        self._ax_map.legend(loc="upper right", fontsize=8)

        # Right panel layout (figure coordinates)
        fig = self._fig
        self._txt_fsm = fig.text(0.76, 0.92, "FSM: NOMINAL", fontsize=16, fontweight="bold", color="#2ecc71")

        axs = fig.add_axes([0.76, 0.78, 0.2, 0.04])
        axs.set_xlim(0, 1)
        axs.set_ylim(0, 1)
        axs.axis("off")
        self._ax_sens = axs
        self._rect_sens_bg = Rectangle((0, 0), 1, 1, facecolor="#ecf0f1", edgecolor="gray", linewidth=1)
        axs.add_patch(self._rect_sens_bg)
        self._rect_sens_fg = Rectangle((0, 0), 0, 1, facecolor="#27ae60", edgecolor="none")
        axs.add_patch(self._rect_sens_fg)
        wn = self._warn_th / max(self._max_noise, 1e-9)
        cr = self._crit_th / max(self._max_noise, 1e-9)
        self._line_sens_warn = axs.axvline(wn, color="orange", linewidth=2)
        self._line_sens_crit = axs.axvline(cr, color="red", linewidth=2)
        fig.text(0.76, 0.83, "Sensor health (noise / max)", fontsize=9)

        axb = fig.add_axes([0.76, 0.68, 0.2, 0.04])
        axb.set_xlim(0, 1)
        axb.set_ylim(0, 1)
        axb.axis("off")
        self._ax_bat = axb
        self._rect_bat_bg = Rectangle((0, 0), 1, 1, facecolor="#ecf0f1", edgecolor="gray", linewidth=1)
        axb.add_patch(self._rect_bat_bg)
        self._rect_bat_fg = Rectangle((0, 0), 1, 1, facecolor="#27ae60", edgecolor="none")
        axb.add_patch(self._rect_bat_fg)
        fig.text(0.76, 0.73, "Battery", fontsize=9)

        self._txt_planner = fig.text(0.76, 0.62, "Planner: —", fontsize=10)
        self._txt_mission = fig.text(0.76, 0.55, "Mission: —", fontsize=10)
        self._txt_mission_bar = fig.text(0.76, 0.50, "", fontsize=8, family="monospace")
        self._txt_faults = fig.text(0.76, 0.38, "Active faults:\n—", fontsize=9, va="top", wrap=True)
        self._txt_step = fig.text(0.76, 0.08, "t = 0", fontsize=11, family="monospace")

        self._fig.canvas.mpl_connect("close_event", self._on_close)

        self._fig.subplots_adjust(left=0.06, right=0.98, bottom=0.08, top=0.94)

    def _on_close(self, _event: Any) -> None:
        self._closed = True

    @property
    def is_closed(self) -> bool:
        return self._closed

    def _build_grid_rgb(self, environment: Environment) -> np.ndarray:
        g = environment.get_grid()
        base = np.ones((self._h, self._w, 3), dtype=np.float32) * 0.96
        ig = self._initial_grid
        assert ig is not None
        for y in range(self._h):
            for x in range(self._w):
                v = int(g[y, x])
                if v == FREE:
                    base[y, x] = (0.96, 0.96, 0.96)
                elif v == HAZARD:
                    base[y, x] = (0.92, 0.15, 0.15)
                elif v == OBSTACLE:
                    if int(ig[y, x]) == OBSTACLE:
                        base[y, x] = (0.32, 0.32, 0.32)
                    else:
                        base[y, x] = (0.85, 0.35, 0.12)
        return base

    def render(
        self,
        drone: Drone,
        environment: Environment,
        planner: BasePlanner,
        fsm: AutonomyFSM,
        mission_manager: MissionManager,
        timestep: int,
        *,
        fault_injector: FaultInjector | None = None,
        planning_position: tuple[int, int] | None = None,
    ) -> None:
        """Update all artists for the current frame and pause for ``1/fps`` seconds."""
        if not self._enabled or self._fig is None:
            return

        state = fsm.get_state()

        # Grid image (rebuild when occupancy changes)
        g = environment.get_grid()
        if self._grid_snapshot is None or not np.array_equal(g, self._grid_snapshot):
            rgb = self._build_grid_rgb(environment)
            self._im.set_data(rgb)
            self._grid_snapshot = g.copy()

        px, py = drone.position
        cx, cy = px + 0.5, py + 0.5
        ppx, ppy = (
            (planning_position[0], planning_position[1])
            if planning_position is not None
            else (px, py)
        )

        # Trail
        self._trail.append((cx, cy))
        if len(self._trail) >= 2:
            xs = [p[0] for p in self._trail]
            ys = [p[1] for p in self._trail]
            self._line_trail.set_data(xs, ys)
        else:
            self._line_trail.set_data([], [])

        # Planned path (anchor to planner's assumed position when degraded)
        if self._show_path:
            path = planner.get_full_path()
            if path:
                xs = [p[0] + 0.5 for p in path]
                ys = [p[1] + 0.5 for p in path]
                ax0, ay0 = ppx + 0.5, ppy + 0.5
                if (ax0, ay0) != (xs[0], ys[0]):
                    xs = [ax0] + xs
                    ys = [ay0] + ys
                self._line_path.set_data(xs, ys)
            else:
                self._line_path.set_data([], [])
        else:
            self._line_path.set_data([], [])

        # Drone (triangle; tip aligns with heading, CCW from +x)
        self._drone_poly.set_xy(_drone_triangle_xy(cx, cy, float(drone.heading)))

        # Sensor disk (effective range shrinks with noise)
        if self._show_sensor:
            r = float(drone.get_effective_sensor_range())
            self._sensor_patch.center = (cx, cy)
            self._sensor_patch.set_radius(r)
            self._sensor_patch.set_visible(True)
        else:
            self._sensor_patch.set_visible(False)

        if planning_position is not None and (ppx, ppy) != (px, py):
            self._planning_dot.center = (ppx + 0.5, ppy + 0.5)
            self._planning_dot.set_visible(True)
        else:
            self._planning_dot.set_visible(False)

        # Waypoint emphasis
        wps = mission_manager.get_progress()
        idx = int(wps["current_waypoint_index"])
        total = int(wps["waypoints_total"])
        st = wps["mission_status"]
        wxc = wyc = 0.0
        if st == "in_progress" and idx < total:
            tx, ty = mission_manager.get_current_target()
            wxc, wyc = tx + 0.5, ty + 0.5
            self._target_ring.center = (wxc, wyc)
            self._target_ring.set_visible(True)
        else:
            self._target_ring.set_visible(False)

        nwp = len(self._waypoints)
        face = np.ones((nwp, 4), dtype=np.float32)
        face[:, :3] = np.array([0.2, 0.45, 0.95])
        face[:, 3] = 1.0
        for i in range(nwp):
            if st == "completed" or (st == "in_progress" and i < idx):
                face[i, :3] = (0.55, 0.55, 0.6)
            elif st == "in_progress" and i == idx:
                face[i, :3] = (0.15, 0.4, 0.95)
        self._wp_scatter.set_facecolors(face)

        # FSM color
        col = {
            "NOMINAL": "#2ecc71",
            "DEGRADED": "#f1c40f",
            "REPLANNING": "#e67e22",
            "SAFE_MODE": "#e74c3c",
            "ABORT": "#1a1a1a",
        }.get(state, "#7f8c8d")
        if self._txt_fsm is not None:
            self._txt_fsm.set_text(f"FSM: {state}")
            self._txt_fsm.set_color(col)

        # Sensor bar
        noise = float(drone.sensor_noise_level)
        frac = min(1.0, noise / max(self._max_noise, 1e-9))
        self._rect_sens_fg.set_width(frac)
        if noise >= self._crit_th:
            self._rect_sens_fg.set_facecolor("#c0392b")
        elif noise >= self._warn_th:
            self._rect_sens_fg.set_facecolor("#f39c12")
        else:
            self._rect_sens_fg.set_facecolor("#27ae60")

        # Battery bar
        bat = float(drone.battery) / max(self._bat_cap, 1e-9)
        self._rect_bat_fg.set_width(bat)
        if bat < 0.2:
            self._rect_bat_fg.set_facecolor("#c0392b")
        elif bat < 0.5:
            self._rect_bat_fg.set_facecolor("#f1c40f")
        else:
            self._rect_bat_fg.set_facecolor("#27ae60")

        self._txt_planner.set_text(f"Planner: {_planner_display_name(planner)}")

        if st == "aborted":
            prog = "Mission: ABORT (home)"
        elif st == "completed":
            prog = "Mission: COMPLETE"
        else:
            prog = f"Waypoint {min(idx + 1, max(total, 1))} / {total}"
        self._txt_mission.set_text(prog)
        bar_w = 20
        filled = int((idx / max(total, 1)) * bar_w) if total else 0
        self._txt_mission_bar.set_text("[" + "#" * filled + "-" * (bar_w - filled) + "]")

        if fault_injector is not None:
            active = fault_injector.get_active_faults()
            if active:
                lines = []
                for f in active[:8]:
                    desc = f.get("description", f.get("type", "?"))
                    lines.append(f"• {desc}")
                self._txt_faults.set_text("Active faults:\n" + "\n".join(lines))
            else:
                self._txt_faults.set_text("Active faults:\n—")
        else:
            self._txt_faults.set_text("Active faults:\n—")

        self._txt_step.set_text(f"t = {timestep}")

        self._fig.canvas.draw_idle()
        self._plt.pause(1.0 / max(self._fps, 0.1))

    def finalize(self) -> None:
        """Keep the figure open until the user closes the window (interactive runs)."""
        if not self._enabled or self._fig is None:
            return
        self._plt.ioff()
        self._fig.canvas.draw_idle()
        self._plt.show(block=True)

