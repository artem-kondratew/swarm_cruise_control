#!/usr/bin/env python3
"""Simple matplotlib GUI for drawing a pacemaker waypoint trajectory.

Usage:
    ros2 run trajectory_drawer draw -o my_trajectory.yaml
    ros2 run trajectory_drawer draw -o my_trajectory.yaml --load existing.yaml

Controls (when the matplotlib window has focus):
    left-click empty area     add a waypoint at the cursor
    left-click + drag on point  pick up and move an existing waypoint
    right-click on point      delete that waypoint
    u                         undo last addition
    c                         clear all waypoints
    r                         toggle resampling preview (uniform spacing)
    l                         toggle close-loop visualisation (last → first)
    b                         toggle cubic spline smoothing
    s                         save to the output yaml path
    q                         quit (warns if unsaved)

Output yaml format (compatible with swarm_controller's pacemaker_controller
in `lanelet` mode):

    /**:
      ros__parameters:
        waypoints_x: [...]
        waypoints_y: [...]

To use the saved trajectory:
1. Either copy waypoints_x / waypoints_y into config/params_pacemaker.yaml,
   or pass the saved yaml as an additional --params-file at launch.
2. Set `trajectory: lanelet` in the pacemaker config.
3. Launch the swarm and toggle `start: true`.

Notes:
* The pacemaker uses pure-pursuit with `lookahead_dist` (default 1.0 m).
  For smooth following, waypoints should be spaced ≤ ~1 m. Use the
  resampling toggle (r) to insert intermediate points uniformly.
* Coordinates are in the simulator's `map` frame (same axes as
  init_x / init_y in params_simulator.yaml).
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

try:
    import yaml
except ImportError:
    print('ERROR: PyYAML is required. `pip install pyyaml` or '
          '`sudo apt install python3-yaml`.', file=sys.stderr)
    sys.exit(1)

try:
    import matplotlib.pyplot as plt
except ImportError:
    print('ERROR: matplotlib is required. `pip install matplotlib` or '
          '`sudo apt install python3-matplotlib`.', file=sys.stderr)
    sys.exit(1)


# Disable matplotlib's default keybindings that conflict with ours.
# Most importantly: 'l' is the y-axis log-scale toggle in matplotlib —
# without disabling, our 'l' for close-loop also flips y to log scale,
# which then crashes on the y-range that includes 0 and negative values.
# Other conflicts: 'c' (back), 's' (save), 'r' (home), 'k' (xscale).
def _disable_default_keymaps() -> None:
    for action, keys_to_remove in [
        ('keymap.yscale', ['l', 'L']),
        ('keymap.xscale', ['k', 'L']),
        ('keymap.back', ['c']),
        ('keymap.forward', ['v']),
        ('keymap.save', ['s', 'ctrl+s']),
        ('keymap.home', ['r', 'h', 'home']),
        ('keymap.fullscreen', ['f', 'ctrl+f']),
        ('keymap.pan', ['p']),
        ('keymap.zoom', ['o']),
        ('keymap.grid', ['g']),
        ('keymap.grid_minor', ['G']),
    ]:
        if action not in plt.rcParams:
            continue
        for k in keys_to_remove:
            if k in plt.rcParams[action]:
                plt.rcParams[action].remove(k)


_disable_default_keymaps()

try:
    from scipy.interpolate import CubicSpline
except ImportError:
    CubicSpline = None  # spline mode just disabled if scipy unavailable


def _config_dir() -> Optional[Path]:
    """Return swarm_controller's source config dir.

    `share/swarm_controller/config` itself is a regular directory, but
    files inside it are symlinks to the source via `build/`. Resolving
    one of those known files via `os.path.realpath` gives us the *source*
    config dir — which is where we want to write so new trajectories
    persist in git and survive a clean `colcon build`. Falls back to
    the share dir if nothing resolves.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share_config = Path(
            get_package_share_directory('swarm_controller')) / 'config'
    except Exception:
        return None
    # Pick any file we know exists in the source config and follow the
    # symlink chain: install → build → src.
    for known in ('params_pacemaker.yaml', 'params_simulator.yaml',
                  'logging_topics.yaml'):
        candidate = share_config / known
        if candidate.exists():
            return Path(os.path.realpath(candidate)).parent
    return share_config  # fallback (no known file → just use install share)


def _resolve_path(path_str: str) -> Path:
    """Absolute paths used as-is. Relative paths / bare filenames are
    resolved against swarm_controller's config dir; if that's unavailable,
    fall back to cwd."""
    p = Path(path_str).expanduser()
    if p.is_absolute():
        return p
    cd = _config_dir()
    return (cd / p) if cd is not None else (Path.cwd() / p)


Point = Tuple[float, float]


def resample_uniform(points: List[Point], spacing: float = 0.5) -> List[Point]:
    """Resample a polyline at uniform arc-length spacing.

    Endpoints are preserved. Returns the input unchanged if there are
    fewer than 2 points.
    """
    if len(points) < 2:
        return list(points)
    arr = np.asarray(points, dtype=float)
    seg = np.diff(arr, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    cum = np.concatenate(([0.0], np.cumsum(seg_len)))
    total = cum[-1]
    if total < 1e-9:
        return list(points)
    n = max(2, int(np.ceil(total / spacing)) + 1)
    s = np.linspace(0.0, total, n)
    xs = np.interp(s, cum, arr[:, 0])
    ys = np.interp(s, cum, arr[:, 1])
    return list(zip(xs.tolist(), ys.tolist()))


def spline_curve(
    points: List[Point],
    n_samples: int = 200,
    closed: bool = False,
) -> List[Point]:
    """Return densely sampled cubic spline through the given waypoints.

    Parametrised by cumulative arc length so the curve handles arbitrary
    geometry (loops, self-intersections, etc). For closed=True the spline
    is `bc_type='periodic'` and the first point is appended to the end
    internally so the loop closes smoothly.

    Returns the input unchanged if scipy is unavailable or n_pts < 3.
    """
    if CubicSpline is None or len(points) < 3:
        return list(points)
    arr = np.asarray(points, dtype=float)
    if closed:
        # close the loop for periodic spline
        if not np.allclose(arr[0], arr[-1]):
            arr = np.vstack([arr, arr[0]])
    seg = np.diff(arr, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    if (seg_len < 1e-9).any():
        # collapse duplicate points (would break the spline param)
        keep = np.concatenate(([True], seg_len > 1e-9))
        arr = arr[keep]
        seg = np.diff(arr, axis=0)
        seg_len = np.linalg.norm(seg, axis=1)
    if len(arr) < 3:
        return list(points)
    cum = np.concatenate(([0.0], np.cumsum(seg_len)))
    bc = 'periodic' if closed else 'not-a-knot'
    try:
        cs_x = CubicSpline(cum, arr[:, 0], bc_type=bc)
        cs_y = CubicSpline(cum, arr[:, 1], bc_type=bc)
    except ValueError:
        # periodic requires y[0] == y[-1] exactly; if numerical error,
        # fall back to natural BC
        cs_x = CubicSpline(cum, arr[:, 0], bc_type='not-a-knot')
        cs_y = CubicSpline(cum, arr[:, 1], bc_type='not-a-knot')
    s_dense = np.linspace(0.0, cum[-1], n_samples)
    return list(zip(cs_x(s_dense).tolist(), cs_y(s_dense).tolist()))


def load_yaml(path: Path) -> List[Point]:
    """Load waypoints_x/waypoints_y from a pacemaker-style yaml."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    # Walk into the first /**: ros__parameters block
    for top_val in data.values():
        if isinstance(top_val, dict) and 'ros__parameters' in top_val:
            params = top_val['ros__parameters']
            xs = params.get('waypoints_x', [])
            ys = params.get('waypoints_y', [])
            if len(xs) != len(ys):
                raise ValueError(
                    f'waypoints_x ({len(xs)}) and waypoints_y ({len(ys)}) '
                    'have different lengths in {path}')
            return list(zip(xs, ys))
    raise ValueError(f'{path}: no /**: ros__parameters block with waypoints')


def save_yaml(path: Path, points: List[Point]) -> None:
    """Write a pacemaker-style yaml with waypoints + trajectory: lanelet.

    The yaml is intended to be loaded as an OVERRIDE on top of the main
    `params_pacemaker.yaml` (last-file-wins), so it sets `trajectory:
    lanelet` to switch the pacemaker into pure-pursuit mode automatically.
    """
    xs = [round(float(x), 4) for x, _ in points]
    ys = [round(float(y), 4) for _, y in points]
    data = {
        '/**': {
            'ros__parameters': {
                'trajectory': 'lanelet',
                'waypoints_x': xs,
                'waypoints_y': ys,
            },
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)


class TrajectoryDrawer:
    """Interactive matplotlib UI for drawing a polyline trajectory."""

    HELP = ('left-click empty: add  |  drag point: move  |  right-click point: del\n'
            'u: undo  |  c: clear  |  r: resample  |  l: close-loop  |  '
            'b: spline  |  s: save  |  q: quit')

    PICK_RADIUS_PX = 10  # pixel radius for click-to-pick on existing points

    def __init__(
        self,
        output_path: Path,
        initial: Optional[List[Point]] = None,
        x_range: Tuple[float, float] = (-5.0, 5.0),
        y_range: Tuple[float, float] = (-5.0, 5.0),
        resample_spacing: float = 0.5,
        spline_samples: int = 200,
    ) -> None:
        self.output_path = output_path
        self.points: List[Point] = list(initial) if initial else []
        self.resample_spacing = resample_spacing
        self.spline_samples = spline_samples
        self.show_resampled = False
        self.show_spline = False
        self.close_loop = False
        self.dirty = bool(initial)  # track unsaved changes
        self._saved_at_count = len(self.points)
        self._drag_idx: Optional[int] = None  # index of point currently dragged

        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_xlim(*x_range)
        self.ax.set_ylim(*y_range)
        self.ax.set_aspect('equal')
        self.ax.grid(True, linestyle=':', alpha=0.6)
        self.ax.set_xlabel('x [m]')
        self.ax.set_ylabel('y [m]')

        # axes:
        #   raw_line       — clicked waypoints (always shown)
        #   spline_line    — smooth cubic spline through points (toggle b)
        #   resampled_line — preview of uniformly spaced points (toggle r)
        #   loop_line      — closing segment (toggle l)
        #   start_marker   — green square at first point
        self.raw_line, = self.ax.plot(
            [], [], 'b-o', markersize=7, lw=1.5, label='waypoints')
        self.spline_line, = self.ax.plot(
            [], [], 'm-', lw=2.0, alpha=0.85, label='spline')
        self.spline_line.set_visible(False)
        self.resampled_line, = self.ax.plot(
            [], [], 'r.', markersize=4, label=f'resampled ({resample_spacing} m)')
        self.resampled_line.set_visible(False)
        self.loop_line, = self.ax.plot([], [], 'b--', lw=1.0, alpha=0.7)
        self.start_marker, = self.ax.plot(
            [], [], 'gs', markersize=12, markerfacecolor='none',
            markeredgewidth=2, label='start')
        self.ax.legend(loc='upper right', fontsize=9)

        self.fig.canvas.mpl_connect('button_press_event', self._on_press)
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_motion)
        self.fig.canvas.mpl_connect('button_release_event', self._on_release)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

        self._refresh()

    # ── helpers ─────────────────────────────────────────────────────────────

    def _pick_point(self, event) -> Optional[int]:
        """Return index of waypoint within PICK_RADIUS_PX of the event,
        or None if cursor is on empty area / outside axes."""
        if not self.points or event.inaxes != self.ax:
            return None
        if event.x is None or event.y is None:
            return None
        # convert all points to display (pixel) coords
        pts_data = np.asarray(self.points, dtype=float)
        pts_disp = self.ax.transData.transform(pts_data)
        dx = pts_disp[:, 0] - event.x
        dy = pts_disp[:, 1] - event.y
        dists = np.hypot(dx, dy)
        idx = int(np.argmin(dists))
        if dists[idx] <= self.PICK_RADIUS_PX:
            return idx
        return None

    # ── refresh ─────────────────────────────────────────────────────────────

    def _refresh(self) -> None:
        if self.points:
            xs, ys = zip(*self.points)
            self.raw_line.set_data(xs, ys)
            self.start_marker.set_data([xs[0]], [ys[0]])
        else:
            self.raw_line.set_data([], [])
            self.start_marker.set_data([], [])

        if self.close_loop and len(self.points) >= 2:
            x0, y0 = self.points[0]
            xn, yn = self.points[-1]
            self.loop_line.set_data([xn, x0], [yn, y0])
        else:
            self.loop_line.set_data([], [])

        # spline curve: dense smooth interpolation (closed if loop is on)
        if self.show_spline and len(self.points) >= 3:
            curve = spline_curve(
                self.points, n_samples=self.spline_samples,
                closed=self.close_loop,
            )
            sxs, sys = zip(*curve)
            self.spline_line.set_data(sxs, sys)
        else:
            self.spline_line.set_data([], [])

        # resampled preview: applied to the SMOOTHED curve when spline is on
        if self.show_resampled and len(self.points) >= 2:
            base = self.points
            if self.show_spline and len(self.points) >= 3:
                base = spline_curve(
                    self.points, n_samples=self.spline_samples,
                    closed=self.close_loop,
                )
            elif self.close_loop and len(self.points) >= 2:
                base = list(self.points) + [self.points[0]]
            rs = resample_uniform(base, self.resample_spacing)
            if rs:
                rxs, rys = zip(*rs)
                self.resampled_line.set_data(rxs, rys)
            else:
                self.resampled_line.set_data([], [])
        else:
            self.resampled_line.set_data([], [])

        suffix = '*' if self.dirty else ''
        loop = ' [closed]' if self.close_loop else ''
        spl = ' [spline]' if self.show_spline else ''
        rs_label = f' [resampled@{self.resample_spacing}m]' if self.show_resampled else ''
        self.ax.set_title(
            f'Trajectory drawer{suffix} — {len(self.points)} pts'
            f'{loop}{spl}{rs_label}\n→ {self.output_path}\n{self.HELP}',
            fontsize=10,
        )
        self.fig.canvas.draw_idle()

    # ── event handlers ──────────────────────────────────────────────────────

    def _on_press(self, event) -> None:
        if event.inaxes != self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        idx = self._pick_point(event)
        if event.button == 1:  # left-click
            if idx is not None:
                # pick existing point for drag
                self._drag_idx = idx
            else:
                # add new point
                self.points.append((float(event.xdata), float(event.ydata)))
                self.dirty = True
                self._refresh()
        elif event.button == 3:  # right-click → delete point under cursor
            if idx is not None:
                self.points.pop(idx)
                self.dirty = True
                self._refresh()

    def _on_motion(self, event) -> None:
        if self._drag_idx is None:
            return
        if event.inaxes != self.ax:
            return
        if event.xdata is None or event.ydata is None:
            return
        self.points[self._drag_idx] = (float(event.xdata), float(event.ydata))
        self.dirty = True
        self._refresh()

    def _on_release(self, event) -> None:
        if self._drag_idx is not None:
            self._drag_idx = None

    def _on_key(self, event) -> None:
        if event.key == 'u':
            if self.points:
                self.points.pop()
                self.dirty = True
                self._refresh()
        elif event.key == 'c':
            if self.points:
                self.points.clear()
                self.dirty = True
                self._refresh()
        elif event.key == 'r':
            self.show_resampled = not self.show_resampled
            self.resampled_line.set_visible(self.show_resampled)
            self._refresh()
        elif event.key == 'l':
            self.close_loop = not self.close_loop
            self._refresh()
        elif event.key == 'b':
            if CubicSpline is None:
                print('scipy is unavailable — spline mode disabled. '
                      '`pip install scipy` or `apt install python3-scipy`.')
                return
            self.show_spline = not self.show_spline
            self.spline_line.set_visible(self.show_spline)
            self._refresh()
        elif event.key == 's':
            self._save()
        elif event.key == 'q':
            self._quit()

    # ── save / quit ─────────────────────────────────────────────────────────

    def _save(self) -> None:
        if not self.points:
            print('Nothing to save (no points).')
            return

        # Save pipeline (in order):
        #   1. start with raw clicked waypoints
        #   2. if spline mode → replace with densely sampled cubic spline
        #      (closed=True triggers periodic boundary conditions)
        #   3. else if close_loop → append first point to close polyline
        #   4. if resampled mode → uniform arc-length resampling
        kinds = []
        if self.show_spline and len(self.points) >= 3:
            pts = spline_curve(
                self.points, n_samples=self.spline_samples,
                closed=self.close_loop,
            )
            kinds.append('spline')
        else:
            pts = list(self.points)
            if self.close_loop and len(pts) >= 2 and pts[0] != pts[-1]:
                pts.append(pts[0])
                kinds.append('closed')
        if self.show_resampled and len(pts) >= 2:
            pts = resample_uniform(pts, self.resample_spacing)
            kinds.append(f'resampled@{self.resample_spacing}m')

        save_yaml(self.output_path, pts)
        self.dirty = False
        self._saved_at_count = len(self.points)
        kind = ', '.join(kinds) if kinds else 'raw'
        print(f'Saved {len(pts)} waypoints ({kind}) → {self.output_path}')
        self._refresh()

    def _quit(self) -> None:
        if self.dirty:
            print('WARNING: unsaved changes. Press s to save, or q again to '
                  'force-quit.')
            self.dirty = False  # next q will close
            return
        plt.close(self.fig)

    def run(self) -> None:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Draw a pacemaker waypoint trajectory by clicking points.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        '-o', '--output', type=str, default='trajectory.yaml',
        help=('Output yaml path. Bare filenames / relative paths resolve '
              'into swarm_controller/config/ (so the pacemaker can find '
              'them via params_pacemaker.yaml: trajectory_file). '
              'Default: trajectory.yaml'))
    parser.add_argument(
        '--load', type=str, default=None,
        help=('Load an existing pacemaker-style yaml as starting points. '
              'Same path resolution as --output.'))
    parser.add_argument(
        '--xlim', type=float, nargs=2, default=(-5.0, 5.0),
        metavar=('MIN', 'MAX'), help='x-axis range (default: -5 5)')
    parser.add_argument(
        '--ylim', type=float, nargs=2, default=(-5.0, 5.0),
        metavar=('MIN', 'MAX'), help='y-axis range (default: -5 5)')
    parser.add_argument(
        '--resample-spacing', type=float, default=0.5,
        help='Uniform spacing for resampling preview (default: 0.5 m)')
    parser.add_argument(
        '--spline-samples', type=int, default=200,
        help='Number of points sampled along the cubic spline (default: 200)')
    args = parser.parse_args()

    output_path = _resolve_path(args.output)
    print(f'Output target: {output_path}')

    initial = None
    if args.load is not None:
        load_path = _resolve_path(args.load)
        if not load_path.exists():
            print(f'ERROR: {load_path} not found', file=sys.stderr)
            sys.exit(1)
        initial = load_yaml(load_path)
        print(f'Loaded {len(initial)} waypoints from {load_path}')

    drawer = TrajectoryDrawer(
        output_path=output_path,
        initial=initial,
        x_range=tuple(args.xlim),
        y_range=tuple(args.ylim),
        resample_spacing=args.resample_spacing,
        spline_samples=args.spline_samples,
    )
    drawer.run()


if __name__ == '__main__':
    main()
