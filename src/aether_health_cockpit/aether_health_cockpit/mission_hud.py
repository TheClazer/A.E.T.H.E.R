"""
A.E.T.H.E.R mission HUD — live matplotlib dashboard (console script ``mission_hud``).

Replaces the old Streamlit trust HUD (``streamlit_app.py``), which depended on
the since-removed ``st.experimental_rerun`` and a browser. This one is a plain
rclpy + matplotlib window that works over WSLg; if no GUI backend is available
it falls back to Agg and saves ``docs/figures/hud_snapshot.png`` every 5 s.

Layout (2x2):
  (a) top-down XY — ground-truth path, VIO estimate path, the AETHER oriented
      protection-level ellipse, the naive ghost's frozen ellipse, truth dot;
  (b) trust strip-chart (0..1) with green/amber/red bands;
  (c) horizontal PL vs |true horizontal error| strip-chart (live coverage view);
  (d) big text: nav state, trust, PL, detection latency, solution separation.

Every subscription is optional — the HUD renders with any topic subset and
shows dashes for whatever has not arrived yet. rclpy spins in a background
thread; matplotlib animates at 5 Hz in the main thread.
"""
import collections
import math
import os
import threading
import time

try:
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.patches import Circle, Ellipse
except Exception:  # allow import without matplotlib installed (CI lint)
    matplotlib = None
    plt = None

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker
from aether_msgs.msg import ProtectionLevel

# Mirrors the cockpit's frozen naive bound radius (m), used when the naive
# ghost markers are not being published.
NAIVE_PL_RADIUS = 0.21
HIST = 600            # strip-chart samples kept (~2 min at 5 Hz)
PATH = 4000           # XY path points kept

GREEN = '#2F7D4F'
AMBER = '#C98A12'
RED = '#C42A1C'
GREY = '#70808F'


class HudNode(Node):
    """Collects the integrity bus into plain attributes the plot thread reads."""

    def __init__(self):
        super().__init__('mission_hud')
        self.t0 = time.monotonic()
        self.gt = None                 # (x, y)
        self.est = None                # (x, y)
        self.gt_path = collections.deque(maxlen=PATH)
        self.est_path = collections.deque(maxlen=PATH)
        self.trust = None
        self.state = None
        self.hpl = None
        self.semi_axes = None          # (a, b)
        self.ellipse_yaw = 0.0
        self.detection_latency = None
        self.solution_separation = None
        self.fault_active = None
        self.naive_pos = None          # (x, y) from /viz/naive_estimate
        self.naive_lost = False        # red naive bound from /viz/naive_bound
        # strip-chart histories: (t, value)
        self.trust_hist = collections.deque(maxlen=HIST)
        self.pl_hist = collections.deque(maxlen=HIST)
        self.err_hist = collections.deque(maxlen=HIST)

        self.create_subscription(Odometry, '/aether/ground_truth', self._on_gt, 20)
        self.create_subscription(Odometry, '/ov_msckf/odomimu', self._on_odom, 20)
        self.create_subscription(Float32, '/nav/trust', self._on_trust, 10)
        self.create_subscription(String, '/nav/state', self._on_state, 10)
        self.create_subscription(ProtectionLevel, '/nav/integrity_bound', self._on_pl, 10)
        self.create_subscription(Float32, '/nav/detection_latency', self._on_latency, 10)
        self.create_subscription(Float32, '/nav/solution_separation', self._on_sep, 10)
        self.create_subscription(Bool, '/fault/active', self._on_fault, 10)
        self.create_subscription(Marker, '/viz/naive_estimate', self._on_naive_est, 10)
        self.create_subscription(Marker, '/viz/naive_bound', self._on_naive_bound, 10)
        self.get_logger().info('A.E.T.H.E.R mission_hud up (matplotlib, 5 Hz).')

    def _t(self):
        return time.monotonic() - self.t0

    def _on_gt(self, msg):
        p = msg.pose.pose.position
        self.gt = (p.x, p.y)
        self.gt_path.append(self.gt)
        self._sample_error()

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.est = (p.x, p.y)
        self.est_path.append(self.est)
        self._sample_error()

    def _sample_error(self):
        if self.gt is not None and self.est is not None:
            err = math.hypot(self.gt[0] - self.est[0], self.gt[1] - self.est[1])
            self.err_hist.append((self._t(), err))

    def _on_trust(self, msg):
        self.trust = msg.data
        self.trust_hist.append((self._t(), msg.data))

    def _on_state(self, msg):
        self.state = msg.data

    def _on_pl(self, msg):
        self.hpl = msg.horizontal_pl
        self.semi_axes = (msg.semi_axes[0], msg.semi_axes[1])
        self.ellipse_yaw = getattr(msg, 'ellipse_yaw', 0.0)
        self.pl_hist.append((self._t(), msg.horizontal_pl))

    def _on_latency(self, msg):
        self.detection_latency = msg.data

    def _on_sep(self, msg):
        self.solution_separation = msg.data

    def _on_fault(self, msg):
        self.fault_active = msg.data

    def _on_naive_est(self, msg):
        self.naive_pos = (msg.pose.position.x, msg.pose.position.y)

    def _on_naive_bound(self, msg):
        self.naive_lost = msg.color.r > 0.9 and msg.color.g < 0.3


def _fmt(value, fmt='%.2f', suffix=''):
    return (fmt % value) + suffix if value is not None else '--'


def build_figure(node):
    """Create the 2x2 dashboard; return (fig, refresh_fn)."""
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    try:
        fig.canvas.manager.set_window_title('A.E.T.H.E.R mission HUD')
    except Exception:
        pass                            # headless backends have no window
    fig.suptitle('A.E.T.H.E.R — navigation integrity HUD', fontsize=13)

    # (a) top-down XY
    ax_xy = axs[0][0]
    ax_xy.set_title('top-down (odom frame)')
    ax_xy.set_xlabel('x [m]')
    ax_xy.set_ylabel('y [m]')
    ax_xy.set_aspect('equal', adjustable='datalim')
    gt_line, = ax_xy.plot([], [], color=GREEN, lw=1.5, label='ground truth')
    est_line, = ax_xy.plot([], [], color='#2A5DB0', lw=1.2, label='VIO estimate')
    truth_dot, = ax_xy.plot([], [], 'o', color=RED, ms=6, label='truth (now)')
    aether_patch = Ellipse((0, 0), 0.1, 0.1, angle=0.0,
                           fill=True, facecolor=GREEN, alpha=0.25,
                           edgecolor=GREEN, lw=1.2, label='AETHER PL')
    naive_patch = Circle((0, 0), NAIVE_PL_RADIUS,
                         fill=True, facecolor=GREY, alpha=0.25,
                         edgecolor=GREY, lw=1.0, label='naive PL (frozen)')
    aether_patch.set_visible(False)
    naive_patch.set_visible(False)
    ax_xy.add_patch(aether_patch)
    ax_xy.add_patch(naive_patch)
    ax_xy.legend(loc='upper right', fontsize=7)

    # (b) trust strip-chart with bands
    ax_tr = axs[0][1]
    ax_tr.set_title('trust (0..1)')
    ax_tr.set_ylim(-0.02, 1.02)
    ax_tr.set_xlabel('t [s]')
    ax_tr.axhspan(0.8, 1.02, color=GREEN, alpha=0.12)
    ax_tr.axhspan(0.4, 0.8, color=AMBER, alpha=0.12)
    ax_tr.axhspan(-0.02, 0.4, color=RED, alpha=0.12)
    trust_line, = ax_tr.plot([], [], color='#1F3A5F', lw=1.4)

    # (c) PL vs |true error|
    ax_pl = axs[1][0]
    ax_pl.set_title('horizontal PL vs |true error| (coverage)')
    ax_pl.set_xlabel('t [s]')
    ax_pl.set_ylabel('[m]')
    pl_line, = ax_pl.plot([], [], color=AMBER, lw=1.4, label='protection level')
    err_line, = ax_pl.plot([], [], color=RED, lw=1.2, label='|true error|')
    ax_pl.legend(loc='upper left', fontsize=7)

    # (d) big text
    ax_txt = axs[1][1]
    ax_txt.axis('off')
    text = ax_txt.text(0.02, 0.95, '', transform=ax_txt.transAxes,
                       fontsize=13, family='monospace', va='top')

    fig.tight_layout(rect=(0, 0, 1, 0.96))

    def refresh(_frame=None):
        # (a)
        if node.gt_path:
            xs, ys = zip(*node.gt_path)
            gt_line.set_data(xs, ys)
        if node.est_path:
            xs, ys = zip(*node.est_path)
            est_line.set_data(xs, ys)
        if node.gt is not None:
            truth_dot.set_data([node.gt[0]], [node.gt[1]])
        if node.est is not None and node.semi_axes is not None:
            aether_patch.set_center(node.est)
            aether_patch.width = max(2.0 * node.semi_axes[0], 0.05)
            aether_patch.height = max(2.0 * node.semi_axes[1], 0.05)
            aether_patch.angle = math.degrees(node.ellipse_yaw)
            aether_patch.set_visible(True)
        naive_xy = node.naive_pos if node.naive_pos is not None else node.est
        if naive_xy is not None:
            naive_patch.center = naive_xy
            col = RED if node.naive_lost else GREY
            naive_patch.set_facecolor(col)
            naive_patch.set_edgecolor(col)
            naive_patch.set_alpha(0.6 if node.naive_lost else 0.25)
            naive_patch.set_visible(True)
        if node.gt_path or node.est_path:
            ax_xy.relim()
            ax_xy.autoscale_view()
        # (b)
        if node.trust_hist:
            ts, vs = zip(*node.trust_hist)
            trust_line.set_data(ts, vs)
            ax_tr.set_xlim(max(0.0, ts[-1] - 120.0), max(10.0, ts[-1] + 1.0))
        # (c)
        ymax = 0.5
        if node.pl_hist:
            ts, vs = zip(*node.pl_hist)
            pl_line.set_data(ts, vs)
            ymax = max(ymax, max(vs) * 1.2)
        if node.err_hist:
            ts, vs = zip(*node.err_hist)
            err_line.set_data(ts, vs)
            ymax = max(ymax, max(vs) * 1.2)
        if node.pl_hist or node.err_hist:
            tlast = (node.pl_hist or node.err_hist)[-1][0]
            ax_pl.set_xlim(max(0.0, tlast - 120.0), max(10.0, tlast + 1.0))
            ax_pl.set_ylim(0.0, ymax)
        # (d)
        text.set_text(
            'STATE     %s\n'
            'TRUST     %s\n'
            'PL (2D)   %s\n'
            'DET LAT   %s\n'
            'SOL SEP   %s\n'
            'FAULT     %s' % (
                node.state if node.state is not None else '--',
                _fmt(node.trust),
                _fmt(node.hpl, suffix=' m'),
                _fmt(node.detection_latency, suffix=' s'),
                _fmt(node.solution_separation, '%.3f', ' m'),
                {True: 'ACTIVE', False: 'none'}.get(node.fault_active, '--')))
        if node.trust is not None:
            text.set_color(GREEN if node.trust >= 0.8 else
                           (AMBER if node.trust >= 0.4 else RED))
        return []

    return fig, refresh


def run_headless(node):
    """Agg fallback: render to docs/figures/hud_snapshot.png every 5 s."""
    plt.switch_backend('Agg')
    fig, refresh = build_figure(node)
    out_dir = os.path.join('docs', 'figures')
    out_png = os.path.join(out_dir, 'hud_snapshot.png')
    node.get_logger().warning('No GUI backend; saving %s every 5 s.' % out_png)
    while rclpy.ok():
        try:
            refresh()
            os.makedirs(out_dir, exist_ok=True)
            fig.savefig(out_png, dpi=110)
        except Exception as exc:
            node.get_logger().warning('snapshot failed: %s' % exc)
        time.sleep(5.0)


def main(args=None):
    if plt is None:
        print('matplotlib not installed; run: pip install matplotlib')
        return
    rclpy.init(args=args)
    node = HudNode()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        try:
            fig, refresh = build_figure(node)
            anim = animation.FuncAnimation(fig, refresh, interval=200,
                                           cache_frame_data=False)
            plt.show()                  # blocks until the window closes
            del anim
        except Exception as exc:        # WSLg absent / backend failure -> Agg
            node.get_logger().warning('GUI backend failed (%s); Agg fallback.' % exc)
            run_headless(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
