"""
A.E.T.H.E.R MISSION CONTROL — live matplotlib dashboard + clickable fault rail
(console script ``mission_hud``).

A plain rclpy + matplotlib window that works over WSLg; if no GUI backend is
available it falls back to Agg and saves ``docs/figures/hud_snapshot.png``
every 5 s (buttons disabled in that mode).

Layout:
  ── state banner (NOMINAL green / DEGRADED amber / INERTIAL red) ──
  (a) top-down XY — ground-truth path, VIO estimate path, the AETHER oriented
      protection-level ellipse, the naive ghost's frozen ellipse, truth dot;
  (b) trust strip-chart (0..1) with green/amber/red bands;
  (c) horizontal PL vs |true horizontal error| strip-chart (live coverage);
  (d) big text: nav state, trust, PL, detection latency, solution separation.
  ── button rail: KILL CAMERA · IMU BIAS · STARVE FEATURES · UWB AID ──

The buttons fire the demo fault services (async; no-ops with a console warning
if a service has no server in the current scene):
  /kill_camera                aether_msgs/KillCamera   (replay scene: replay_node;
                                                        live3d scene: sensor_bridge)
  /inject/imu_bias            std_srvs/SetBool
  /inject/feature_starvation  std_srvs/SetBool
  /uwb/enable                 std_srvs/SetBool

Every subscription is optional — the HUD renders with any topic subset. rclpy
spins in a background thread; matplotlib animates at 5 Hz in the main thread.
``--ros-args -p source_label:='...'`` sets the honesty label in the header.
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
    from matplotlib.widgets import Button
except Exception:  # allow import without matplotlib installed (CI lint)
    matplotlib = None
    plt = None

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker
from aether_msgs.msg import ProtectionLevel
from aether_msgs.srv import KillCamera

# Mirrors the cockpit's frozen naive bound radius (m), used when the naive
# ghost markers are not being published.
NAIVE_PL_RADIUS = 0.21
HIST = 600            # strip-chart samples kept (~2 min at 5 Hz)
PATH = 4000           # XY path points kept

GREEN = '#2F7D4F'
AMBER = '#C98A12'
RED = '#C42A1C'
GREY = '#70808F'
INK = '#1B2A36'
PAPER = '#F7F6F2'

STATE_COLOR = {'NOMINAL': GREEN, 'DEGRADED': AMBER,
               'INERTIAL': RED, 'RE_ACQUIRE': '#2A5DB0'}

SPINE = ('"When the camera dies, our drift bound still covers the true position '
         '95%+ of the time - and we know within half a second."')


class HudNode(Node):
    """Collects the integrity bus into plain attributes the plot thread reads,
    and owns the fault-service clients the buttons fire."""

    def __init__(self):
        super().__init__('mission_hud')
        self.declare_parameter('source_label', 'LIVE RIG · VIO-CLASS ERROR MODEL')
        self.source_label = str(self.get_parameter('source_label').value)
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
        # button toggle states (local intent; the bus reflects the real effect)
        self.camera_dead = False
        self.bias_on = False
        self.starve_on = False
        self.uwb_on = False

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

        self._cli_kill = self.create_client(KillCamera, '/kill_camera')
        self._cli_bias = self.create_client(SetBool, '/inject/imu_bias')
        self._cli_starve = self.create_client(SetBool, '/inject/feature_starvation')
        self._cli_uwb = self.create_client(SetBool, '/uwb/enable')
        self.get_logger().info('A.E.T.H.E.R MISSION CONTROL up (matplotlib, 5 Hz).')

    # ---------------- fault buttons ----------------
    def _fire(self, client, request, label):
        if not client.service_is_ready():
            self.get_logger().warning(
                '%s: no service server in this scene (%s)' % (label, client.srv_name))
            return False
        client.call_async(request)
        self.get_logger().info('%s -> %s' % (label, client.srv_name))
        return True

    def toggle_camera(self):
        req = KillCamera.Request()
        req.enable = not self.camera_dead
        if self._fire(self._cli_kill, req, 'KILL CAMERA' if req.enable else 'RESTORE CAMERA'):
            self.camera_dead = req.enable
        return self.camera_dead

    def toggle_bias(self):
        req = SetBool.Request()
        req.data = not self.bias_on
        if self._fire(self._cli_bias, req, 'IMU BIAS %s' % ('ON' if req.data else 'OFF')):
            self.bias_on = req.data
        return self.bias_on

    def toggle_starve(self):
        req = SetBool.Request()
        req.data = not self.starve_on
        if self._fire(self._cli_starve, req, 'STARVATION %s' % ('ON' if req.data else 'OFF')):
            self.starve_on = req.data
        return self.starve_on

    def toggle_uwb(self):
        req = SetBool.Request()
        req.data = not self.uwb_on
        if self._fire(self._cli_uwb, req, 'UWB AID %s' % ('ON' if req.data else 'OFF')):
            self.uwb_on = req.data
        return self.uwb_on

    # ---------------- bus callbacks ----------------
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


def build_figure(node, interactive=True):
    """Create the dashboard; return (fig, refresh_fn, widgets_keepalive)."""
    fig = plt.figure(figsize=(13, 8.6))
    fig.patch.set_facecolor(PAPER)
    try:
        fig.canvas.manager.set_window_title('A.E.T.H.E.R MISSION CONTROL')
    except Exception:
        pass                            # headless backends have no window

    # header: banner + honesty label; footer: the spine + button rail
    banner = fig.text(0.5, 0.965, 'WAITING FOR TELEMETRY ...', ha='center',
                      fontsize=19, family='monospace', fontweight='bold', color=GREY)
    fig.text(0.5, 0.928, 'A.E.T.H.E.R - navigation integrity mission control   ·   %s'
             % node.source_label, ha='center', fontsize=9.5, color=INK)
    fig.text(0.5, 0.012, SPINE, ha='center', fontsize=8, color=GREY, style='italic')

    grid = fig.add_gridspec(2, 2, left=0.06, right=0.975, top=0.895, bottom=0.165,
                            hspace=0.33, wspace=0.22)
    ax_xy = fig.add_subplot(grid[0, 0])
    ax_tr = fig.add_subplot(grid[0, 1])
    ax_pl = fig.add_subplot(grid[1, 0])
    ax_txt = fig.add_subplot(grid[1, 1])

    # (a) top-down XY
    ax_xy.set_title('top-down (odom frame)', fontsize=10)
    ax_xy.set_xlabel('x [m]', fontsize=8)
    ax_xy.set_ylabel('y [m]', fontsize=8)
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
    ax_tr.set_title('trust (0..1)', fontsize=10)
    ax_tr.set_ylim(-0.02, 1.02)
    ax_tr.set_xlabel('t [s]', fontsize=8)
    ax_tr.axhspan(0.8, 1.02, color=GREEN, alpha=0.12)
    ax_tr.axhspan(0.4, 0.8, color=AMBER, alpha=0.12)
    ax_tr.axhspan(-0.02, 0.4, color=RED, alpha=0.12)
    trust_line, = ax_tr.plot([], [], color='#1F3A5F', lw=1.4)

    # (c) PL vs |true error|
    ax_pl.set_title('horizontal PL vs |true error| (coverage)', fontsize=10)
    ax_pl.set_xlabel('t [s]', fontsize=8)
    ax_pl.set_ylabel('[m]', fontsize=8)
    pl_line, = ax_pl.plot([], [], color=AMBER, lw=1.4, label='protection level')
    err_line, = ax_pl.plot([], [], color=RED, lw=1.2, label='|true error|')
    ax_pl.legend(loc='upper left', fontsize=7)

    # (d) big text
    ax_txt.axis('off')
    text = ax_txt.text(0.02, 0.95, '', transform=ax_txt.transAxes,
                       fontsize=13, family='monospace', va='top')

    # ---------------- button rail ----------------
    widgets = []
    if interactive:
        defs = [
            ('KILL CAMERA', node.toggle_camera,
             lambda on: 'RESTORE CAMERA' if on else 'KILL CAMERA'),
            ('IMU BIAS: off', node.toggle_bias,
             lambda on: 'IMU BIAS: %s' % ('ON' if on else 'off')),
            ('STARVE FEATS: off', node.toggle_starve,
             lambda on: 'STARVE FEATS: %s' % ('ON' if on else 'off')),
            ('UWB AID: off', node.toggle_uwb,
             lambda on: 'UWB AID: %s' % ('ON' if on else 'off')),
        ]
        n = len(defs)
        w, gap = 0.20, 0.025
        x0 = 0.5 - (n * w + (n - 1) * gap) / 2.0
        for i, (label, action, relabel) in enumerate(defs):
            bax = fig.add_axes((x0 + i * (w + gap), 0.045, w, 0.055))
            btn = Button(bax, label, color='#E8E6E0', hovercolor='#D8D4CA')
            btn.label.set_fontsize(9)
            btn.label.set_family('monospace')

            def _cb(_event, a=action, r=relabel, b=btn):
                try:
                    on = a()
                    b.label.set_text(r(on))
                    b.label.set_color(RED if on else INK)
                except Exception as exc:   # never let a click kill the HUD
                    print('button error:', exc)

            btn.on_clicked(_cb)
            widgets.append(btn)

    def refresh(_frame=None):
        # banner
        st = node.state or '--'
        banner.set_text('%s      trust %s      PL %s      det %s' % (
            st, _fmt(node.trust), _fmt(node.hpl, suffix=' m'),
            _fmt(node.detection_latency, suffix=' s')))
        banner.set_color(STATE_COLOR.get(st, GREY))
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

    return fig, refresh, widgets


def run_headless(node):
    """Agg fallback: render to docs/figures/hud_snapshot.png every 5 s."""
    plt.switch_backend('Agg')
    fig, refresh, _ = build_figure(node, interactive=False)
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
            fig, refresh, widgets = build_figure(node, interactive=True)
            anim = animation.FuncAnimation(fig, refresh, interval=200,
                                           cache_frame_data=False)
            plt.show()                  # blocks until the window closes
            del anim, widgets
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
