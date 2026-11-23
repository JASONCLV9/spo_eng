"""
ME5314 Sports Engineering - E2
Topspin (table tennis) - Vizard affordance scene

CONTROLS
  SPACE    play / pause
  R        restart (t = 0)
  S        change speed: real time 1.0x / slow motion 0.25x
  1-4      select condition (ball position)
  V        show / hide criterion result
  <- / ->  previous / next frame (while paused)
  T        show / hide paddle proxy trajectory
  A        show / hide avatar
  K        show / hide skeleton
  C        change camera view
  P        print current frame values to console
  G        jump to next validation instant 
  H        show / hide help
  NM       rotate table -5 / +5 degrees
  IO       move table closer / farther 5 cm
"""

import viz
import vizact
import vizshape
import csv
import math
import os

# ============================================================
# PARAMETERS
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, '..', 'motion-data'))

ORIENTATION_CSV = os.path.join(DATA_DIR, 'Top_spin_SegmentOrientation.csv')
POSITION_CSV = os.path.join(DATA_DIR, 'Top_spin_SegmentPosition.csv')

if not os.path.exists(ORIENTATION_CSV):
    ORIENTATION_CSV = os.path.join(SCRIPT_DIR, 'Top_spin_SegmentOrientation.csv')
if not os.path.exists(POSITION_CSV):
    POSITION_CSV = os.path.join(SCRIPT_DIR, 'Top_spin_SegmentPosition.csv')

FRAME_RATE = 60.0            # Hz (MVNX frameRate)
SLOW_FACTOR = 0.25           # slow motion (labeled on screen)
AVATAR_FILE = 'vcc_male.cfg'
AVATAR_SIDE_OFFSET = 1.5     # m, avatar is drawn beside the skeleton

# Hand holding the paddle (Topspin played with the left hand)
PADDLE_SIDE = 'Left'

# Proxy assumptions (document in the report)
PADDLE_OFFSET = 0.10         # m, paddle center beyond the wrist
PADDLE_RADIUS = 0.075        # m, approx. paddle radius (blade ~15 cm)
BALL_RADIUS = 0.020          # m, 40 mm ball
CONTACT_THRESHOLD = PADDLE_RADIUS + BALL_RADIUS

# Ordered conditions: ball distance d (m) outward
# from the point of maximum proxy speed
CONDITIONS = [-0.08, 0.00, 0.08, 0.15]

# Instants for OpenSim vs Vizard validation (s): preparation, impact, end
VALIDATION_TIMES = [2.500, 3.083, 3.600]

# Regulation table (m)
TABLE_LENGTH = 2.74
TABLE_WIDTH = 1.525
TABLE_HEIGHT = 0.76
NET_HEIGHT = 0.1525
TABLE_GAP = 0.40             # m, distance from initial pelvis to table edge
TABLE_EXTRA_YAW = 0.0        # degrees, manual adjustment if the table is rotated
TABLE_SHIFT = [0.0, 0.0]     # m (x, z), manual position adjustment

SEGMENTS = [
    'Pelvis', 'L5', 'L3', 'T12', 'T8', 'Neck', 'Head',
    'Right Shoulder', 'Right Upper Arm', 'Right Forearm', 'Right Hand',
    'Left Shoulder', 'Left Upper Arm', 'Left Forearm', 'Left Hand',
    'Right Upper Leg', 'Right Lower Leg', 'Right Foot', 'Right Toe',
    'Left Upper Leg', 'Left Lower Leg', 'Left Foot', 'Left Toe',
]

LINKS = [
    ('Pelvis', 'L5'), ('L5', 'L3'), ('L3', 'T12'), ('T12', 'T8'),
    ('T8', 'Neck'), ('Neck', 'Head'),
    ('T8', 'Right Shoulder'), ('Right Shoulder', 'Right Upper Arm'),
    ('Right Upper Arm', 'Right Forearm'), ('Right Forearm', 'Right Hand'),
    ('T8', 'Left Shoulder'), ('Left Shoulder', 'Left Upper Arm'),
    ('Left Upper Arm', 'Left Forearm'), ('Left Forearm', 'Left Hand'),
    ('Pelvis', 'Right Upper Leg'), ('Right Upper Leg', 'Right Lower Leg'),
    ('Right Lower Leg', 'Right Foot'), ('Right Foot', 'Right Toe'),
    ('Pelvis', 'Left Upper Leg'), ('Left Upper Leg', 'Left Lower Leg'),
    ('Left Lower Leg', 'Left Foot'), ('Left Foot', 'Left Toe'),
]

BONE_MAP = {
    'Pelvis': 'Bip01 Pelvis', 'L5': 'Bip01 Spine', 'L3': 'Bip01 Spine1',
    'T12': 'Bip01 Spine2', 'Neck': 'Bip01 Neck', 'Head': 'Bip01 Head',
    'Right Shoulder': 'Bip01 R Clavicle', 'Right Upper Arm': 'Bip01 R UpperArm',
    'Right Forearm': 'Bip01 R Forearm', 'Right Hand': 'Bip01 R Hand',
    'Left Shoulder': 'Bip01 L Clavicle', 'Left Upper Arm': 'Bip01 L UpperArm',
    'Left Forearm': 'Bip01 L Forearm', 'Left Hand': 'Bip01 L Hand',
    'Right Upper Leg': 'Bip01 R Thigh', 'Right Lower Leg': 'Bip01 R Calf',
    'Right Foot': 'Bip01 R Foot', 'Right Toe': 'Bip01 R Toe0',
    'Left Upper Leg': 'Bip01 L Thigh', 'Left Lower Leg': 'Bip01 L Calf',
    'Left Foot': 'Bip01 L Foot', 'Left Toe': 'Bip01 L Toe0',
}


# ============================================================
# UTILITIES
# ============================================================
def load_csv(filename):
    with open(filename, 'r') as f:
        return list(csv.DictReader(f))


def vsub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vadd(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vscale(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def vnorm(a):
    return math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)


def vunit(a):
    n = vnorm(a)
    return [0.0, 0.0, 0.0] if n < 1e-9 else vscale(a, 1.0 / n)


def angle_deg(a, b):
    na, nb = vnorm(a), vnorm(b)
    if na < 1e-9 or nb < 1e-9:
        return float('nan')
    c = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (na * nb)
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def make_box(size, color, parent=None):
    node = vizshape.addBox(size=size)
    node.color(color)
    if parent is not None:
        node.setParent(parent)
    return node


def make_sphere(radius, color, parent=None):
    node = vizshape.addSphere(radius=radius)
    node.color(color)
    if parent is not None:
        node.setParent(parent)
    return node


def xsens_pos(row, seg):
    x = float(row[seg + ' x'])
    y = float(row[seg + ' y'])
    z = float(row[seg + ' z'])
    return [-y, z, x]


def xsens_quat(row, seg):
    w = float(row[seg + ' q0'])
    x = float(row[seg + ' q1'])
    y = float(row[seg + ' q2'])
    z = float(row[seg + ' q3'])
    return viz.Quat(-y, z, x, -w)


def xsens_forward(row, seg='Pelvis'):
    """X axis (forward) of the segment, in Vizard coordinates."""
    w = float(row[seg + ' q0'])
    x = float(row[seg + ' q1'])
    y = float(row[seg + ' q2'])
    z = float(row[seg + ' q3'])
    fx = 1 - 2 * (y * y + z * z)
    fy = 2 * (x * y + w * z)
    fz = 2 * (x * z - w * y)
    return [-fy, fz, fx]


# ============================================================
# DATA
# ============================================================
orientation_rows = load_csv(ORIENTATION_CSV)
position_rows = load_csv(POSITION_CSV)
nframes = min(len(orientation_rows), len(position_rows))
if nframes == 0:
    raise RuntimeError('No MVN frames loaded.')

missing = [s for s in SEGMENTS if (s + ' x') not in position_rows[0]]
if missing:
    print('WARNING: position columns not found for:', missing)
    print('Available columns (first 10):', list(position_rows[0].keys())[:10])
SEG_OK = [s for s in SEGMENTS if s not in missing]

# Origin: initial pelvis at (0, real height, 0). Vertical is preserved.
pelvis0 = xsens_pos(position_rows[0], 'Pelvis')
ORIGIN_SHIFT = [-pelvis0[0], 0.0, -pelvis0[2]]

positions = []   # positions[i][seg] = [x, y, z] in Vizard (m)
for i in range(nframes):
    frame = {}
    for s in SEG_OK:
        frame[s] = vadd(xsens_pos(position_rows[i], s), ORIGIN_SHIFT)
    positions.append(frame)

HAND = PADDLE_SIDE + ' Hand'
FOREARM = PADDLE_SIDE + ' Forearm'
UPPERARM = PADDLE_SIDE + ' Upper Arm'


def paddle_proxy(i):
    """Paddle proxy center: wrist + PADDLE_OFFSET in elbow->wrist direction."""
    wrist = positions[i][HAND]
    elbow = positions[i][FOREARM]
    return vadd(wrist, vscale(vunit(vsub(wrist, elbow)), PADDLE_OFFSET))


def elbow_flexion(i):
    """Elbow flexion = 180 - included angle (shoulder-elbow-wrist), degrees."""
    shoulder = positions[i][UPPERARM]
    elbow = positions[i][FOREARM]
    wrist = positions[i][HAND]
    return 180.0 - angle_deg(vsub(shoulder, elbow), vsub(wrist, elbow))


proxy_path = [paddle_proxy(i) for i in range(nframes)]

# Frame of maximum proxy speed (approx. impact)
speeds = [0.0] + [vnorm(vsub(proxy_path[i], proxy_path[i - 1])) * FRAME_RATE
                  for i in range(1, nframes)]
i_ref = max(range(nframes), key=lambda k: speeds[k])
P_ref = proxy_path[i_ref]

# "Outward" direction: horizontal, from pelvis to proxy at that frame
out = vsub(P_ref, positions[i_ref]['Pelvis'])
out[1] = 0.0
U_OUT = vunit(out)

balls = [vadd(P_ref, vscale(U_OUT, d)) for d in CONDITIONS]


def evaluate(ball):
    dmin, imin = 1e9, 0
    for i, p in enumerate(proxy_path):
        d = vnorm(vsub(p, ball))
        if d < dmin:
            dmin, imin = d, i
    return dmin, imin


results = [evaluate(b) for b in balls]

print('==========================================================')
print('Frames:', nframes, '| Frame rate:', FRAME_RATE, 'Hz | Duration:',
      round((nframes - 1) / FRAME_RATE, 3), 's')
print('Paddle hand:', PADDLE_SIDE, '| proxy offset:', PADDLE_OFFSET, 'm')
print('Reference frame (peak proxy speed): %d (t = %.3f s, %.2f m/s)'
      % (i_ref, i_ref / FRAME_RATE, speeds[i_ref]))
print('Contact threshold: %.3f m (paddle %.3f + ball %.3f)'
      % (CONTACT_THRESHOLD, PADDLE_RADIUS, BALL_RADIUS))
print('Cond | d (m) | min dist (m) | t closest (s) | result')
for k, (d, (dmin, imin)) in enumerate(zip(CONDITIONS, results)):
    res = 'INTERCEPT' if dmin <= CONTACT_THRESHOLD else 'NO INTERCEPT'
    print('  %d  | %.2f  |    %.3f     |    %.3f      | %s'
          % (k + 1, d, dmin, imin / FRAME_RATE, res))
print('==========================================================')

# ============================================================
# SCENE
# ============================================================
viz.setMultiSample(4)
viz.go()
viz.clearcolor(viz.SKYBLUE)
viz.add('ground.osgb')
viz.add('sky_day.osgb')

# ---- Table ----
# Table direction = horizontal direction of the swing at the maximum-speed
# frame (the topspin goes toward the table). It can be adjusted live with
# [ ] (rotate) and - = (closer/farther); the values are printed to the console.
k0, k1 = max(0, i_ref - 2), min(nframes - 1, i_ref + 2)
fwd = vsub(proxy_path[k1], proxy_path[k0])
fwd[1] = 0.0
fwd = vunit(fwd)
if vnorm(fwd) < 1e-6:
    fwd = xsens_forward(orientation_rows[0], 'Pelvis')
    fwd[1] = 0.0
    fwd = vunit(fwd)
BASE_YAW = math.degrees(math.atan2(fwd[0], fwd[2]))
table_cfg = {'yaw': BASE_YAW + TABLE_EXTRA_YAW, 'gap': TABLE_GAP}
f_yaw = [0.0, 0.0, 1.0]
center = [0.0, 0.0, 0.0]


def place_table():
    global f_yaw, center
    yaw = table_cfg['yaw']
    f_yaw = [math.sin(math.radians(yaw)), 0.0, math.cos(math.radians(yaw))]
    center = vadd(vadd(positions[0]['Pelvis'],
                       vscale(f_yaw, table_cfg['gap'] + TABLE_LENGTH / 2.0)),
                  [TABLE_SHIFT[0], 0.0, TABLE_SHIFT[1]])
    table_root.setPosition([center[0], 0, center[2]])
    table_root.setEuler([yaw, 0, 0])

table_root = viz.addGroup()
top = make_box([TABLE_WIDTH, 0.03, TABLE_LENGTH], [0.05, 0.25, 0.55], table_root)
top.setPosition([0, TABLE_HEIGHT - 0.015, 0])
net = make_box([TABLE_WIDTH + 0.30, NET_HEIGHT, 0.01], viz.WHITE, table_root)
net.setPosition([0, TABLE_HEIGHT + NET_HEIGHT / 2.0, 0])
net.alpha(0.6)
for sx in (-1, 1):
    for sz in (-1, 1):
        leg = make_box([0.05, TABLE_HEIGHT - 0.03, 0.05], [0.2, 0.2, 0.2], table_root)
        leg.setPosition([sx * (TABLE_WIDTH / 2 - 0.1), (TABLE_HEIGHT - 0.03) / 2.0,
                         sz * (TABLE_LENGTH / 2 - 0.2)])
place_table()


def adjust_table(dyaw=0.0, dgap=0.0):
    table_cfg['yaw'] += dyaw
    table_cfg['gap'] = max(0.0, table_cfg['gap'] + dgap)
    place_table()
    print('TABLE: TABLE_EXTRA_YAW = %.1f   TABLE_GAP = %.2f'
          % (table_cfg['yaw'] - BASE_YAW, table_cfg['gap']))

# ---- Scale reference: 1 m ruler with marks every 10 cm ----
ruler = viz.addGroup()
for k in range(10):
    seg = make_box([0.1, 0.01, 0.04], viz.BLACK if k % 2 == 0 else viz.WHITE, ruler)
    seg.setPosition([0.05 + 0.1 * k, 0.005, 0])
label = viz.addText3D('1 m')
label.setParent(ruler)
label.setPosition([0.42, 0.03, -0.03])
label.setScale([0.08, 0.08, 0.08])
label.setEuler([0, 90, 0])
label.color(viz.BLACK)
_c0 = positions[0]['Pelvis']
_side = [f_yaw[2], 0.0, -f_yaw[0]]
_rp = vadd([_c0[0], 0.0, _c0[2]], vscale(_side, 0.7))
ruler.setPosition(_rp)
ruler.setEuler([table_cfg['yaw'] - 90.0, 0, 0])

# ---- Segment skeleton (actual positions) ----
skeleton = viz.addGroup()
joint_nodes = {}
for s in SEG_OK:
    color = [1.0, 0.45, 0.0] if s in (HAND, FOREARM, UPPERARM) else [0.9, 0.9, 0.9]
    joint_nodes[s] = make_sphere(0.025, color, skeleton)

viz.startLayer(viz.LINES)
viz.vertexColor(viz.BLACK)
link_list = [(a, b) for (a, b) in LINKS if a in SEG_OK and b in SEG_OK]
for a, b in link_list:
    viz.vertex(0, 0, 0)
    viz.vertex(0, 0, 0)
bones_lines = viz.endLayer()
bones_lines.setParent(skeleton)
bones_lines.dynamic()
try:
    bones_lines.lineWidth(4)
except Exception:
    pass

# ---- Paddle proxy ----
paddle = make_sphere(PADDLE_RADIUS, [0.8, 0.1, 0.1])
paddle.alpha(0.35)

# ---- Proxy trajectory (recorded sweep) ----
viz.startLayer(viz.LINE_STRIP)
viz.vertexColor([0.8, 0.1, 0.1])
for p in proxy_path:
    viz.vertex(p[0], p[1], p[2])
trail = viz.endLayer()
trail.visible(viz.OFF)

# ---- Simulated ball ----
ball = make_sphere(BALL_RADIUS, [1.0, 0.55, 0.0])

# ---- Avatar (optional) ----
avatar = viz.addAvatar(AVATAR_FILE)
avatar.setPosition([AVATAR_SIDE_OFFSET, 0, 0])
avatar_bones = {}
for seg, bone_name in BONE_MAP.items():
    try:
        b = avatar.getBone(bone_name)
        b.lock()
        avatar_bones[seg] = b
    except Exception as e:
        print('Avatar bone warning:', seg, e)

ADJ = -36.0
if 'Right Upper Arm' in avatar_bones:
    avatar_bones['Right Upper Arm'].setEuler([-ADJ, 0, 0], viz.REL_PARENT)
if 'Left Upper Arm' in avatar_bones:
    avatar_bones['Left Upper Arm'].setEuler([ADJ, 0, 0], viz.REL_PARENT)

sensor_q0, avatar_q0 = {}, {}
for seg, b in avatar_bones.items():
    try:
        sensor_q0[seg] = xsens_quat(orientation_rows[0], seg)
        avatar_q0[seg] = b.getQuat(viz.ABS_GLOBAL)
    except Exception as e:
        print('Calibration warning:', seg, e)
avatar_origin = avatar.getPosition(viz.ABS_GLOBAL)
avatar.visible(viz.OFF)

# ============================================================
# HUD
# ============================================================
def hud_text(y, size=0.32):
    t = viz.addText('', viz.SCREEN)
    t.setPosition([0.01, y, 0])
    t.setScale([size, size, size])
    t.color(viz.BLACK)
    t.setBackdrop(viz.BACKDROP_OUTLINE)
    t.setBackdropColor(viz.WHITE)
    return t


hud_time = hud_text(0.97, 0.20)
hud_mode = hud_text(0.94, 0.24)
hud_cond = hud_text(0.91, 0.20)
hud_crit = hud_text(0.885, 0.20)
hud_val = hud_text(0.86, 0.20)
hud_help = hud_text(0.10, 0.16)
HELP = ('SPACE play/pause | R restart | S speed 1.0x/0.25x\n'
        '1-4 condition | V show criterion | <- -> step frame\n'
        'T trail | A avatar | K skeleton | C camera | P print | H help\n'
        '[ ] rotate table | - = move table | G validation times')
hud_help.message(HELP)
hud_help.visible(viz.OFF)

# ============================================================
# STATE AND UPDATE
# ============================================================
state = {'t': 0.0, 'playing': True, 'speed': 1.0, 'cond': 0,
         'reveal': False, 'frame': 0, 'view': 0}


def apply_frame(i):
    fr = positions[i]
    for s, node in joint_nodes.items():
        node.setPosition(fr[s])
    for k, (a, b) in enumerate(link_list):
        bones_lines.setVertex(2 * k, fr[a])
        bones_lines.setVertex(2 * k + 1, fr[b])
    paddle.setPosition(proxy_path[i])

    if avatar.getVisible():
        row = orientation_rows[i]
        for seg, b in avatar_bones.items():
            if seg in sensor_q0:
                try:
                    q = avatar_q0[seg] * (sensor_q0[seg].inverse() * xsens_quat(row, seg))
                    b.setQuat(q, viz.ABS_GLOBAL)
                except (ValueError, KeyError):
                    pass
        p = positions[i]['Pelvis']
        p0 = positions[0]['Pelvis']
        avatar.setPosition(vadd(avatar_origin, vsub(p, p0)), viz.ABS_GLOBAL)


def update_hud(i):
    t = i / FRAME_RATE
    hud_time.message('t = %.3f s   frame %d / %d   %s'
                     % (t, i, nframes - 1, 'PLAYING' if state['playing'] else 'PAUSED'))
    if state['speed'] == 1.0:
        hud_mode.message('REAL TIME (1.0x)')
        hud_mode.color(viz.BLACK)
    else:
        hud_mode.message('SLOW MOTION (%.2fx)' % state['speed'])
        hud_mode.color(viz.RED)
    k = state['cond']
    hud_cond.message('Condition %d / %d:  ball offset d = %.2f m'
                     % (k + 1, len(CONDITIONS), CONDITIONS[k]))
    d_frame = vnorm(vsub(proxy_path[i], balls[k]))
    if state['reveal']:
        dmin, imin = results[k]
        res = 'INTERCEPT' if dmin <= CONTACT_THRESHOLD else 'NO INTERCEPT'
        hud_crit.message('Criterion: min dist = %.3f m (t = %.3f s)  |  ball-racket dist = %.3f m (thresh %.3f m)  ->  %s'
                         % (dmin, imin / FRAME_RATE, d_frame, CONTACT_THRESHOLD, res))
    else:
        hud_crit.message('Criterion: hidden (press V)')
    hud_val.message('%s elbow flexion = %.1f deg    %s hand height = %.3f m'
                    % (PADDLE_SIDE, elbow_flexion(i), PADDLE_SIDE, positions[i][HAND][1]))


def on_update():
    if state['playing']:
        state['t'] += viz.getFrameElapsed() * state['speed']
        i = int(state['t'] * FRAME_RATE)
        if i >= nframes:
            state['t'] = 0.0
            i = 0
        state['frame'] = i
    apply_frame(state['frame'])
    update_hud(state['frame'])


vizact.ontimer(0, on_update)


def set_condition(k):
    state['cond'] = k
    ball.setPosition(balls[k])


def toggle_play():
    state['playing'] = not state['playing']
    state['t'] = state['frame'] / FRAME_RATE


def restart():
    state['t'] = 0.0
    state['frame'] = 0


def toggle_speed():
    state['speed'] = SLOW_FACTOR if state['speed'] == 1.0 else 1.0


def step(n):
    if not state['playing']:
        state['frame'] = max(0, min(nframes - 1, state['frame'] + n))
        state['t'] = state['frame'] / FRAME_RATE


def toggle(node):
    node.visible(viz.TOGGLE)


VIEWS = []


def build_views():
    c = positions[0]['Pelvis']
    side = [f_yaw[2], 0, -f_yaw[0]]
    VIEWS.append((vadd(vadd(c, vscale(side, -3.0)), [0, 0.6, -0.5]), [c[0], 1.0, c[2]]))
    VIEWS.append((vadd(vadd(c, vscale(f_yaw, 3.5)), [0, 1.0, 0]), [c[0], 1.0, c[2]]))
    VIEWS.append((vadd(c, [0, 4.0, 0.01]), [c[0], 0.8, c[2]]))
    VIEWS.append((vadd(vadd(c, vscale(f_yaw, -2.5)), [0, 0.8, 0]),
                  [center[0], TABLE_HEIGHT, center[2]]))


def next_view():
    state['view'] = (state['view'] + 1) % len(VIEWS)
    pos, look = VIEWS[state['view']]
    viz.MainView.setPosition(pos)
    viz.MainView.lookAt(look)


def goto_validation():
    state.setdefault('vidx', -1)
    state['vidx'] = (state['vidx'] + 1) % len(VALIDATION_TIMES)
    state['playing'] = False
    state['frame'] = min(nframes - 1, int(round(VALIDATION_TIMES[state['vidx']] * FRAME_RATE)))
    state['t'] = state['frame'] / FRAME_RATE
    print_values()


def print_values():
    i = state['frame']
    k = state['cond']
    d_frame = vnorm(vsub(proxy_path[i], balls[k]))
    print('t = %.3f s | frame %d | %s elbow flexion = %.1f deg | %s hand height = %.3f m | ball-racket dist = %.3f m'
          % (i / FRAME_RATE, i, PADDLE_SIDE, elbow_flexion(i), PADDLE_SIDE,
             positions[i][HAND][1], d_frame))


vizact.onkeydown(' ', toggle_play)
vizact.onkeydown('r', restart)
vizact.onkeydown('s', toggle_speed)
vizact.onkeydown('v', lambda: state.update(reveal=not state['reveal']))
vizact.onkeydown('t', toggle, trail)
vizact.onkeydown('a', toggle, avatar)
vizact.onkeydown('k', toggle, skeleton)
vizact.onkeydown('h', toggle, hud_help)
vizact.onkeydown('c', next_view)
vizact.onkeydown('p', print_values)
vizact.onkeydown('g', goto_validation)
vizact.onkeydown('n', adjust_table, -5.0, 0.0)
vizact.onkeydown('m', adjust_table, 5.0, 0.0)
vizact.onkeydown('u', adjust_table, 0.0, -0.05)
vizact.onkeydown('i', adjust_table, 0.0, 0.05)
vizact.onkeydown(viz.KEY_RIGHT, step, 1)
vizact.onkeydown(viz.KEY_LEFT, step, -1)
for k in range(len(CONDITIONS)):
    vizact.onkeydown(str(k + 1), set_condition, k)

build_views()
state['view'] = -1
next_view()
set_condition(0)