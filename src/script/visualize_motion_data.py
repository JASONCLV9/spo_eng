"""
Vizard offline Xsens MVN playback - MOTION SELECTION (Topspin, Forward, Smash)
  - Retargeting using relative LOCAL quaternion motion.
  - Pelvis translation from MVN Segment Position.
  - Xsens -> Vizard coordinate mapping:
        position:   (x,y,z)_Xsens   -> (-y, z, x)_Vizard
        quaternion: (w,x,y,z)_Xsens -> (-y, z, x, -w)_Vizard

KEYBOARD CONTROLS:
  [1] or [T]: Topspin
  [2] or [F]: Forward
  [3] or [S]: Smash
"""

import viz
import viztask
import vizact
import csv
import os

# ------------------------------------------------------------
# MOTION CONFIGURATION AND RELATIVE PATHS (os.path)
# ------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, '..', 'motion-data'))

# Initial motion by default: 'topspin', 'forward', or 'smash'
INITIAL_MOTION = 'topspin'

MOTIONS = {
    'topspin': {
        'label': 'Topspin',
        'orientation_file': 'Top_spin_SegmentOrientation.csv',
        'position_file': 'Top_spin_SegmentPosition.csv',
    },
    'forward': {
        'label': 'Forward',
        'orientation_file': 'Forward_SegmentOrientation.csv',
        'position_file': 'Forward_SegmentPosition.csv',
    },
    'smash': {
        'label': 'Smash',
        'orientation_file': 'Smash_SegmentOrientation.csv',
        'position_file': 'Smash_SegmentPosition.csv',
    }
}


def resolve_csv_path(filename):
    """Look for the CSV file in motion-data or in the same folder as the script."""
    path_data = os.path.join(DATA_DIR, filename)
    if os.path.exists(path_data):
        return path_data
    path_script = os.path.join(SCRIPT_DIR, filename)
    if os.path.exists(path_script):
        return path_script
    return path_data


# Variables kept for backward compatibility
ORIENTATION_CSV = resolve_csv_path(MOTIONS[INITIAL_MOTION]['orientation_file'])
POSITION_CSV = resolve_csv_path(MOTIONS[INITIAL_MOTION]['position_file'])

FRAME_RATE = 60.0
DT = 1.0 / FRAME_RATE
LOOP_PLAYBACK = True

AVATAR_FILE = 'vcc_male.cfg'

# 1.0 = real MVN meters
ROOT_SCALE = 1.0

BONE_MAP = {
    'Pelvis':          'Bip01 Pelvis',

    'L5':              'Bip01 Spine',
    'L3':              'Bip01 Spine1',
    'T12':             'Bip01 Spine2',

    'Neck':            'Bip01 Neck',
    'Head':            'Bip01 Head',

    'Right Shoulder':  'Bip01 R Clavicle',
    'Right Upper Arm': 'Bip01 R UpperArm',
    'Right Forearm':   'Bip01 R Forearm',
    'Right Hand':      'Bip01 R Hand',

    'Left Shoulder':   'Bip01 L Clavicle',
    'Left Upper Arm':  'Bip01 L UpperArm',
    'Left Forearm':    'Bip01 L Forearm',
    'Left Hand':       'Bip01 L Hand',

    'Right Upper Leg': 'Bip01 R Thigh',
    'Right Lower Leg': 'Bip01 R Calf',
    'Right Foot':      'Bip01 R Foot',
    'Right Toe':       'Bip01 R Toe0',

    'Left Upper Leg':  'Bip01 L Thigh',
    'Left Lower Leg':  'Bip01 L Calf',
    'Left Foot':       'Bip01 L Foot',
    'Left Toe':        'Bip01 L Toe0',
}


def load_csv(filename):
    rows = []
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def xsens_to_vizard_quat(row, segment):
    """MVN: q0 = w, q1 = x, q2 = y, q3 = z  ->  Vizard (x, y, z, w)."""
    w = float(row[segment + ' q0'])
    x = float(row[segment + ' q1'])
    y = float(row[segment + ' q2'])
    z = float(row[segment + ' q3'])
    return viz.Quat(-y, z, x, -w)


def xsens_position_to_vizard(row, segment='Pelvis'):
    """
    Xsens: X = forward, Y = left, Z = up
    Vizard: X = right, Y = up, Z = forward
    => [Xv, Yv, Zv] = [-Yx, Zx, Xx]
    """
    x = float(row[segment + ' x'])
    y = float(row[segment + ' y'])
    z = float(row[segment + ' z'])
    return [-y, z, x]


viz.go()

# Basic environment included with Vizard
viz.add('ground.osgb')
viz.add('sky_day.osgb')

avatar = viz.addAvatar(AVATAR_FILE)
avatar.setPosition([0, 0, 0])

# Front-side camera to view the stroke
viz.MainView.setPosition([3.0, 1.5, 3.0])
viz.MainView.lookAt([0, 1.0, 0])

bones = {}
for segment, bone_name in BONE_MAP.items():
    try:
        b = avatar.getBone(bone_name)
        b.lock()
        bones[segment] = b
        print('Mapped:', segment, '->', bone_name)
    except Exception as e:
        print('WARNING:', segment, bone_name, e)

# ------------------------------------------------------------
# INITIAL AVATAR CALIBRATION (Rest pose)
# ------------------------------------------------------------
ADJUSTMENT_ANGLE = -36.0
if 'Right Upper Arm' in bones:
    bones['Right Upper Arm'].setEuler([-ADJUSTMENT_ANGLE, 0, 0], viz.REL_PARENT)
if 'Left Upper Arm' in bones:
    bones['Left Upper Arm'].setEuler([ADJUSTMENT_ANGLE, 0, 0], viz.REL_PARENT)

avatar_q0 = {}
for segment, bone in bones.items():
    try:
        avatar_q0[segment] = bone.getQuat(viz.ABS_GLOBAL)
    except Exception as e:
        print('Avatar q0 warning:', segment, e)

avatar_origin = avatar.getPosition(viz.ABS_GLOBAL)

# ------------------------------------------------------------
# CACHE AND LOADING OF MOTION DATA
# ------------------------------------------------------------
motion_cache = {}


def get_motion_data(motion_key):
    """Load and prepare the data for the requested motion."""
    if motion_key in motion_cache:
        return motion_cache[motion_key]

    cfg = MOTIONS[motion_key]
    orient_path = resolve_csv_path(cfg['orientation_file'])
    pos_path = resolve_csv_path(cfg['position_file'])

    print('Loading data for:', cfg['label'])
    o_rows = load_csv(orient_path)
    p_rows = load_csv(pos_path)

    n_frames = min(len(o_rows), len(p_rows))
    if n_frames == 0:
        raise RuntimeError('Could not load frames for: ' + cfg['label'])

    first_q = o_rows[0]
    sensor_q = {}
    for segment in bones.keys():
        try:
            sensor_q[segment] = xsens_to_vizard_quat(first_q, segment)
        except Exception as e:
            print('Calibration warning:', segment, e)

    p_zero = xsens_position_to_vizard(p_rows[0], 'Pelvis')

    data = {
        'label': cfg['label'],
        'orientation_rows': o_rows,
        'position_rows': p_rows,
        'nframes': n_frames,
        'sensor_q0': sensor_q,
        'p0': p_zero
    }
    motion_cache[motion_key] = data
    return data


def apply_orientation(row, sensor_q0):
    """q_delta_local = inv(q_initial) * q_current ;  q_final = q_avatar0 * q_delta_local"""
    for segment, bone in bones.items():
        if segment not in sensor_q0:
            continue
        try:
            q_current = xsens_to_vizard_quat(row, segment)
            q_delta_local = sensor_q0[segment].inverse() * q_current
            q_final = avatar_q0[segment] * q_delta_local
            bone.setQuat(q_final, viz.ABS_GLOBAL)
        except (ValueError, KeyError):
            pass


def apply_root_translation(row, p0):
    p = xsens_position_to_vizard(row, 'Pelvis')
    dx = (p[0] - p0[0]) * ROOT_SCALE
    dy = (p[1] - p0[1]) * ROOT_SCALE
    dz = (p[2] - p0[2]) * ROOT_SCALE
    avatar.setPosition([
        avatar_origin[0] + dx,
        avatar_origin[1] + dy,
        avatar_origin[2] + dz
    ], viz.ABS_GLOBAL)


# ------------------------------------------------------------
# MOTION STATE AND ON-SCREEN HUD
# ------------------------------------------------------------
current_motion_state = {'name': INITIAL_MOTION.lower()}

hud_title = viz.addText('', viz.SCREEN)
hud_title.setPosition([0.02, 0.95, 0])
hud_title.setScale([0.28, 0.28, 0.28])
hud_title.color(viz.WHITE)
hud_title.setBackdrop(viz.BACKDROP_OUTLINE)
hud_title.setBackdropColor(viz.BLACK)

hud_info = viz.addText('', viz.SCREEN)
hud_info.setPosition([0.02, 0.91, 0])
hud_info.setScale([0.20, 0.20, 0.20])
hud_info.color(viz.YELLOW)
hud_info.setBackdrop(viz.BACKDROP_OUTLINE)
hud_info.setBackdropColor(viz.BLACK)
hud_info.message('Keys: [1/T] Topspin   |   [2/F] Forward   |   [3/S] Smash')


def update_hud():
    m_name = current_motion_state['name']
    label = MOTIONS.get(m_name, {}).get('label', m_name).upper()
    hud_title.message('CURRENT MOTION: ' + label)


def set_motion(motion_name):
    motion_name = motion_name.lower()
    if motion_name == 'fordward':
        motion_name = 'forward'
    if motion_name not in MOTIONS:
        print('Unknown motion:', motion_name)
        return
    if current_motion_state['name'] == motion_name:
        return
    print('\n==================================================')
    print('Switching motion to:', MOTIONS[motion_name]['label'])
    print('==================================================')
    current_motion_state['name'] = motion_name
    update_hud()


vizact.onkeydown('1', set_motion, 'topspin')
vizact.onkeydown('2', set_motion, 'forward')
vizact.onkeydown('3', set_motion, 'smash')

vizact.onkeydown('t', set_motion, 'topspin')
vizact.onkeydown('T', set_motion, 'topspin')
vizact.onkeydown('f', set_motion, 'forward')
vizact.onkeydown('F', set_motion, 'forward')
vizact.onkeydown('s', set_motion, 'smash')
vizact.onkeydown('S', set_motion, 'smash')


def playback():
    while True:
        m_name = current_motion_state['name']
        data = get_motion_data(m_name)
        o_rows = data['orientation_rows']
        p_rows = data['position_rows']
        s_q0 = data['sensor_q0']
        p_zero = data['p0']
        total_frames = data['nframes']

        update_hud()
        print('Playing: %s (%d frames, %.2f s)' % (
            data['label'], total_frames, round(total_frames / FRAME_RATE, 2)))

        for i in range(total_frames):
            # If the motion was changed at runtime
            if current_motion_state['name'] != m_name:
                break

            apply_orientation(o_rows[i], s_q0)
            apply_root_translation(p_rows[i], p_zero)

            if i % 60 == 0:
                print('[%s] Frame %d / %d' % (data['label'], i, total_frames - 1))

            yield viztask.waitTime(DT)

        avatar.setPosition(avatar_origin, viz.ABS_GLOBAL)

        if not LOOP_PLAYBACK and current_motion_state['name'] == m_name:
            break

        if current_motion_state['name'] == m_name:
            print('Restarting playback (%s)...' % data['label'])


update_hud()
viztask.schedule(playback())

print('')
print('MVN offline playback started.')
print('Press [1/T] Topspin, [2/F] Forward, [3/S] Smash to switch between motions.')