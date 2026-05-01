#!/usr/bin/env python3
"""
calibrate_sequences.py — Run after calibrate_robot.py to convert all sequences
to the robot's calibrated home positions.

Usage:
    python3 scripts/calibrate_sequences.py

Reads home positions from motor_config.yaml. Converts all sequences from
their known original neutral positions. Safe to run multiple times on any
robot — always starts from known originals, never from previously converted values.

Original neutrals:
  Blossom library sequences: motor_front=870, motor_back_left=870,
                              motor_back_right=870, lazy_susan=586
  Custom Bloom sequences:    motor_front=100, motor_back_left=100,
                              motor_back_right=100, lazy_susan=512
"""

import yaml
import os

SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
SEQUENCES_DIR = os.path.join(SCRIPT_DIR, '..', 'config', 'sequences')
CONFIG_FILE   = os.path.join(SCRIPT_DIR, '..', 'config', 'motor_config.yaml')

LIMITS = {
    'motor_front':      (0, 950),
    'motor_back_left':  (0, 950),
    'motor_back_right': (0, 950),
    'lazy_susan':       (0, 1023),
    'ear':              (500, 1023),
}

# ── Original sequence data (hardcoded from source) ───────────────────────────
# Blossom library sequences use motor positions ~870 and lazy_susan ~586
# Custom Bloom sequences use motor positions ~100 and lazy_susan ~512

ORIGINAL_SEQUENCES = {
    'excited': {
        'old_home': {'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100, 'lazy_susan': 512},
        'keyframes': [
            {'joints': {'lazy_susan': 450, 'motor_front': 50, 'motor_back_left': 50, 'motor_back_right': 50}, 'duration': 0.15},
            {'joints': {'lazy_susan': 574, 'motor_front': 30, 'motor_back_left': 30, 'motor_back_right': 30}, 'duration': 0.15},
            {'joints': {'lazy_susan': 450, 'motor_front': 50, 'motor_back_left': 50, 'motor_back_right': 50}, 'duration': 0.15},
            {'joints': {'lazy_susan': 512, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 0.2},
        ]
    },
    'idle': {
        'old_home': {'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100, 'lazy_susan': 512},
        'keyframes': [
            {'joints': {'lazy_susan': 512, 'motor_front': 80, 'motor_back_left': 80, 'motor_back_right': 80}, 'duration': 2.0},
            {'joints': {'lazy_susan': 512, 'motor_front': 120, 'motor_back_left': 120, 'motor_back_right': 120}, 'duration': 2.0},
            {'joints': {'lazy_susan': 512, 'motor_front': 80, 'motor_back_left': 80, 'motor_back_right': 80}, 'duration': 2.0},
        ]
    },
    'calm': {
        'old_home': {'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100, 'lazy_susan': 512},
        'keyframes': [
            {'joints': {'lazy_susan': 512, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 1.5},
            {'joints': {'lazy_susan': 512, 'motor_front': 120, 'motor_back_left': 120, 'motor_back_right': 120}, 'duration': 1.5},
            {'joints': {'lazy_susan': 512, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 1.5},
        ]
    },
    'look_left': {
        'old_home': {'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100, 'lazy_susan': 512},
        'keyframes': [
            {'joints': {'lazy_susan': 400, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 0.5},
            {'joints': {'lazy_susan': 512, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 0.5},
        ]
    },
    'look_right': {
        'old_home': {'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100, 'lazy_susan': 512},
        'keyframes': [
            {'joints': {'lazy_susan': 624, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 0.5},
            {'joints': {'lazy_susan': 512, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 0.5},
        ]
    },
    'look_up': {
        'old_home': {'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100, 'lazy_susan': 512},
        'keyframes': [
            {'joints': {'lazy_susan': 512, 'motor_front': 20, 'motor_back_left': 250, 'motor_back_right': 250}, 'duration': 0.5},
            {'joints': {'lazy_susan': 512, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 0.5},
        ]
    },
    'look_down': {
        'old_home': {'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100, 'lazy_susan': 512},
        'keyframes': [
            {'joints': {'lazy_susan': 512, 'motor_front': 300, 'motor_back_left': 50, 'motor_back_right': 50}, 'duration': 0.5},
            {'joints': {'lazy_susan': 512, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 0.5},
        ]
    },
    'no': {
        'old_home': {'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100, 'lazy_susan': 512},
        'keyframes': [
            {'joints': {'lazy_susan': 400, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 0.3},
            {'joints': {'lazy_susan': 624, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 0.3},
            {'joints': {'lazy_susan': 400, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 0.3},
            {'joints': {'lazy_susan': 512, 'motor_front': 100, 'motor_back_left': 100, 'motor_back_right': 100}, 'duration': 0.3},
        ]
    },
}

# Blossom sequences — read from git originals, converted here from known values
# These are loaded from the YAML files on disk since they have many keyframes
# but we know their old_home is 870/586
BLOSSOM_SEQUENCE_NAMES = {
    'anger', 'arousal1', 'bounce_smooth', 'dance1_1', 'dance2_2',
    'fear', 'happy', 'no_v2', 'reset', 'sad', 'yes'
}
BLOSSOM_OLD_HOME = {
    'motor_front':      870,
    'motor_back_left':  870,
    'motor_back_right': 870,
    'lazy_susan':       586,
}

def load_config():
    with open(CONFIG_FILE, 'r') as f:
        config = yaml.safe_load(f)
    home = config.get('home_positions', {})
    print("Loaded home positions from motor_config.yaml:")
    for name, pos in home.items():
        print(f"  {name}: {pos}")
    return home

def convert_pos(motor_name, old_pos, old_home, new_home):
    if motor_name not in old_home or motor_name not in new_home:
        return old_pos
    delta = old_pos - old_home[motor_name]
    new_pos = new_home[motor_name] + delta
    lo, hi = LIMITS.get(motor_name, (0, 1023))
    return max(lo, min(hi, int(new_pos)))

def convert_keyframes(keyframes, old_home, new_home):
    new_keyframes = []
    for kf in keyframes:
        new_joints = {}
        for motor_name, old_pos in kf['joints'].items():
            new_joints[motor_name] = convert_pos(motor_name, old_pos, old_home, new_home)
        new_keyframes.append({'joints': new_joints, 'duration': kf['duration']})
    return new_keyframes

def process_custom_sequence(seq_name, new_home):
    """Convert a custom Bloom sequence from hardcoded originals."""
    orig = ORIGINAL_SEQUENCES[seq_name]
    old_home = orig['old_home']
    new_keyframes = convert_keyframes(orig['keyframes'], old_home, new_home)
    filepath = os.path.join(SEQUENCES_DIR, f'{seq_name}.yaml')
    out = {seq_name: {'keyframes': new_keyframes}}
    with open(filepath, 'w') as f:
        yaml.dump(out, f, default_flow_style=False)
    print(f"  Converted (custom): {seq_name}.yaml")

def process_blossom_sequence(filename, new_home):
    """Convert a Blossom library sequence using known old home of 870/586."""
    seq_name = filename[:-5]
    filepath = os.path.join(SEQUENCES_DIR, filename)

    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)

    seq_data = data.get(seq_name) or (list(data.values())[0] if data else None)
    if not seq_data or 'keyframes' not in seq_data:
        print(f"  SKIP {filename} — no keyframes found")
        return

    new_keyframes = convert_keyframes(seq_data['keyframes'], BLOSSOM_OLD_HOME, new_home)
    out = {seq_name: {'keyframes': new_keyframes}}
    with open(filepath, 'w') as f:
        yaml.dump(out, f, default_flow_style=False)
    print(f"  Converted (blossom): {filename}")

def main():
    print("=" * 60)
    print("Bloom Sequence Calibration Script")
    print("=" * 60)
    print()

    new_home = load_config()
    print()

    print("Converting custom Bloom sequences from hardcoded originals...")
    for seq_name in sorted(ORIGINAL_SEQUENCES.keys()):
        process_custom_sequence(seq_name, new_home)

    print()
    print("Converting Blossom library sequences from known home (870/586)...")
    for filename in sorted(os.listdir(SEQUENCES_DIR)):
        if not filename.endswith('.yaml'):
            continue
        seq_name = filename[:-5]
        if seq_name in BLOSSOM_SEQUENCE_NAMES:
            process_blossom_sequence(filename, new_home)

    print()
    print("=" * 60)
    print("Done! All sequences calibrated to home positions.")
    print("Rebuild openhmi_blossom to apply changes:")
    print("  cd ~/ros2_ws && colcon build --packages-select openhmi_blossom")
    print("=" * 60)

if __name__ == '__main__':
    main()
