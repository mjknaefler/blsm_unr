#!/usr/bin/env python3
"""
Blossom Robot Motor Calibration Script

Guides the user through leveling the robot head and saves the
calibrated home positions to motor_config.yaml.

Usage:
    python3 calibrate_robot.py [port]          # default: /dev/ttyUSB0
    python3 calibrate_robot.py /dev/ttyUSB0

This script should be run BEFORE launching the robot for the first time
on a new physical unit. The saved config will be used as the home position
on every subsequent startup.
"""

import sys
import time
import yaml
import os

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    print("ERROR: dynamixel_sdk not installed. Run: pip install dynamixel-sdk")
    sys.exit(1)

PORT     = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
BAUDRATE = 1_000_000
PROTOCOL = 2.0

ADDR_TORQUE_ENABLE = 24
ADDR_MOVING_SPEED  = 32
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POS   = 37

MOTOR_IDS = {
    'lazy_susan':       1,
    'motor_front':      2,
    'motor_back_left':  3,
    'motor_back_right': 4,
    'ear':              5,
}

MOTOR_LIMITS = {
    'lazy_susan':       (0, 1023),
    'motor_front':      (0, 950),
    'motor_back_left':  (0, 950),
    'motor_back_right': (0, 950),
    'ear':              (500, 1023),
}

MOTOR_SPEEDS = {
    'lazy_susan':       200,
    'motor_front':      200,
    'motor_back_left':  200,
    'motor_back_right': 200,
    'ear':              200,
}

CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'config', 'motor_config.yaml'
)


def open_port():
    ph = PortHandler(PORT)
    pkt = PacketHandler(PROTOCOL)
    if not ph.openPort():
        sys.exit(f'ERROR: Cannot open port {PORT}')
    if not ph.setBaudRate(BAUDRATE):
        sys.exit(f'ERROR: Cannot set baudrate {BAUDRATE}')
    print(f'Connected to {PORT} at {BAUDRATE} baud')
    return ph, pkt


def read_position(ph, pkt, motor_id):
    pos, result, error = pkt.read2ByteTxRx(ph, motor_id, ADDR_PRESENT_POS)
    if result == COMM_SUCCESS and error == 0:
        return pos
    return None


def set_position(ph, pkt, motor_id, position, speed=150):
    pkt.write2ByteTxRx(ph, motor_id, ADDR_MOVING_SPEED, speed)
    pkt.write2ByteTxRx(ph, motor_id, ADDR_GOAL_POSITION, position)


def enable_torque(ph, pkt, motor_id, enable=True):
    pkt.write1ByteTxRx(ph, motor_id, ADDR_TORQUE_ENABLE, 1 if enable else 0)


def scan_motors(ph, pkt):
    found = []
    print('Scanning for motors...')
    for name, mid in MOTOR_IDS.items():
        _, result, _ = pkt.ping(ph, mid)
        if result == COMM_SUCCESS:
            found.append((name, mid))
            print(f'  Found: {name} (ID: {mid})')
        else:
            print(f'  Not found: {name} (ID: {mid})')
    return found


def read_all_positions(ph, pkt, motors):
    positions = {}
    for name, mid in motors:
        pos = read_position(ph, pkt, mid)
        positions[name] = pos if pos is not None else 0
        status = str(pos) if pos is not None else 'ERROR'
        print(f'  {name} (ID: {mid}): {status}')
    return positions


def interactive_adjust(ph, pkt, motors):
    """Let user interactively adjust motor positions."""
    print('\n--- INTERACTIVE ADJUSTMENT ---')
    print('Commands:')
    print('  <motor_name> <position>  — move a motor (e.g. "motor_front 350")')
    print('  read                     — read all current positions')
    print('  done                     — save and exit')
    print('  quit                     — exit without saving')
    print()

    motor_map = {name: mid for name, mid in motors}
    current = {}
    for name, mid in motors:
        pos = read_position(ph, pkt, mid)
        current[name] = pos if pos is not None else 0

    while True:
        try:
            cmd = input('> ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            print('\nAborted.')
            return None

        if cmd == 'done':
            return current
        elif cmd == 'quit':
            return None
        elif cmd == 'read':
            print('Current positions:')
            for name, mid in motors:
                pos = read_position(ph, pkt, mid)
                current[name] = pos if pos is not None else current.get(name, 0)
                print(f'  {name}: {current[name]}')
        else:
            parts = cmd.split()
            if len(parts) == 2:
                name, val = parts[0], parts[1]
                if name not in motor_map:
                    print(f'Unknown motor: {name}. Options: {list(motor_map.keys())}')
                    continue
                try:
                    position = int(val)
                except ValueError:
                    print(f'Invalid position: {val}')
                    continue
                lo, hi = MOTOR_LIMITS.get(name, (0, 1023))
                if not (lo <= position <= hi):
                    print(f'Position {position} out of range [{lo}, {hi}] for {name}')
                    continue
                set_position(ph, pkt, motor_map[name], position)
                current[name] = position
                print(f'  {name} -> {position}')
            else:
                print('Invalid command. Type "done" to save or "quit" to exit.')


def save_config(home_positions):
    config = {
        'motor_ids': {name: mid for name, mid in MOTOR_IDS.items()},
        'motor_limits': {name: list(limits) for name, limits in MOTOR_LIMITS.items()},
        'home_positions': home_positions,
        'motor_speeds': MOTOR_SPEEDS,
    }
    
    # Save to source config (relative to script)
    source_path = os.path.abspath(CONFIG_PATH)
    os.makedirs(os.path.dirname(source_path), exist_ok=True)
    with open(source_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    print(f'Config saved to source: {source_path}')

    # Also save to installed path if it exists
    installed_path = os.path.expanduser(
        '~/ros2_ws/install/openhmi_blossom/share/openhmi_blossom/config/motor_config.yaml'
    )
    if os.path.exists(os.path.dirname(installed_path)):
        with open(installed_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        print(f'Config saved to install: {installed_path}')
        print('No rebuild needed — installed config updated directly.')
    else:
        print('NOTE: Rebuild required to apply config:')
        print('  cd ~/ros2_ws && colcon build --packages-select openhmi_blossom')


def main():
    print('=' * 50)
    print('  Blossom Robot Motor Calibration')
    print('=' * 50)

    ph, pkt = open_port()

    print()
    motors = scan_motors(ph, pkt)
    if not motors:
        print('ERROR: No motors found. Check connections and power.')
        ph.closePort()
        sys.exit(1)

    print('\nEnabling torque...')
    for name, mid in motors:
        enable_torque(ph, pkt, mid, True)
        pkt.write2ByteTxRx(ph, mid, ADDR_MOVING_SPEED, 100)

    print('\nStep 1: Moving to approximate level position...')
    approx_home = {
        'lazy_susan':       512,
        'motor_front':      350,
        'motor_back_left':  18,
        'motor_back_right': 133,
        'ear':              761,
    }
    for name, mid in motors:
        if name in approx_home:
            set_position(ph, pkt, mid, approx_home[name])
    time.sleep(3)

    print('\nStep 2: Current positions after moving to approximate level:')
    read_all_positions(ph, pkt, motors)

    print('\nStep 3: Fine-tune the head level.')
    print('Physically observe the robot and adjust motors until the head is level.')
    print('The face screen should be vertical and the head should not tilt in any direction.')
    print()
    final_positions = interactive_adjust(ph, pkt, motors)

    if final_positions is None:
        print('Calibration cancelled — no changes saved.')
        ph.closePort()
        return

    print('\nFinal calibrated home positions:')
    for name, pos in final_positions.items():
        print(f'  {name}: {pos}')

    confirm = input('\nSave these as home positions? [y/N]: ').strip().lower()
    if confirm == 'y':
        save_config(final_positions)
        print('\nCalibration complete!')
        print('Update your launch file to pass the motor_config parameter:')
        print(f'  motor_config:={os.path.abspath(CONFIG_PATH)}')
    else:
        print('Calibration cancelled — no changes saved.')

    ph.closePort()


if __name__ == '__main__':
    main()