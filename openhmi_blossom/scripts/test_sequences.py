#!/usr/bin/env python3
"""
Standalone motor sequence tester — no ROS required.
Plays each sequence one at a time, logs voltage and errors to a file.
"""

import sys
import os
import time
import yaml
import datetime

try:
    from dynamixel_sdk import *
    SDK_AVAILABLE = True
except ImportError:
    print("ERROR: dynamixel_sdk not available")
    sys.exit(1)

# ── Config ──────────────────────────────────────────────────────────────────
PORT            = '/dev/ttyUSB0'
BAUDRATE        = 1000000
PROTOCOL        = 2.0

ADDR_GOAL_POS   = 30
ADDR_MOVING_SPD = 32
ADDR_TORQUE_EN  = 24
ADDR_TORQUE_LIM = 35
ADDR_VOLTAGE    = 45
ADDR_HW_ERROR   = 50

MOTOR_IDS = {
    'lazy_susan':       1,
    'motor_front':      2,
    'motor_back_left':  3,
    'motor_back_right': 4,
}

HOME = {
    'lazy_susan':       512,
    'motor_front':      352,
    'motor_back_left':  18,
    'motor_back_right': 133,
}

SEQUENCES_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'config', 'sequences'
)

LOG_FILE = os.path.expanduser('~/motor_test_log.txt')

# ── Helpers ──────────────────────────────────────────────────────────────────
port_handler   = None
packet_handler = None
log_lines      = []

def log(msg):
    ts = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
    line = f"[{ts}] {msg}"
    print(line)
    log_lines.append(line)

def read_voltage(motor_id):
    raw, result, _ = packet_handler.read1ByteTxRx(port_handler, motor_id, ADDR_VOLTAGE)
    if result != COMM_SUCCESS:
        return None
    return raw * 0.1

def read_hw_error(motor_id):
    val, result, _ = packet_handler.read1ByteTxRx(port_handler, motor_id, ADDR_HW_ERROR)
    if result != COMM_SUCCESS:
        return None
    return val

def set_position(motor_id, position, speed=200):
    packet_handler.write2ByteTxRx(port_handler, motor_id, ADDR_MOVING_SPD, speed)
    packet_handler.write2ByteTxRx(port_handler, motor_id, ADDR_GOAL_POS, int(position))

def go_home():
    log("Moving to home positions...")
    for name, motor_id in MOTOR_IDS.items():
        set_position(motor_id, HOME[name], speed=150)
        time.sleep(0.05)
    time.sleep(1.5)

def check_all_voltages():
    for name, motor_id in MOTOR_IDS.items():
        v = read_voltage(motor_id)
        err = read_hw_error(motor_id)
        v_str = f"{v:.1f}V" if v is not None else "N/A"
        err_str = f"0x{err:02x}" if err is not None else "N/A"
        status = "OK" if (err == 0) else "ERROR"
        log(f"  {name} (ID {motor_id}): {v_str} | hw_error={err_str} | {status}")

def play_sequence(name, seq_data):
    log(f"\n{'='*50}")
    log(f"SEQUENCE: {name}")
    log(f"{'='*50}")
    
    log("Voltages BEFORE:")
    check_all_voltages()
    
    keyframes = seq_data.get('keyframes', [])
    log(f"Playing {len(keyframes)} keyframes...")
    
    errors_during = []
    
    for i, kf in enumerate(keyframes):
        joints = kf.get('joints', {})
        duration = kf.get('duration', 0.5)
        
        # Calculate speed from duration
        for motor_name, position in joints.items():
            if motor_name not in MOTOR_IDS:
                continue
            motor_id = MOTOR_IDS[motor_name]
            
            # Check hw error before sending command
            err = read_hw_error(motor_id)
            if err and err != 0:
                msg = f"  HW ERROR on {motor_name} (ID {motor_id}) before keyframe {i+1}: 0x{err:02x}"
                log(msg)
                errors_during.append(msg)
            
            speed = max(50, min(1023, int(abs(position - HOME.get(motor_name, 512)) / max(duration, 0.01) * 0.3)))
            set_position(motor_id, position, speed=speed)
            time.sleep(0.003)
        
        time.sleep(duration)
        
        # Check voltages mid-sequence every other keyframe
        if i % 2 == 0:
            for motor_name, motor_id in MOTOR_IDS.items():
                v = read_voltage(motor_id)
                err = read_hw_error(motor_id)
                if v is not None and v < 6.5:
                    msg = f"  LOW VOLTAGE on {motor_name} (ID {motor_id}): {v:.1f}V at keyframe {i+1}"
                    log(msg)
                    errors_during.append(msg)
                if err and err != 0:
                    msg = f"  HW ERROR on {motor_name} (ID {motor_id}) at keyframe {i+1}: 0x{err:02x}"
                    log(msg)
                    errors_during.append(msg)
    
    log("Voltages AFTER:")
    check_all_voltages()
    
    if errors_during:
        log(f"RESULT: FAILED — {len(errors_during)} error(s) detected")
    else:
        log("RESULT: PASSED — no errors detected")
    
    return len(errors_during) == 0

def main():
    global port_handler, packet_handler
    
    log(f"Motor Sequence Test — {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    log(f"Port: {PORT} | Baudrate: {BAUDRATE}")
    log(f"Sequences dir: {SEQUENCES_DIR}")
    
    # Connect
    port_handler = PortHandler(PORT)
    packet_handler = PacketHandler(PROTOCOL)
    
    if not port_handler.openPort():
        log("ERROR: Failed to open port")
        sys.exit(1)
    if not port_handler.setBaudRate(BAUDRATE):
        log("ERROR: Failed to set baudrate")
        sys.exit(1)
    
    log("Connected to motors")
    
    # Enable torque on all motors
    for name, motor_id in MOTOR_IDS.items():
        packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_EN, 1)
        packet_handler.write2ByteTxRx(port_handler, motor_id, ADDR_TORQUE_LIM, 400)
        time.sleep(0.05)
    
    log("\nInitial voltage check:")
    check_all_voltages()
    
    # Move to home
    go_home()
    
    # Find all sequence files
    seq_files = sorted([
        f for f in os.listdir(SEQUENCES_DIR) 
        if f.endswith('.yaml')
    ])
    
    log(f"\nFound {len(seq_files)} sequence files: {[f[:-5] for f in seq_files]}")
    
    results = {}
    
    for seq_file in seq_files:
        seq_name = seq_file[:-5]
        seq_path = os.path.join(SEQUENCES_DIR, seq_file)
        
        try:
            with open(seq_path) as f:
                data = yaml.safe_load(f)
        except Exception as e:
            log(f"ERROR loading {seq_file}: {e}")
            continue
        
        if seq_name not in data:
            # Try first key
            if data:
                seq_data = list(data.values())[0]
            else:
                log(f"SKIP {seq_name}: no data found")
                continue
        else:
            seq_data = data[seq_name]
        
        print(f"\nReady to test '{seq_name}'. Press ENTER to run, 's' to skip, 'q' to quit: ", end='')
        inp = input().strip().lower()
        
        if inp == 'q':
            break
        if inp == 's':
            log(f"SKIPPED: {seq_name}")
            results[seq_name] = 'skipped'
            continue
        
        passed = play_sequence(seq_name, seq_data)
        results[seq_name] = 'PASSED' if passed else 'FAILED'
        
        # Return home between sequences
        go_home()
        time.sleep(0.5)
    
    # Summary
    log(f"\n{'='*50}")
    log("TEST SUMMARY")
    log(f"{'='*50}")
    for name, result in results.items():
        log(f"  {name}: {result}")
    
    # Disable torque
    for name, motor_id in MOTOR_IDS.items():
        packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_EN, 0)
    
    port_handler.closePort()
    
    # Save log
    with open(LOG_FILE, 'w') as f:
        f.write('\n'.join(log_lines))
    log(f"\nLog saved to {LOG_FILE}")

if __name__ == '__main__':
    main()
