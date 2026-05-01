#!/usr/bin/env python3
"""
Incremental motor test — moves motors one at a time to isolate the fault.
"""
import sys
import time
import datetime

try:
    from dynamixel_sdk import *
except ImportError:
    print("ERROR: dynamixel_sdk not available")
    sys.exit(1)

PORT     = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL = 2.0

ADDR_GOAL_POS   = 30
ADDR_MOVING_SPD = 32
ADDR_TORQUE_EN  = 24
ADDR_TORQUE_LIM = 35
ADDR_VOLTAGE    = 45
ADDR_HW_ERROR   = 50
ADDR_SHUTDOWN   = 18

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

# Test positions — moderate movement from home
TEST_POS = {
    'lazy_susan':       624,   # ~110 units right
    'motor_front':      200,
    'motor_back_left':  100,
    'motor_back_right': 200,
}

log_lines = []

def log(msg):
    ts = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
    line = f"[{ts}] {msg}"
    print(line)
    log_lines.append(line)

port_handler   = None
packet_handler = None

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

def read_shutdown(motor_id):
    val, result, _ = packet_handler.read1ByteTxRx(port_handler, motor_id, ADDR_SHUTDOWN)
    if result != COMM_SUCCESS:
        return None
    return val

def set_pos(motor_id, position, speed=200):
    packet_handler.write2ByteTxRx(port_handler, motor_id, ADDR_MOVING_SPD, speed)
    packet_handler.write2ByteTxRx(port_handler, motor_id, ADDR_GOAL_POS, int(position))

def status_all(label):
    log(f"  [{label}]")
    for name, mid in MOTOR_IDS.items():
        v   = read_voltage(mid)
        err = read_hw_error(mid)
        v_s = f"{v:.1f}V" if v is not None else "N/A"
        e_s = f"0x{err:02x}" if err is not None else "N/A"
        ok  = "OK" if err == 0 else "ERROR"
        log(f"    {name} (ID {mid}): {v_s} | hw_error={e_s} | {ok}")

def go_home():
    log("  -> Moving to home...")
    for name, mid in MOTOR_IDS.items():
        set_pos(mid, HOME[name], speed=150)
        time.sleep(0.05)
    time.sleep(2.0)

def run_test(test_name, motors_to_move):
    log(f"\n{'='*50}")
    log(f"TEST: {test_name}")
    log(f"Moving: {list(motors_to_move.keys())}")
    log(f"{'='*50}")
    
    go_home()
    status_all("BEFORE")
    
    log("  -> Sending commands...")
    for name, pos in motors_to_move.items():
        mid = MOTOR_IDS[name]
        set_pos(mid, pos, speed=200)
        time.sleep(0.003)
    
    # Sample every 100ms for 2 seconds
    errors = []
    for i in range(20):
        time.sleep(0.1)
        for name, mid in MOTOR_IDS.items():
            err = read_hw_error(mid)
            if err and err != 0:
                v = read_voltage(mid)
                msg = f"    ERROR {name} (ID {mid}): hw=0x{err:02x} v={v:.1f}V at t={i*100}ms"
                log(msg)
                errors.append(msg)
    
    status_all("AFTER")
    
    result = "PASSED" if not errors else f"FAILED ({len(errors)} errors)"
    log(f"  RESULT: {result}")
    return len(errors) == 0

def main():
    global port_handler, packet_handler
    
    log(f"Incremental Motor Test — {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    port_handler   = PortHandler(PORT)
    packet_handler = PacketHandler(PROTOCOL)
    
    if not port_handler.openPort():
        log("ERROR: Failed to open port"); sys.exit(1)
    if not port_handler.setBaudRate(BAUDRATE):
        log("ERROR: Failed to set baudrate"); sys.exit(1)
    
    log("Connected")
    
    # Enable torque, disable voltage shutdown on all motors
    log("\nInitializing motors...")
    for name, mid in MOTOR_IDS.items():
        # Read and clear voltage shutdown bit
        shutdown, result, _ = packet_handler.read1ByteTxRx(port_handler, mid, ADDR_SHUTDOWN)
        if result == COMM_SUCCESS:
            log(f"  {name} shutdown reg: 0x{shutdown:02x}")
            new_shutdown = shutdown & ~0x01  # clear voltage bit
            packet_handler.write1ByteTxRx(port_handler, mid, ADDR_SHUTDOWN, new_shutdown)
        packet_handler.write1ByteTxRx(port_handler, mid, ADDR_TORQUE_EN, 1)
        packet_handler.write2ByteTxRx(port_handler, mid, ADDR_TORQUE_LIM, 400)
        time.sleep(0.05)
    
    log("\nInitial status:")
    status_all("INIT")
    
    input("\nPress ENTER to start incremental tests...")
    
    results = {}
    
    # Test 1: lazy_susan alone
    input("\nPress ENTER: Test lazy_susan alone")
    results['1_lazy_susan_only'] = run_test(
        "lazy_susan alone",
        {'lazy_susan': TEST_POS['lazy_susan']}
    )
    
    # Test 2: motor_front alone
    input("\nPress ENTER: Test motor_front alone")
    results['2_motor_front_only'] = run_test(
        "motor_front alone",
        {'motor_front': TEST_POS['motor_front']}
    )
    
    # Test 3: motor_back_left alone
    input("\nPress ENTER: Test motor_back_left alone")
    results['3_motor_back_left_only'] = run_test(
        "motor_back_left alone",
        {'motor_back_left': TEST_POS['motor_back_left']}
    )
    
    # Test 4: motor_back_right alone
    input("\nPress ENTER: Test motor_back_right alone")
    results['4_motor_back_right_only'] = run_test(
        "motor_back_right alone",
        {'motor_back_right': TEST_POS['motor_back_right']}
    )
    
    # Test 5: lazy_susan + motor_front
    input("\nPress ENTER: Test lazy_susan + motor_front")
    results['5_lazy_susan_front'] = run_test(
        "lazy_susan + motor_front",
        {'lazy_susan': TEST_POS['lazy_susan'],
         'motor_front': TEST_POS['motor_front']}
    )
    
    # Test 6: all motors
    input("\nPress ENTER: Test ALL motors")
    results['6_all_motors'] = run_test(
        "ALL motors",
        TEST_POS
    )
    
    # Test 7: lazy_susan left vs right
    input("\nPress ENTER: Test lazy_susan LEFT (400)")
    results['7_lazy_susan_left'] = run_test(
        "lazy_susan LEFT (400)",
        {'lazy_susan': 400}
    )
    
    input("\nPress ENTER: Test lazy_susan RIGHT (624)")
    results['8_lazy_susan_right'] = run_test(
        "lazy_susan RIGHT (624)",
        {'lazy_susan': 624}
    )
    
    # Summary
    log(f"\n{'='*50}")
    log("FINAL SUMMARY")
    log(f"{'='*50}")
    for name, passed in results.items():
        log(f"  {name}: {'PASSED' if passed else 'FAILED'}")
    
    # Disable torque
    for name, mid in MOTOR_IDS.items():
        packet_handler.write1ByteTxRx(port_handler, mid, ADDR_TORQUE_EN, 0)
    port_handler.closePort()
    
    with open('/home/max/incremental_test_log.txt', 'w') as f:
        f.write('\n'.join(log_lines))
    log("Log saved to ~/incremental_test_log.txt")

if __name__ == '__main__':
    main()
