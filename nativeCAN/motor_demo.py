import can
import time
import struct
import math

# Configuration
CHANNEL = 'can0'
BITRATE = 1000000  # 1 Mbps
MOTOR_ID = 0x01    # Default ID, change if needed
DT = 0.01          # Loop period

# MIT Protocol Ranges (Estimated for RS02)
P_MIN = -12.57
P_MAX = 12.57
V_MIN = -30.0
V_MAX = 30.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -17.0
T_MAX = 17.0

def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    if x > x_max: x = x_max
    elif x < x_min: x = x_min
    return int((x - offset) * ((1 << bits) - 1) / span)

def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    return (float(x_int) * span / ((1 << bits) - 1)) + offset

def pack_cmd(p_des, v_des, kp, kd, t_ff):
    """
    Pack the command into 8 bytes using MIT protocol.
    """
    p_int = float_to_uint(p_des, P_MIN, P_MAX, 16)
    v_int = float_to_uint(v_des, V_MIN, V_MAX, 12)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12)

    # Packing (Standard MIT/Mini Cheetah format)
    # 0: p_int[15:8]
    # 1: p_int[7:0]
    # 2: v_int[11:4]
    # 3: v_int[3:0] | kp_int[11:8]
    # 4: kp_int[7:0]
    # 5: kd_int[11:4]
    # 6: kd_int[3:0] | t_int[11:8]
    # 7: t_int[7:0]
    
    data = [0] * 8
    data[0] = (p_int >> 8) & 0xFF
    data[1] = p_int & 0xFF
    data[2] = (v_int >> 4) & 0xFF
    data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF)
    data[4] = kp_int & 0xFF
    data[5] = (kd_int >> 4) & 0xFF
    data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF)
    data[7] = t_int & 0xFF
    
    return data

def unpack_reply(data):
    """
    Unpack the reply from the motor.
    """
    id = data[0]
    p_int = (data[1] << 8) | data[2]
    v_int = (data[3] << 4) | (data[4] >> 4)
    t_int = ((data[4] & 0xF) << 8) | data[5]
    
    p = uint_to_float(p_int, P_MIN, P_MAX, 16)
    v = uint_to_float(v_int, V_MIN, V_MAX, 12)
    t = uint_to_float(t_int, T_MIN, T_MAX, 12)
    
    return p, v, t

def enable_motor(bus, motor_id):
    """
    Send the Enable command.
    """
    print(f"Enabling motor {motor_id}...")
    # Common Enable command for MIT mode: 0xFF...0xFC
    msg = can.Message(arbitration_id=motor_id, data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC], is_extended_id=False)
    bus.send(msg)
    time.sleep(0.1)

def disable_motor(bus, motor_id):
    """
    Send the Disable command.
    """
    print(f"Disabling motor {motor_id}...")
    # Common Disable command for MIT mode: 0xFF...0xFD
    msg = can.Message(arbitration_id=motor_id, data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD], is_extended_id=False)
    bus.send(msg)
    time.sleep(0.1)

def zero_motor(bus, motor_id):
    """
    Set the current position as zero.
    """
    print(f"Zeroing motor {motor_id}...")
    # Common Zero command: 0xFF...0xFE
    msg = can.Message(arbitration_id=motor_id, data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE], is_extended_id=False)
    bus.send(msg)
    time.sleep(0.1)

def main():
    try:
        bus = can.interface.Bus(channel=CHANNEL, bustype='socketcan')
    except OSError:
        print(f"Error: Could not open {CHANNEL}. Make sure the interface is up.")
        print("Try: sudo ip link set can0 up type can bitrate 1000000")
        return

    print("Starting Motor Demo...")
    
    try:
        enable_motor(bus, MOTOR_ID)
        
        # Spin slowly
        # We use velocity control: p_des=0, v_des=target, kp=0, kd=1.0, t_ff=0
        target_vel = 2.0 # rad/s
        
        start_time = time.time()
        while time.time() - start_time < 5.0: # Run for 5 seconds
            data = pack_cmd(0.0, target_vel, 0.0, 1.0, 0.0)
            msg = can.Message(arbitration_id=MOTOR_ID, data=data, is_extended_id=False)
            bus.send(msg)
            
            # Try to read reply (non-blocking)
            msg = bus.recv(timeout=0.001)
            if msg:
                # print(f"Received: {msg.data.hex()}")
                pass
                
            time.sleep(DT)
            
        print("Stopping...")
        # Stop the motor
        data = pack_cmd(0.0, 0.0, 0.0, 1.0, 0.0)
        msg = can.Message(arbitration_id=MOTOR_ID, data=data, is_extended_id=False)
        bus.send(msg)
        time.sleep(0.5)
        
        disable_motor(bus, MOTOR_ID)
        
    except KeyboardInterrupt:
        print("\nInterrupted!")
        disable_motor(bus, MOTOR_ID)
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
