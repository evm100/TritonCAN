import sys
import time
import math
import signal

# Adjust path to find the SDK
sys.path.append('./seeed-projects/robstride_control/RobStride_Control-e0a01d38335972c2fa1dd1e15bb222c4e15e866b/python')
from robstride_dynamics import RobstrideBus, Motor, ParameterType

# --- CONFIGURATION ---
MOTOR_ID = 127          # Target Motor ID
INTERFACE = 'can0'      # CAN Interface
SWING_FREQ = 0.5        # Frequency in Hz (0.5 = 2 seconds per full swing)
SWING_AMP_DEG = 90.0    # Amplitude in degrees (Swings +/- 90 deg)

# RS02 Motor Limits (Crucial for correct scaling)
# These values define how the 16-bit CAN integers map to real-world floats
RS02_PARAMS = {
    "rs-02": {
        "position": 12.57,  # 4 * PI
        "velocity": 44.0,
        "torque": 17.0,
        "kp": 500.0,
        "kd": 5.0
    }
}

# Monkey-patch the table in the SDK if RS-02 isn't defined correctly there
import robstride_dynamics.table as table
table.MODEL_MIT_POSITION_TABLE["rs-02"] = RS02_PARAMS["rs-02"]["position"]
table.MODEL_MIT_VELOCITY_TABLE["rs-02"] = RS02_PARAMS["rs-02"]["velocity"]
table.MODEL_MIT_TORQUE_TABLE["rs-02"]   = RS02_PARAMS["rs-02"]["torque"]
table.MODEL_MIT_KP_TABLE["rs-02"]       = RS02_PARAMS["rs-02"]["kp"]
table.MODEL_MIT_KD_TABLE["rs-02"]       = RS02_PARAMS["rs-02"]["kd"]

def main():
    print(f"🤖 Connecting to Motor {MOTOR_ID} on {INTERFACE}...")
    
    # Define Motor
    motor_name = f"motor_{MOTOR_ID}"
    motors = {motor_name: Motor(id=MOTOR_ID, model="rs-02")}
    
    # Initialize Bus
    bus = RobstrideBus(INTERFACE, motors, {})
    bus.connect(handshake=True)

    try:
        # 1. Setup: Enable and Switch to MIT Mode
        print("⚡ Enabling and switching to MIT Mode (Mode 0)...")
        bus.enable(motor_name)
        bus.write(motor_name, ParameterType.MODE, 0) 
        time.sleep(0.5)

        # 2. Set Gains (Stiffness & Damping)
        # Kp=40 is stiff enough to move, but soft enough to be safe
        kp = 40.0 
        kd = 1.5
        
        print(f"\n🌊 Starting Swing: ±{SWING_AMP_DEG}° at {SWING_FREQ}Hz")
        print("   Press Ctrl+C to stop safely.\n")
        print(f"{'TIME':<8} | {'POS (deg)':<10} | {'VEL (rad/s)':<12} | {'TRQ (Nm)':<10} | {'TEMP':<6}")
        print("-" * 60)

        start_time = time.time()
        
        while True:
            # --- A. Calculate Trajectory ---
            t = time.time() - start_time
            
            # Sine Wave: Amp * sin(2 * pi * freq * t)
            # Result is in Radians
            target_pos_rad = math.radians(SWING_AMP_DEG) * math.sin(2 * math.pi * SWING_FREQ * t)
            
            # Calculate feed-forward velocity (derivative of position) for smoother motion
            # v = d/dt (A sin(wt)) = A * w * cos(wt)
            target_vel_rads = math.radians(SWING_AMP_DEG) * (2 * math.pi * SWING_FREQ) * math.cos(2 * math.pi * SWING_FREQ * t)

            # --- B. Send Command (Write) ---
            # In MIT mode, we send Target Pos, Vel, Kp, Kd, and Feed-Forward Torque
            bus.write_operation_frame(
                motor_name,
                target_pos_rad, 
                kp, 
                kd, 
                target_vel_rads, 
                0.0 # No extra feed-forward torque
            )

            # --- C. Read Telemetry (Read) ---
            # The motor replies to every write with a status frame
            p_act, v_act, t_act, temp = bus.read_operation_frame(motor_name)
            
            # --- D. Print Status ---
            # Convert act pos to degrees for readability
            p_deg = math.degrees(p_act)
            
            # Print overwriting the same line (or new line if you prefer logs)
            # Using \r to keep the terminal clean, remove it to log history
            sys.stdout.write(f"\r{t:6.2f}s | {p_deg:8.1f}°  | {v_act:10.2f}   | {t_act:8.2f}   | {temp:4.1f}°C")
            sys.stdout.flush()

            # Loop at ~50Hz (20ms)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\n🛑 Stopping...")
    
    except Exception as e:
        print(f"\n❌ Error: {e}")

    finally:
        # Safe Shutdown Sequence
        try:
            # 1. Command Zero Position with Zero Gains (Limp)
            bus.write_operation_frame(motor_name, 0, 0, 0, 0, 0)
            time.sleep(0.1)
            # 2. Disable Motor
            bus.disable(motor_name)
        except:
            pass
        bus.disconnect()
        print("👋 Motor Disabled.")

if __name__ == "__main__":
    main()
