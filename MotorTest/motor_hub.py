#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RoboStride All-Inclusive Command Hub (RS-02 Edition)
----------------------------------------------------
A terminal-based tool for exploring motor functionality, testing commands,
and debugging the RoboStride RS-02 motor.

Usage: sudo python3 motor_hub.py [motor_id] [interface]
"""

import sys
import os
import time
import struct
import threading
import math
import signal

# --- 1. Environment Setup ---
# Add the SDK path (assuming running from python/ directory)
sys.path.append(os.path.join(os.path.dirname(__file__), 'robstride_dynamics'))
# Also try standard install location
sys.path.append('/usr/local/lib/python3/dist-packages')

try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
    import robstride_dynamics.table as table
except ImportError as e:
    print(f"❌ Critical Error: Could not import RobStride SDK.")
    print(f"   Details: {e}")
    print("   Please ensure you are running this from the 'python/' directory.")
    sys.exit(1)

# --- 2. RS-02 Parameter Patch (Crucial for correct physics) ---
RS02_PARAMS = {
    "rs-02": {
        "position": 12.57,  # 4 * PI
        "velocity": 44.0,
        "torque": 17.0,
        "kp": 500.0,
        "kd": 5.0
    }
}
# Apply patch to SDK tables
table.MODEL_MIT_POSITION_TABLE["rs-02"] = RS02_PARAMS["rs-02"]["position"]
table.MODEL_MIT_VELOCITY_TABLE["rs-02"] = RS02_PARAMS["rs-02"]["velocity"]
table.MODEL_MIT_TORQUE_TABLE["rs-02"]   = RS02_PARAMS["rs-02"]["torque"]
table.MODEL_MIT_KP_TABLE["rs-02"]       = RS02_PARAMS["rs-02"]["kp"]
table.MODEL_MIT_KD_TABLE["rs-02"]       = RS02_PARAMS["rs-02"]["kd"]

# --- 3. Helper: Parameter Map ---
# Create a dictionary of all available parameters from the SDK for easy menu selection
PARAM_MAP = {k: v for k, v in vars(ParameterType).items() if not k.startswith('__')}

class MotorHub:
    def __init__(self, motor_id=127, interface='can0'):
        self.motor_id = motor_id
        self.motor_name = f"motor_{motor_id}"
        self.interface = interface
        self.bus = None
        self.connected = False
        
    def connect(self):
        print(f"\n🔌 Connecting to Motor ID {self.motor_id} on {self.interface}...")
        motors = {self.motor_name: Motor(id=self.motor_id, model="rs-02")}
        try:
            self.bus = RobstrideBus(self.interface, motors, {})
            self.bus.connect(handshake=True)
            self.connected = True
            print("✅ Connected successfully.")
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            self.connected = False

    def disconnect(self):
        if self.bus:
            print("\n🔌 Disconnecting...")
            try:
                # Try to disable first for safety
                self.bus.disable(self.motor_name)
            except:
                pass
            self.bus.disconnect()
            self.connected = False

    def _check_connection(self):
        if not self.connected:
            print("⚠️ Error: Motor not connected. Please select 'Connect' first.")
            return False
        return True

    # --- COMMAND IMPLEMENTATIONS ---

    def cmd_enable(self):
        if not self._check_connection(): return
        print(f"Sending ENABLE command to ID {self.motor_id}...")
        self.bus.enable(self.motor_name)
        print("✅ Motor Enabled (Torque On)")

    def cmd_disable(self):
        if not self._check_connection(): return
        print(f"Sending DISABLE command to ID {self.motor_id}...")
        self.bus.disable(self.motor_name)
        print("✅ Motor Disabled (Torque Off)")

    def cmd_set_zero(self):
        if not self._check_connection(): return
        print(f"Sending SET_ZERO command...")
        print("⚠️  Warning: This redefines the current position as 0.0 rad.")
        confirm = input("   Continue? (y/n): ")
        if confirm.lower() == 'y':
            # Sending Command Type 6
            self.bus.transmit(CommunicationType.SET_ZERO_POSITION, self.bus.host_id, self.motor_id)
            print("✅ Zero position set.")

    def cmd_clear_faults(self):
        if not self._check_connection(): return
        print(f"Sending CLEAR_FAULTS command...")
        # Fault report/clear uses Type 21 (0x15) with extra data
        # Specifics depend on SDK, but usually writing 0xFF clears it
        # Alternatively, re-enabling often clears transient faults.
        # We will use a raw transmit to be sure.
        # 0x15 (Type 21) | 0xFF (Clear) << 8 | MotorID
        # NOTE: The SDK's bus.py might throw an error on fault frames, so raw handling is safer here.
        try:
            # Command 5 in manual: FF FF ... FB
            # Using SDK transmit:
            # Type 21 (FAULT_REPORT), Extra=0xFF (Clear command), ID=MotorID
            self.bus.transmit(CommunicationType.FAULT_REPORT, 0xFF, self.motor_id)
            print("✅ Fault clear command sent.")
        except Exception as e:
            print(f"⚠️ Error sending clear command: {e}")

    def cmd_save_config(self):
        if not self._check_connection(): return
        print(f"Sending SAVE_PARAMETERS command...")
        self.bus.transmit(CommunicationType.SAVE_PARAMETERS, self.bus.host_id, self.motor_id)
        print("✅ Parameters saved to non-volatile memory.")

    def cmd_read_parameter(self):
        if not self._check_connection(): return
        print("\n📖 --- READ PARAMETER ---")
        print("Available Parameters:")
        
        keys = list(PARAM_MAP.keys())
        for i, key in enumerate(keys):
            param_id, dtype, name = PARAM_MAP[key]
            print(f"  {i+1:2}. {key:<25} (ID: 0x{param_id:04X}, Type: {dtype.__name__})")
        
        try:
            idx = int(input("\nSelect parameter number to read (0 to cancel): ")) - 1
            if idx < 0: return
            
            param_key = keys[idx]
            param_tuple = PARAM_MAP[param_key]
            
            print(f"reading {param_key}...")
            val = self.bus.read(self.motor_name, param_tuple)
            print(f"\n✅ VALUE: {val}")
            
        except ValueError:
            print("❌ Invalid input.")
        except Exception as e:
            print(f"❌ Read Error: {e}")

    def cmd_write_parameter(self):
        if not self._check_connection(): return
        print("\n✍️ --- WRITE PARAMETER ---")
        print("⚠️  BE CAREFUL: Writing incorrect values can damage the motor.")
        
        keys = list(PARAM_MAP.keys())
        # Filter for writable? Protocol doesn't explicitly flag in python struct, 
        # but usually Status/Meas vars are read-only. User discretion advised.
        for i, key in enumerate(keys):
            param_id, dtype, name = PARAM_MAP[key]
            print(f"  {i+1:2}. {key:<25} (ID: 0x{param_id:04X})")
            
        try:
            idx = int(input("\nSelect parameter number to write (0 to cancel): ")) - 1
            if idx < 0: return
            
            param_key = keys[idx]
            param_tuple = PARAM_MAP[param_key]
            
            val_str = input(f"Enter new value for {param_key}: ")
            # Simple type conversion
            if 'float' in str(param_tuple[1]):
                val = float(val_str)
            else:
                val = int(val_str)
                
            print(f"Writing {val} to {param_key}...")
            self.bus.write(self.motor_name, param_tuple, val)
            print("✅ Write command sent.")
            
        except ValueError:
            print("❌ Invalid input format.")
        except Exception as e:
            print(f"❌ Write Error: {e}")

    def cmd_control_mit(self):
        if not self._check_connection(): return
        print("\n🎮 --- MIT CONTROL MODE ---")
        print("This mode gives you direct control over Pos, Vel, Kp, Kd, Torque.")
        print("Steps:")
        print("  1. Switching motor to MIT Mode (Mode 0)")
        print("  2. Entering interactive loop")
        print("  3. Type 'q' to exit loop")
        
        try:
            self.bus.write(self.motor_name, ParameterType.MODE, 0)
            time.sleep(0.2)
            self.bus.enable(self.motor_name)
            
            kp = 0.0
            kd = 0.0
            pos = 0.0
            
            print("\nReady. Commands:")
            print("  'k <val>'  -> Set Stiffness (Kp) (0-500) [Start low! e.g. 5]")
            print("  'd <val>'  -> Set Damping (Kd) (0-5)     [e.g. 0.5]")
            print("  'p <val>'  -> Set Position (deg)         [e.g. 90]")
            print("  'z'        -> Zero all gains (Limp)")
            
            while True:
                # Send frame
                self.bus.write_operation_frame(self.motor_name, pos, kp, kd, 0.0, 0.0)
                # Read feedback
                p_fb, v_fb, t_fb, temp = self.bus.read_operation_frame(self.motor_name)
                
                # Print Status
                status = f"\rStatus: Pos={math.degrees(p_fb):6.1f}° | Trq={t_fb:5.2f}Nm | Cmd: P={math.degrees(pos):.1f}° Kp={kp} Kd={kd}   "
                sys.stdout.write(status)
                sys.stdout.flush()
                
                # Non-blocking input would be ideal, but for simplicity in this hub
                # we will use blocking input. This means the motor "holds" last command 
                # while you type.
                # To keep the watchdog happy, we'll use a small trick: 
                # A background thread sending heartbeats could be used, 
                # but let's just use simple prompt for this educational tool.
                
                # Actually, for safety in MIT mode, we must keep sending. 
                # We will break the live update to ask for input.
                print("\n")
                user_in = input("Command > ").strip().lower()
                
                if user_in == 'q': break
                elif user_in == 'z': 
                    kp, kd = 0.0, 0.0
                    print("Gains zeroed.")
                elif user_in.startswith('k '):
                    try: kp = float(user_in.split()[1])
                    except: print("Bad value")
                elif user_in.startswith('d '):
                    try: kd = float(user_in.split()[1])
                    except: print("Bad value")
                elif user_in.startswith('p '):
                    try: pos = math.radians(float(user_in.split()[1]))
                    except: print("Bad value")
                
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"Error: {e}")
        
        # Cleanup
        try:
            self.bus.write_operation_frame(self.motor_name, 0, 0, 0, 0, 0)
            self.bus.disable(self.motor_name)
        except: pass
        print("\nExited MIT Mode.")

    def cmd_control_velocity(self):
        if not self._check_connection(): return
        print("\n🏎️ --- VELOCITY CONTROL MODE ---")
        
        try:
            print("Setting Mode 2 (Velocity)...")
            self.bus.write(self.motor_name, ParameterType.MODE, 2)
            
            # Safety Limits
            print("Setting Limits (Cur=5A, Acc=10)...")
            self.bus.write(self.motor_name, ParameterType.CURRENT_LIMIT, 5.0)
            self.bus.write(self.motor_name, ParameterType.VEL_ACCELERATION_TARGET, 10.0)
            self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, 0.0)
            
            self.bus.enable(self.motor_name)
            print("Motor Enabled.")
            
            while True:
                val = input("\nEnter Target Velocity (rad/s) or 'q' to quit: ")
                if val.lower() == 'q': break
                
                try:
                    vel = float(val)
                    self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, vel)
                    print(f"Velocity set to {vel} rad/s")
                except ValueError:
                    print("Invalid number.")
                    
        except Exception as e:
            print(f"Error: {e}")
            
        # Stop
        try:
            self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, 0.0)
            self.bus.disable(self.motor_name)
        except: pass
        print("Exited Velocity Mode.")

    # --- MENU SYSTEM ---

    def run(self):
        while True:
            print("\n" + "="*40)
            print("   🤖 ROBOSTRIDE MOTOR COMMAND HUB")
            print("="*40)
            conn_status = "🟢 ONLINE" if self.connected else "🔴 OFFLINE"
            print(f"Status: {conn_status} (Motor ID: {self.motor_id})")
            print("-"*40)
            print("1.  Connect to Motor")
            print("2.  Disconnect")
            print("--- BASIC CONTROLS ---")
            print("3.  Enable Motor")
            print("4.  Disable Motor")
            print("5.  Set Zero Position")
            print("6.  Clear Faults")
            print("--- PARAMETERS ---")
            print("7.  Read Parameter (All types)")
            print("8.  Write Parameter (Config/Limits)")
            print("9.  Save Configuration")
            print("--- MOTION MODES ---")
            print("10. MIT Control (Pos + Stiffness)")
            print("11. Velocity Control")
            print("--- SYSTEM ---")
            print("0.  Exit")
            print("-"*40)
            
            choice = input("Enter choice: ")
            
            if choice == '1': self.connect()
            elif choice == '2': self.disconnect()
            elif choice == '3': self.cmd_enable()
            elif choice == '4': self.cmd_disable()
            elif choice == '5': self.cmd_set_zero()
            elif choice == '6': self.cmd_clear_faults()
            elif choice == '7': self.cmd_read_parameter()
            elif choice == '8': self.cmd_write_parameter()
            elif choice == '9': self.cmd_save_config()
            elif choice == '10': self.cmd_control_mit()
            elif choice == '11': self.cmd_control_velocity()
            elif choice == '0': 
                self.disconnect()
                print("Goodbye!")
                sys.exit(0)
            else:
                print("Invalid selection.")
            
            if choice != '0':
                input("\nPress Enter to continue...")

def main():
    # Handle arguments: python motor_hub.py [ID] [Interface]
    mid = 127
    iface = 'can0'
    
    if len(sys.argv) > 1:
        try: mid = int(sys.argv[1])
        except: pass
    if len(sys.argv) > 2:
        iface = sys.argv[2]
        
    hub = MotorHub(mid, iface)
    
    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, lambda s, f: (hub.disconnect(), sys.exit(0)))
    
    hub.run()

if __name__ == "__main__":
    main()
