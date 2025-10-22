
#!/usr/bin/env python3
import argparse, sys, yaml, os
from pathlib import Path

DEVICE_TEMPLATES = {
    # Each template returns a list of rx_frame entries to add for this device.
    # Each entry: (key_name, spec_dict)
    'motor_rs02': lambda topic_prefix, id1, id2: [
        (f"RS02_Status1@{id1}", {
            'topic': f"{topic_prefix}/angle_deg",
            'type': 'std_msgs/msg/Float32',
            'fields': {'mech_angle_deg': 'data'},
            'frame_id': int(id1, 16)
        }),
        (f"RS02_Status1__vel@{id1}", {
            'topic': f"{topic_prefix}/velocity_rad_s",
            'type': 'std_msgs/msg/Float32',
            'fields': {'mech_velocity_rads': 'data'},
            'frame_id': int(id1, 16)
        }),
        (f"RS02_Status1__current@{id1}", {
            'topic': f"{topic_prefix}/phase_current_a",
            'type': 'std_msgs/msg/Float32',
            'fields': {'phase_current_A': 'data'},
            'frame_id': int(id1, 16)
        }),
        (f"RS02_Status1__busv@{id1}", {
            'topic': f"{topic_prefix}/bus_voltage_v",
            'type': 'std_msgs/msg/Float32',
            'fields': {'dc_bus_V': 'data'},
            'frame_id': int(id1, 16)
        }),
        (f"RS02_Status2__mtemp@{id2}", {
            'topic': f"{topic_prefix}/motor_temp_c",
            'type': 'std_msgs/msg/Float32',
            'fields': {'motor_temp_C': 'data'},
            'frame_id': int(id2, 16)
        }),
        (f"RS02_Status2__dtemp@{id2}", {
            'topic': f"{topic_prefix}/driver_temp_c",
            'type': 'std_msgs/msg/Float32',
            'fields': {'driver_temp_C': 'data'},
            'frame_id': int(id2, 16)
        }),
        (f"RS02_Status2__faults@{id2}", {
            'topic': f"{topic_prefix}/fault_bits",
            'type': 'std_msgs/msg/UInt32',
            'fields': {'fault_bits': 'data'},
            'frame_id': int(id2, 16)
        }),
        (f"RS02_Status2__status@{id2}", {
            'topic': f"{topic_prefix}/status_bits",
            'type': 'std_msgs/msg/UInt32',
            'fields': {'status_bits': 'data'},
            'frame_id': int(id2, 16)
        }),
    ],
    # Simple one-frame devices, customize as needed:
    'foot_sensor': lambda topic_prefix, id_hex: [
        (f"FootForce@{id_hex}", {
            'topic': f"{topic_prefix}/force_n",
            'type': 'std_msgs/msg/Float32',
            'fields': {'forceN': 'data'},
            'frame_id': int(id_hex, 16)
        })
    ],
    'imu': lambda topic_prefix, id_hex: [
        (f"IMU_Data@{id_hex}", {
            'topic': f"{topic_prefix}/temp_c",
            'type': 'std_msgs/msg/Float32',
            'fields': {'temp_C': 'data'},
            'frame_id': int(id_hex, 16)
        })
    ],
    'pdb': lambda topic_prefix, id_hex: [
        (f"PDB_Status@{id_hex}", {
            'topic': f"{topic_prefix}/bus_voltage_v",
            'type': 'std_msgs/msg/Float32',
            'fields': {'bus_V': 'data'},
            'frame_id': int(id_hex, 16)
        })
    ],
}

def pick_bus(buses, interface):
    for b in buses:
        if b.get('interface') == interface or b.get('name') == interface:
            return b
    raise SystemExit(f"No bus matches interface/name: {interface}. Available: {[b.get('interface') for b in buses]}")

def main():
    ap = argparse.ArgumentParser(description='Register a CAN node in td_can_bridges config by prompting for ID and device type.')
    ap.add_argument('--config', default=str(Path(__file__).resolve().parents[1] / 'config' / 'example_multibus.yaml'),
                    help='Path to td_can_bridges YAML config to modify.')
    args = ap.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.exists():
        sys.exit(f"Config not found: {cfg_path}")

    cfg = yaml.safe_load(cfg_path.read_text())
    buses = cfg.get('buses', [])
    if not buses:
        sys.exit("No 'buses' in config.")

    print("=== td_can_register ===")
    print("Available buses:")
    for b in buses:
        print(f" - name={b.get('name')} interface={b.get('interface')}")
    interface = input("Enter bus name or interface to register on (e.g., 'motor_bus' or 'can0'): ").strip()
    bus = None
    for b in buses:
        if b.get('name') == interface or b.get('interface') == interface:
            bus = b
            break
    if bus is None:
        sys.exit("No matching bus found.")

    dtype = input("Device type [motor_rs02 | foot_sensor | imu | pdb]: ").strip().lower()
    if dtype not in DEVICE_TEMPLATES:
        sys.exit(f"Unsupported device type: {dtype}")

    topic_prefix = input("Topic prefix (e.g., /td/rs02/1): ").strip() or "/td/device"

    if dtype == 'motor_rs02':
        id1 = input("Enter CAN ID (hex) for RS02_Status1 (e.g., 0x210): ").strip()
        id2 = input("Enter CAN ID (hex) for RS02_Status2 (e.g., 0x211): ").strip()
        entries = DEVICE_TEMPLATES[dtype](topic_prefix, id1, id2)
    else:
        idx = input("Enter CAN ID (hex) for device (e.g., 0x300): ").strip()
        entries = DEVICE_TEMPLATES[dtype](topic_prefix, idx)

    rx_frames = bus.setdefault('rx_frames', {})
    for key, spec in entries:
        # Key names must reference existing DBC message names used in mapping;
        # the key itself is just a label, but we will store a field 'dbc_msg' for clarity.
        # For compatibility with our parser, the key should be the DBC message name or any label.
        # We'll ensure DBC messages exist in motors.dbc / sensors.dbc.
        # Here we embed 'dbc_message' so future tooling can read it (not required by runtime).
        if 'RS02_Status1' in key:
            spec['dbc_message'] = 'RS02_Status1'
        elif 'RS02_Status2' in key:
            spec['dbc_message'] = 'RS02_Status2'
        elif 'FootForce' in key:
            spec['dbc_message'] = 'FootForce'
        elif 'IMU_Data' in key:
            spec['dbc_message'] = 'IMU_Data'
        elif 'PDB_Status' in key:
            spec['dbc_message'] = 'PDB_Status'
        rx_frames[key] = spec

    cfg_path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    print(f"Updated {cfg_path}")
    print("Added rx_frames entries:")
    for k,_ in entries:
        print(" -", k)

if __name__ == '__main__':
    main()
