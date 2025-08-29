import random
import struct

# --- Configuration ---
REMOTE_FILE = "example/remote_control_sample.txt"
LIDAR_FILE = "example/lidar_sample.txt"
GPS_FILE = "example/gps_sample.txt"

# --- I-Bus Packet Generation ---
IBUS_HEADER = b'\x20\x40'

def create_ibus_packet(channels):
    """Helper to create a valid I-Bus packet from 14 channel values."""
    if len(channels) != 14:
        raise ValueError("Channel list must contain 14 values.")
    packet = IBUS_HEADER
    packet += struct.pack('<' + 'H' * 14, *channels)
    checksum = 0xFFFF - sum(packet)
    packet += struct.pack('<H', checksum)
    return packet

def generate_remote_data(num_packets=50):
    """Generates a sequence of I-Bus packets."""
    packets = []
    ch = [1500] * 14 # Start with centered sticks

    # Sequence of events
    events = [
        {'ch': 1, 'val': 2000, 'dur': 5},  # Full throttle
        {'ch': 0, 'val': 1000, 'dur': 5},  # Full left steer
        {'ch': 0, 'val': 1500, 'dur': 3},  # Center steer
        {'ch': 1, 'val': 1000, 'dur': 5},  # Full reverse
        {'ch': 4, 'val': 2000, 'dur': 2},  # Switch to auto mode
        {'ch': 4, 'val': 1000, 'dur': 2},  # Switch back to manual
        {'ch': 1, 'val': 1500, 'dur': 5},  # Stop
    ]

    for event in events:
        for _ in range(event['dur']):
            ch[event['ch']] = event['val']
            packets.append(create_ibus_packet(ch))

    # Fill remaining with random small movements
    while len(packets) < num_packets:
        ch[0] = random.randint(1400, 1600)
        ch[1] = random.randint(1400, 1600)
        packets.append(create_ibus_packet(ch))

    with open(REMOTE_FILE, 'wb') as f: # Write in binary mode
        for packet in packets:
            f.write(packet)
    print(f"Generated {len(packets)} I-Bus packets in {REMOTE_FILE}")


def generate_lidar_data(num_readings=100):
    """Generates simulated Lidar distance readings."""
    with open(LIDAR_FILE, 'w') as f:
        for i in range(num_readings):
            angle = (i * 3.6) % 360  # Simulate a 360-degree scan
            distance = random.uniform(1.0, 8.0)
            # Add some noise/outliers
            if random.random() < 0.05:
                distance = random.uniform(10.0, 20.0)
            f.write(f"Angle: {angle:.1f}, Distance: {distance:.2f}m\n")
    print(f"Generated {num_readings} Lidar readings in {LIDAR_FILE}")

def generate_gps_data(num_sentences=50):
    """Generates simulated GPS NMEA sentences."""
    lat_start, lon_start = 13.7563, 100.5018  # Bangkok
    with open(GPS_FILE, 'w') as f:
        for i in range(num_sentences):
            # Simulate slight movement
            lat = lat_start + (random.random() - 0.5) * 0.001
            lon = lon_start + (random.random() - 0.5) * 0.001

            # Alternate between GPGGA and GPRMC sentences
            if i % 2 == 0:
                # GPGGA: Global Positioning System Fix Data
                sentence = f"$GPGGA,{180000 + i:06d}.00,{abs(lat)*100:.4f},{'N' if lat>0 else 'S'},{abs(lon)*100:.4f},{'E' if lon>0 else 'W'},1,08,0.9,15.0,M,-2.0,M,,*47"
            else:
                # GPRMC: Recommended Minimum Specific GNSS Data
                sentence = f"$GPRMC,{180000 + i:06d}.00,A,{abs(lat)*100:.4f},{'N' if lat>0 else 'S'},{abs(lon)*100:.4f},{'E' if lon>0 else 'W'},0.1,180.0,260824,,,A*6A"
            f.write(f"{sentence}\n")
    print(f"Generated {num_sentences} GPS sentences in {GPS_FILE}")


if __name__ == "__main__":
    print("--- Generating Sample Data ---")
    generate_remote_data()
    generate_lidar_data()
    generate_gps_data()
    print("--- Sample Data Generation Complete ---")
