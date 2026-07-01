"""Logs distance readings from a LightWare SF30/D LiDAR over USB.

Uses the binary protocol (Full communication mode) to configure the
sensor programmatically and stream distance data to a CSV file.

Setup:
    In LightWare Studio, set "Output type (legacy)" to
    "Full communication mode" before running this script.

Wiring:
    Connect the SF30/D to the Raspberry Pi using a micro-USB cable.
    No separate power supply or GPIO wiring is needed.

Usage:
    python3 lidar_logger.py
"""

import csv
import datetime
import struct
import time

import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
OUTPUT_CSV = "lidar_log.csv"

# Command IDs (from manual section 10.1.6)
CMD_PRODUCT_NAME = 0
CMD_STREAM = 30
CMD_DISTANCE_DATA = 44
CMD_RETURN_MODE = 77

# Stream values
STREAM_DISABLED = 0
STREAM_DISTANCE = 5  # streams command 44 (distance data in cm)

# Return mode values
RETURN_FIRST = 0
RETURN_LAST = 1


def compute_crc(data: bytes) -> int:
    """Compute CRC-16-CCITT checksum (as specified in manual section 10.1.2)."""
    crc = 0
    for byte in data:
        code = (crc >> 8) & 0xFF
        code ^= byte & 0xFF
        code ^= code >> 4
        crc = (crc << 8) & 0xFFFF
        crc ^= code
        code = (code << 5) & 0xFFFF
        crc ^= code
        code = (code << 7) & 0xFFFF
        crc ^= code
    return crc


def build_packet(command_id: int, data: bytes = b"", write: bool = False) -> bytes:
    """Build a binary protocol request packet.

    Packet structure: start | flags_low | flags_high | id | data | crc_low | crc_high
    """
    payload = bytes([command_id | (1 if write else 0)]) + data
    payload_length = len(payload)

    # Flags: payload length in upper bits, write bit in bit 0 of flags_low
    flags = payload_length << 6
    flags_low = flags & 0xFF
    flags_high = (flags >> 8) & 0xFF

    header_and_payload = bytes([0xAA, flags_low, flags_high]) + payload
    crc = compute_crc(header_and_payload)
    crc_low = crc & 0xFF
    crc_high = (crc >> 8) & 0xFF

    return header_and_payload + bytes([crc_low, crc_high])


def read_packet(ser: serial.Serial) -> tuple[int, bytes] | None:
    """Read and validate one binary protocol packet from the serial port.

    Returns (command_id, data) on success, or None on failure/timeout.
    """
    # Wait for start byte 0xAA
    while True:
        byte = ser.read(1)
        if not byte:
            return None
        if byte[0] == 0xAA:
            break

    # Read 2-byte flags
    flags_bytes = ser.read(2)
    if len(flags_bytes) < 2:
        return None

    flags = flags_bytes[0] | (flags_bytes[1] << 8)
    payload_length = flags >> 6

    if payload_length < 1 or payload_length > 1023:
        return None

    # Read payload
    payload = ser.read(payload_length)
    if len(payload) < payload_length:
        return None

    # Read 2-byte CRC
    crc_bytes = ser.read(2)
    if len(crc_bytes) < 2:
        return None

    received_crc = crc_bytes[0] | (crc_bytes[1] << 8)

    # Validate CRC
    packet_without_crc = bytes([0xAA]) + flags_bytes + payload
    expected_crc = compute_crc(packet_without_crc)
    if received_crc != expected_crc:
        return None

    command_id = payload[0] >> 1  # strip write bit
    data = payload[1:]
    return command_id, data


def send_read_command(ser: serial.Serial, command_id: int) -> tuple[int, bytes] | None:
    """Send a read request and return the response packet, or None on timeout."""
    packet = build_packet(command_id, write=False)
    ser.write(packet)
    return read_packet(ser)


def send_write_command(ser: serial.Serial, command_id: int, data: bytes) -> bool:
    """Send a write request and confirm the response matches."""
    packet = build_packet(command_id, data=data, write=True)
    ser.write(packet)
    response = read_packet(ser)
    if response is None:
        return False
    return response[0] == command_id


def initialize(ser: serial.Serial) -> bool:
    """Initialize sensor communication per manual section 10.

    The first command after powerup is ignored by the sensor (used to
    detect interface type), so we send Product name twice and expect
    a response only on the second attempt.
    """
    # First request — no response expected
    ser.write(build_packet(CMD_PRODUCT_NAME, write=False))
    time.sleep(0.1)
    ser.reset_input_buffer()

    # Second request — should return "SF30..."
    response = send_read_command(ser, CMD_PRODUCT_NAME)
    if response is None:
        return False

    product_name = response[1].decode("ascii", errors="ignore").rstrip("\x00")
    print(f"Sensor identifierad: {product_name}")
    return True


def main():
    """Configure the sensor and log distance readings to CSV."""
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, open(
        OUTPUT_CSV, mode="w", newline=""
    ) as csv_file:

        ser.reset_input_buffer()

        print("Initierar sensor...")
        if not initialize(ser):
            print("Fel: kunde inte kommunicera med sensorn. Kontrollera anslutning och att sensorn är i Full communication mode.")
            return

        # Set return mode to First return
        print("Sätter Return mode: Last return...")
        if not send_write_command(ser, CMD_RETURN_MODE, bytes([RETURN_FIRST])):
            print("Varning: kunde inte sätta Return mode.")

        # Start streaming distance data
        print("Startar streaming...")
        stream_value = struct.pack("<I", STREAM_DISTANCE)
        if not send_write_command(ser, CMD_STREAM, stream_value):
            print("Varning: kunde inte starta streaming.")

        writer = csv.writer(csv_file)
        writer.writerow(["timestamp_utc", "timestamp_unix", "distance_m"])
        print(f"Loggar till {OUTPUT_CSV}. Avbryt med Ctrl+C.")

        try:
            while True:
                response = read_packet(ser)
                if response is None:
                    continue

                cmd_id, data = response
                if cmd_id != CMD_DISTANCE_DATA:
                    continue
                if len(data) < 2:
                    continue

                # First two bytes = first (or last, per return mode) return raw, in cm
                distance_cm = struct.unpack_from("<h", data, 0)[0]
                if distance_cm <= 0:
                    continue  # out of range or invalid

                distance_m = distance_cm / 100.0
                now = datetime.datetime.now(datetime.timezone.utc)
                writer.writerow([now.isoformat(), time.time(), distance_m])
                csv_file.flush()
                print(f"{now.isoformat()}  {distance_m:.2f} m")

        except KeyboardInterrupt:
            # Stop streaming before exiting
            stream_off = struct.pack("<I", STREAM_DISABLED)
            send_write_command(ser, CMD_STREAM, stream_off)
            print("\nLoggning avbruten av användaren.")


if __name__ == "__main__":
    main()