"""Logs distance readings from a LightWare SF30/D LiDAR over USB.

Reads plain-text distance values (one per line, in meters) from the
sensor's USB serial output and writes them to a timestamped CSV file.

Wiring:
    Connect the SF30/D to the Raspberry Pi using a micro-USB cable,
    plugged into any of the Pi's USB ports. No separate power supply
    or GPIO wiring is needed; the sensor is powered via USB.

Before running, set the sensor's "Output type (legacy)" parameter to
"Distance over USB" using LightWare Studio.

Find the correct serial port by running, before and after plugging in
the sensor, and noting which new device appears:
    ls /dev/tty*
It is usually /dev/ttyACM0 or /dev/ttyUSB0.

Usage:
    python3 lidar_logger.py
"""

import csv
import datetime
import time

import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
OUTPUT_CSV = "lidar_log.csv"


def main():
    """Open the serial port and continuously log distance readings to CSV."""
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser, open(
        OUTPUT_CSV, mode="w", newline=""
    ) as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["timestamp_utc", "timestamp_unix", "distance_m"])

        print(f"Loggar till {OUTPUT_CSV}. Avbryt med Ctrl+C.")

        try:
            while True:
                raw_line = ser.readline()
                if not raw_line:
                    continue

                # Sensor lines look like "1.00 m" or "1.00m" — strip the unit
                # suffix and any whitespace before parsing the number.
                text = raw_line.decode("utf-8", errors="ignore").strip()
                text = text.replace("m", "").strip()

                try:
                    distance_m = float(text)
                except ValueError:
                    # Ignore lines that aren't valid numbers (e.g. startup noise)
                    continue

                now = datetime.datetime.now(datetime.timezone.utc)
                writer.writerow([now.isoformat(), time.time(), distance_m])
                csv_file.flush()
                print(f"{now.isoformat()}  {distance_m:.2f} m")

        except KeyboardInterrupt:
            print("\nLoggning avbruten av användaren.")


if __name__ == "__main__":
    main()