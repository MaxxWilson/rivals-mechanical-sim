import serial
import time
import numpy as np
import argparse
import json
import os

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
SAMPLE_SIZE = 25 # Keep this high for precise calibration
CALIBRATION_FILE = 'calibration.json'

def get_stable_reading(ser, samples=SAMPLE_SIZE):
    """Reads multiple samples and returns the average."""
    ser.reset_input_buffer()
    values = []
    while len(values) < samples:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                values.append(int(line))
            except (ValueError, UnicodeDecodeError):
                continue
    return np.mean(values)

def load_calibration():
    """Loads calibration data from a JSON file, returning defaults if not found."""
    if not os.path.exists(CALIBRATION_FILE):
        return 1.0, 0.0
    try:
        with open(CALIBRATION_FILE, 'r') as f:
            data = json.load(f)
        return data.get('calibration_factor', 1.0), data.get('raw_offset', 0.0)
    except (json.JSONDecodeError, KeyError):
        print("Warning: Could not read calibration.json. Using default values.")
        return 1.0, 0.0

def save_calibration(factor=None, offset=None):
    """Saves updated calibration data, preserving existing values."""
    existing_factor, existing_offset = load_calibration()
    data = {
        'calibration_factor': factor if factor is not None else existing_factor,
        'raw_offset': offset if offset is not None else existing_offset
    }
    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(data, f, indent=4)
    print(f"Calibration data updated in {CALIBRATION_FILE}")

def calibrate_offset(ser):
    """Measures the zero-point and saves it as the raw offset."""
    print("\n--- Calibrate Offset (Tare & Save) ---")
    input("Ensure scale is completely empty, then press Enter to capture zero point...")
    print(f"Capturing {SAMPLE_SIZE} samples for zero offset...")
    new_offset = get_stable_reading(ser, SAMPLE_SIZE)
    save_calibration(offset=new_offset)
    print(f"\nNew raw offset saved: {new_offset:.2f}")

def calibrate_scale(ser):
    """Performs multi-point calibration to find the scale factor (slope)."""
    print("\n--- Calibrate Scale Factor ---")
    print("Use at least two different known weights for best results.")
    calibration_points = []

    while True:
        try:
            known_weight_str = input("\nEnter known weight in grams (or 'c' to finish): ").strip()
            if known_weight_str.lower() == 'c':
                break
            known_weight_g = float(known_weight_str)
            input(f"Place {known_weight_g}g on the scale and press Enter to capture...")
            
            print(f"Capturing {SAMPLE_SIZE} samples...")
            avg_raw_value = get_stable_reading(ser, SAMPLE_SIZE)
            calibration_points.append((known_weight_g, avg_raw_value))
            print(f"  > Captured: Weight={known_weight_g}g, Raw Value={avg_raw_value:.2f}")
        except ValueError:
            print("Invalid input. Please enter a number or 'c'.")

    if len(calibration_points) < 2:
        print("\nScale calibration requires at least two data points. Exiting.")
        return

    known_weights = np.array([p[0] for p in calibration_points])
    raw_values = np.array([p[1] for p in calibration_points])
    
    slope, _ = np.polyfit(known_weights, raw_values, 1)

    print("\n--- Scale Calibration Complete ---")
    save_calibration(factor=slope)
    print(f"New calibration factor (slope) saved: {slope:.2f}")

def run_scale(ser):
    """Main measurement loop. Outputs in grams using saved calibration."""
    print("Loading calibration...")
    cal_factor, raw_offset = load_calibration()

    if cal_factor == 1.0:
        print("Warning: Scale is not calibrated. Please run --calibrate-scale first.")
        return
    
    print("\nStreaming weight in grams... (Press Ctrl+C to stop)")
    while True:
        # --- THIS LINE IS THE FIX ---
        # We only want 1 sample for live mode to get max refresh rate.
        raw_value = get_stable_reading(ser, samples=1)
        mass_g = (raw_value - raw_offset) / cal_factor
        print(f"Weight: {mass_g: 8.2f} g", end='\r')

def main():
    parser = argparse.ArgumentParser(description="A tool to calibrate and read from an HX711 load cell via a Raspberry Pi Pico.")
    parser.add_argument('--calibrate-offset', action='store_true', help="Set the permanent zero-point (tare).")
    parser.add_argument('--calibrate-scale', action='store_true', help="Calibrate the scale factor using known weights.")
    args = parser.parse_args()

    print(f"Connecting to serial port {SERIAL_PORT}...")
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2) as ser:
            time.sleep(2)
            if args.calibrate_scale:
                calibrate_scale(ser)
            elif args.calibrate_offset:
                calibrate_offset(ser)
            else:
                run_scale(ser)
    except serial.SerialException as e:
        print(f"\nError: Could not open serial port. {e}")
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("\nExiting.")

if __name__ == "__main__":
    main()