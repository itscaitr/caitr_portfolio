#
# very simple python program for the spectrometer board
# takes in can messages from the board, parses them, then prints outputs
# and stores host time (your computer's time), the canID, voltage, and the actual 
# message in a .csv file. 
#
# version 1.0
# author: caitr
#

import serial
import time
import csv
import struct
import os

PORT = "COM5" # for my machine -- change depending on urs
BAUD = 115200
CSV_FILENAME = "samples.csv" # this file should be saved in the same directory as this code


def parse_rcv_line(line_text):

    # parses rcv lines, returns (can_id:int, data_bytes:bytes)

    parts = line_text.split()
    if len(parts) < 3:
        return None, None

    if parts[0].lower() != "rcv":
        return None, None

    # id is hex :)
    try:
        can_id = int(parts[1], 16)
    except ValueError:
        return None, None

    data_bytes = bytearray()

    # start from parts[2:], consume tokens that are pure hex
    for token in parts[2:]:
        # stop when hit non-hex
        if not all(c in "0123456789abcdefABCDEF" for c in token):
            break

        if len(token) % 2 == 1:
            token = token[:-1]

        for i in range(0, len(token), 2):
            byte_str = token[i:i+2]
            try:
                data_bytes.append(int(byte_str, 16))
            except ValueError:
                # just stop if there's something wacky
                break

    return can_id, bytes(data_bytes)



def main():
    print("Current working directory:", os.getcwd()) # tells u where ur file is
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(1.0)
    ser.reset_input_buffer()

    with open(CSV_FILENAME, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["host_time_s", "can_id", "voltage", "raw_line"])

        def send(cmd):
            ser.write((cmd + "\n").encode())
            resp = ser.readline().decode(errors="ignore").strip()
            print("→", cmd, "|", resp)
            # log config replies too, for testing sake
            if resp:
                t = time.time()
                writer.writerow([f"{t:.6f}", "", "", resp])
                csvfile.flush()
            if not resp.startswith("OK"):
                raise RuntimeError(f"Command failed: {cmd} → {resp}")

        # configure fdcanusb (thanks david)
        send("can off")
        send("conf set can.bitrate 1000000")
        send("conf set can.bitrate_switch off")
        # optional: ensure fdcan_frame is on (default usually is)
        # send("conf set can.fdcan_frame on")
        send("can on")

        print("Bus on, waiting for rcv lines... Ctrl+C to stop.") # or any other line break command

        try:
            while True:
                line = ser.readline()
                if not line:
                    continue
                line_text = line.decode(errors="ignore").strip()
                if not line_text:
                    continue

                t = time.time()
                print(f"{t:.3f} ← {line_text}")

                can_id, data = parse_rcv_line(line_text)

                if can_id is not None and len(data) >= 4:
                    voltage = struct.unpack("<f", data[0:4])[0]
                    writer.writerow([f"{t:.6f}", f"0x{can_id:X}", f"{voltage:.6f}", line_text])
                else:
                    writer.writerow([f"{t:.6f}", "", "", line_text])

                csvfile.flush()

        except KeyboardInterrupt:
            print("\nStopping logging.") # currently, no autostop exists. one can be implemented.

    ser.close()


if __name__ == "__main__":
    main()
