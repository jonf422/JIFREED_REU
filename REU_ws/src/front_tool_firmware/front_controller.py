import sys
import serial
import serial.tools.list_ports

# ============================================================
#  CONFIG
# ============================================================
BAUD_RATE = 115200
TIMEOUT   = 30      # seconds to wait for a response

RETURN_CODES = {
    "0": "OK",
    "1": "Error: move unsafe (tool stuck or not homed)",
    "2": "Error: reserved (2)",
    "3": "Error: reserved (3)",
    "4": "Error: reserved (4)",
    "5": "Error: reserved (5)",
    "6": "Error: reserved (6)",
    "7": "Error: reserved (7)",
    "8": "Error: reserved (8)",
    "9": "Error: reserved (9)",
}

VALID_COMMANDS = {"home", "drill", "stop"}

# ============================================================
#  PLATFORM DETECTION
# ============================================================
IS_WINDOWS = sys.platform.startswith("win")
IS_LINUX   = sys.platform.startswith("linux")

# ============================================================
#  PORT SELECTION
# ============================================================
WINDOWS_IGNORED_PORTS = {"COM5", "COM6"}

def find_port():
    ports = list(serial.tools.list_ports.comports())
    if IS_WINDOWS:
        ports = [p for p in ports if p.device.upper() not in WINDOWS_IGNORED_PORTS]
    if not ports:
        print("No serial ports found.")
        sys.exit(1)

    if len(ports) == 1:
        return ports[0].device

    print("Available serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  —  {p.description}")

    while True:
        try:
            choice = int(input("Select port number: "))
            if 0 <= choice < len(ports):
                return ports[choice].device
        except ValueError:
            pass
        print("Invalid selection.")


# ============================================================
#  SERIAL COMMAND
# ============================================================
def send_command(ser, command):
    if command == "stop":
        # Kill outputs on device immediately via RTS pulse, then send command
        ser.rts = True
        ser.reset_input_buffer()
        ser.write((command + "\n").encode())
        ser.flush()
        ser.rts = False
        return "stop"

    ser.reset_input_buffer()
    ser.write((command + "\n").encode())
    while True:
        response = ser.readline().decode().strip()
        if response:
            return response


def interpret(response):
    if response == "stop":
        return "Stop sent — device resetting"
    return RETURN_CODES.get(response, f"Unknown response: '{response}'")


# ============================================================
#  WINDOWS — TESTING MODE
# ============================================================
def run_windows(ser):
    print("=" * 50)
    print("  DRILL CONTROLLER  —  TESTING MODE (Windows)")
    print("=" * 50)
    print(f"  Port : {ser.port}")
    print(f"  Baud : {BAUD_RATE}")
    print()
    print("  Commands: home | drill | stop | quit")
    print()

    while True:
        try:
            raw = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break

        if raw in ("quit", "exit", "q"):
            print("Exiting.")
            break

        if raw not in VALID_COMMANDS:
            print(f"  Unknown command '{raw}'. Valid: home, drill, quit")
            continue

        print(f"  Sending '{raw}'...")
        response = send_command(ser, raw)
        code_str = interpret(response)
        print(f"  Response code : {response}  —  {code_str}")
        print()


# ============================================================
#  LINUX — LIVE MODE
# ============================================================
def run_linux(ser):
    for line in sys.stdin:
        command = line.strip().lower()
        if command not in VALID_COMMANDS:
            continue
        response = send_command(ser, command)
        print(response, flush=True)


# ============================================================
#  MAIN
# ============================================================
def main():
    port = find_port()

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"Failed to open port {port}: {e}")
        sys.exit(1)

    with ser:
        if IS_WINDOWS:
            run_windows(ser)
        elif IS_LINUX:
            run_linux(ser)
        else:
            print(f"Unsupported platform: {sys.platform}")
            sys.exit(1)


if __name__ == "__main__":
    main()