#! MIT License

# Sets SC09 serial bus servo IDs

# Majority of code is AI generated

# TODO: add feature to calibrate servo end points

import time
from machine import UART, Pin, I2C

# ==========================================
# 1. HARDWARE INITIALISATION & DRIVERS
# ==========================================

# --- UART Setup for Waveshare SC09 Servos ---
# Default half-duplex configuration at 1Mbps
uart = UART(0, baudrate=1_000_000, tx=Pin(0), rx=Pin(1), timeout=5)

# --- Matrix Keypad Driver ---
ROW_PINS = [3, 4, 5, 6]
COL_PINS = [7, 8, 9]

KEY_MAP = [
    ['1', '2', '3'],
    ['4', '5', '6'],
    ['7', '8', '9'],
    ['*', '0', '#']
]

# Set up row pins as outputs (initially HIGH)
rows = [Pin(p, Pin.OUT, value=1) for p in ROW_PINS]
# Set up column pins as inputs with internal pull-up resistors
cols = [Pin(p, Pin.IN, Pin.PULL_UP) for p in COL_PINS]

def read_keypad():
    """Scans the 4x3 matrix keypad and returns the character pressed, or None."""
    for row_idx, row_pin in enumerate(rows):
        row_pin.value(0) # Drive row LOW
        for col_idx, col_pin in enumerate(cols):
            if col_pin.value() == 0: # Check if column goes LOW
                time.sleep_ms(20) # Simple debounce delay
                if col_pin.value() == 0:
                    while col_pin.value() == 0:
                        time.sleep_ms(10) # Wait for key release
                    row_pin.value(1) # Restore row HIGH
                    return KEY_MAP[row_idx][col_idx]
        row_pin.value(1) # Restore row HIGH
    return None

def wait_for_any_key():
    """Blocks execution until any valid key is pressed and released."""
    while True:
        key = read_keypad()
        if key is not None:
            return key
        time.sleep_ms(20)

class LCD2004_4Bit:
    """Integrated 4-bit parallel driver for HD44780 20x4 LCD displays."""
    def __init__(self, rs, e, d4, d5, d6, d7):
        self.rs = Pin(rs, Pin.OUT)
        self.e = Pin(e, Pin.OUT)
        self.d4 = Pin(d4, Pin.OUT)
        self.d5 = Pin(d5, Pin.OUT)
        self.d6 = Pin(d6, Pin.OUT)
        self.d7 = Pin(d7, Pin.OUT)
        
        self.e.value(0)
        self.rs.value(0)
        
        # Row memory offsets for a 20x4 display
        self.row_offsets = [0x00, 0x40, 0x14, 0x54]
        self.init_display()

    def pulse_enable(self):
        self.e.value(0)
        time.sleep_us(1)
        self.e.value(1)
        time.sleep_us(1)  # Enable pulse width > 450ns
        self.e.value(0)
        time.sleep_us(100) # Execution time delay

    def write_4bit(self, value):
        self.d4.value((value >> 0) & 0x01)
        self.d5.value((value >> 1) & 0x01)
        self.d6.value((value >> 2) & 0x01)
        self.d7.value((value >> 3) & 0x01)
        self.pulse_enable()

    def send(self, data, mode):
        self.rs.value(mode)
        # High nibble
        self.write_4bit(data >> 4)
        # Low nibble
        self.write_4bit(data & 0x0F)

    def write_cmd(self, cmd):
        self.send(cmd, 0)
        time.sleep_ms(2) # Robust delay for heavy commands

    def write_data(self, data):
        self.send(data, 1)

    def init_display(self):
        time.sleep_ms(50) # Wait for LCD power up
        
        # Soft reset sequence into 4-bit mode
        self.write_4bit(0x03)
        time.sleep_ms(5)
        self.write_4bit(0x03)
        time.sleep_us(150)
        self.write_4bit(0x03)
        self.write_4bit(0x02) # Set to 4-bit interface
        
        # Function Set: 4-bit, 2 lines, 5x8 font
        self.write_cmd(0x28)
        # Display Control: Display ON, Cursor OFF, Blink OFF
        self.write_cmd(0x0C)
        # Entry Mode Set: Increment cursor, No shift
        self.write_cmd(0x06)
        self.clear()

    def clear(self):
        self.write_cmd(0x01)
        time.sleep_ms(2)

    def move_to(self, col, row):
        if 0 <= row < 4:
            self.write_cmd(0x80 | (col + self.row_offsets[row]))

    def text(self, string, col, row):
        self.move_to(col, row)
        # Trim text to fit remaining line space safely
        max_len = 20 - col
        for char in string[:max_len]:
            self.write_data(ord(char))


# Initialize the LCD with safe, unassigned Pico pins
lcd = LCD2004_4Bit(rs=10, e=11, d4=16, d5=17, d6=18, d7=19)

# ==========================================
# 2. WAVESHARE SC SERIES BUS SERVO PROTOCOL
# ==========================================

# SC09 Protocol Constants
INST_PING = 0x01
INST_WRITE = 0x03
REG_EPROM_LOCK = 0x30	# Eprom memory address of EPROM protection lock flag
REG_ID = 0x05 # Eprom memory address register for Servo ID
REG_TARGET_POS = 0x2A  # 2 Bytes: Big-Endian Position Register (0-1023)
REG_TARGET_TIME = 0x2C # 2 Bytes: Big-Endian Move Duration Time Register

def send_packet(servo_id, instruction, parameters):
    """Assembles and writes a Feetech/Waveshare SC-series protocol frame."""
    length = len(parameters) + 2
    packet = bytearray([0xFF, 0xFF, servo_id, length, instruction])
    packet.extend(parameters)
    
    # Calculate Checksum: ~ (ID + Length + Instruction + Params)
    checksum = sum(packet[2:])
    packet.append((~checksum) & 0xFF)
    
    # Flush remaining RX garbage due to half-duplex echo loops before transmission
    if uart.any():
        uart.read()
        
    uart.write(packet)
    # Give the hardware bus driver line time to complete physical transmission
    time.sleep_ms(2)

def ping_servo(servo_id):
    """Pings a specific servo ID and checks for an acknowledgement frame."""
    send_packet(servo_id, INST_PING, [])
    
    # Read the response packet from the shared single data bus line
    time.sleep_ms(5)
    if uart.any():
        response = uart.read()
        # A valid echo or response back should match protocol sizes
        if response and len(response) >= 6:
            # Look for returning sequence headers inside half-duplex streams
            for i in range(len(response) - 4):
                if response[i] == 0xFF and response[i+1] == 0xFF and response[i+2] == servo_id:
                    return True
    return False

def change_servo_id(old_id, new_id):
    """Writes a new ID value into the EPROM memory layout of the target servo."""
    # Step A: Unlock EPROM protection lock flag (Address 0x30 = 0 to write registers)
    send_packet(old_id, INST_WRITE, [REG_EPROM_LOCK, 0x00])
    time.sleep_ms(10)
    
    # Step B: Write the new target ID into Register 0x05
    send_packet(old_id, INST_WRITE, [REG_ID, new_id])
    time.sleep_ms(20)
    
    # Step C: Re-lock EPROM protection flag (Address 0x30 = 1)
    send_packet(new_id, INST_WRITE, [REG_EPROM_LOCK, 0x01])
    time.sleep_ms(10)

def set_servo_position(servo_id, position, duration_ms):
    """Moves SC09 servo to a position (0-1023) over a specified time duration."""
    # Constrain boundaries safely
    position = max(0, min(1023, position))
    
    # Split 16-bit parameters into Big-Endian layout byte arrays
    pos_h = (position >> 8) & 0xFF
    pos_l = position & 0xFF
    time_h = (duration_ms >> 8) & 0xFF
    time_l = duration_ms & 0xFF
    
    # Combined write parameters payload starting at Target Position address
    payload = [REG_TARGET_POS, pos_h, pos_l, time_h, time_l]
    send_packet(servo_id, INST_WRITE, payload)

def scan_bus():
    """Scans the bus for valid servo IDs between 1 and 253."""
    display_message("Scanning", "", "Please wait...")
    found_servos = []
    # Test typical valid range of IDs
    for servo_id in range(1, 254):
        if ping_servo(servo_id):
            found_servos.append(servo_id)
    return found_servos

# ==========================================
# 3. INTERACTIVE UI FUNCTIONS
# ==========================================

def display_message(line1, line2="", line3="", line4=""):
    """Convenience helper preserved to output formatted lines to the 20x4 LCD."""
    lcd.clear()
    # Pad strings with spaces up to 20 characters to clear artifacts
    lcd.text(line1[:20], 0, 0)
    lcd.text(line2[:20], 0, 1)
    lcd.text(line3[:20], 0, 2)
    lcd.text(line4[:20], 0, 3)
    
def get_numeric_input():
    """Collects input characters from matrix layout until user hits '#'."""
    input_str = ""
    while True:
        key = wait_for_any_key()
        if key == '#':
            if len(input_str) > 0:
                return int(input_str)
        elif key == '*':
            # Handle backspace behavior if key '*' is tapped
            if len(input_str) > 0:
                input_str = input_str[:-1]
        else:
            # Allow strings up to 3 digits (Valid IDs: 1-253)
            if len(input_str) < 3:
                input_str += key
        
        display_message("Enter New ID:", f"Value: {input_str}", "Press # to confirm", "Press * to backspace")

# ==========================================
# 4. MAIN PROGRAM WORKFLOW ENGINE
# ==========================================

def test_servo(servo_id):
    
    MIN_POS = 0
    MAX_POS = 1023
    MID_POS = (MAX_POS - MIN_POS) // 2 + MIN_POS
    
    # Move to maximum limit position 1023 over 1.2 seconds
    display_message(f"ID {servo_id} Moving:", f"-> Max: {MAX_POS}", "", "Please wait...")
    set_servo_position(servo_id, MAX_POS, 1200)
    time.sleep(1.5) # Time for servo to arrive and settle
    
    # Move to minmum limit position 0 over 1.2 seconds
    display_message(f"ID {servo_id} Moving:", f"-> Min: {MIN_POS}", "", "Please wait...")
    set_servo_position(servo_id, MIN_POS, 1200)
    time.sleep(1.5) # Time for servo to arrive and settle

    # Move back to midpoint position 511 over 1.2 seconds
    display_message(f"ID {servo_id} Moving:", f"-> Centre: {MID_POS}", "", "Please wait...")
    set_servo_position(servo_id, MID_POS, 1200)
    time.sleep(1.5)
    display_message("Test complete!", f"ID {servo_id} ready.", "", "Press any key")
    wait_for_any_key()
    
def id_setter():
    # Requirement 1: Welcome Screen
    display_message("=" * 20, "WWRM".center(20), "SC09 Servo ID Writer".center(20), "=" * 20)
    time.sleep(2.5)

    while True:
        # Requirement 2: Scan the bus
        attached_servos = scan_bus()
        num_servos = len(attached_servos)

        # Requirement 3: Check for empty bus condition
        if num_servos == 0:
            display_message("No servos found!", "Connect a servo", "Then press a key", "to retry...")
            wait_for_any_key()
            continue

        # Requirement 4: Check for multi-servo conflict condition
        if num_servos > 1:
            display_message("Error: Found", f"{num_servos} servos!", f"Disconnect {num_servos-1}", "Then press a key")
            wait_for_any_key()
            continue

        # Requirement 5: Exactly one servo detected
        current_id = attached_servos[0]
        display_message(f"Current ID: {current_id}", "Enter New ID...", "Type digits", "Then press #")
        
        # Pull numeric entry values from user
        new_id = get_numeric_input()
        
        if new_id < 1 or new_id > 253:
            display_message("ERROR!", "ID must be 1 - 253", "Press any key to", "retry")
            wait_for_any_key()
            continue

        # Requirement 6: Update entry if change condition is met
        if new_id != current_id:
            display_message("Writing ID...", "", "Do not unplug!")
            change_servo_id(current_id, new_id)
            
            # Double-check verify cycle
            time.sleep(0.5)
            if ping_servo(new_id):
                display_message("Success!", f"ID changed to {new_id}", "", "Press any key")
                test_servo(new_id)
            else:                      
                display_message("Write Failed!", "Check wiring", "Press any key", "to retry")
                wait_for_any_key()
        else:
            display_message(f"IDs are both {new_id}", "Nothing to do", "", "Press any key")
            wait_for_any_key()
            test_servo(current_id)
            
        display_message("Ready for next servo", "Press any key to", "continue", "or turn power off")
        wait_for_any_key()

if __name__ == '__main__':
    id_setter()
