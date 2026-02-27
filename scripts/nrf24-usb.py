import argparse
import sys
import threading
import time

import serial
import serial.tools.list_ports


# Added: Class to hold ANSI color codes
class Colors:
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    BOLD = "\033[1m"
    RESET = "\033[0m"


# try to import pynput
try:
    from pynput import keyboard
except ImportError:
    print(f"{Colors.RED}Error: 'pynput' library is missing.{Colors.RESET}")
    print(
        f"Please run: {Colors.YELLOW}pip install pynput{Colors.RESET} or {Colors.YELLOW}sudo apt install python3-pynput{Colors.RESET}"
    )
    sys.exit(1)


class nRF24L01_Controller:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.1):
        """
        Initializes the serial connection.
        Note: This __init__ expects a specific port.
        Use the auto_connect() static method to find and connect automatically.
        """
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )

        if self.ser.is_open:
            print(
                f"{Colors.GREEN}Connected to {port} at {baudrate} baud{Colors.RESET}"
            )

    @staticmethod
    def auto_connect(baudrate=115200, timeout=2):
        """
        Find the nRF24L01 USB adapter by its VID/PID and return an
        initialized controller instance.
        """
        # These IDs are from dmesg output:
        # idVendor=1a86, idProduct=7523
        TARGET_VID = 0x1A86
        TARGET_PID = 0x7523

        print(
            f"{Colors.BLUE}Searching for nRF24L01 device (VID:{TARGET_VID:x}, PID:{TARGET_PID:x})...{Colors.RESET}"
        )

        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.vid == TARGET_VID and port.pid == TARGET_PID:
                print(
                    f"{Colors.GREEN}Found device at: {port.device}{Colors.RESET}"
                )
                try:
                    # Return a new instance of this class
                    return nRF24L01_Controller(
                        port=port.device, baudrate=baudrate, timeout=timeout
                    )
                except serial.serialutil.SerialException as e:
                    print(
                        f"{Colors.RED}Error connecting to {port.device}: {e}{Colors.RESET}"
                    )
                    return None

        print(
            f"\n{Colors.RED}Error: Could not find nRF24L01 USB adapter.{Colors.RESET}"
        )
        print(
            f"{Colors.RED}Please ensure the device is plugged in.{Colors.RESET}"
        )
        return None

    @staticmethod
    def decode_gb2312(hex_string):
        """Decode GB2312 Chinese text from hex string"""
        try:
            hex_values = hex_string.strip().split()
            byte_data = bytes(int(x, 16) for x in hex_values)
            decoded_text = byte_data.decode("gb2312")
            return decoded_text
        except Exception as e:
            return f"Decoding error: {e}"

    @staticmethod
    def translate_chinese(text):
        """Translate common Chinese terms to English"""
        translations = {
            "系统信息": "System Information",
            "波特率": "Baud Rate",
            "目标地址": "Target Address",
            "本地接收地址": "Local Receive Address",
            "通讯频率": "Communication Frequency",
            "校验模式": "Check Mode",
            "发射功率": "Transmit Power",
            "空中传输速率": "Air Data Rate",
            "低噪声放大增益": "Low Noise Amplifier Gain",
            "开启": "Enabled",
            "关闭": "Disabled",
            "传输速率设置成功": "Data rate setting successful",
            "通讯波特率设置成功": "Baud rate setting successful",
            "地址设置成功": "Address setting successful",
            "设置成功": "Setting successful",
            "成功": "successful",
            "CRC校验": "CRC Check",
            "位": " bits ",
            "传输速率": "Data Rate",
        }

        for chinese, english in translations.items():
            text = text.replace(chinese, english)

        return text

    def send_command(self, command, wait_time=0.1):
        """Send serial command and return the response"""
        self.ser.reset_input_buffer()
        self.ser.write((command + "\r\n").encode("ascii"))
        time.sleep(wait_time)
        response = self.ser.read(self.ser.in_waiting)
        return response.decode("ascii", errors="ignore")

    def send_at_command(self, command, wait_time=0.1):
        """Send AT command and read response"""
        print(f"{Colors.BLUE}>>> {command}{Colors.RESET}")

        # Clear buffer and send command
        self.ser.reset_input_buffer()
        self.ser.write((command + "\r\n").encode("ascii"))
        self.ser.flush()

        # Wait and read response
        time.sleep(wait_time)

        response = b""
        start_time = time.time()
        while time.time() - start_time < 5:
            if self.ser.in_waiting > 0:
                response += self.ser.read(self.ser.in_waiting)
                time.sleep(0.1)
            else:
                if response and time.time() - start_time > 0.5:
                    break

        if response:
            hex_str = " ".join([f"{b:02x}" for b in response])
            decoded = nRF24L01_Controller.decode_gb2312(hex_str)

            if decoded.startswith("Decoding error"):
                print(f"{Colors.RED}<<< {decoded}{Colors.RESET}")
                return decoded

            translated = nRF24L01_Controller.translate_chinese(decoded)
            print(f"{Colors.GREEN}<<< {translated}{Colors.RESET}")
            return translated
        else:
            print(f"{Colors.YELLOW}<<< No response{Colors.RESET}")
            return ""

    def get_system_info(self):
        """Get and display system information"""
        print("\n" + "=" * 50)
        print(f"{Colors.BOLD}SYSTEM INFORMATION{Colors.RESET}")
        print("=" * 50)

        response = self.send_at_command("AT?")
        if not response:
            return

        # Parse and display the system information cleanly
        lines = response.split("\r\n")
        for line in lines:
            line = line.strip()
            if not line or line in ["OK", "System Information"]:
                continue

            if ":" in line:
                key, value = line.split(":", 1)
                key = key.strip()
                value = value.strip()

                # Map the keys to consistent English names
                # Map substrings to consistent English display labels
                key_map = {
                    "Baud Rate": "Baud Rate",
                    "Target Address": "Target Address",
                    "Local Receive Address": "Local Receive Address",
                    "Communication Frequency": "Communication Frequency",
                    "Check Mode": "Check Mode",
                    "Transmit Power": "Transmit Power",
                    "Air Data Rate": "Air Data Rate",
                    "Low Noise Amplifier Gain": "Low Noise Amplifier Gain",
                }

                # Find the first map key that appears in the parsed key (fallback to the original key)
                display_key = next(
                    (label for sub, label in key_map.items() if sub in key),
                    key,
                )

                print(f"{display_key:25}: {value}")

    def set_receive_address(self, address_bytes):
        """Set receive pipe address using AT+RXA command"""
        if len(address_bytes) != 5:
            print(
                f"{Colors.RED}Error: Address must be exactly 5 bytes{Colors.RESET}"
            )
            return False

        addr_str = ",".join([f"0x{byte:02X}" for byte in address_bytes])
        response = self.send_at_command(f"AT+RXA={addr_str}", wait_time=3)

        if "successful" in response.lower():
            print(
                f"{Colors.GREEN}✓ Receive address set to {[hex(x) for x in address_bytes]}{Colors.RESET}"
            )
            return True
        else:
            print(f"{Colors.RED}✗ Failed to set receive address{Colors.RESET}")
            return False

    def set_transmit_address(self, address_bytes):
        """Set transmit pipe address using AT+TXA command"""
        if len(address_bytes) != 5:
            print(
                f"{Colors.RED}Error: Address must be exactly 5 bytes{Colors.RESET}"
            )
            return False

        addr_str = ",".join([f"0x{byte:02X}" for byte in address_bytes])
        response = self.send_at_command(f"AT+TXA={addr_str}", wait_time=3)

        if "successful" in response.lower():
            print(
                f"{Colors.GREEN}✓ Transmit address set to {[hex(x) for x in address_bytes]}{Colors.RESET}"
            )
            return True
        else:
            print(
                f"{Colors.RED}✗ Failed to set transmit address{Colors.RESET}"
            )
            return False

    def set_addresses(self, rx_address, tx_address):
        """Set both receive and transmit addresses"""
        print(f"\n{Colors.BLUE}Setting addresses:{Colors.RESET}")
        print(
            f"{Colors.BLUE}  RX (listen): {[hex(x) for x in rx_address]}{Colors.RESET}"
        )
        print(
            f"{Colors.BLUE}  TX (send to): {[hex(x) for x in tx_address]}{Colors.RESET}"
        )

        success1 = self.set_receive_address(rx_address)
        success2 = self.set_transmit_address(tx_address)

        return success1 and success2

    def set_frequency(self, frequency_ghz):
        """Set operating frequency in GHz (e.g., 2.404 for 2.404GHz)"""
        if not 2.400 <= frequency_ghz <= 2.525:
            print(
                f"{Colors.RED}Error: Frequency must be between 2.400 and 2.525 GHz{Colors.RESET}"
            )
            return False

        # Format frequency to always have 3 decimal places
        formatted_freq = f"{frequency_ghz:.3f}"
        print(
            f"\n{Colors.BLUE}Setting frequency to {formatted_freq} GHz{Colors.RESET}"
        )

        response = self.send_at_command(f"AT+FREQ={formatted_freq}")

        if "successful" in response.lower():
            print(
                f"{Colors.GREEN}✓ Frequency set to {formatted_freq} GHz{Colors.RESET}"
            )
            return True
        else:
            print(f"{Colors.RED}✗ Frequency change failed{Colors.RESET}")
            return False

    def set_data_rate(self, rate):
        """Set data rate (1=250Kbps, 2=1Mbps, 3=2Mbps)"""
        if rate not in [1, 2, 3]:
            print(
                f"{Colors.RED}Error: Data rate must be 1 (250Kbps), 2 (1Mbps), or 3 (2Mbps){Colors.RESET}"
            )
            return False

        rates = {1: "250Kbps", 2: "1Mbps", 3: "2Mbps"}
        print(
            f"\n{Colors.BLUE}Setting data rate to {rates[rate]}{Colors.RESET}"
        )

        response = self.send_at_command(f"AT+RATE={rate}")
        if "successful" in response.lower():
            print(
                f"{Colors.GREEN}✓ Data rate set to {rates[rate]}{Colors.RESET}"
            )
            return True
        else:
            print(f"{Colors.RED}✗ Data rate change failed{Colors.RESET}")
            return False

    def set_baudrate(self, new_baud):
        """Change baud rate (1-7 for different rates)"""
        baud_rates = {
            1: 4800,
            2: 9600,
            3: 14400,
            4: 19200,
            5: 38400,
            6: 57600,
            7: 115200,
        }

        if new_baud not in baud_rates:
            print(
                f"{Colors.RED}Error: Invalid baud rate code: {new_baud}{Colors.RESET}"
            )
            return False

        print(
            f"\n{Colors.BLUE}Setting baud rate to {baud_rates[new_baud]}{Colors.RESET}"
        )
        response = self.send_at_command(f"AT+BAUD={new_baud}")

        if "successful" in response.lower():
            print(
                f"{Colors.GREEN}✓ Baud rate changed to {baud_rates[new_baud]}{Colors.RESET}"
            )
            return True
        else:
            print(f"{Colors.RED}✗ Baud rate change failed{Colors.RESET}")
            return False

    def send_data(self, data):
        """Send data through wireless serial"""
        if len(data) > 31:
            data = data[:31]
            print(
                f"{Colors.YELLOW}Note: Data truncated to 31 bytes{Colors.RESET}"
            )
        self.ser.write(data.encode())

    def receive_data(self, timeout=0.5):
        """Receive data with proper parsing"""
        print(
            f"\n{Colors.BLUE}Listening for data ({timeout}s)...{Colors.RESET}"
        )
        start_time = time.time()

        while time.time() - start_time < timeout:
            if self.ser.in_waiting > 0:
                received = self.ser.read(self.ser.in_waiting)

                # Parse data (first byte is length)
                if len(received) >= 1:
                    length_byte = received[0]

                    if 0 < length_byte <= len(received) - 1:
                        actual_data = received[1 : 1 + length_byte]
                        try:
                            decoded = actual_data.decode("ascii")
                            print(
                                f"{Colors.GREEN}✓ Received: '{decoded}' ({length_byte} bytes){Colors.RESET}"
                            )
                            return decoded
                        except AttributeError:
                            hex_str = " ".join(
                                [f"{b:02x}" for b in actual_data]
                            )
                            print(
                                f"{Colors.GREEN}✓ Received (hex): {hex_str}{Colors.RESET}"
                            )
                            return hex_str
                    else:
                        try:
                            decoded = received.decode("ascii", errors="ignore")
                            print(
                                f"{Colors.GREEN}✓ Received: '{decoded}'{Colors.RESET}"
                            )
                            return decoded
                        except AttributeError:
                            hex_str = " ".join([f"{b:02x}" for b in received])
                            print(
                                f"{Colors.GREEN}✓ Received (hex): {hex_str}{Colors.RESET}"
                            )
                            return hex_str
                time.sleep(0.1)

        print(f"{Colors.YELLOW}✗ No data received{Colors.RESET}")
        return None

    def close(self):
        """Close serial connection"""
        if self.ser.is_open:
            self.ser.close()
            print(f"{Colors.BLUE}Serial connection closed{Colors.RESET}")


class RobotJoystick:
    def __init__(self, device):
        self.device = device
        self.last_command = None
        self.running = True

        # Define key mappings
        self.key_map = {
            "w": "f",
            "s": "b",
            "a": "l",
            "d": "r",
            "l": "o",
            "k": "k",
            "space": "s",
        }

    def on_press(self, key):
        try:
            # Handle standard keys (a, w, s, d)
            char = key.char.lower() if hasattr(key, "char") else None
        except AttributeError:
            char = None

        # Handle special keys
        if key == keyboard.Key.space:
            command = "s"
        elif key == keyboard.Key.esc:
            print(f"\n{Colors.YELLOW}Exiting...{Colors.RESET}")
            self.running = False
            return False  # Stop listener
        elif char in self.key_map:
            command = self.key_map[char]
        else:
            return  # Ignore unmapped keys

        # Only send if the command changed (prevents flooding serial buffer)
        if command != self.last_command:
            print(
                f"\r{Colors.BLUE}Sending: {command: <15}{Colors.RESET}",
                end="",
                flush=True,
            )
            self.device.send_data(command)
            self.last_command = command

    def on_release(self, key):
        # When W, A, S, or D are released, send stop.
        # Note: This logic stops if ANY mapped key is released.
        try:
            char = key.char.lower() if hasattr(key, "char") else None
        except AttributeError:
            char = None

        if char in ["w", "s", "a", "d"] and self.last_command in [
            "f",
            "b",
            "l",
            "r",
        ]:
            # Only stop if we are currently moving (don't stop if we just toggled LED)
            print(
                f"\r{Colors.YELLOW}Sending: stop           {Colors.RESET}",
                end="",
                flush=True,
            )
            self.device.send_data("s")
            self.last_command = "stop"

    def start(self):
        print(f"\n{Colors.BOLD}=== ROBOT JOYSTICK CONTROL ==={Colors.RESET}")
        print(f" {Colors.GREEN}[W]{Colors.RESET} Forward")
        print(f" {Colors.GREEN}[S]{Colors.RESET} Backward")
        print(f" {Colors.GREEN}[A]{Colors.RESET} Left")
        print(f" {Colors.GREEN}[D]{Colors.RESET} Right")
        print(
            f" {Colors.GREEN}[L]{Colors.RESET} LED On  / {Colors.GREEN}[K]{Colors.RESET} LED Off"
        )
        print(f" {Colors.RED}[ESC]{Colors.RESET} Quit")
        print("=" * 40 + "\n")

        # Collect events until released
        with keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        ) as listener:
            listener.join()


def main():
    parser = argparse.ArgumentParser(
        description="nRF24L01 Wireless Module Controller",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-p",
        "--port",
        type=str,
        default="auto",
        help="Serial port device. Use 'auto' to auto-detect by VID/PID.",
    )
    parser.add_argument(
        "-b", "--baud", type=int, default=115200, help="Serial baud rate."
    )
    parser.add_argument(
        "-f",
        "--freq",
        type=float,
        default=2.401,
        help="Communication frequency in GHz (e.g., 2.401 to 2.525)",
    )
    parser.add_argument(
        "-r",
        "--rate",
        type=int,
        default=1,
        choices=[1, 2, 3],
        help="Air data rate: 1 (250Kbps), 2 (1Mbps), 3 (2Mbps)",
    )

    parser.add_argument(
        "-a",
        "--address",
        type=str,
        default="ESP32",
        help="5-byte address string for the target device (e.g., 'ESP32').",
    )

    args = parser.parse_args()

    device = None
    try:
        # Initialize the controller
        print(
            f"{Colors.BOLD}=== nRF24L01 Wireless Module Controller ==={Colors.RESET}\n"
        )

        if args.port.lower() == "auto":
            device = nRF24L01_Controller.auto_connect(baudrate=args.baud)
        else:
            # Connect to the specified port
            print(
                f"{Colors.BLUE}Attempting to connect to specified port: {args.port}{Colors.RESET}"
            )
            device = nRF24L01_Controller(port=args.port, baudrate=args.baud)

        # Exit if connection failed
        if not device:
            return

        # Configure the module using values from argparse
        print(f"\n{Colors.BOLD}CONFIGURING MODULE:{Colors.RESET}")

        if len(args.address) != 5:
            print(
                f"{Colors.RED}Error: Address must be exact  ly 5 characters long.{Colors.RESET}"
            )
            return

        # Convert the address string to a list of byte values (integers)
        address_bytes = list(args.address.encode("ascii"))

        # Set both RX (adapter's) and TX (target's) addresses
        # We set RX to something different so it doesn't just listen to itself
        rx_address_bytes = [int(ord(c)) for c in "ADMIN"]
        device.set_addresses(rx_address_bytes, address_bytes)

        # Use arguments for frequency and rate
        # NOTE: The firmware is hardcoded to channel 76 (2.476 GHz)
        # We must match that here.
        esp32_freq_ghz = 2.400 + (76 * 0.001)  # Channel 76
        print(
            f"{Colors.YELLOW}Warning: Overriding frequency to {esp32_freq_ghz:.3f} GHz to match ESP32 channel 76.{Colors.RESET}"
        )
        device.set_frequency(esp32_freq_ghz)
        device.set_data_rate(args.rate)

        # Verify configuration
        print(f"\n{Colors.BOLD}UPDATED MODULE STATUS:{Colors.RESET}")
        device.get_system_info()

        # Start Joystick Loop
        joystick = RobotJoystick(device)
        joystick.start()

    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Stopped by user{Colors.RESET}")
    except serial.serialutil.SerialException as e:
        print(f"\n{Colors.RED}Serial Error: {e}{Colors.RESET}")
        print(
            f"{Colors.YELLOW}Hint: Do you have permissions? Try 'sudo usermod -a -G dialout $USER'{Colors.RESET}"
        )
    except Exception as e:
        print(f"{Colors.RED}An unexpected error occurred: {e}{Colors.RESET}")
    finally:
        if device and device.ser.is_open:
            device.close()


if __name__ == "__main__":
    main()
