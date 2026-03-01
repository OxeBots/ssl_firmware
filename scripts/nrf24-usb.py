import argparse
import sys
import time
import platform
import os
import subprocess

try:
    import ssl_robot_protocol_bp
except ImportError:
    print("Generating Python bitproto module...")
    try:
        subprocess.run(
            ["bitproto", "py", "proto/ssl_robot_protocol.bitproto", "."], check=True
        )
        import ssl_robot_protocol_bp
    except Exception as e:
        print(
            f"Failed to generate bitproto python module. Ensure bitproto is installed: pip install bitproto. Error: {e}"
        )
        sys.exit(1)

import serial
import serial.tools.list_ports


class Colors:
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    BOLD = "\033[1m"
    RESET = "\033[0m"


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
            print(f"{Colors.GREEN}Connected to {port} at {baudrate} baud{Colors.RESET}")

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
                print(f"{Colors.GREEN}Found device at: {port.device}{Colors.RESET}")
                try:
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

    def send_at_command(self, command, wait_time=0.1):
        """Send AT command and read response"""
        print(f"{Colors.BLUE}>>> {command}{Colors.RESET}")

        self.ser.reset_input_buffer()
        self.ser.write((command + "\r\n").encode("ascii"))
        self.ser.flush()

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
            return False

        addr_str = ",".join([f"0x{byte:02X}" for byte in address_bytes])
        response = self.send_at_command(f"AT+RXA={addr_str}", wait_time=3)
        return "successful" in response.lower()

    def set_transmit_address(self, address_bytes):
        """Set transmit pipe address using AT+TXA command"""
        if len(address_bytes) != 5:
            return False

        addr_str = ",".join([f"0x{byte:02X}" for byte in address_bytes])
        response = self.send_at_command(f"AT+TXA={addr_str}", wait_time=3)
        return "successful" in response.lower()

    def set_addresses(self, rx_address, tx_address):
        """Set both receive and transmit addresses"""
        print(f"\n{Colors.BLUE}Setting addresses:{Colors.RESET}")
        print(
            f"{Colors.BLUE}  RX (listen): {[hex(x) for x in rx_address]}{Colors.RESET}"
        )
        print(
            f"{Colors.BLUE}  TX (send to): {[hex(x) for x in tx_address]}{Colors.RESET}"
        )

        return self.set_receive_address(rx_address) and self.set_transmit_address(
            tx_address
        )

    def set_frequency(self, frequency_ghz):
        """Set operating frequency in GHz (e.g., 2.404 for 2.404GHz)"""
        if not 2.400 <= frequency_ghz <= 2.525:
            return False

        formatted_freq = f"{frequency_ghz:.3f}"
        print(f"\n{Colors.BLUE}Setting frequency to {formatted_freq} GHz{Colors.RESET}")
        response = self.send_at_command(f"AT+FREQ={formatted_freq}")
        return "successful" in response.lower()

    def set_data_rate(self, rate):
        """Set data rate (1=250Kbps, 2=1Mbps, 3=2Mbps)"""
        if rate not in [1, 2, 3]:
            return False

        rates = {1: "250Kbps", 2: "1Mbps", 3: "2Mbps"}
        print(f"\n{Colors.BLUE}Setting data rate to {rates[rate]}{Colors.RESET}")
        response = self.send_at_command(f"AT+RATE={rate}")
        return "successful" in response.lower()

    def send_data(self, command_bytes):
        """
        Send raw bitproto bytes.
        DO NOT ZERO PAD! The dongle automatically counts our UART bytes,
        prepends the length inside the RF packet, and fires it over the air!
        """
        self.ser.reset_input_buffer()
        self.ser.write(command_bytes)
        self.ser.flush()

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

    def receive_data(self, timeout=0.5):
        """
        Receive exact length of Telemetry packet.
        The USB dongle strips its internal length byte and pushes raw payload to us.
        """
        start_time = time.time()
        buffer = bytearray()

        expected_len = ssl_robot_protocol_bp.RobotTelemetry.BYTES_LENGTH

        while time.time() - start_time < timeout:
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                buffer.extend(chunk)

                # USB DONGLE QUIRK:
                # The adapter spits out a 6-byte success status code (02 00 00 00 00 00)
                # immediately after it finishes transmitting our command. We MUST strip it!
                while len(buffer) >= 6 and buffer.startswith(
                    b"\x02\x00\x00\x00\x00\x00"
                ):
                    buffer = buffer[6:]

                if len(buffer) >= expected_len:
                    packet = buffer[:expected_len]
                    try:
                        telemetry = ssl_robot_protocol_bp.RobotTelemetry()
                        telemetry.decode(packet)
                        print(
                            f"\r{Colors.GREEN}✓ [RX Telemetry] Timestamp: {telemetry.timestamp} | Batt: {telemetry.battery_percentage}% | Kicker: {telemetry.kicker_voltage / 100:.2f}V{Colors.RESET}"
                            + " " * 15
                        )
                        return telemetry
                    except Exception as e:
                        print(
                            f"\n{Colors.RED}[DEBUG RX] Decode error: {e}{Colors.RESET}"
                        )

                    # Drop parsed bytes to resync
                    buffer = buffer[expected_len:]
            else:
                time.sleep(0.01)

        print(
            f"\n{Colors.YELLOW}[DEBUG RX] Timeout reached. No valid telemetry decoded.{Colors.RESET}"
        )
        return None

    def close(self):
        """Close serial connection"""
        if self.ser.is_open:
            self.ser.close()
            print(f"{Colors.BLUE}Serial connection closed{Colors.RESET}")


class CLIController:
    """A blocking, synchronous CLI command loop for testing the Bitproto protocol"""

    def __init__(self, device):
        self.device = device
        self.running = True

    def start(self):
        print(f"\n{Colors.BOLD}=== ROBOT CLI CONTROL ==={Colors.RESET}")
        print("Type a command like: {x: 1000, y: 500, kick: 128}")
        print("Type 'q' or 'quit' to exit.")
        print("=========================\n")

        while self.running:
            try:
                cmd_input = input(f"\n{Colors.BLUE}Cmd > {Colors.RESET}").strip()
                if cmd_input.lower() in ["q", "quit", "exit"]:
                    break
                if not cmd_input:
                    continue

                cmd_input = cmd_input.strip("{} ")
                pairs = [p.strip() for p in cmd_input.split(",")]

                x, y, kick = 0, 0, 0
                for p in pairs:
                    if ":" in p:
                        k, v = p.split(":", 1)
                        k = k.strip(" \"'")
                        if k == "x":
                            x = int(v.strip())
                        elif k == "y":
                            y = int(v.strip())
                        elif k == "kick":
                            kick = int(v.strip())

                cmd = ssl_robot_protocol_bp.RobotCommand()

                # Keep timestamp under 32-bit limits
                timestamp_val = int(time.time() * 1000) % (2**31 - 1)
                cmd.timestamp = timestamp_val
                cmd.target_pose.x = x
                cmd.target_pose.y = y
                cmd.kick_velocity = kick

                encoded = cmd.encode()

                print(
                    f"{Colors.CYAN}Sending -> Timestamp={timestamp_val}, x={x}, y={y}, kick={kick} {Colors.RESET}",
                    end="",
                    flush=True,
                )

                self.device.send_data(encoded)

                # Await a response from the robot firmware via NRF24 USB dongle
                self.device.receive_data(timeout=0.5)

            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"{Colors.RED}Error: {e}{Colors.RESET}")


def main():
    parser = argparse.ArgumentParser(
        description="nRF24L01 Wireless Module Controller",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-p", "--port", type=str, default="auto", help="Serial port device."
    )
    parser.add_argument(
        "-b", "--baud", type=int, default=115200, help="Serial baud rate."
    )
    parser.add_argument(
        "-f",
        "--freq",
        type=float,
        default=2.401,
        help="Communication frequency in GHz.",
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
        "-a", "--address", type=str, default="ESP32", help="5-byte address string."
    )
    parser.add_argument(
        "--skip-config",
        action="store_true",
        help="Skip sending AT config commands to the USB adapter.",
    )

    args = parser.parse_args()

    device = None
    try:
        print(
            f"{Colors.BOLD}=== nRF24L01 Wireless Module Controller ==={Colors.RESET}\n"
        )

        if args.port.lower() == "auto":
            device = nRF24L01_Controller.auto_connect(baudrate=args.baud)
        else:
            device = nRF24L01_Controller(port=args.port, baudrate=args.baud)

        if not device:
            return

        if not args.skip_config:
            print(f"\n{Colors.BOLD}CONFIGURING MODULE:{Colors.RESET}")
            if len(args.address) != 5:
                print(
                    f"{Colors.RED}Error: Address must be exactly 5 characters long.{Colors.RESET}"
                )
                return

            address_bytes = list(args.address.encode("ascii"))
            rx_address_bytes = [int(ord(c)) for c in "ADMIN"]
            device.set_addresses(rx_address_bytes, address_bytes)

            esp32_freq_ghz = 2.400 + (76 * 0.001)
            device.set_frequency(esp32_freq_ghz)
            device.set_data_rate(args.rate)
            device.get_system_info()
        else:
            print(
                f"\n{Colors.YELLOW}Skipping USB Adapter AT-Configuration...{Colors.RESET}"
            )

        joystick = CLIController(device)
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
