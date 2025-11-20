import serial
import serial.tools.list_ports
import pyaudio
import threading
import time
import sys

# ==========================================
# CONFIGURATION (Adjust based on Firmware)
# ==========================================
BAUD_RATE = 115200  # Search results suggest v5 firmware uses 115200
# BAUD_RATE = 921600 # Use this if you are on older firmware
CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100  # KV4P uses 44.1kHz or 22.05kHz. Check firmware!


class KV4PRadio:
    def __init__(self):
        self.ser = None
        self.audio = pyaudio.PyAudio()
        self.input_stream = None
        self.output_stream = None
        self.running = False
        self.freq = "144.0000"

    def select_serial_port(self):
        print("\n--- STEP 1: Select Serial Port ---")
        ports = list(serial.tools.list_ports.comports())

        if not ports:
            print("No COM ports found! Check your USB connection.")
            return False

        for i, p in enumerate(ports):
            print(f"[{i}] {p.device} - {p.description}")

        while True:
            try:
                selection = int(input("Enter port number (e.g., 0): "))
                if 0 <= selection < len(ports):
                    self.port_name = ports[selection].device
                    print(f"Selected: {self.port_name}")
                    return True
                else:
                    print("Invalid selection.")
            except ValueError:
                print("Please enter a number.")

    def select_audio_devices(self):
        print("\n--- STEP 2: Select Audio Devices ---")
        info = self.audio.get_host_api_info_by_index(0)
        numdevices = info.get('deviceCount')

        # Input Devices
        print("\nAvailable INPUT (Microphone) Devices:")
        input_indices = []
        for i in range(0, numdevices):
            if (self.audio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
                name = self.audio.get_device_info_by_host_api_device_index(0, i).get('name')
                print(f"[{i}] {name}")
                input_indices.append(i)

        while True:
            try:
                self.input_idx = int(input("Select Input Device ID: "))
                if self.input_idx in input_indices:
                    break
                print("Invalid ID.")
            except ValueError:
                pass

        # Output Devices
        print("\nAvailable OUTPUT (Speaker) Devices:")
        output_indices = []
        for i in range(0, numdevices):
            if (self.audio.get_device_info_by_host_api_device_index(0, i).get('maxOutputChannels')) > 0:
                name = self.audio.get_device_info_by_host_api_device_index(0, i).get('name')
                print(f"[{i}] {name}")
                output_indices.append(i)

        while True:
            try:
                self.output_idx = int(input("Select Output Device ID: "))
                if self.output_idx in output_indices:
                    break
                print("Invalid ID.")
            except ValueError:
                pass

        return True

    def start_streams(self):
        # Setup Audio Streams
        self.input_stream = self.audio.open(format=FORMAT, channels=CHANNELS,
                                            rate=RATE, input=True,
                                            input_device_index=self.input_idx,
                                            frames_per_buffer=CHUNK_SIZE)

        self.output_stream = self.audio.open(format=FORMAT, channels=CHANNELS,
                                             rate=RATE, output=True,
                                             output_device_index=self.output_idx,
                                             frames_per_buffer=CHUNK_SIZE)

        # Setup Serial
        try:
            self.ser = serial.Serial(self.port_name, BAUD_RATE, timeout=0.1)
            print(f"\nSuccessfully connected to {self.port_name} at {BAUD_RATE} baud.")
        except Exception as e:
            print(f"Error opening serial port: {e}")
            return False

        self.running = True

        # Start Threads
        self.rx_thread = threading.Thread(target=self.rx_loop)
        self.rx_thread.start()

        return True

    def rx_loop(self):
        """Receives data from Radio -> Plays on Speakers"""
        print("RX Thread Started...")
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    # CRITICAL: You must modify this to match the KV4P Protocol!
                    # Current logic: Reads everything and tries to play it as audio.
                    # In reality, you need to split 'Data Packets' from 'Audio Packets'.

                    data = self.ser.read(CHUNK_SIZE)

                    # Placeholder for packet filtering:
                    # if is_audio_packet(data):
                    #    decoded = opus_decode(data)
                    #    self.output_stream.write(decoded)
                    # else:
                    #    print(f"Data Received: {data}")

                    # For now, just print raw data to help you debug the protocol
                    # print(f"RAW: {data[:10]}...")
                    pass
            except Exception as e:
                print(f"RX Error: {e}")
                break

    def send_frequency(self, frequency):
        """
        Formats the frequency command and sends it to the device.
        You MUST verify this format in 'SerialService.java' or 'protocol.cpp'
        """
        print(f"\nConfiguring Radio to {frequency} MHz...")

        # --- PROTOCOL GUESS 1: Standard SA818 Command ---
        # Format: AT+DMOSETGROUP=bw,tx_freq,rx_freq,tx_ctcss,sq,rx_ctcss
        # Example: AT+DMOSETGROUP=0,145.0000,145.0000,0,4,0
        cmd_str = f"AT+DMOSETGROUP=0,{frequency},{frequency},0,4,0\r\n"

        # --- PROTOCOL GUESS 2: Custom KV4P Packet ---
        # It might look like: HEAD(1) + TYPE(1) + DATA(...)
        # cmd_str = f"FREQ={frequency}\n"

        if self.ser and self.ser.is_open:
            self.ser.write(cmd_str.encode('utf-8'))
            print(f"Sent: {cmd_str.strip()}")
        else:
            print("Serial port not open.")

    def main_loop(self):
        print("\n--- SYSTEM READY ---")
        print("Commands: 'FREQ <145.000>' to set frequency, 'Q' to quit.")

        while self.running:
            user_input = input("CMD> ").strip().upper()

            if user_input == 'Q':
                print("Stopping...")
                self.running = False
                break

            elif user_input.startswith("FREQ"):
                try:
                    parts = user_input.split()
                    if len(parts) == 2:
                        freq = "{:.4f}".format(float(parts[1]))
                        self.send_frequency(freq)
                    else:
                        print("Usage: FREQ 144.500")
                except ValueError:
                    print("Invalid frequency format.")

            else:
                # Send raw input for testing
                if self.ser:
                    self.ser.write((user_input + "\n").encode('utf-8'))

        # Cleanup
        if self.ser: self.ser.close()
        self.input_stream.stop_stream()
        self.input_stream.close()
        self.output_stream.stop_stream()
        self.output_stream.close()
        self.audio.terminate()


if __name__ == "__main__":
    app = KV4PRadio()
    if app.select_serial_port():
        if app.select_audio_devices():
            if app.start_streams():
                app.main_loop()