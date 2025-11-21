import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import struct
import pyaudio
import os
import sys
import json
import ctypes
import traceback

# Add the current directory to the DLL search path to find opus.dll
if sys.platform == 'win32':
    os.add_dll_directory(os.path.dirname(os.path.abspath(__file__)))

from pyogg.opus import (
    OpusDecoder, OpusEncoder, opus_decode, opus_encode,
    opus_encoder_ctl, opus_decoder_ctl,
    OPUS_SET_BANDWIDTH_REQUEST, OPUS_BANDWIDTH_NARROWBAND,
    OPUS_SET_VBR_REQUEST
)
from pyogg import PyOggError


# --- Protocol Constants ---
COMMAND_DELIMITER = b'\xDE\xAD\xBE\xEF'
COMMAND_HOST_PTT_DOWN = 0x01
COMMAND_HOST_PTT_UP = 0x02
COMMAND_HOST_GROUP = 0x03
COMMAND_HOST_FILTERS = 0x04
COMMAND_HOST_CONFIG = 0x06
COMMAND_RX_AUDIO = 0x07
COMMAND_HOST_TX_AUDIO = 0x07
COMMAND_HOST_HL = 0x08
COMMAND_HOST_RSSI = 0x09
COMMAND_HELLO = 0x06
COMMAND_VERSION = 0x08


# --- Audio Parameters ---
AUDIO_SAMPLE_RATE = 8000
AUDIO_CHANNELS = 1
AUDIO_WIDTH = 2  # 16-bit
SAMPLES_PER_FRAME = 160  # 20ms frame at 8kHz
BUFFER_SIZE = SAMPLES_PER_FRAME
MAX_DECODED_FRAME_SIZE = SAMPLES_PER_FRAME * AUDIO_CHANNELS * AUDIO_WIDTH
MAX_ENCODED_FRAME_SIZE = 4000

CONFIG_FILE = 'config.json'

class HandshakeState:
    WAITING_FOR_HELLO = 0
    WAITING_FOR_VERSION = 1
    COMPLETE = 2

class KV4P_Driver:
    def __init__(self, port, baudrate, input_device_index, output_device_index, initial_freq):
        self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
        # Set DTR and RTS to match the Android application's behavior
        self.serial_port.rts = True
        self.serial_port.dtr = True
        
        self.p_audio = pyaudio.PyAudio()
        self.initial_freq = initial_freq
        self.handshake_state = HandshakeState.WAITING_FOR_HELLO

        # --- RX (Receiving) Audio Setup ---
        self.opus_decoder = OpusDecoder(sampling_frequency=AUDIO_SAMPLE_RATE, channels=AUDIO_CHANNELS)
        opus_decoder_ctl(self.opus_decoder, OPUS_SET_BANDWIDTH_REQUEST, OPUS_BANDWIDTH_NARROWBAND)
        self.pcm_buffer = (ctypes.c_short * (SAMPLES_PER_FRAME * AUDIO_CHANNELS))()

        self.output_stream = self.p_audio.open(
            format=self.p_audio.get_format_from_width(AUDIO_WIDTH),
            channels=AUDIO_CHANNELS,
            rate=AUDIO_SAMPLE_RATE,
            output=True,
            output_device_index=output_device_index
        )

        # --- TX (Transmitting) Audio Setup ---
        self.opus_encoder = OpusEncoder(sampling_frequency=AUDIO_SAMPLE_RATE, channels=AUDIO_CHANNELS,
                                        application='voip')
        opus_encoder_ctl(self.opus_encoder, OPUS_SET_BANDWIDTH_REQUEST, OPUS_BANDWIDTH_NARROWBAND)
        opus_encoder_ctl(self.opus_encoder, OPUS_SET_VBR_REQUEST, 1)
        
        self.encoded_buffer = (ctypes.c_ubyte * MAX_ENCODED_FRAME_SIZE)()

        self.input_stream = self.p_audio.open(
            format=self.p_audio.get_format_from_width(AUDIO_WIDTH),
            channels=AUDIO_CHANNELS,
            rate=AUDIO_SAMPLE_RATE,
            input=True,
            input_device_index=input_device_index,
            frames_per_buffer=BUFFER_SIZE,
            stream_callback=self._audio_callback
        )
        self.input_stream.stop_stream()

        # --- Serial Reader Thread ---
        self.reader_thread = threading.Thread(target=self._read_serial)
        self.reader_thread.daemon = True
        self.reader_thread.start()

    def _audio_callback(self, in_data, frame_count, time_info, status):
        pcm_pointer = ctypes.cast(in_data, ctypes.POINTER(ctypes.c_short))
        encoded_len = opus_encode(
            self.opus_encoder, pcm_pointer, SAMPLES_PER_FRAME,
            self.encoded_buffer, MAX_ENCODED_FRAME_SIZE
        )
        if encoded_len > 0:
            encoded_packet = ctypes.string_at(self.encoded_buffer, encoded_len)
            self._send_command(COMMAND_HOST_TX_AUDIO, encoded_packet)
        return (None, pyaudio.paContinue)

    def _read_serial(self):
        print("[INFO] Serial reader thread started.")
        buffer = b''
        while self.serial_port.is_open:
            try:
                data_in = self.serial_port.read(1024)
                if data_in:
                    buffer += data_in
                
                while True:
                    start_index = buffer.find(COMMAND_DELIMITER)
                    if start_index == -1: break
                    
                    header_start = start_index + len(COMMAND_DELIMITER)
                    if len(buffer) < header_start + 3: break
                        
                    command, payload_len = struct.unpack('<BH', buffer[header_start:header_start + 3])
                    packet_len = len(COMMAND_DELIMITER) + 3 + payload_len
                    if len(buffer) < start_index + packet_len: break

                    payload_start = header_start + 3
                    payload = buffer[payload_start:payload_start + payload_len]

                    self.process_command(command, payload)
                    
                    buffer = buffer[start_index + packet_len:]
            except Exception as e:
                print(f"\n[ERROR] Unhandled exception in serial reader thread: {e}")
                traceback.print_exc()
                break

    def process_command(self, command, payload):
        if self.handshake_state == HandshakeState.WAITING_FOR_HELLO:
            if command == COMMAND_HELLO:
                print("[INFO] HELLO received. Sending config.")
                self._send_command(COMMAND_HOST_CONFIG, b'\x01') # High power
                self.handshake_state = HandshakeState.WAITING_FOR_VERSION
        
        elif self.handshake_state == HandshakeState.WAITING_FOR_VERSION:
            if command == COMMAND_VERSION:
                print(f"[INFO] Version received: {payload.hex()}. Handshake complete.")
                self.handshake_state = HandshakeState.COMPLETE
                # Now that handshake is done, send the final initialization sequence.
                self.finish_initialization()
        
        elif self.handshake_state == HandshakeState.COMPLETE:
            if command == COMMAND_RX_AUDIO:
                if len(payload) < 2: return
                try:
                    payload_ctype = (ctypes.c_ubyte * len(payload)).from_buffer_copy(payload)
                    samples_decoded = opus_decode(
                        self.opus_decoder, payload_ctype, len(payload),
                        self.pcm_buffer, SAMPLES_PER_FRAME, 0
                    )
                    if samples_decoded > 0:
                        bytes_to_write = samples_decoded * AUDIO_CHANNELS * AUDIO_WIDTH
                        decoded_pcm = ctypes.string_at(self.pcm_buffer, bytes_to_write)
                        self.output_stream.write(decoded_pcm)
                except PyOggError as e:
                    print(f"    [!] Opus decoding error: {e}")

    def finish_initialization(self):
        print("[INFO] Finishing initialization...")
        # This sequence is critical and mirrors the Android app's logic.
        # 1. Set High/Low power state
        self._send_command(COMMAND_HOST_HL, b'\x01') # High power
        # 2. Set filters (default: all off)
        self._send_command(COMMAND_HOST_FILTERS, b'\x00')
        # 3. Set the initial frequency.
        self.set_frequency(self.initial_freq)
        # 4. Enable RSSI reporting (and audio forwarding)
        self._send_command(COMMAND_HOST_RSSI, b'\x01')

    def set_frequency(self, mhz_float):
        print(f"Setting frequency to {mhz_float} MHz")
        payload = struct.pack('<BffBBB', 0, mhz_float, mhz_float, 0, 1, 0)
        self._send_command(COMMAND_HOST_GROUP, payload)

    def ptt_on(self):
        self._send_command(COMMAND_HOST_PTT_DOWN)
        if not self.input_stream.is_active():
            self.input_stream.start_stream()

    def ptt_off(self):
        if self.input_stream.is_active():
            self.input_stream.stop_stream()
        self._send_command(COMMAND_HOST_PTT_UP)

    def _send_command(self, command, payload=b''):
        try:
            header = struct.pack('<4sBH', COMMAND_DELIMITER, command, len(payload))
            self.serial_port.write(header + payload)
        except serial.SerialException as e:
            print(f"Serial write error: {e}")

    def close(self):
        if self.input_stream.is_active(): self.input_stream.stop_stream()
        self.input_stream.close()
        self.output_stream.stop_stream()
        self.output_stream.close()
        self.p_audio.terminate()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("KV4P-HT Desktop Driver")
        self.driver = None
        self.config = self.load_config()
        self.p_audio = pyaudio.PyAudio()
        self.create_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def create_widgets(self):
        ttk.Label(self.root, text="COM Port:").grid(row=0, column=0, padx=5, pady=5, sticky='w')
        self.port_combo = ttk.Combobox(self.root, values=[p.device for p in serial.tools.list_ports.comports()])
        self.port_combo.set(self.config.get('com_port', ''))
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky='ew')

        self.input_devices = {info['name']: info['index'] for info in self.get_audio_devices('input')}
        self.output_devices = {info['name']: info['index'] for info in self.get_audio_devices('output')}

        ttk.Label(self.root, text="Audio Input:").grid(row=1, column=0, padx=5, pady=5, sticky='w')
        self.input_combo = ttk.Combobox(self.root, values=list(self.input_devices.keys()))
        self.input_combo.set(self.config.get('audio_input', ''))
        self.input_combo.grid(row=1, column=1, padx=5, pady=5, sticky='ew')

        ttk.Label(self.root, text="Audio Output:").grid(row=2, column=0, padx=5, pady=5, sticky='w')
        self.output_combo = ttk.Combobox(self.root, values=list(self.output_devices.keys()))
        self.output_combo.set(self.config.get('audio_output', ''))
        self.output_combo.grid(row=2, column=1, padx=5, pady=5, sticky='ew')

        self.connect_button = ttk.Button(self.root, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=2, rowspan=3, padx=5, pady=5, sticky='ns')

        ttk.Label(self.root, text="Frequency (MHz):").grid(row=3, column=0, padx=5, pady=5, sticky='w')
        self.freq_entry = ttk.Entry(self.root)
        self.freq_entry.insert(0, self.config.get('frequency', '146.520'))
        self.freq_entry.grid(row=3, column=1, padx=5, pady=5, sticky='ew')
        self.set_freq_button = ttk.Button(self.root, text="Set Freq", command=self.set_frequency_from_ui)
        self.set_freq_button.grid(row=3, column=2, padx=5, pady=5, sticky='ew')

        self.ptt_button = ttk.Button(self.root, text="PTT")
        self.ptt_button.grid(row=4, column=0, columnspan=3, padx=5, pady=10, sticky='ew')
        self.ptt_button.bind("<ButtonPress-1>", self.ptt_on)
        self.ptt_button.bind("<ButtonRelease-1>", self.ptt_off)

        self.root.grid_columnconfigure(1, weight=1)

    def get_audio_devices(self, kind='input'):
        devices = []
        for i in range(self.p_audio.get_device_count()):
            info = self.p_audio.get_device_info_by_index(i)
            if kind == 'input' and info['maxInputChannels'] > 0:
                devices.append(info)
            elif kind == 'output' and info['maxOutputChannels'] > 0:
                devices.append(info)
        return devices

    def connect(self):
        if self.driver:
            self.driver.close()
            self.driver = None
            self.connect_button.config(text="Connect")
            print("Disconnected.")
            return

        port = self.port_combo.get()
        input_dev_name = self.input_combo.get()
        output_dev_name = self.output_combo.get()

        if not all([port, input_dev_name, output_dev_name]):
            print("Error: Please select COM port and audio devices.")
            return

        try:
            input_idx = self.input_devices[input_dev_name]
            output_idx = self.output_devices[output_dev_name]
            initial_freq = float(self.freq_entry.get())
            self.driver = KV4P_Driver(port, 115200, input_idx, output_idx, initial_freq)
            self.connect_button.config(text="Disconnect")
            print(f"Connected to {port}")
        except Exception as e:
            print(f"Error connecting: {e}")
            traceback.print_exc()
            self.driver = None

    def set_frequency_from_ui(self):
        if self.driver and self.driver.handshake_state == HandshakeState.COMPLETE:
            try:
                freq = float(self.freq_entry.get())
                if 144.0 <= freq <= 148.0:
                    self.driver.set_frequency(freq)
                else:
                    print("Frequency must be between 144.000 and 148.000 MHz")
            except ValueError:
                print("Invalid frequency format")

    def ptt_on(self, event):
        if self.driver: self.driver.ptt_on()

    def ptt_off(self, event):
        if self.driver: self.driver.ptt_off()

    def load_config(self):
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    return json.load(f)
            except (json.JSONDecodeError, IOError):
                return {}
        return {}

    def save_config(self):
        config_data = {
            'com_port': self.port_combo.get(),
            'audio_input': self.input_combo.get(),
            'audio_output': self.output_combo.get(),
            'frequency': self.freq_entry.get()
        }
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(config_data, f, indent=4)
        except IOError as e:
            print(f"Error saving config: {e}")

    def on_closing(self):
        self.save_config()
        if self.driver:
            self.driver.close()
        self.p_audio.terminate()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
