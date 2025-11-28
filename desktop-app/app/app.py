import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import sounddevice as sd
from serial.tools import list_ports
import threading
import logging
import wave
from datetime import datetime
import time
import os
from .config import Config
from .audio import AudioEngine
from .radio_controller import RadioController, RadioControllerError

TONE_MAPPINGS = {
    "None": 0, "67.0": 1, "71.9": 2, "74.4": 3, "77.0": 4, "79.7": 5,
    "82.5": 6, "85.4": 7, "88.5": 8, "91.5": 9, "94.8": 10, "97.4": 11,
    "100.0": 12, "103.5": 13, "107.2": 14, "110.9": 15, "114.8": 16,
    "118.8": 17, "123.0": 18, "127.3": 19, "131.8": 20, "136.5": 21,
    "141.3": 22, "146.2": 23, "151.4": 24, "156.7": 25, "162.2": 26,
    "167.9": 27, "173.8": 28, "179.9": 29, "186.2": 30, "192.8": 31,
    "203.5": 32, "210.7": 33, "218.1": 34, "225.7": 35, "233.6": 36,
    "241.8": 37, "250.3": 38
}

class RadioApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("KV4P Radio Desktop Controller ~ Malaka Wickremasinghe ~")
        self.geometry("500x700")

        self.config = Config('config.json')
        self.audio_engine = AudioEngine()
        self.controller = None
        self.is_transmitting = False
        self.is_recording = False
        self.wave_file = None
        self.audio_file_path = None
        self.is_playing_file = False
        self.play_thread = None
        self.stop_play_event = threading.Event()

        self._build_ui()
        self.load_settings()
        self.populate_devices()
        self.update_tx_controls_state()

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.bind('<KeyPress>', self.on_key_press)
        self.bind('<KeyRelease>', self.on_key_release)

    def _build_ui(self):
        main_frame = ttk.Frame(self, padding="10")
        main_frame.pack(fill="both", expand=True)

        # Status Frame
        status_frame = ttk.LabelFrame(main_frame, text="Status")
        status_frame.pack(fill="x", expand=True, pady=5)
        
        ttk.Label(status_frame, text="RSSI:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.rssi_var = tk.IntVar()
        self.rssi_bar = ttk.Progressbar(status_frame, variable=self.rssi_var, maximum=255)
        self.rssi_bar.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.rssi_label = ttk.Label(status_frame, text="0")
        self.rssi_label.grid(row=0, column=2, padx=5, pady=5)
        status_frame.columnconfigure(1, weight=1)

        # Audio Settings
        audio_frame = ttk.LabelFrame(main_frame, text="Audio Devices")
        audio_frame.pack(fill="x", expand=True, pady=5)

        ttk.Label(audio_frame, text="Input:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.input_device_var = tk.StringVar()
        self.input_device_menu = ttk.Combobox(audio_frame, textvariable=self.input_device_var, state="readonly")
        self.input_device_menu.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.input_device_menu.bind("<<ComboboxSelected>>", self.on_device_change)

        ttk.Label(audio_frame, text="Output:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.output_device_var = tk.StringVar()
        self.output_device_menu = ttk.Combobox(audio_frame, textvariable=self.output_device_var, state="readonly")
        self.output_device_menu.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        self.output_device_menu.bind("<<ComboboxSelected>>", self.on_device_change)
        
        audio_frame.columnconfigure(1, weight=1)

        # Radio Settings
        radio_frame = ttk.LabelFrame(main_frame, text="Radio Settings")
        radio_frame.pack(fill="x", expand=True, pady=5)

        ttk.Label(radio_frame, text="RX Frequency:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.rx_frequency_var = tk.StringVar()
        self.rx_frequency_entry = ttk.Entry(radio_frame, textvariable=self.rx_frequency_var)
        self.rx_frequency_entry.grid(row=0, column=1, columnspan=2, padx=5, pady=5, sticky="ew")
        self.rx_frequency_entry.bind("<FocusOut>", self.on_frequency_change)

        ttk.Label(radio_frame, text="TX Frequency:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.tx_frequency_var = tk.StringVar()
        self.tx_frequency_entry = ttk.Entry(radio_frame, textvariable=self.tx_frequency_var)
        self.tx_frequency_entry.grid(row=1, column=1, columnspan=2, padx=5, pady=5, sticky="ew")
        self.tx_frequency_entry.bind("<FocusOut>", self.on_frequency_change)

        ttk.Label(radio_frame, text="Tone:").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        self.tone_var = tk.StringVar()
        self.tone_menu = ttk.Combobox(radio_frame, textvariable=self.tone_var, values=list(TONE_MAPPINGS.keys()), state="readonly")
        self.tone_menu.grid(row=2, column=1, columnspan=2, padx=5, pady=5, sticky="ew")
        self.tone_menu.bind("<<ComboboxSelected>>", self.on_tone_change)

        ttk.Label(radio_frame, text="Squelch:").grid(row=3, column=0, padx=5, pady=5, sticky="w")
        self.squelch_var = tk.IntVar()
        self.squelch_scale = ttk.Scale(radio_frame, from_=0, to=9, orient="horizontal", variable=self.squelch_var, command=self.on_squelch_change)
        self.squelch_scale.grid(row=3, column=1, padx=5, pady=5, sticky="ew")
        self.squelch_label = ttk.Label(radio_frame, text="0")
        self.squelch_label.grid(row=3, column=2, padx=5, pady=5)
        
        ttk.Label(radio_frame, text="Band:").grid(row=4, column=0, padx=5, pady=5, sticky="w")
        band_frame = ttk.Frame(radio_frame)
        band_frame.grid(row=4, column=1, columnspan=2, sticky="w")
        self.band_selection_var = tk.StringVar()
        ttk.Radiobutton(band_frame, text="Wide Band", variable=self.band_selection_var, value="Wide", command=self.on_band_change).pack(side="left")
        ttk.Radiobutton(band_frame, text="Narrow Band", variable=self.band_selection_var, value="Narrow", command=self.on_band_change).pack(side="left")

        ttk.Label(radio_frame, text="Tx Power:").grid(row=5, column=0, padx=5, pady=5, sticky="w")
        power_frame = ttk.Frame(radio_frame)
        power_frame.grid(row=5, column=1, columnspan=2, sticky="w")
        self.power_var = tk.StringVar()
        ttk.Radiobutton(power_frame, text="High", variable=self.power_var, value="High", command=self.on_power_change).pack(side="left")
        ttk.Radiobutton(power_frame, text="Low", variable=self.power_var, value="Low", command=self.on_power_change).pack(side="left")
        ttk.Radiobutton(power_frame, text="No TX", variable=self.power_var, value="No TX", command=self.on_power_change).pack(side="left")

        radio_frame.columnconfigure(1, weight=1)

        # Audio File Player
        player_frame = ttk.LabelFrame(main_frame, text="Audio File Player")
        player_frame.pack(fill="x", expand=True, pady=5)

        self.browse_button = ttk.Button(player_frame, text="Browse", command=self.browse_file)
        self.browse_button.grid(row=0, column=0, padx=5, pady=5)
        self.file_label = ttk.Label(player_frame, text="No file selected")
        self.file_label.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        self.play_button = ttk.Button(player_frame, text="Play Audio File", command=self.toggle_play_file)
        self.play_button.grid(row=1, column=0, padx=5, pady=5)
        self.loop_var = tk.BooleanVar()
        self.loop_check = ttk.Checkbutton(player_frame, text="Loop", variable=self.loop_var)
        self.loop_check.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        # Connection
        conn_frame = ttk.LabelFrame(main_frame, text="Connection")
        conn_frame.pack(fill="x", expand=True, pady=5)

        ttk.Label(conn_frame, text="Serial Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.port_var = tk.StringVar()
        self.port_menu = ttk.Combobox(conn_frame, textvariable=self.port_var, state="readonly")
        self.port_menu.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.port_menu.bind("<<ComboboxSelected>>", self.on_port_change)
        
        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=2, padx=5, pady=5)

        conn_frame.columnconfigure(1, weight=1)

        # Controls
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill="x", expand=True, pady=5)

        self.record_button = ttk.Button(control_frame, text="Record", command=self.toggle_recording)
        self.record_button.pack(side="left", fill="x", expand=True, padx=(0, 5))

        self.ptt_button = ttk.Button(control_frame, text="Push to Talk (Space)")
        self.ptt_button.pack(side="left", fill="x", expand=True, padx=(5, 0))
        self.ptt_button.bind("<ButtonPress-1>", lambda e: self.start_transmit())
        self.ptt_button.bind("<ButtonRelease-1>", lambda e: self.stop_transmit())

    def populate_devices(self):
        self.input_devices = {d['name']: i for i, d in enumerate(self.audio_engine.get_input_devices())}
        self.output_devices = {d['name']: i for i, d in enumerate(self.audio_engine.get_output_devices())}
        self.input_device_menu['values'] = list(self.input_devices.keys())
        self.output_device_menu['values'] = list(self.output_devices.keys())
        self.ports = {p.device: p.description for p in list_ports.comports()}
        self.port_menu['values'] = list(self.ports.keys())

    def load_settings(self):
        self.input_device_var.set(self.config.get('audio_input_device', ''))
        self.output_device_var.set(self.config.get('audio_output_device', ''))
        squelch_val = self.config.get('squelch_level', 0)
        self.squelch_var.set(squelch_val)
        self.squelch_label.config(text=str(squelch_val))
        self.rx_frequency_var.set(self.config.get('rx_frequency', '146.520'))
        self.tx_frequency_var.set(self.config.get('tx_frequency', '146.520'))
        self.tone_var.set(self.config.get('tone', 'None'))
        self.band_selection_var.set(self.config.get('band_mode', 'Narrow'))
        self.power_var.set(self.config.get('power', 'High'))
        self.port_var.set(self.config.get('serial_port', ''))

    def on_device_change(self, event):
        self.config.set('audio_input_device', self.input_device_var.get())
        self.config.set('audio_output_device', self.output_device_var.get())
        in_dev_idx = self.input_devices.get(self.input_device_var.get())
        out_dev_idx = self.output_devices.get(self.output_device_var.get())
        self.audio_engine.set_devices(in_dev_idx, out_dev_idx)

    def on_port_change(self, event):
        self.config.set('serial_port', self.port_var.get())

    def on_frequency_change(self, event):
        self.config.set('rx_frequency', self.rx_frequency_var.get())
        self.config.set('tx_frequency', self.tx_frequency_var.get())
        self.apply_radio_settings()

    def on_tone_change(self, event):
        self.config.set('tone', self.tone_var.get())
        self.apply_radio_settings()

    def on_band_change(self):
        self.config.set('band_mode', self.band_selection_var.get())
        self.apply_radio_settings()

    def on_power_change(self):
        self.config.set('power', self.power_var.get())
        self.update_tx_controls_state()
        self.apply_radio_settings()

    def on_squelch_change(self, value):
        squelch_level = int(float(value))
        self.squelch_label.config(text=str(squelch_level))
        self.config.set('squelch_level', squelch_level)
        self.apply_radio_settings()

    def update_rssi(self, value):
        self.rssi_var.set(value)
        self.rssi_label.config(text=str(value))

    def on_physical_ptt_change(self, is_down):
        if is_down:
            self.start_transmit()
        else:
            self.stop_transmit()

    def update_tx_controls_state(self):
        state = "disabled" if self.power_var.get() == "No TX" else "normal"
        self.tx_frequency_entry.config(state=state)
        self.play_button.config(state=state)
        self.ptt_button.config(state=state)

    def toggle_connection(self):
        if self.controller and self.controller.is_connected:
            self.disconnect()
        else:
            threading.Thread(target=self.connect, daemon=True).start()

    def connect(self):
        port = self.port_var.get()
        if not port:
            self.after(0, lambda: messagebox.showerror("Error", "Please select a serial port."))
            return

        self.after(0, lambda: self.connect_button.config(text="Connecting...", state="disabled"))
        
        try:
            self.controller = RadioController(port)
            self.controller.open_connection()
            self.controller.initialize()
            self.controller.start_rx_mode()
            self.controller.add_audio_listener(self.audio_engine.enqueue_playback)
            self.controller.add_rssi_listener(lambda value: self.after(0, self.update_rssi, value))
            self.controller.add_ptt_state_listener(lambda is_down: self.after(0, self.on_physical_ptt_change, is_down))
            self.audio_engine.start_playback()
            self.apply_radio_settings()
            self.after(0, lambda: self.connect_button.config(text="Disconnect", state="enabled"))
        except RadioControllerError as e:
            self.after(0, lambda: messagebox.showerror("Connection Error", str(e)))
            self.after(0, lambda: self.connect_button.config(text="Connect", state="enabled"))
            self.controller = None

    def disconnect(self):
        if self.is_playing_file:
            self.stop_play_event.set()
        if self.is_recording:
            self.toggle_recording()
        if self.controller:
            self.controller.close_connection()
            self.controller = None
        self.audio_engine.stop_playback()
        self.connect_button.config(text="Connect")
        self.update_rssi(0)

    def apply_radio_settings(self):
        if self.controller and self.controller.is_connected:
            try:
                power_mode = self.power_var.get()
                high_power = (power_mode == "High")
                self.controller.set_power(high_power)

                tone_name = self.tone_var.get()
                tone_value = TONE_MAPPINGS.get(tone_name, 0)
                wideband = self.band_selection_var.get() == "Wide"
                
                self.controller.tune_to_frequency(
                    rx_frequency=self.rx_frequency_var.get(),
                    tx_frequency=self.tx_frequency_var.get(),
                    tone=tone_value,
                    squelch_level=self.squelch_var.get(),
                    wideband=wideband
                )
            except RadioControllerError as e:
                self.after(0, lambda: messagebox.showerror("Error", f"Failed to apply settings: {e}"))

    def start_transmit(self):
        if self.power_var.get() == "No TX" or self.is_playing_file or not self.controller or not self.controller.is_connected or self.is_transmitting:
            return
        self.is_transmitting = True
        self.ptt_button.state(['pressed'])
        self.controller.start_tx_mode()
        self.audio_engine.start_recording(self.controller.send_audio_data)

    def stop_transmit(self):
        if not self.is_transmitting:
            return
        self.is_transmitting = False
        self.ptt_button.state(['!pressed'])
        self.audio_engine.stop_recording()
        if self.controller:
            self.controller.end_tx_mode()

    def toggle_recording(self):
        if not self.controller or not self.controller.is_connected:
            messagebox.showinfo("Info", "Connect to the radio to start recording.")
            return

        self.is_recording = not self.is_recording
        if self.is_recording:
            self.record_button.config(text="Stop Recording")
            filename = f"recording_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.wav"
            self.wave_file = wave.open(filename, 'wb')
            self.wave_file.setnchannels(1)
            self.wave_file.setsampwidth(2) # 16-bit PCM
            self.wave_file.setframerate(self.audio_engine.sample_rate)
            self.controller.add_audio_listener(self._write_to_wav_file)
        else:
            self.record_button.config(text="Record")
            if self.wave_file:
                self.wave_file.close()
                self.wave_file = None
            self.controller.remove_audio_listener(self._write_to_wav_file)

    def _write_to_wav_file(self, audio_data):
        if self.wave_file:
            self.wave_file.writeframes(audio_data)

    def browse_file(self):
        path = filedialog.askopenfilename(filetypes=[("WAV files", "*.wav")])
        if path:
            self.audio_file_path = path
            self.file_label.config(text=os.path.basename(path))

    def toggle_play_file(self):
        if self.is_playing_file:
            self.stop_play_event.set()
            self.play_button.config(text="Stopping...", state="disabled")
        else:
            if not self.audio_file_path:
                messagebox.showinfo("Info", "Please select an audio file first.")
                return
            if not self.controller or not self.controller.is_connected:
                messagebox.showinfo("Info", "Connect to the radio to play an audio file.")
                return
            
            self.is_playing_file = True
            self.play_button.config(text="Stop")
            self.ptt_button.config(state="disabled")
            self.stop_play_event.clear()
            self.play_thread = threading.Thread(target=self._play_audio_file, daemon=True)
            self.play_thread.start()

    def _play_audio_file(self):
        chunk_size = 1920 * 2
        try:
            while not self.stop_play_event.is_set():
                with wave.open(self.audio_file_path, 'rb') as wf:
                    if wf.getframerate() != self.audio_engine.sample_rate or wf.getnchannels() != 1 or wf.getsampwidth() != 2:
                        self.after(0, lambda: messagebox.showerror("Error", "Audio file must be 48kHz, 16-bit, mono WAV."))
                        break
                    
                    self.controller.start_tx_mode()
                    
                    data = wf.readframes(chunk_size // 2)
                    while data and not self.stop_play_event.is_set():
                        self.controller.send_audio_data(data)
                        time.sleep(0.040)
                        data = wf.readframes(chunk_size // 2)

                if not self.loop_var.get() or self.stop_play_event.is_set():
                    break
        except Exception as e:
            self.after(0, lambda: messagebox.showerror("Error", f"Failed to play audio file: {e}"))
        finally:
            if self.controller:
                self.controller.end_tx_mode()
            self.after(0, self._playback_finished)

    def _playback_finished(self):
        self.is_playing_file = False
        self.play_button.config(text="Play Audio File", state="enabled")
        self.ptt_button.config(state="!disabled")
        if self.play_thread:
            self.play_thread = None

    def on_key_press(self, event):
        if event.keysym == 'space' and not self.is_transmitting:
            self.start_transmit()

    def on_key_release(self, event):
        if event.keysym == 'space' and self.is_transmitting:
            self.stop_transmit()

    def on_close(self):
        self.disconnect()
        self.destroy()

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
    app = RadioApp()
    app.mainloop()
