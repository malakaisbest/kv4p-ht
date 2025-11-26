import sounddevice as sd
import numpy as np
import queue
import threading

class AudioEngine:
    def __init__(self, sample_rate=48000, frames_per_buffer=1920):
        self.sample_rate = sample_rate
        self.frames_per_buffer = frames_per_buffer
        self.input_device = None
        self.output_device = None
        self.input_stream = None
        self.output_stream = None
        self.playback_queue = queue.Queue()
        self.on_input_chunk = None
        self.lock = threading.Lock()

    def get_input_devices(self):
        return [device for device in sd.query_devices() if device['max_input_channels'] > 0]

    def get_output_devices(self):
        return [device for device in sd.query_devices() if device['max_output_channels'] > 0]

    def set_devices(self, input_device, output_device):
        self.input_device = input_device
        self.output_device = output_device

    def start_recording(self, on_chunk):
        with self.lock:
            if self.input_stream:
                return
            self.on_input_chunk = on_chunk
            self.input_stream = sd.InputStream(
                samplerate=self.sample_rate,
                channels=1,
                dtype='float32',
                device=self.input_device,
                callback=self._input_callback,
                blocksize=self.frames_per_buffer
            )
            self.input_stream.start()

    def stop_recording(self):
        with self.lock:
            if not self.input_stream:
                return
            self.input_stream.stop()
            self.input_stream.close()
            self.input_stream = None
            self.on_input_chunk = None

    def start_playback(self):
        with self.lock:
            if self.output_stream:
                return
            self.output_stream = sd.OutputStream(
                samplerate=self.sample_rate,
                channels=1,
                dtype='float32',
                device=self.output_device,
                callback=self._output_callback,
                blocksize=self.frames_per_buffer
            )
            self.output_stream.start()

    def stop_playback(self):
        with self.lock:
            if not self.output_stream:
                return
            self.output_stream.stop()
            self.output_stream.close()
            self.output_stream = None

    def enqueue_playback(self, data):
        samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        self.playback_queue.put(samples)

    def _input_callback(self, indata, frames, time, status):
        if status:
            print(status)
        if self.on_input_chunk:
            pcm_data = (indata * 32767).astype(np.int16).tobytes()
            self.on_input_chunk(pcm_data)

    def _output_callback(self, outdata, frames, time, status):
        if status:
            print(status)
        try:
            data = self.playback_queue.get_nowait()
            if len(data) < len(outdata):
                outdata[:len(data)] = data.reshape(-1, 1)
                outdata[len(data):] = 0
            else:
                outdata[:] = data[:len(outdata)].reshape(-1, 1)
        except queue.Empty:
            outdata.fill(0)
