import json
import os

class Config:
    def __init__(self, file_path='config.json'):
        self.file_path = file_path
        self.settings = {
            'audio_input_device': None,
            'audio_output_device': None,
            'squelch_level': 0,
            'rx_frequency': '146.520',
            'tx_frequency': '146.520',
            'tone': 'None',
            'band_mode': 'Narrow',
            'power': 'High',
            'ptt_hotkey': 'space'
        }
        self.load()

    def load(self):
        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as f:
                self.settings.update(json.load(f))

    def save(self):
        with open(self.file_path, 'w') as f:
            json.dump(self.settings, f, indent=4)

    def get(self, key, default=None):
        return self.settings.get(key, default)

    def set(self, key, value):
        self.settings[key] = value
        self.save()
