import mido
import logging

class Midi:
    def __init__(self, port):
        self.port = port
        try:
            self.midi = mido.open_input(port)
            self.midi_out = mido.open_output(port)
        except Exception as e:
            logging.error(f"MIDI open error: {e}")
            return None
    
    def __enter__(self):
        return self
    
    def clear_events(self):
        for msg in self.midi.iter_pending():
            pass
    
    def set_led_state(self, note, state):
        if state:
            v = 127
        else:
            v = 0
        logging.debug(f"setting led {note}, {state}")
        m = mido.Message('note_on', channel=0, note=note, velocity=v)
        self.midi_out.send(m)

    def __exit__(self, exc_type, exc_value, traceback):
        # I think this one cleans up after itself?
        pass
