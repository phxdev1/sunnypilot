"""Canopy prompt sound slots.

Each Canopy prompt has its own sound slot. soundd plays /data/canopy/sounds/<slot>.wav when that
file is there and valid, and the slot's stock default otherwise. The files are made on Forge
(ElevenLabs sound effects or a short line in Zia's voice), leveled and trimmed there, and copied
here while the car is parked. Nothing is fetched or generated on the car.

openpilot's own safety sounds (engage, disengage, take control, distracted driver) are not slots
and never change, so they stay recognizable.

Valid file: WAV, mono, 16 bit, 48 kHz, at most MAX_SECONDS long. Anything else falls back to the
default, so a bad file can only ever cost a custom sound, never a silent safety prompt.
"""
import os
import wave

import numpy as np

SOUND_DIR = os.environ.get("CANOPY_SOUND_DIR", "/data/canopy/sounds")
SAMPLE_RATE = 48000
MAX_SECONDS = 4.0

# slot name -> (AudibleAlertSP field name, stock default file in assets/sounds, or None for silent)
SLOTS = {
  "red_light": ("canopyRedLight", "warning.wav"),
  "green_light": ("canopyGreenLight", "warning.wav"),
  "ramp_meter": ("canopyRampMeter", "warning.wav"),
  "move_over": ("canopyMoveOver", "warning.wav"),
  "lane_drop": ("canopyLaneDrop", "warning.wav"),
  "coast": ("canopyCoast", "prompt_single_low.wav"),
  # Garage mode start and end: a chime, then Zia "Garage mode." / "Garage mode off." Stock fallback is
  # a soft single chime so the start and end are never silent.
  "garage": ("canopyGarage", "prompt_single_high.wav"),
  "garage_end": ("canopyGarageEnd", "prompt_single_low.wav"),
  # Spoken lane cues: a chime, then Zia saying which way (made on Forge as one file). The stock
  # fallback is only a chime, so these are worth little until the spoken file is on the comma.
  "merge_left": ("canopyMergeLeft", "prompt_single_high.wav"),
  "merge_right": ("canopyMergeRight", "prompt_single_high.wav"),
  "hand_back": ("canopyHandBack", "warning.wav"),
  "blinker_off": ("canopyBlinkerOff", "prompt_single_low.wav"),  # lane change done, blinker still on
  # A right change starting while the blind spot reads occupied: a loud chime, then Zia "Blind spot."
  # It warns only; the change goes on (he checks his right). Stock fallback is the loud warning chime.
  "right_blindspot": ("canopyRightBlindspot", "warning.wav"),
  # A trouble spot ahead (he has taken over there before): a chime, then Zia "Heads up, tricky spot."
  "trouble_spot": ("canopyTroubleSpot", "prompt_single_high.wav"),
  # A corner where he keeps taking over: a chime, then Zia "Your turn.", and steering hands that turn back.
  "your_turn": ("canopyCornerHandBack", "prompt_single_high.wav"),
}


def read_wav(path: str, max_s: float | None = MAX_SECONDS) -> np.ndarray | None:
  """Float samples in -1..1, or None if the file is missing or not in the one format soundd plays."""
  try:
    with wave.open(path, "r") as w:
      if w.getnchannels() != 1 or w.getsampwidth() != 2 or w.getframerate() != SAMPLE_RATE:
        return None
      n = w.getnframes()
      if n == 0 or (max_s is not None and n > max_s * SAMPLE_RATE):
        return None
      return np.frombuffer(w.readframes(n), dtype=np.int16).astype(np.float32) / (2**16 / 2)
  except (OSError, EOFError, wave.Error):
    return None


def dir_mtime() -> float | None:
  try:
    return max([os.path.getmtime(SOUND_DIR)] + [os.path.getmtime(os.path.join(SOUND_DIR, f))
                                                 for f in os.listdir(SOUND_DIR) if f.endswith(".wav")])
  except OSError:
    return None


def load_slot(slot: str, assets_dir: str) -> np.ndarray:
  """The slot's custom sound if valid, else its default, else a short silence."""
  custom = read_wav(os.path.join(SOUND_DIR, f"{slot}.wav"))
  if custom is not None:
    return custom
  default = SLOTS[slot][1]
  if default:
    d = read_wav(os.path.join(assets_dir, default), max_s=None)
    if d is not None:
      return d
  return np.zeros(SAMPLE_RATE // 10, dtype=np.float32)
