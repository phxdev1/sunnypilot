"""Canopy: Leaf blind spot, read off the car bus (bus 0).

0x3F0 byte0 bit0 is set whenever either rear side radar warns (20 Hz). Decoded from
Mark's own drives. Which side is not decoded yet: 0x3F1 byte0 with bits 1-3 clear shows
up only during right-side warnings, but only for a moment inside each warning, so it can't
clear a side on its own.

Until the side is proven, both sides read occupied whenever 0x3F0 warns, and both read
occupied if 0x3F0 goes quiet (unknown counts as occupied). right_hint is kept for logging.
"""
ANY_ADDR = 0x3F0
RIGHT_HINT_ADDR = 0x3F1
STALE_NS = 300_000_000  # six missed 20 Hz frames


class LeafBlindspot:
  def __init__(self):
    self.any_on = False
    self.t_any: int | None = None
    self.t_now = 0
    self.right_hint = False  # seen during the current warning

  def update(self, can_list) -> None:
    for nanos, frames in can_list:
      self.t_now = max(self.t_now, nanos)
      for addr, dat, src in frames:
        if src != 0 or len(dat) < 1:
          continue
        if addr == ANY_ADDR:
          self.any_on = bool(dat[0] & 1)
          self.t_any = nanos
          if not self.any_on:
            self.right_hint = False
        elif addr == RIGHT_HINT_ADDR and self.any_on and dat[0] != 0xEF and (dat[0] & 0x0E) == 0:
          self.right_hint = True

  def sides(self) -> tuple[bool, bool]:
    """(left_occupied, right_occupied)"""
    if self.t_any is None or self.t_now - self.t_any > STALE_NS:
      return True, True
    return self.any_on, self.any_on
