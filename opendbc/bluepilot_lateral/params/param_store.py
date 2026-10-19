"""
BluePilot: the UI params the fork publishes in CarControlSP.params, by key. The lateral code reads
every setting through params/manifest.py's value(), which parses the published text and takes the
manifest default for a key that is missing, empty or unparsable. A fork with the #6 change
publishes every lateral key anyway, using its params_keys.h default for one never written.
"""


class ParamStore:
  def __init__(self, cc_sp_params):
    self._values = {p.key: p.value for p in cc_sp_params}

  @classmethod
  def from_cc_sp(cls, CC_SP):
    return cls(CC_SP.params)

  def get(self, key):
    """The published value as text, or None if the fork didn't publish the key."""
    v = self._values.get(key)
    return v.decode("utf-8", errors="replace") if isinstance(v, (bytes, bytearray)) else v
