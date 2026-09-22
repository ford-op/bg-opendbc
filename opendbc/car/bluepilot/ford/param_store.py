"""
BluePilot: the per-frame params in CarControlSP.params, behind the two calls the lateral code
already makes on openpilot's Params (get / get_bool). Missing keys raise, like Params does, so the
callers' existing defaults apply.
"""


class ParamStore:
  def __init__(self, cc_sp_params):
    self._values = {p.key: p.value for p in cc_sp_params}

  @classmethod
  def from_cc_sp(cls, CC_SP):
    return cls(CC_SP.params)

  def get(self, key, return_default=False):
    if key not in self._values:
      if return_default:
        return None
      raise KeyError(key)
    v = self._values[key]
    return v.decode("utf-8", errors="replace") if isinstance(v, (bytes, bytearray)) else v

  def get_bool(self, key):
    v = self.get(key)
    return v in ("1", "true", "True")
