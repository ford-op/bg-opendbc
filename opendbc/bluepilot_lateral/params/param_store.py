"""
BluePilot: the UI params the fork publishes in CarControlSP.params, behind the two calls the
lateral code already makes on openpilot's Params.

Semantics match Params for a key the fork did not publish:
  get_bool(key)                    -> False   (Params: unset bool reads as False)
  get(key)                         -> raises
  get(key, return_default=True)    -> None

The lateral code reads every setting through params/manifest.py's value(), which takes the
manifest default for a key that is missing, empty or unparsable. A fork with the #6 change
publishes every lateral key anyway, using its params_keys.h default for one never written.
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
    v = self._values.get(key)
    if v is None:
      return False
    if isinstance(v, (bytes, bytearray)):
      v = v.decode("utf-8", errors="replace")
    return v == "1"
