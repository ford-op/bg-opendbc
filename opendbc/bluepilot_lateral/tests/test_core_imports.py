import ast
import unittest
from pathlib import Path

LIBRARY = Path(__file__).resolve().parents[1]
# the host-independent parts: only hosts/ (an adapter per host) may know a host's types
CORE_DIRS = ("core", "can", "params", "safety")
FORBIDDEN = ("opendbc.bluepilot_lateral.hosts", "opendbc.sunnypilot", "openpilot", "cereal")


def _imports(path):
  tree = ast.parse(path.read_text())
  for node in ast.walk(tree):
    if isinstance(node, ast.Import):
      yield from (alias.name for alias in node.names)
    elif isinstance(node, ast.ImportFrom):
      # relative imports count too (from ..hosts import x), and so does importing hosts by name
      # (from opendbc.bluepilot_lateral import hosts, from .. import hosts)
      module = ("." * node.level) + (node.module or "")
      yield module
      yield from (f"{module.rstrip('.')}.{alias.name}" if node.module else module + alias.name for alias in node.names)


class TestCoreImports(unittest.TestCase):
  def test_core_imports_no_host(self):
    files = [p for d in CORE_DIRS for p in sorted((LIBRARY / d).rglob("*.py"))]
    self.assertTrue(files)
    for path in files:
      for module in _imports(path):
        rel = path.relative_to(LIBRARY)
        self.assertFalse(module.startswith(FORBIDDEN), f"{rel} imports {module}")
        self.assertFalse(module.startswith(".") and "hosts" in module, f"{rel} imports {module}")


if __name__ == "__main__":
  unittest.main()
