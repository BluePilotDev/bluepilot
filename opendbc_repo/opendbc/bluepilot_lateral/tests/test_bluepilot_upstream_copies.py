"""BluePilot carries near-copies of these upstream functions. An upstream change to one of them syncs
in without a conflict and never reaches the copy, so each upstream body is pinned here by a hash
that ignores whitespace, comments and docstrings. When upstream changes one, this fails on the sync
PR: the PR that resolves it ports what applies to the BluePilot copy and updates the hash (#47)."""
import ast
import hashlib
import io
import re
import tokenize
import unittest
from pathlib import Path

from . import host

# the host's opendbc package (opendbc, iqdbc): the one the library is mounted in
HOST = Path(__file__).resolve().parents[2]

# per host (tests/host.py NAME; sunnypilot and StarPilot share the package name opendbc): (upstream file,
# function) -> hash of its normalized body; the copy is named in the comment
PINNED = {
  "sunnypilot": {  # sunnypilot/opendbc
    ("safety/lateral.h", "steer_curvature_cmd_checks"): "9b13985a8d01e88b",  # ford_bp_curvature_cmd_checks, bluepilot_lateral/safety/lateral.h
    ("car/ford/fordcan.py", "create_lka_msg"): "c7e38e4f0215dcf4",           # bluepilot_lateral/can/fordcan_ext.py
    ("car/ford/fordcan.py", "create_lat_ctl_msg"): "f00a87f31389cb91",       # bluepilot_lateral/can/fordcan_ext.py
    ("car/ford/fordcan.py", "create_lat_ctl2_msg"): "1653ca5622eccc77",      # bluepilot_lateral/can/fordcan_ext.py
  },
  "iqpilot": {  # IQ.Pilot: Ford on the native-curvature names; its packer leaves the LMC2 checksum to fordcan.py
    ("safety/lateral.h", "steer_native_curvature_cmd_checks"): "adb2981b8d1b69ac",  # ford_bp_curvature_cmd_checks, bluepilot_lateral/safety/lateral.h
    ("car/ford/fordcan.py", "create_lka_msg"): "c7e38e4f0215dcf4",                  # bluepilot_lateral/can/fordcan_ext.py
    ("car/ford/fordcan.py", "create_lat_ctl_msg"): "f00a87f31389cb91",              # bluepilot_lateral/can/fordcan_ext.py
    ("car/ford/fordcan.py", "create_lat_ctl2_msg"): "4147e71aa7c6c51f",             # bluepilot_lateral/can/fordcan_ext.py
  },
  # StarPilot: the lateral checks gate on lateral_controls_allowed() (its AOL; BP_HOST_LAT_ALLOWED), and its
  # Transit adds active-LKA arguments to create_lka_msg and a stock-LMC passthrough to create_lat_ctl_msg, neither
  # used on the cars BluePilot lateral runs on (hosts/starpilot/safety_param.py)
  "starpilot": {
    ("safety/lateral.h", "steer_curvature_cmd_checks"): "bac2b4c0de5b2a97",  # ford_bp_curvature_cmd_checks, bluepilot_lateral/safety/lateral.h
    ("car/ford/fordcan.py", "create_lka_msg"): "a3876bd67e579195",           # bluepilot_lateral/can/fordcan_ext.py
    ("car/ford/fordcan.py", "create_lat_ctl_msg"): "59eec05e601ac97e",       # bluepilot_lateral/can/fordcan_ext.py
    ("car/ford/fordcan.py", "create_lat_ctl2_msg"): "1653ca5622eccc77",      # bluepilot_lateral/can/fordcan_ext.py
  },
}


def c_function(src: str, name: str) -> str:
  """The definition of a top-level C function (not a prototype or a call), comments and whitespace removed."""
  src = re.sub(r"/\*.*?\*/|//[^\n]*", "", src, flags=re.S)
  m = re.search(rf"^\w[^\n;]*\b{name}\([^;{{]*\{{", src, re.M)  # starts in column 0, ends in "{"
  assert m, f"{name} not defined"
  depth = 0
  for i in range(m.end() - 1, len(src)):
    depth += {"{": 1, "}": -1}.get(src[i], 0)
    if depth == 0:
      return re.sub(r"\s+", "", src[m.start():i + 1])
  raise AssertionError(f"{name}: unbalanced braces")


def py_function(src: str, name: str) -> str:
  """A top-level Python function as its token strings, without comments, layout or docstring."""
  func = next(n for n in ast.parse(src).body if isinstance(n, ast.FunctionDef | ast.AsyncFunctionDef) and n.name == name)
  doc = func.body[0]
  doc_line = doc.lineno if isinstance(doc, ast.Expr) and isinstance(doc.value, ast.Constant) and isinstance(doc.value.value, str) else None
  lines = src.splitlines(keepends=True)[func.lineno - 1:func.end_lineno]
  skip = {tokenize.COMMENT, tokenize.NL, tokenize.NEWLINE, tokenize.INDENT, tokenize.DEDENT, tokenize.ENDMARKER}
  toks = [t for t in tokenize.generate_tokens(io.StringIO("".join(lines)).readline)
          if t.type not in skip and not (t.type == tokenize.STRING and t.start[0] + func.lineno - 1 == doc_line)]
  return " ".join(t.string for t in toks)


def body_hash(path: str, name: str) -> str:
  src = (HOST / path).read_text()
  body = c_function(src, name) if path.endswith(".h") else py_function(src, name)
  return hashlib.sha256(body.encode()).hexdigest()[:16]


class TestBluePilotUpstreamCopies(unittest.TestCase):
  def test_upstream_bodies_unchanged(self):
    if host.NAME not in PINNED:
      self.skipTest(f"no pins for host {host.NAME}: add its upstream hashes to PINNED")
    for (path, name), pinned in PINNED[host.NAME].items():
      with self.subTest(function=name):
        self.assertEqual(body_hash(path, name), pinned,
                         f"upstream {name} ({path}) changed: port what applies to the BluePilot copy, then update the hash")

  def test_normalization_ignores_layout_but_not_code(self):
    c = "int f(int a) { return a; }\n"
    self.assertEqual(c_function(c, "f"), c_function("int f(int a)\n{\n  // note\n  return a; /* x */\n}\n", "f"))
    self.assertNotEqual(c_function(c, "f"), c_function("int f(int a) { return a + 1; }\n", "f"))
    self.assertEqual(c_function("int f(int a);\n" + c, "f"), c_function(c, "f"))  # prototype skipped
    self.assertEqual(c_function("void g(void) {\n  if (f(1)) {\n  }\n}\n" + c, "f"), c_function(c, "f"))  # call skipped
    self.assertEqual(c_function("int f(int a) {\n  // a } in a comment\n  return a;\n}\n", "f"), c_function(c, "f"))

    py = 'def f(a):\n  """doc"""\n  return a\n'
    self.assertEqual(py_function(py, "f"), py_function('def f(a):\n  """other doc"""\n\n  return a  # note\n', "f"))
    self.assertNotEqual(py_function(py, "f"), py_function("def f(a):\n  return a + 1\n", "f"))
    self.assertEqual(py_function(py + "\ndef g():\n  pass\n", "f"), py_function(py, "f"))  # stops at the next def
    self.assertEqual(py_function('def f(a):\n  """doc"""\n# column-0 comment\n  return a\n', "f"), py_function(py, "f"))
    self.assertNotEqual(py_function("def f(a): return a\n", "f"), py_function("def f(a): return a + 1\n", "f"))  # one-liner


if __name__ == "__main__":
  unittest.main()
