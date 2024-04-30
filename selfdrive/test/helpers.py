import http.server
import os
import threading
import time

from functools import wraps

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.selfdrive.manager.process_config import managed_processes
from openpilot.system.hardware import PC
from openpilot.system.version import training_version, terms_version


def set_params_enabled():
  os.environ['FINGERPRINT'] = "TOYOTA COROLLA TSS2 2019"
  os.environ['LOGPRINT'] = "debug"

  params = Params()
  params.put("HasAcceptedTerms", terms_version)
  params.put("CompletedTrainingVersion", training_version)
  params.put_bool("OpenpilotEnabledToggle", True)

  # valid calib
  msg = messaging.new_message('liveCalibration')
  msg.liveCalibration.validBlocks = 20
  msg.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
  params.put("CalibrationParams", msg.to_bytes())

def phone_only(f):
  @wraps(f)
  def wrap(self, *args, **kwargs):
    if PC:
      self.skipTest("This test is not meant to run on PC")
    f(self, *args, **kwargs)
  return wrap

def release_only(f):
  @wraps(f)
  def wrap(self, *args, **kwargs):
    if "RELEASE" not in os.environ:
      self.skipTest("This test is only for release branches")
    f(self, *args, **kwargs)
  return wrap

def with_processes(processes, init_time=0, ignore_stopped=None):
  ignore_stopped = [] if ignore_stopped is None else ignore_stopped

  def wrapper(func):
    @wraps(func)
    def wrap(*args, **kwargs):
      # start and assert started
      for n, p in enumerate(processes):
        managed_processes[p].start()
        if n < len(processes) - 1:
          time.sleep(init_time)
      assert all(managed_processes[name].proc.exitcode is None for name in processes)

      # call the function
      try:
        func(*args, **kwargs)
        # assert processes are still started
        assert all(managed_processes[name].proc.exitcode is None for name in processes if name not in ignore_stopped)
      finally:
        for p in processes:
          managed_processes[p].stop()

    return wrap
  return wrapper


def noop(*args, **kwargs):
  pass


def read_segment_list(segment_list_path):
  with open(segment_list_path) as f:
    seg_list = f.read().splitlines()

  return [(platform[2:], segment) for platform, segment in zip(seg_list[::2], seg_list[1::2], strict=True)]


def with_http_server(func, handler=http.server.BaseHTTPRequestHandler, setup=None):
  @wraps(func)
  def inner(*args, **kwargs):
    host = '127.0.0.1'
    server = http.server.HTTPServer((host, 0), handler)
    port = server.server_port
    t = threading.Thread(target=server.serve_forever)
    t.start()

    if setup is not None:
      setup(host, port)

    try:
      return func(*args, f'http://{host}:{port}', **kwargs)
    finally:
      server.shutdown()
      server.server_close()
      t.join()

  return inner
