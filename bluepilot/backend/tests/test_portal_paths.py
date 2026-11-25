"""The portal's file routes serve only files inside their own folder, and its internal broadcast only to this device.

Run against the real request handler on a local socket, with the server's state (enabled, offroad, its folders) patched.
"""
import http.client
import json
import shutil
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock

from bluepilot.backend import bp_portal
from bluepilot.backend.routes import processing

SECRET = b"not for the network"
INDEX = b"<html>index</html>"
APP_JS = b"console.log('app')"
VIDEO = b"qcamera bytes"
THUMB = b"\xff\xd8 a route's thumbnail"
ROUTE = "00000020--d54af86e0e"
LAN_CLIENT = ("192.168.43.20", 50000)


class Server(ThreadingHTTPServer):
  """The portal's handler on 127.0.0.1; `client` sets the address the handler sees, as a client on the network would."""
  daemon_threads = True
  client = None

  def finish_request(self, request, client_address):
    self.RequestHandlerClass(request, self.client or client_address, self)


class TestPortalPaths(unittest.TestCase):
  def setUp(self):
    self.tmp = Path(tempfile.mkdtemp())
    self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
    web = self.tmp / "webapp"
    (web / "assets").mkdir(parents=True)
    (web / "index.html").write_bytes(INDEX)
    (web / "assets" / "app.js").write_bytes(APP_JS)
    (self.tmp / "secret.txt").write_bytes(SECRET)
    (self.tmp / "secret.jpg").write_bytes(SECRET)
    thumbs = self.tmp / "thumbnails"
    thumbs.mkdir()
    (thumbs / f"{ROUTE}.jpg").write_bytes(THUMB)
    seg = self.tmp / "realdata" / f"{ROUTE}--0"
    seg.mkdir(parents=True)
    (seg / "qcamera.ts").write_bytes(VIDEO)
    self.segments = [{"path": str(seg), "name": seg.name, "segment": 0}]

    self.broadcaster = mock.Mock()
    for target, value in [
      ("should_server_run", lambda: True),
      ("is_onroad", lambda: False),
      ("WEBAPP_DIR", web),
      ("get_route_segments", lambda base: self.segments if base == ROUTE else []),
      ("scan_routes", lambda: [{"baseName": ROUTE}]),
      ("get_disk_space_info", dict),
    ]:
      patcher = mock.patch.object(bp_portal, target, value)
      patcher.start()
      self.addCleanup(patcher.stop)
    for patcher in (mock.patch.object(processing, "THUMBNAIL_CACHE", str(thumbs)),
                    mock.patch.object(bp_portal.process_manager, "check_rate_limit", return_value=(True, 0)),
                    mock.patch.object(bp_portal.server_state, "get_broadcaster", return_value=self.broadcaster)):
      patcher.start()
      self.addCleanup(patcher.stop)

    self.server = Server(("127.0.0.1", 0), bp_portal.WebRoutesHandler)
    threading.Thread(target=self.server.serve_forever, daemon=True).start()
    self.addCleanup(self.server.server_close)
    self.addCleanup(self.server.shutdown)

  def request(self, method, raw_path, body=None):
    # http.client sends the path as written: no client-side removal of '..' or decoding.
    conn = http.client.HTTPConnection("127.0.0.1", self.server.server_address[1], timeout=10)
    try:
      conn.request(method, raw_path, body=body, headers={"Content-Type": "application/json"} if body else {})
      resp = conn.getresponse()
      return resp.status, resp.read()
    finally:
      conn.close()

  # Files outside the web app's folder

  def test_raw_dotdot_path_is_not_served(self):
    status, body = self.request("GET", "/../secret.txt")
    self.assertNotIn(SECRET, body)
    self.assertEqual((status, body), (200, INDEX))

  def test_deep_dotdot_path_is_not_served(self):
    status, body = self.request("GET", "/assets/" + "../" * 30 + str(self.tmp / "secret.txt").lstrip("/"))
    self.assertNotIn(SECRET, body)
    self.assertEqual((status, body), (200, INDEX))

  def test_encoded_dotdot_path_is_not_served(self):
    for path in ("/%2e%2e/secret.txt", "/%2E%2E%2Fsecret.txt", "/..%2fsecret.txt"):
      with self.subTest(path=path):
        status, body = self.request("GET", path)
        self.assertNotIn(SECRET, body)
        self.assertEqual((status, body), (200, INDEX))

  def test_thumbnail_outside_the_thumbnail_folder_is_not_served(self):
    status, body = self.request("GET", "/api/thumbnail/../secret")
    self.assertNotIn(SECRET, body)
    self.assertEqual(status, 200)
    self.assertTrue(body.startswith(b"\x89PNG"), "the placeholder image")

  def test_file_viewer_refuses_a_folder_that_only_starts_like_an_allowed_one(self):
    status, body = self.request("GET", "/api/file-content?path=/data/openpilot_other/notes.txt")
    self.assertEqual(status, 403, body)
    status, body = self.request("GET", "/api/file-content?path=/data/logs2/notes.txt")
    self.assertEqual(status, 403, body)

  # The internal broadcast

  def test_broadcast_from_the_network_is_refused(self):
    self.server.client = LAN_CLIENT
    status, _ = self.request("POST", "/_internal/broadcast", json.dumps({"type": "x", "data": {}}))
    self.assertEqual(status, 403)
    self.broadcaster.broadcast.assert_not_called()

  def test_broadcast_from_this_device_is_sent(self):
    status, _ = self.request("POST", "/_internal/broadcast", json.dumps({"type": "x", "data": {"a": 1}}))
    self.assertEqual(status, 200)
    self.broadcaster.broadcast.assert_called_once_with("x", {"a": 1})

  # What the portal is for, still served (from the network too)

  def test_normal_pages_route_list_video_and_thumbnail_are_served(self):
    self.server.client = LAN_CLIENT
    self.assertEqual(self.request("GET", "/"), (200, INDEX))
    self.assertEqual(self.request("GET", "/routes"), (200, INDEX))
    self.assertEqual(self.request("GET", "/assets/app.js"), (200, APP_JS))
    self.assertEqual(self.request("GET", "/no/such/page"), (200, INDEX))
    status, body = self.request("GET", "/api/routes")
    self.assertEqual(status, 200)
    self.assertEqual(json.loads(body)["routes"], [{"baseName": ROUTE}])
    self.assertEqual(self.request("GET", f"/api/video/{ROUTE}/0/lq"), (200, VIDEO))
    self.assertEqual(self.request("GET", f"/api/thumbnail/{ROUTE}"), (200, THUMB))
    status, body = self.request("GET", "/api/file-content?path=/data/openpilot/no-such-file.txt")
    self.assertEqual(status, 404, body)


if __name__ == "__main__":
  unittest.main()
