#!/usr/bin/env python3
import re
import time
import json
import base64
import requests
import subprocess
from http.server import BaseHTTPRequestHandler, HTTPServer
from os.path import expanduser
from threading import Thread

from common.params import Params
import os


MASTER_HOST = "testing.comma.life"


def get_workdir():
  continue_sh = open('/data/data/com.termux/files/continue.sh').read()
  for l in continue_sh.split('\n'):
    if l.startswith('#'):
      continue

    if 'cd "$HOME/one"' in l:
      work_dir = expanduser('~/one')
      return work_dir

  work_dir = '/data/openpilot'
  return work_dir


def heartbeat():
  work_dir = get_workdir()
  env = {
    "LD_LIBRARY_PATH": "",
    "ANDROID_DATA": "/data",
    "ANDROID_ROOT": "/system",
  }

  while True:

    with open(os.path.join(work_dir, "selfdrive", "common", "version.h")) as _versionf:
      version = _versionf.read().split('"')[1]

    # subprocess.check_output(["/system/bin/screencap", "-p", "/tmp/screen.png"], cwd=work_dir, env=env)
    # screenshot = base64.b64encode(open('/tmp/screen.png').read())
    tmux = ""

    try:
      tmux = os.popen('tail -n 100 /tmp/tmux_out').read()
    except:
      pass

    params = Params()
    msg = {
      'version': version,
      'dongle_id': params.get("DongleId").rstrip().decode('utf8'),
      'remote': subprocess.check_output(["git", "config", "--get", "remote.origin.url"], cwd=work_dir).decode('utf8').rstrip(),
      'revision': subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=work_dir).decode('utf8').rstrip(),
      'serial': subprocess.check_output(["getprop", "ro.boot.serialno"]).decode('utf8').rstrip(),
      # 'screenshot': screenshot,
      'tmux': tmux,
    }

    try:
      requests.post('http://%s/eon/heartbeat/' % MASTER_HOST, json=msg, timeout=10.0)
    except:
      print("Unable to reach master")

    time.sleep(5)


class HTTPHandler(BaseHTTPRequestHandler):
    def _set_headers(self, response=200, content='text/html'):
        self.send_response(response)
        self.send_header('Content-type', content)
        self.end_headers()

    def do_GET(self):
        self._set_headers()
        self.wfile.write("EON alive")

    def do_HEAD(self):
        self._set_headers()

    def do_POST(self):
        # Doesn't do anything with posted data
        self._set_headers(response=204)

        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        post_data = json.loads(post_data)

        if 'command' not in post_data or 'dongle_id' not in post_data:
          return

        params = Params()
        if params.get("DongleId").rstrip() != post_data['dongle_id']:
          return

        if post_data['command'] == "reboot":
          subprocess.check_output(["reboot"])

        if post_data['command'] == "update":
          print("Pulling new version")
          work_dir = get_workdir()
          env = {
            "GIT_SSH_COMMAND": "ssh -i /data/gitkey",
            "LD_LIBRARY_PATH": "/data/data/com.termux/files/usr/lib/",
            "ANDROID_DATA": "/data",
            "ANDROID_ROOT": "/system",
          }

          subprocess.check_output(["git", "reset", "--hard"], cwd=work_dir, env=env)
          # subprocess.check_output(["git", "clean", "-xdf"], cwd=work_dir, env=env)
          try:
            subprocess.check_output(["git", "fetch", "--unshallow"], cwd=work_dir, env=env)
          except subprocess.CalledProcessError:
            pass

          if 'revision' in post_data and re.match(r'\b[0-9a-f]{5,40}\b', post_data['revision']):
            subprocess.check_output(["git", "fetch", "origin"], cwd=work_dir, env=env)
            subprocess.check_output(["git", "checkout", post_data['revision']], cwd=work_dir, env=env)
          else:
            subprocess.check_output(["git", "pull"], cwd=work_dir, env=env)

          subprocess.check_output(["git", "submodule", "update"], cwd=work_dir, env=env)
          subprocess.check_output(["git", "lfs", "pull"], cwd=work_dir, env=env)
          subprocess.check_output(["reboot"], cwd=work_dir, env=env)


def control_server(server_class=HTTPServer, handler_class=HTTPHandler, port=8080):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print('Starting httpd...')
    httpd.serve_forever()


if __name__ == "__main__":
  heartbeat_thread = Thread(target=heartbeat)
  heartbeat_thread.daemon = True
  heartbeat_thread.start()

  control_thread = Thread(target=control_server)
  control_thread.daemon = True
  control_thread.start()

  while True:
    time.sleep(1)
