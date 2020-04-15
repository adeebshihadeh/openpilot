#!/usr/bin/env python3
import os
import subprocess
from common.basedir import BASEDIR
from selfdrive.swaglog import cloudlog

def run_cmd(cmd, default=None):
  try:
    return subprocess.check_output(cmd.split(), cwd=BASEIDR, encoding='utf8').strip()
  except subprocess.CalledProcessError:
    return default

def get_git_commit(default=None):
  return run_cmd("git rev-parse HEAD", default)

def get_git_branch(default=None):
  return run_cmd("git rev-parse --abbrev-ref HEAD", default)

def get_git_full_branchname(default=None):
  return run_cmd("git rev-parse --abbrev-ref --symbolic-full-name @{u}")

def get_git_remote(default=None):
  local_branch = run_cmd("git name-rev --name-only HEAD")
  tracking_remote = run_cmd("git config branch.%s.remote" % local_branch)
  branch_name = run_cmd("git config remote.%s.url" % tracking_remote, default)

  if branch_name is None:
    # Not on a branch, fallback
    return run_cmd("git config --get remote.origin.url")

  return branch_name

with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "common", "version.h")) as _versionf:
  version = _versionf.read().split('"')[1]

training_version = b"0.2.0"
terms_version = b"2"

dirty = True
origin = get_git_remote()
branch = get_git_full_branchname()

# This is needed otherwise touched files might show up as modified
run_cmd("git update-index --refresh")

if (origin is not None) and (branch is not None):
  comma_remote = origin.startswith('git@github.com:commaai') or origin.startswith('https://github.com/commaai')

  dirty = not comma_remote
  dirty = dirty or ('master' in branch)
  dirty = dirty or (subprocess.call(["git", "diff-index", "--quiet", branch, "--"]) != 0)

  if dirty:
    dirty_files = run_cmd("git diff-index %s --" % branch)
    commit = run_cmd("git rev-parse --verify HEAD")
    origin_commit = run_cmd("git rev-parse --verify %s" % branch)
    if any(n is None for n in (dirty_files, commit, origin_commit)):
      dirty = True
      cloudlog.exception("git subprocess failed while checking dirty")
    else:
      cloudlog.event("dirty comma branch", version=version, dirty=dirty, origin=origin, branch=branch, dirty_files=dirty_files, commit=commit, origin_commit=origin_commit)


if __name__ == "__main__":
  print("Dirty: %s" % dirty)
  print("Version: %s" % version)
  print("Remote: %s" % origin)
  print("Branch %s" % branch)
