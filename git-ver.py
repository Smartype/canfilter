#!/usr/bin/env python

import subprocess
import sys
import os

ver = subprocess.check_output(["git", "rev-parse", "--short=8", "HEAD"]).strip()
ver = ver.decode()
s = "const uint8_t gitversion[] = " + "{" +  f"0x{ver[:2]}, 0x{ver[2:4]}, 0x{ver[4:6]}, 0x{ver[6:]}" + "};"

fn = sys.argv[1]
t = None
if os.path.exists(fn):
  with open(fn, 'r') as f:
    t = f.read()

if t != s:
  with open(fn, 'w') as f:
    f.write(s)

