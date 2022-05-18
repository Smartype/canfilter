#!/usr/bin/env python3
import sys
import time
import struct
import argparse
from panda import Panda
from canfilter import CanFilter, FEATURE_LOCKSPEED, FEATURE_FAKELEAD, FEATURE_PASSTHRU, FEATURE_MIRROR_MSG, FEATURE_SET_DISTANCE

if __name__ == "__main__":

  if len(sys.argv) < 3:
    print(f'''usage:
    {sys.argv[0]} <features> <save-flash>
    feaures: LOCKSPEED,FAKELEAD,PASSTHRU,MIRRORMSG,DISTANCEREQ''')
    sys.exit(1)

  FEATURES = sys.argv[1].split(",")
  save = bool(int(sys.argv[2]))
  features = 0
  for f in FEATURES:
    if f == 'LOCKSPEED':
      features |= FEATURE_LOCKSPEED
    elif f == 'FAKELEAD':
      features |= FEATURE_FAKELEAD
    elif f == 'PASSTHRU':
      features |= FEATURE_PASSTHRU
    elif f == 'MIRRORMSG':
      features |= FEATURE_MIRROR_MSG
    elif f == 'DISTANCEREQ':
      features |= FEATURE_SET_DISTANCE
    else:
      raise f

  print(f"features: {f_str}, save: {SAVE}")

  p = Panda()
  p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)

  while 1:
    if len(p.can_recv()) == 0:
      break

  CanFilter.set_features(p, features, save) 
