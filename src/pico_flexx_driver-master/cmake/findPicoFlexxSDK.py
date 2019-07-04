#!/usr/bin/python

import os, sys

class SDK:
  def __init__(self, directory, version):
    (major, minor, patch, build) = version.split('.', 3)
    self.major = int(major)
    self.minor = int(minor)
    self.patch = int(patch)
    self.build = int(build)
    self.directory = directory

  def __lt__(self, other):
    return self.major < other.major or self.major == other.major and\
      (self.minor < other.minor or self.minor == other.minor and\
      (self.patch < other.patch or self.patch == other.patch and self.build < other.build))

  def __str__(self):
    return "%d.%d.%d.%d %s" % (self.major, self.minor, self.patch, self.build, self.directory)

def findSDKs(path):
  directories = [directory for directory in os.listdir(path) if os.path.isdir(os.path.join(path, directory))]
  sdks = []
  for directory in directories:
    try:
      (name, version, platform) = directory.split('-', 2)
      if name != "libroyale":
        continue;

      share = os.path.join(path, directory, "share")
      if not os.path.exists(share) or not os.path.isdir(share):
        continue;

      sdks.append(SDK(os.path.join(path, directory, "share"), version));
    except:
      continue
  sdks.sort()
  sdks.reverse()
  return sdks

if __name__ == "__main__":
  if len(sys.argv) != 2:
    print >> sys.stderr, "no search directory given as first argument!"
    sys.exit(-1)

  path = sys.argv[1]
  if not os.path.exists(path) or not os.path.isdir(path):
    print >> sys.stderr, "argument is not a valid directory!"
    sys.exit(-1)

  sdks = findSDKs(path)
  if len(sdks) == 0:
    print >> sys.stderr, "no SDKs found!"
    sys.exit(-1)

  for sdk in sdks:
    print os.path.abspath(sdk.directory)

  sys.exit(0)
