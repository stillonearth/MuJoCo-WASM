import os
import json
os.chdir(os.path.realpath(os.path.dirname(__file__)))

files_to_download = []

for root, dirs, files in os.walk(".", topdown=False):
   for name in files:
      files_to_download.append(os.path.join(root, name).replace("\\", "/")[2:])

with open("index.json", mode="w") as f:
    json.dump(files_to_download, f, indent=2)
