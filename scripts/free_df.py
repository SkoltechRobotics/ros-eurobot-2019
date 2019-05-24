#!/usr/bin/env python3

import os
import shutil

total, used, free = shutil.disk_usage("/")

mem_used = float(used)/float(total)
bash_command = "rm -rf ~/.ros/log/*"

if mem_used > 0.85:
     os.system(bash_command)
     print ("Logs were deleted")

