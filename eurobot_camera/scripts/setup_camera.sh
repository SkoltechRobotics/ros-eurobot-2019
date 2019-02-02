#!/usr/bin/bash

v4l2-ctl -c auto_shutter=0
v4l2-ctl -c gain_auto=0
v4l2-ctl -c gain=0
v4l2-ctl -c exposure_time_us=30000
