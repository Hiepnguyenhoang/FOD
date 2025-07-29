#!/bin/bash

/usr/bin/python3.6 cam.py &
PID1=$!

python3  jetson_bbox.py &
PID2=$!

sleep 3000

kill $PID1
kill $PID2

echo "done"
