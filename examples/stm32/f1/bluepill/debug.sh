#!/bin/bash

# start xterm with openocd in the background
xterm -e openocd -f ../bluepill.cfg &

# save the PID of the background process
XTERM_PID=$!

# wait a bit to be sure the hardware is ready
sleep 2

# execute some initialisation commands via gdb
gdb --command=../init.gdb "$1"

# start the gdb gui
#nemiver --remote=localhost:3333 $1

# close xterm when the user has exited nemiver
kill "${XTERM_PID}"
