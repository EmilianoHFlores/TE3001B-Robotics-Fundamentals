#!/bin/sh

python3 tkinter_drawer.py
echo "Loading Processing..."
processing-java --sketch=sketchbook/draw_csv_round --run $1