#Servo Positional Control
#This code works by moving the servo in increments

import sys, termios, tty
from gpiozero import Servo
from time import sleep

servo = Servo(18)
pos = 0.0

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

while True:
    ch = getch()
    if ch == 'a': #Use a to turn left
        pos -= 0.1
    elif ch == 'd': #Use d to turn right
        pos += 0.1
    elif ch == 'q': #Press q to quit
        break
    pos = max(-1, min(1, pos))
    servo.value = pos
    print(f"Position: {pos:.2f}")
