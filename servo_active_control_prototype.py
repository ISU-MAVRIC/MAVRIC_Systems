#Servo Active Control
#This code lets the user control a servo in real time using curses and PWM

import pigpio
import curses
from time import sleep

SERVO_GPIO = 18
pi = pigpio.pi()
if not pi.connected: exit()

MIN_PW = 500
MAX_PW = 2500
MID_PW = 1500
pulse = MID_PW
pi.set_servo_pulsewidth(SERVO_GPIO, pulse)

def main(stdscr):
    global pulse
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)
    stdscr.addstr(0,0,"Servo Terminal UI - Up/Down arrows to move, q to quit")
    while True:
        key = stdscr.getch()
        if key == ord('q'):
            break
        elif key == curses.KEY_UP:
            pulse += 50
        elif key == curses.KEY_DOWN:
            pulse -= 50
        pulse = max(MIN_PW, min(MAX_PW, pulse))
        pi.set_servo_pulsewidth(SERVO_GPIO, pulse)
        stdscr.addstr(2,0,f"Pulse width: {pulse} μs       ")

curses.wrapper(main)
pi.set_servo_pulsewidth(SERVO_GPIO, 0)
pi.stop()
