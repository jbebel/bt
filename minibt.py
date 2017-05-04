#!/usr/bin/python

#import io
import RPi.GPIO as GPIO
import serial
import threading

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
state_change = 16
cmd_mode = 18
trigger = 12
state_change_event = threading.Event()


def SignalEvent(unused_channel):
  state_change_event.set()


def main():
  ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
  #sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser), newline='\r\n',
  #                       line_buffering=True)

  GPIO.setup(state_change, GPIO.IN)
  GPIO.setup(cmd_mode, GPIO.OUT, initial=GPIO.HIGH)
  GPIO.setup(trigger, GPIO.OUT, initial=GPIO.LOW)
  GPIO.add_event_detect(state_change, GPIO.FALLING)
  GPIO.add_event_callback(state_change, SignalEvent)

  while True:
    print "Waiting for state change"
    state_change_event.wait()
    state_change_event.clear()
    ser.flushInput()
    print "State changed, entering command mode"
    GPIO.output(cmd_mode, GPIO.LOW)
    cmd = ser.readline()
    if not cmd == 'CMD\r\n':
      print "Command mode failed! RESET"
      GPIO.output(cmd_mode, GPIO.HIGH)
      continue

    ser.write('Q\r\n')
    status = ser.readline()
    print "status is %s" % status.strip()
    status_int = int(status, base=16)
    state = status_int & 0x0f
    if state == 13:
      print "playing audio"
      GPIO.output(trigger, GPIO.HIGH)
    else:
      GPIO.output(trigger, GPIO.LOW)

    print "Exiting command mode"
    GPIO.output(cmd_mode, GPIO.HIGH)
    end = ser.readline()
    if not end == 'END\r\n':
      print "END statement missing!"
      print "Got %s instead" % end

  GPIO.cleanup()

if __name__ == "__main__":
  main()
