#!/usr/bin/python

#import io
import RPi.GPIO as GPIO
import serial
import threading

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
state_change = 16
cmd_mode = 18
state_change_event = threading.Event()


def State(state):
  """Convert an integer state to its meaning."""
  if state == 0:
    return ("Limbo: "
            "Logically off, but physically on")
  elif state == 1:
    return ("Connectable: "
            "The module is connectable, page scanning")
  elif state == 2:
    return ("Connectable and discoverable: "
            "The module is connectable and discoverable, page and inquiry scanning")
  elif state == 3:
    return ("Connected: "
            "The module is connected to an audio gateway")
  elif state == 4:
    return ("Outgoing call established: "
            "The connected audio gateway has an outgoing call in progress")
  elif state == 5:
    return ("Incoming call established: "
            "The connected audio gateway has an active call in progress and the audio is in the headset")
  elif state == 6:
    return ("Active call: "
            "The connected audio gateway has an active call in progress and the audio is in the headset")
  elif state == 7:
    return ("Test mode: "
            "The headset is in Test mode")
  elif state == 8:
    return ("Three-way call waiting: "
            "The connected audio gateway has an active call and a second call on hold")
  elif state == 9:
    return ("Three-way call on hold: "
            "The connected audio gateway has an active call and a second call on hold")
  elif state == 10:
    return ("Three-way call multi-call: "
            "The connected audio gateway has an active call and a second call on hold")
  elif state == 11:
    return ("Incoming call on hold: "
            "The connected audio gateway has an incoming call on hold")
  elif state == 12:
    return ("Active call: "
            "The connected audio gateway has an active call and the audio is in the handset")
  elif state == 13:
    return ("Audio streaming: "
            "The headset is streaming A2DP audio")
  elif state == 14:
    return ("Low battery: "
            "The system has a low battery")
  else:
    return "INVALID: State is not valid."


def StatusDict(status_int):
  """Converts a status integer into a dictionary of status values."""
  status_dict = {}
  status_dict['state'] = status_int & 0x0f
  status_dict['volchange'] = bool(status_int >> 4 & 1)
  status_dict['micchange'] = bool(status_int >> 5 & 1)
  status_dict['maybe_hfp_mute'] = bool(status_int >> 6 & 1)
  status_dict['maybe_timeout'] = bool(status_int >> 7 & 1)
  status_dict['iap'] = bool(status_int >> 8 & 1)
  status_dict['spp'] = bool(status_int >> 9 & 1)
  status_dict['a2dp'] = bool(status_int >> 10 & 1)
  status_dict['hfp_hsp'] = bool(status_int >> 11 & 1)
  status_dict['caller_id'] = bool(status_int >> 12 & 1)
  status_dict['track_change'] = bool(status_int >> 13 & 1)
  status_dict['reserved_14'] = bool(status_int >> 14 & 1)
  status_dict['reserved_15'] = bool(status_int >> 15 & 1)

  return status_dict


def ProcessStatus(status):
  """Processes a status string from the Q command."""
  status_int = int(status, base=16)
  status_dict = StatusDict(status_int)

  profiles = []
  if status_dict['iap']:
    profiles.append('iAP')
  if status_dict['spp']:
    profiles.append('SPP')
  if status_dict['a2dp']:
    profiles.append('A2DP')
  if status_dict['hfp_hsp']:
    profiles.append('HFP/HSP')
  
  if profiles:
    print 'Connected profiles: %s' % ','.join(profiles)
  else:
    print 'Not Connected'

  print "state is %s" % status_dict['state']
  print State(status_dict['state'])

  if status_dict['caller_id']:
    print "Caller ID event"
    #TODO(jbebel): get caller ID with "T" command
  if status_dict['track_change']:
    print "Track change"
    #TODO(jbebel): get track data with "AD" command
  if status_dict['volchange']:
    print "Volume change"
    #TODO(jbebel): get volume change with "Y,0" command
  if status_dict['micchange']:
    print "Mic level change"
    #TODO(jbebel): get mic level change with "Y,1" command
  if status_dict['maybe_hfp_mute']:
    print "Unexpected HFP Mute signal detected"
  if status_dict['maybe_timeout']:
    print "Unexpected reconnect timeout signal detected"
  if status_dict['reserved_14']:
    print "Unexpected bit 14 signal"
  if status_dict['reserved_15']:
    print "Unexpected bit 15 signal"

def SignalEvent(unused_channel):
  state_change_event.set()


def main():
  ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
  #sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser), newline='\r\n',
  #                       line_buffering=True)

  GPIO.setup(state_change, GPIO.IN)
  GPIO.setup(cmd_mode, GPIO.OUT, initial=GPIO.HIGH)
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
    ProcessStatus(status)

    print "Exiting command mode"
    GPIO.output(cmd_mode, GPIO.HIGH)
    end = ser.readline()
    if not end == 'END\r\n':
      print "END statement missing!"
      print "Got %s instead" % end

  GPIO.cleanup()

if __name__ == "__main__":
  main()
