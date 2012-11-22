#!/usr/bin/python

import code
import math
import pyfirmata
import subprocess
# import pyunit
import struct
import time

from Adafruit_CharLCD import Adafruit_CharLCD
from datetime import datetime
from subprocess import * 

# BrainScan jig settings
PORT = '/dev/ttyACM0'
EXPECTED_12V = 12.25
EXPECTED_5V = 5.04
TOLERANCE_12V = 0.5
TOLERANCE_5V = 0.25
# 0.844 coefficient to all analog readings from arduino?
COEFFICIENT_12V = 20.151515 # 13300 / 3300 * 5
COEFFICIENT_5V = 10 # 6600 / 3300 * 5

# avrdude params
AVR_CMD = '/usr/bin/avrdude'
AVRISP = 'avrispmkII'
AVR_MCU = 'at90usb646'
AVRDUDE = [AVR_CMD, '-p', AVR_MCU, '-c', AVRISP, '-P', 'usb']
AVRDUDEBOOT = [AVR_CMD, '-p', AVR_MCU, '-c', 'avr109', '-P', '/dev/ttyACM1']
FUSES = ['-U', 'lfuse:w:0xde:m', '-U', 'hfuse:w:0x9b:m', '-U', 'efuse:w:0xf0:m']

# Pin modes.
# except from UNAVAILABLE taken from Firmata.h
UNAVAILABLE = -1 
INPUT = 0          # as defined in wiring.h
OUTPUT = 1         # as defined in wiring.h
ANALOG = 2         # analog pin in analogInput mode
PWM = 3            # digital pin in PWM output mode
SERVO = 4          # digital pin in SERVO mode

# Pin types
DIGITAL = OUTPUT   # same as OUTPUT below
# ANALOG is already defined above

# INA219 constants
SENSE_LSB = 0.00001
SENSE_OHMS = 0.1

# MCP4462 constants
DIGITAL_POT = 0x2e
EXT_POT_REG = 0
BED_POT_REG = 7

# jig digital pins
PIN_Z_MIN = 2
PIN_Y_MIN = 3
PIN_X_MIN = 4
PIN_HWB = 5
PIN_RESET = 6
PIN_BUTTON = 7
PIN_RELAY = 8
PIN_BLUE = 9
PIN_GREEN = 10
PIN_RED = 11

# jig analog pins
PIN_12V_SENSE = 0
PIN_5V_SENSE = 1
PIN_SDA = 4
PIN_SCL = 5

# Brainwave pins for firmata
BW_PIN_B_HEAT = 18
BW_PIN_E_HEAT = 32
BW_PIN_B_TEMP = 6 # Analog
BW_PIN_E_TEMP = 7 # Analog
BW_PIN_FAN = 31
BW_PIN_STATUS = 19
# stepper pins (step, dir, enable, attenuate, endstop, coil_a, coil_b, name)
BW_X_AXIS = (3, 5, 4, 2, 35, 0x40, 0x41, "X")
BW_Y_AXIS = (7, 9, 8, 6, 34, 0x42, 0x43, "Y")
BW_Z_AXIS = (11, 13, 12, 10, 33, 0x44, 0x45, "Z")
BW_E_AXIS = (15, 17, 16, 14, -1, 0x46, 0x47, "E")
STEP = 0
DIR = 1
EN = 2
ATT = 3
ENDSTOP = 4
COIL_A = 5
COIL_B = 6
NAME = 7

# Useful constants
CW = 0
CCW = 1


brainwave_layout = {
  'digital' : tuple(x for x in range(38)),
  'analog' : tuple(x for x in range(8)),
  'pwm' : (0,1,14,15,16,24,25,26,27),
  'use_ports' : True,
  'disabled' : ()
  }

class BrainScanTestFailure(Exception):
  def __init__(self, msg):
    self.msg = msg

class BrainScan(object):
  _harness = None
  _it = None
  
  # harness pins
  _reset = None
  _hwb = None
  _button = None
  _5v_sense = None
  _12v_sense = None
  _x_min = None
  _y_min = None
  _z_min = None
  _red = None
  _green = None
  _blue = None

  def __init__(self, port):
    print "Brainscan Init"
    self._harness = pyfirmata.Arduino(port)

    # Enable i2c
    self._harness.i2c_config(0)
    
    # Configure ic2 devices
    # INA219 current sensors
    self._harness.i2c_write(0x40, 0x00, bytearray(b'\x29\xff'))
    self._harness.i2c_write(0x41, 0x00, bytearray(b'\x29\xff'))
    self._harness.i2c_write(0x42, 0x00, bytearray(b'\x29\xff'))
    self._harness.i2c_write(0x43, 0x00, bytearray(b'\x29\xff'))
    self._harness.i2c_write(0x44, 0x00, bytearray(b'\x29\xff'))
    self._harness.i2c_write(0x45, 0x00, bytearray(b'\x29\xff'))
    self._harness.i2c_write(0x46, 0x00, bytearray(b'\x29\xff'))
    self._harness.i2c_write(0x47, 0x00, bytearray(b'\x29\xff'))
    self._harness.i2c_write(0b01001111, 0x00, bytearray(b'9\xff'))

    # Set RESET, HWB and BUTTON pins to high impedence
    self._reset = self._harness.get_pin("d:%s:i" % PIN_RESET)
    self._hwb = self._harness.get_pin("d:%s:i" % PIN_HWB)
    self._button = self._harness.get_pin("d:%s:i" % PIN_BUTTON)

    # Make sure the target is off
    self._relay = self._harness.get_pin("d:%s:o" % PIN_RELAY)
    self._relay.write(0)

    # Set voltage sense lines as analog input, but disabled
    self._5v_sense = self._harness.get_pin("a:%s:i" % PIN_5V_SENSE)
    self._5v_sense.disable_reporting()
    self._12v_sense = self._harness.get_pin("a:%s:i" % PIN_12V_SENSE)
    self._12v_sense.disable_reporting()
    # Iterator keeps analog reads from overflowing the serial port buffer
    self._it = pyfirmata.util.Iterator(self._harness)
    self._it.start()

    # Set endstop inputs to active low to simulate untriggered switches
    self._x_min = self._harness.get_pin("d:%s:o" % PIN_X_MIN)
    self._x_min.write(0)
    self._y_min = self._harness.get_pin("d:%s:o" % PIN_Y_MIN)
    self._y_min.write(0)
    self._z_min = self._harness.get_pin("d:%s:o" % PIN_Z_MIN)
    self._z_min.write(0)

    # Configure LED output pins
    self._red = self._harness.get_pin("d:%s:p" % PIN_RED)
    self._red.write(0)
    self._green = self._harness.get_pin("d:%s:p" % PIN_GREEN)
    self._green.write(0)
    self._blue = self._harness.get_pin("d:%s:p" % PIN_BLUE)
    self._blue.write(0)
    
  def terminate(self):
    if self._harness:
      self._harness.exit()
      del self._harness

  def powerTargetOn(self):
    """Apply 12V to the target."""
    
    try:
      self._12v_sense.enable_reporting()
      self._5v_sense.enable_reporting()
      self._relay.write(1)
      time.sleep(0.5) # let power stabilize

      vmot = self._12v_sense.read() * COEFFICIENT_12V
      print "vmotor: %s" % vmot
      if vmot > EXPECTED_12V + TOLERANCE_12V:
        raise BrainScanTestFailure("vmot too high: %s" % vmot)
      elif vmot < EXPECTED_12V - TOLERANCE_12V:
        raise BrainScanTestFailure("vmot too low: %s" % vmot)

      vcc = self._5v_sense.read() * COEFFICIENT_5V
      print "vcc: %s" % vcc
      if vcc > EXPECTED_5V + TOLERANCE_5V:
        raise BrainScanTestFailure("vcc too high: %s" % vcc)
      elif vcc < EXPECTED_5V - TOLERANCE_5V:
        raise BrainScanTestFailure("vcc too low: %s" % vcc)
     
      # Check we're below 1.6A or so (all motors on, no FETS)
      target_current = self.readTargetCurrent()
      print "target current: %s" % target_current
      if target_current > 4:
        raise BrainScanTestFailure("target current too high: %s" % target_current)

    except:
      self._relay.write(0) # slightly redundant, but safer
      raise
    finally:
      self._12v_sense.disable_reporting()
      self._5v_sense.disable_reporting()

  def powerTargetDown(self):
    """Power down target board."""
    self._relay.write(0)

  def resetTarget(self):
    """Reset the target board."""
    self._reset.mode = OUTPUT
    self._reset.write(0)
    time.sleep(0.5)
    self._reset.mode = INPUT

  def activateBootloader(self):
    """Put the target in bootloader mode."""
    self._hwb.mode = OUTPUT
    self._hwb.write(0)
    self._reset.mode = OUTPUT
    self._reset.write(0)
    time.sleep(0.1)
    self._reset.mode = INPUT
    time.sleep(0.1)
    self._hwb.mode = INPUT

  def setLEDColor(self, color):
    self._red.write((color >> 16 & 0xFF)/255)
    self._green.write((color >> 8 & 0xFF)/255)
    self._blue.write((color & 0xFF)/255)
  
  def readHWB(self):
    return self._hwb.read()
  
  def readReset(self):
    return self._reset.read()

  def writeMCP4462Reg(self, address, reg, value):
    """Write 9 bit value to given 4 bit register."""
    byte0 = reg << 4 | (value >> 8 & 0x01) 
    self._harness.i2c_write(address, None, bytearray([byte0, value]))

  def readMCP4462Reg(self, address, reg):
    """Read 9 bit value from 4 bit register."""
    byte0 = reg << 4 | 0x0c
    return self._harness.i2c_read(address, byte0, 2)

  def setBedPot(self, value):
    self.writeMCP4462Reg(DIGITAL_POT, BED_POT_REG, value)
    
  def setExtruderPot(self, value):
    self.writeMCP4462Reg(DIGITAL_POT, EXT_POT_REG, value)
    
  def readINA219Current(self, address, sense=SENSE_OHMS):
    raw = self._harness.i2c_read(address, 0x01, 2)
    value = struct.unpack( '!h', bytes(raw[0:2]) )[0]
    return value * SENSE_LSB / sense
    
  def readTargetCurrent(self):
    return self.readINA219Current(0b01001111, 0.02)
  
  def readAxisCurrent(self, axis):
    return (self.readINA219Current(axis[COIL_A]),
            self.readINA219Current(axis[COIL_B]))
  
  def testEndstop(self, target, axis):
    if (axis[ENDSTOP] < 0):
      return
    if axis[NAME] == "X":
      endstop = self._x_min
    elif axis[NAME] == "Y":
      endstop = self._y_min
    elif axis[NAME] == "Z":
      endstop = self._z_min
    else:
      return
    
    endstop.write(0)
    time.sleep(0.05)
    if target.readEndstop(axis) != 0:
      raise BrainScanTestFailure("%s endstop read failure" % axis[NAME])
    endstop.write(1)
    time.sleep(0.05)
    if target.readEndstop(axis) != 1:
      raise BrainScanTestFailure("%s endstop read failure" % axis[NAME])
  
  def testAxis(self, target, axis):
    #(step, direction, enable, attenuate, endstop, coil_a, coil_b, name) = axis
    print "Testing %s Axis" % axis[NAME]
    target.disableAxis(axis)
    (coil_a, coil_b) = self.readAxisCurrent(axis)
    if coil_a > 0.01:
      raise BrainScanTestFailure("%s axis coil A non-zero" % axis[NAME])
    if coil_b > 0.01:
      raise BrainScanTestFailure("%s axis coil B non-zero" % axis[NAME])
    
    # Single step mode (CW):
    phases = { 0: (-1,  1), (-1,  1): 0,
               1: (-1, -1), (-1, -1): 1,
               2: ( 1, -1), ( 1, -1): 2,
               3: ( 1,  1), ( 1,  1): 3 }
    
    try:
      target.enableAxis(axis)
      time.sleep(0.1)
      (coil_a, coil_b) = self.readAxisCurrent(axis)
      if ( (coil_a > 0.7) or (coil_b > 0.7) ):
        raise BrainScanTestFailure("%s axis current too high! %s %s" % axis[NAME], coil_a, coil_b)

      errors = 0

      start = phases[(math.copysign(1, coil_a), math.copysign(1, coil_b))]
      for step in range(start, start + 16):
        print "step %s: (%s, %s) = %s" % (step, coil_a, coil_b, phases[step % 4])
        if (math.copysign(1, coil_a), math.copysign(1, coil_b)) != phases[step % 4]:
          errors += 1
        target.stepAxis(axis, CW)
        time.sleep(0.1)
        (coil_a, coil_b) = self.readAxisCurrent(axis)
        if ( (coil_a > 0.7) or (coil_b > 0.7) ):
          raise BrainScanTestFailure("%s axis current too high! %s %s" % axis[NAME], coil_a, coil_b)
      print "Errors: %s" % errors
      if errors > 0:
        raise BrainScanTestFailure("%s axis failed step test" % axis[NAME])

      start = phases[(math.copysign(1, coil_a), math.copysign(1, coil_b))]
      for step in range(start, start - 16, -1):
        print "step %s: (%s, %s) = %s" % (step, coil_a, coil_b, phases[step % 4])
        if (math.copysign(1, coil_a), math.copysign(1, coil_b)) != phases[step % 4]:
          errors += 1
        target.stepAxis(axis, CCW)
        time.sleep(0.1)
        (coil_a, coil_b) = self.readAxisCurrent(axis)
        if ( (coil_a > 0.7) or (coil_b > 0.7) ):
          raise BrainScanTestFailure("%s axis current too high! %s %s" % axis[NAME], coil_a, coil_b)
      print "Errors: %s" % errors

      target.enableAxis(axis, attenuate=True)
      time.sleep(0.1)
      start = phases[(math.copysign(1, coil_a), math.copysign(1, coil_b))]
      for step in range(start, start + 16):
        print "step %s: (%s, %s) = %s" % (step, coil_a, coil_b, phases[step % 4])
        if (math.copysign(1, coil_a), math.copysign(1, coil_b)) != phases[step % 4]:
          errors += 1
        target.stepAxis(axis, CW)
        time.sleep(0.1)
        (coil_a, coil_b) = self.readAxisCurrent(axis)
        if ( (coil_a > 0.7) or (coil_b > 0.7) ):
          raise BrainScanTestFailure("%s axis current too high! %s %s" % axis[NAME], coil_a, coil_b)

      print "Errors: %s" % errors
      if errors > 0:
        raise BrainScanTestFailure("%s axis failed step test" % axis[NAME])
      
    finally:
      target.disableAxis(axis)

  def testResetButton(self):
    """Only call when the target object doesn't exist."""
    self._reset.mode = INPUT
    if self._reset.read() is False:
      raise BrainScanTestFailure("RESET pin LOW when not pressed")
    print "Please press the PROGRAM button"
    timeout = time.time() + 5
    count = 0
    while timeout > time.time():
      if self._reset.read() is False:
        count += 1
      else:
        count = 0
      if count > 50:
        print "RESET button test passed"
        break
    if timeout < time.time():
      raise BrainScanTestFailure("RESET button test timed out")

  def testProgramButton(self):
    self._hwb.mode = INPUT
    if self._hwb.read() is False:
      raise BrainScanTestFailure("HWB pin LOW when not pressed")
    print "Please press the PROGRAM button"
    timeout = time.time() + 5
    count = 0
    while timeout > time.time():
      if self._hwb.read() is False:
        count += 1
      else:
        count = 0
      if count > 50:
        print "PROGRAM button test passed"
        break
    if timeout < time.time():
      raise BrainScanTestFailure("PROGRAM button test timed out")
    
  def runTestSuite(self, target):
    idle_current = self.readTargetCurrent()
    print "idle current: %s" % idle_current
    if idle_current > 0.2:
      raise BrainScanTestFailure("idle current too high: %s" % idle_current)
    
    target.assertFan(True)
    time.sleep(0.1)
    fan_current = self.readTargetCurrent() - idle_current
    target.assertFan(False)
    print "fan current: %s" % fan_current
    if fan_current > 0.15:
      raise BrainScanTestFailure("fan current too high: %s" % fan_current)
    if fan_current < 0.08:
      raise BrainScanTestFailure("fan current too low: %s" % fan_current)
    
    target.assertBedHeat(True)
    time.sleep(0.1)
    bed_current = self.readTargetCurrent() - idle_current
    target.assertBedHeat(False)
    print "bed current: %s" % bed_current
    if bed_current > 13:
      raise BrainScanTestFailure("bed current too high: %s" % bed_current)
    if bed_current < 10:
      raise BrainScanTestFailure("bed current too low: %s" % bed_current)
      
    time.sleep(0.1)
    self.testAxis(target, BW_X_AXIS)
    time.sleep(0.1)
    self.testAxis(target, BW_Y_AXIS)
    time.sleep(0.1)
    self.testAxis(target, BW_Z_AXIS)
    time.sleep(0.1)
    self.testAxis(target, BW_E_AXIS)

    target.assertExtruderHeat(True)
    time.sleep(0.1)
    extruder_current = self.readTargetCurrent() - idle_current
    target.assertExtruderHeat(False)
    print "extruder current: %s" % extruder_current
    if extruder_current > 2.5:
      raise BrainScanTestFailure("extruder current too high: %s" % extruder_current)
    if extruder_current < 1.5:
      raise BrainScanTestFailure("extruder current too low: %s" % extruder_current)

    self.testEndstop(target, BW_X_AXIS)
    self.testEndstop(target, BW_Y_AXIS)
    self.testEndstop(target, BW_Z_AXIS)


class Brainwave(object):
  def __init__(self, port):
    self._target = pyfirmata.Board(port, brainwave_layout)
    
    # Iterator keeps analog reads from overflowing the serial port buffer
    self._it = pyfirmata.util.Iterator(self._target)
    self._it.start()

    self._bed_heat = self._target.get_pin("d:%s:o" % BW_PIN_B_HEAT)
    self._ext_heat = self._target.get_pin("d:%s:o" % BW_PIN_E_HEAT)
    self._fan = self._target.get_pin("d:%s:o" % BW_PIN_FAN)
    
    # Make sure things start off right (i.e. off)
    self.assertBedHeat(False)
    self.assertExtruderHeat(False)
    self.assertFan(False)
    self.setupAxis(BW_X_AXIS)
    self.setupAxis(BW_Y_AXIS)
    self.setupAxis(BW_Z_AXIS)
    self.setupAxis(BW_E_AXIS)
    self._b_temp = self._target.get_pin("a:%s:i" % BW_PIN_B_TEMP)
    self._e_temp = self._target.get_pin("a:%s:i" % BW_PIN_E_TEMP)

  def terminate(self):
    if self._target:
      self._target.exit()
      del self._target


  def assertBedHeat(self, state):
    self._bed_heat.write(state)

  def assertExtruderHeat(self, state):
    self._ext_heat.write(state)

  def assertFan(self, state):
    self._fan.write(state)

  def setupAxis(self, axis):
    self._target.digital[axis[STEP]].mode = OUTPUT
    self._target.digital[axis[DIR]].mode = OUTPUT
    self._target.digital[axis[ATT]].mode = OUTPUT
    self._target.digital[axis[EN]].mode = OUTPUT
    self._target.digital[axis[ENDSTOP]].mode = INPUT
    self._target.digital[axis[ENDSTOP]].write(1) # enable pullup
    self.disableAxis(axis)

  def enableAxis(self, axis, attenuate=False):
    if attenuate:
      self._target.digital[axis[ATT]].write(1);
    else:
      self._target.digital[axis[ATT]].write(0);
    self._target.digital[axis[EN]].write(0);

  def disableAxis(self, axis):
    self._target.digital[axis[EN]].write(1);

  def stepAxis(self, axis, direction):
    """stepper driver steps on rising edge"""
    self._target.digital[axis[DIR]].write(direction)
    self._target.digital[axis[STEP]].write(0)
    time.sleep(0.01)  # do we need this sleep?
    self._target.digital[axis[STEP]].write(1)

  def readEndstop(self, axis):
    return self._target.digital[axis[ENDSTOP]].read()

  def readBedTemp(self):
    return self._target.analog[BW_B_HEAT].read()

  def readExtruderTemp(self):
    return self._target.analog[BW_E_HEAT].read()


lcd = Adafruit_CharLCD()
lcd.clear()
scanner = None
quit = False

lcd.message("   Brainscan\n Initializing")

while not quit:
  try:
    scanner = BrainScan(PORT)
    while not quit:
      try:
        scanner.setLEDColor(0xFFFFFF)
        lcd.clear()
        lcd.message(" Starting Test")
        print "Starting test"
        
        scanner.powerTargetOn()

        # Perform chip erase
        subprocess.check_call(AVRDUDE + ['-e'])
        # Set fuses for flashing
        subprocess.check_call(AVRDUDE + FUSES)
        # Write firmata
        subprocess.check_call(AVRDUDE + ['-D', '-U', 'flash:w:BrainwaveFirmata.cpp.hex:i'])

        time.sleep(2) # give the target a chance to start
        target = Brainwave("/dev/ttyACM1")

        #scanner.runTestSuite(target)
        code.interact(local=locals())
        quit = True

        target.terminate()
        del target
        
        # write bootloader
        subprocess.check_call(AVRDUDE + ['-U', 'flash:w:BootloaderCDC.hex:i'])
        # Reset fuses and writelock bootloader area
        subprocess.check_call(AVRDUDE + FUSES + ['-U', 'lock:w:0x2f:m'])
        scanner.activateBootloader()
        time.sleep(2) # wait for linux to find the device
        # This hangs:
        #subprocess.check_call(AVRDUDEBOOT + ['-U', 'flash:w:Sprinter.cpp.hex:i'])
        #scanner.resetTarget()
        

        code.interact(local=locals())
        scanner.powerTargetDown()
        scanner.setLEDColor(0x00FF00)
        time.sleep(2)
      except BrainScanTestFailure as e:
        print "Test failure"
        scanner.powerTargetDown()
        scanner.setLEDColor(0xFF0000)
        lcd.clear()
        lcd.message(" Test Failure\n%s" % e.msg)
        print e.msg
        time.sleep(10)
  except subprocess.CalledProcessError:
    scanner.setLEDColor(0xFF0000)
    time.sleep(120)
    raise BrainScanTestFailure("avrdude failure")
  finally:
    lcd.clear()
    if scanner:
      scanner.powerTargetDown()
      scanner.terminate()
      del scanner
