#!/usr/bin/python

import code
import math
import firmata
import subprocess
import struct
import time

#from Adafruit_CharLCD import Adafruit_CharLCD
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
COEFFICIENT_5V = 10.0 # 6600 / 3300 * 5

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
SHIFT = 5          # no idea, it's in the headers
I2C = 6            # analog pin in I2C mode

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


class BrainScanTestFailure(Exception):
  def __init__(self, msg):
    self.msg = msg

class BrainScan(object):
  _harness = None
  _it = None

  def __init__(self, port):
    print "Brainscan Init"
    self._harness = firmata.FirmataInit(port, 57600, '/tmp/brainscan_log')

    # Enable i2c
    self._harness.I2CConfig(0)

    # Configure ic2 devices
    # INA219 stepper current sensors
    self._harness._i2c_device.I2CWrite(0x40, 0x00, [0x29, 0xFF])
    self._harness._i2c_device.I2CWrite(0x41, 0x00, [0x29, 0xFF])
    self._harness._i2c_device.I2CWrite(0x42, 0x00, [0x29, 0xFF])
    self._harness._i2c_device.I2CWrite(0x43, 0x00, [0x29, 0xFF])
    self._harness._i2c_device.I2CWrite(0x44, 0x00, [0x29, 0xFF])
    self._harness._i2c_device.I2CWrite(0x45, 0x00, [0x29, 0xFF])
    self._harness._i2c_device.I2CWrite(0x46, 0x00, [0x29, 0xFF])
    self._harness._i2c_device.I2CWrite(0x47, 0x00, [0x29, 0xFF])
    # INA219 board input current sensor
    self._harness._i2c_device.I2CWrite(0x4F, 0x00, [0x39, 0xFF])

    # Set RESET, HWB and BUTTON pins to high impedence
    self._harness.pinMode(PIN_RESET, INPUT)
    self._harness.pinMode(PIN_HWB, INPUT)
    self._harness.pinMode(PIN_BUTTON, INPUT)

    # Make sure the target is off (redundant, but better safe than sorry)
    self._harness.digitalWrite(PIN_RELAY, 0)

  def powerTargetOn(self):
    """Apply 12V to the target."""
    
    try:
      self._harness.EnableAnalogReporting(PIN_12V_SENSE)
      self._harness.EnableAnalogReporting(PIN_5V_SENSE)
      self._harness.digitalWrite(PIN_RELAY, 1)
      time.sleep(0.5) # let power stabilize

      vmot = self._harness.analogRead(PIN_12V_SENSE) * COEFFICIENT_12V / 1024
      print "vmotor: %s" % vmot
      if vmot > EXPECTED_12V + TOLERANCE_12V:
        raise BrainScanTestFailure("vmot too high: %s" % vmot)
      elif vmot < EXPECTED_12V - TOLERANCE_12V:
        raise BrainScanTestFailure("vmot too low: %s" % vmot)

      vcc = self._harness.analogRead(PIN_5V_SENSE) * COEFFICIENT_5V / 1024
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
      self._harness.digitalWrite(PIN_RELAY, 0) # slightly redundant, but safer
      raise
    finally:
      self._harness.DisableAnalogReporting(PIN_12V_SENSE)
      self._harness.DisableAnalogReporting(PIN_5V_SENSE)

  def powerTargetDown(self):
    """Power down target board."""
    self._harness.digitalWrite(PIN_RELAY, 0)

  def resetTarget(self):
    """Reset the target board."""
    self._harness.pinMode(PIN_RESET, OUTPUT)
    self._harness.digitalWrite(PIN_RESET, 0)
    time.sleep(0.5)
    self._harness.pinMode(PIN_RESET, INPUT)

  def activateBootloader(self):
    """Put the target in bootloader mode."""
    self._harness.pinMode(PIN_HWB, OUTPUT)
    self._harness.digitalWrite(PIN_HWB, 0)
    self._harness.pinMode(PIN_RESET, OUTPUT)
    self._harness.digitalWrite(PIN_RESET, 0)
    time.sleep(0.1)
    self._harness.pinMode(PIN_RESET, INPUT)
    time.sleep(0.1)
    self._harness.pinMode(PIN_HWB, INPUT)

  def setLEDColor(self, color):
    return
    self._harness.analogWrite(PIN_RED, color >> 16 & 0xFF)
    self._harness.analogWrite(PIN_GREEN, color >> 8 & 0xFF)
    self._harness.analogWrite(PIN_BLUE, color & 0xFF)

  def readHWB(self):
    return self_harness.digitalRead(PIN_HWB)

  def readReset(self):
    return self_harness.digitalRead(PIN_RESET)

  def writeMCP4462Reg(self, address, reg, value):
    """Write 9 bit value to given 4 bit register."""
    byte0 = reg << 4 | (value >> 8 & 0x01) 
    self._harness._i2c_device.I2CWrite(address, None, [byte0, value])

  def readMCP4462Reg(self, address, reg):
    """Read 9 bit value from 4 bit register."""
    byte0 = reg << 4 | 0x0c
    return self._harness._i2c_device.I2CRead(address, byte0, 2)

  def setBedPot(self, value):
    self.writeMCP4462Reg(DIGITAL_POT, BED_POT_REG, value)

  def setExtruderPot(self, value):
    self.writeMCP4462Reg(DIGITAL_POT, EXT_POT_REG, value)

  def readINA219Current(self, address, sense=SENSE_OHMS):
    reply = self._harness._i2c_device.I2CRead(address, 0x01, 2)
    assert type(reply) == list
    assert len(reply) == 2
    shunt = reply[0] | reply[1] << 8
    return shunt * SENSE_LSB / sense

  def readTargetCurrent(self):
    return self.readINA219Current(0x4F, 0.02)

  def readAxisCurrent(self, axis):
    return (self.readINA219Current(axis[COIL_A]),
            self.readINA219Current(axis[COIL_B]))

  def testEndstop(self, target, axis):
    print "Testing %s axis endstop" % axis[NAME]
    if (axis[ENDSTOP] < 0):
      return

    self._harness.digitalWrite(axis[ENDSTOP], 0)
    time.sleep(0.05)
    if target.readEndstop(axis) != 0:
      raise BrainScanTestFailure("%s endstop read failure" % axis[NAME])
    self._harness.digitalWrite(axis[ENDSTOP], 1)
    time.sleep(0.05)
    if target.readEndstop(axis) != 1:
      raise BrainScanTestFailure("%s endstop read failure" % axis[NAME])

  def testExtruderTempSet(self, target, value, min, max):
    self.setExtruderPot(value)
    time.sleep(0.1)
    extruder_temp = target.readExtruderTemp()
    print "setting bed pot to %s, expecting between %s and %s, got %s" % (value, min, max, extruder_temp)
    if not min < extruder_temp < max:
      raise BrainScanTestFailure("Extruder temp failure %s" % value)

  def testExtruderTemp(self, target):
    self.testExtruderTempSet(target, 0x00, 0.00, 0.04)
    self.testExtruderTempSet(target, 0x01, 0.10, 0.13)
    self.testExtruderTempSet(target, 0x02, 0.17, 0.20)
    self.testExtruderTempSet(target, 0x03, 0.23, 0.26)
    self.testExtruderTempSet(target, 0x04, 0.28, 0.31)
    self.testExtruderTempSet(target, 0x05, 0.33, 0.36)
    self.testExtruderTempSet(target, 0x06, 0.36, 0.39)
    self.testExtruderTempSet(target, 0x07, 0.40, 0.43)
    self.testExtruderTempSet(target, 0x08, 0.43, 0.46)
    self.testExtruderTempSet(target, 0x09, 0.46, 0.49)
    self.testExtruderTempSet(target, 0x10, 0.60, 0.63)
    self.testExtruderTempSet(target, 0x20, 0.735, 0.76)
    self.testExtruderTempSet(target, 0x30, 0.81, 0.84)
    self.testExtruderTempSet(target, 0x40, 0.84, 0.87)
    self.testExtruderTempSet(target, 0x50, 0.87, 0.90)
    self.testExtruderTempSet(target, 0x60, 0.89, 0.92)
    self.testExtruderTempSet(target, 0x70, 0.90, 0.93)
    self.testExtruderTempSet(target, 0x80, 0.91, 0.94)
    self.testExtruderTempSet(target, 0x90, 0.92, 0.95)
    self.testExtruderTempSet(target, 0xff, 0.94, 1.00)
  
  def testBedTempSet(self, target, value, min, max):
    self.setBedPot(value)
    time.sleep(0.1)
    bed_temp = target.readBedTemp()
    print "setting bed pot to %s, expecting between %s and %s, got %s" % (value, min, max, bed_temp)
    if not min < bed_temp < max:
      raise BrainScanTestFailure("Bed temp failure %s" % value)

  def testBedTemp(self, target):
    self.testBedTempSet(target, 0x00, 0.00, 0.04)
    self.testBedTempSet(target, 0x01, 0.10, 0.13)
    self.testBedTempSet(target, 0x02, 0.17, 0.20)
    self.testBedTempSet(target, 0x03, 0.23, 0.26)
    self.testBedTempSet(target, 0x04, 0.28, 0.31)
    self.testBedTempSet(target, 0x05, 0.33, 0.36)
    self.testBedTempSet(target, 0x06, 0.36, 0.39)
    self.testBedTempSet(target, 0x07, 0.40, 0.43)
    self.testBedTempSet(target, 0x08, 0.43, 0.46)
    self.testBedTempSet(target, 0x09, 0.46, 0.49)
    self.testBedTempSet(target, 0x10, 0.60, 0.63)
    self.testBedTempSet(target, 0x20, 0.735, 0.76)
    self.testBedTempSet(target, 0x30, 0.81, 0.84)
    self.testBedTempSet(target, 0x40, 0.84, 0.87)
    self.testBedTempSet(target, 0x50, 0.87, 0.90)
    self.testBedTempSet(target, 0x60, 0.89, 0.92)
    self.testBedTempSet(target, 0x70, 0.90, 0.93)
    self.testBedTempSet(target, 0x80, 0.91, 0.94)
    self.testBedTempSet(target, 0x90, 0.92, 0.95)
    self.testBedTempSet(target, 0xff, 0.94, 1.00)
  
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
        raise BrainScanTestFailure("%s axis current too high! %s %s" % (axis[NAME], coil_a, coil_b))

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
    self._harness.pinMode(PIN_RESET, INPUT)
    if self._harness.digitalRead(PIN_RESET) is False:
      raise BrainScanTestFailure("RESET pin LOW when not pressed")
    print "Please press the PROGRAM button"
    timeout = time.time() + 5
    count = 0
    while timeout > time.time():
      if self._harness.digitalRead(PIN_RESET) is False:
        count += 1
      else:
        count = 0
      if count > 50:
        print "RESET button test passed"
        break
    if timeout < time.time():
      raise BrainScanTestFailure("RESET button test timed out")

  def testProgramButton(self):
    self._harness.pinMode(PIN_HWB, INPUT)
    if self._harness.digitalRead(PIN_HWB) is False:
      raise BrainScanTestFailure("HWB pin LOW when not pressed")
    print "Please press the PROGRAM button"
    timeout = time.time() + 5
    count = 0
    while timeout > time.time():
      if self._harness.digitalRead(PIN_HWB) is False:
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

    self.testBedTemp(target)
    self.testExtruderTemp(target)


class Brainwave(object):
  def __init__(self, port):
    self._target = firmata.FirmataInit(port, 57600, '/tmp/brainwave_log')

    # Make sure things start off right (i.e. off)
    self.assertBedHeat(False)
    self.assertExtruderHeat(False)
    self.assertFan(False)
    self.setupAxis(BW_X_AXIS)
    self.setupAxis(BW_Y_AXIS)
    self.setupAxis(BW_Z_AXIS)
    self.setupAxis(BW_E_AXIS)

  def assertBedHeat(self, state):
    self._target.digitalWrite(BW_PIN_B_HEAT, state)

  def assertExtruderHeat(self, state):
    self._target.digitalWrite(BW_PIN_E_HEAT, state)

  def assertFan(self, state):
    self._target.digitalWrite(BW_PIN_FAN, state)

  def setupAxis(self, axis):
    self._target.pinMode(axis[STEP], OUTPUT)
    self._target.pinMode(axis[STEP], OUTPUT)
    self._target.pinMode(axis[DIR], OUTPUT)
    self._target.pinMode(axis[ATT], OUTPUT)
    self._target.pinMode(axis[EN], OUTPUT)
    self._target.pinMode(axis[ENDSTOP], INPUT)
    #self._target.digitalWrite(axis[ENDSTOP], 1) # enable pullup
    self.disableAxis(axis)

  def enableAxis(self, axis, attenuate=False):
    if attenuate:
      self._target.digitalWrite(axis[ATT], 1)
    else:
      self._target.digitalWrite(axis[ATT], 0)
    self._target.digitalWrite(axis[EN], 0)

  def disableAxis(self, axis):
    self._target.digitalWrite(axis[EN], 1)

  def stepAxis(self, axis, direction):
    """stepper driver steps on rising edge"""
    self._target.digitalWrite(axis[DIR], direction)
    self._target.digitalWrite(axis[STEP], 0)
    time.sleep(0.01)  # do we need this sleep?
    self._target.digitalWrite(axis[STEP], 1)

  def readEndstop(self, axis):
    return self._target.digitalRead(axis[ENDSTOP])

  def readBedTemp(self):
    self._target.EnableAnalogReporting(BW_PIN_B_TEMP)
    ret = self._target.analogRead(BW_PIN_B_TEMP)
    self._target.DisableAnalogReporting(BW_PIN_B_TEMP)
    return ret

  def readExtruderTemp(self):
    self._target.EnableAnalogReporting(BW_PIN_E_TEMP)
    ret = self._target.analogRead(BW_PIN_E_TEMP)
    self._target.DisableAnalogReporting(BW_PIN_E_TEMP)
    return ret


#lcd = Adafruit_CharLCD()
#lcd.clear()
scanner = None
quit = False

#lcd.message("   Brainscan\n Initializing")

while not quit:
  try:
    scanner = BrainScan(PORT)
    while not quit:
      try:
        scanner.setLEDColor(0xFFFFFF)
        #lcd.clear()
        #lcd.message(" Starting Test")
        print "Starting test"
        
        code.interact(local=locals())
        scanner.powerTargetOn()

        # Perform chip erase
        subprocess.check_call(AVRDUDE + ['-e'])
        # Set fuses for flashing
        subprocess.check_call(AVRDUDE + FUSES)
        # Write firmata
        subprocess.check_call(AVRDUDE + ['-D', '-U', 'flash:w:BrainwaveFirmata.cpp.hex:i'])

        time.sleep(2) # give the target a chance to start
        target = Brainwave("/dev/ttyACM1")

        def test():
          scanner.runTestSuite(target)
        #scanner.runTestSuite(target)
        code.interact(local=locals())
        quit = True

        # write bootloader
        subprocess.check_call(AVRDUDE + ['-U', 'flash:w:BrainwaveBootloaderCDC.hex:i'])
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
        #lcd.clear()
        #lcd.message(" Test Failure\n%s" % e.msg)
        print e.msg
        time.sleep(10)
  except subprocess.CalledProcessError:
    scanner.setLEDColor(0xFF0000)
    time.sleep(120)
    raise BrainScanTestFailure("avrdude failure")
  finally:
    #lcd.clear()
    if scanner:
      scanner.powerTargetDown()
