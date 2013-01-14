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
PORT = '/dev/ttyUSB0'
#PORT = '/dev/ttyACM0'
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
AVRDUDEBOOT = [AVR_CMD, '-p', AVR_MCU, '-c', 'avr109', '-P', '/dev/ttyACM0']
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
PIN_E_POT_HIGH = 17
PIN_E_POT_LOW = 16
PIN_B_POT_HIGH = 12
PIN_B_POT_LOW = 13

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
  pass

class BrainScan(object):
  _harness = None
  _it = None

  def __init__(self, port):
    self._harness = firmata.FirmataInit(port, 57600, '/tmp/brainscan_log')

    # Enable i2c
    self._harness.I2CConfig(4) #INA219 registers are delayed by 4uS

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
      self._harness.digitalWrite(PIN_RELAY, 1)
      start_time = time.time()
      while time.time() < start_time + 2:
        # Check we're below 4A or so (all motors on, no FETS)
        target_current = self.readTargetCurrent()
        print "target current: %s" % target_current
        if target_current > 4:
          raise BrainScanTestFailure("target current too high: %s" % target_current)

      self._harness.EnableAnalogReporting(PIN_12V_SENSE)
      self._harness.EnableAnalogReporting(PIN_5V_SENSE)
      start_time = time.time()
      while time.time() < start_time + 1:
        vmot = self._harness.analogRead(PIN_12V_SENSE) * COEFFICIENT_12V / 1024
        print "vmotor: %s" % vmot
        if vmot > EXPECTED_12V + TOLERANCE_12V:
          raise BrainScanTestFailure("vmot too high: %s" % vmot)
      if vmot < EXPECTED_12V - TOLERANCE_12V:
        raise BrainScanTestFailure("vmot too low: %s" % vmot)

      start_time = time.time()
      while time.time() < start_time + 1:
        vcc = self._harness.analogRead(PIN_5V_SENSE) * COEFFICIENT_5V / 1024
        print "vcc: %s" % vcc
        if vcc > EXPECTED_5V + TOLERANCE_5V:
          raise BrainScanTestFailure("vcc too high: %s" % vcc)
      if vcc < EXPECTED_5V - TOLERANCE_5V:
        raise BrainScanTestFailure("vcc too low: %s" % vcc)
      self._harness.DisableAnalogReporting(PIN_12V_SENSE)
      self._harness.DisableAnalogReporting(PIN_5V_SENSE)

      time.sleep(1) # wait for serial buffer to clean analog messages

      # Test each stepper coil to ensure current below 0.7A
      start_time = time.time()
      while time.time() < start_time + 2:
        for axis in (BW_X_AXIS, BW_Y_AXIS, BW_Z_AXIS, BW_E_AXIS):
          (coil_a, coil_b) = self.readAxisCurrent(axis)
          if coil_a > 0.7:
            raise BrainScanTestFailure("%s axis coil A current too high" % axis[NAME])
          if coil_b > 0.7:
            raise BrainScanTestFailure("%s axis coil B current too high" % axis[NAME])

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
    self._harness.analogWrite(PIN_RED, color >> 16 & 0xFF)
    self._harness.analogWrite(PIN_GREEN, color >> 8 & 0xFF)
    self._harness.analogWrite(PIN_BLUE, color & 0xFF)

  def readHWB(self):
    return self_harness.digitalRead(PIN_HWB)

  def readReset(self):
    return self_harness.digitalRead(PIN_RESET)

  def readINA219Current(self, address, sense=SENSE_OHMS):
    reply = self._harness._i2c_device.I2CRead(address, 0x01, 2)
    assert type(reply) == list
    assert len(reply) == 2
    shunt = struct.unpack('!h', bytes(chr(reply[0])) + bytes(chr(reply[1])))[0]
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

    # Compensate for poor architectural decisions
    if axis == BW_X_AXIS:
      endstop_pin = PIN_X_MIN
    elif axis == BW_Y_AXIS:
      endstop_pin = PIN_Y_MIN
    elif axis == BW_Z_AXIS:
      endstop_pin = PIN_Z_MIN
    else:
      raise BrainScanTestFailure("Trying to test non-existant Axis.")

    # ensure at least one state transistion before testing
    self._harness.digitalWrite(endstop_pin, 1)
    time.sleep(0.25)
    self._harness.digitalWrite(endstop_pin, 0)
    time.sleep(0.25)
    if target.readEndstop(axis):
      raise BrainScanTestFailure("%s endstop read failure, failed to read LOW" % axis[NAME])
    self._harness.digitalWrite(endstop_pin, 1)
    time.sleep(0.25)
    if not target.readEndstop(axis):
      raise BrainScanTestFailure("%s endstop read failure, failed to read HIGH" % axis[NAME])
    self._harness.digitalWrite(endstop_pin, 0)

    """
    >>> target.readExtruderTemp()
    0.9990234375
    >>> scanner._harness.pinMode(16, OUTPUT)
    >>> target.readExtruderTemp()
    0.53125
    >>> scanner._harness.pinMode(17, OUTPUT)
    >>> target.readExtruderTemp()
    0.4375
    >>> scanner._harness.pinMode(16, INPUT)
    >>> target.readExtruderTemp()
    0.7060546875
    """

  def testExtruderTempSet(self, target, min, max):
    time.sleep(0.25)
    extruder_temp = target.readExtruderTemp()
    print "testing bed temp, expecting between %s and %s, got %s" % (min, max, extruder_temp)
    if not min < extruder_temp < max:
      raise BrainScanTestFailure("Extruder temp failure")

  def testExtruderTemp(self, target):
    try:
      target._target.EnableAnalogReporting(BW_PIN_E_TEMP)

      self._harness.pinMode(PIN_E_POT_HIGH, INPUT)
      self._harness.pinMode(PIN_E_POT_LOW, INPUT)
      self.testExtruderTempSet(target, 0.99, 1)

      self._harness.pinMode(PIN_E_POT_HIGH, OUTPUT)
      self._harness.pinMode(PIN_E_POT_LOW, INPUT)
      self.testExtruderTempSet(target, 0.70, 0.71)

      self._harness.pinMode(PIN_E_POT_HIGH, INPUT)
      self._harness.pinMode(PIN_E_POT_LOW, OUTPUT)
      self.testExtruderTempSet(target, 0.53, 0.54)

      self._harness.pinMode(PIN_E_POT_HIGH, OUTPUT)
      self._harness.pinMode(PIN_E_POT_LOW, OUTPUT)
      self.testExtruderTempSet(target, 0.43, 0.44)
    finally:
      target._target.DisableAnalogReporting(BW_PIN_E_TEMP)

  def testBedTempSet(self, target, min, max):
    time.sleep(0.25)
    bed_temp = target.readBedTemp()
    print "testing bed temp, expecting between %s and %s, got %s" % (min, max, bed_temp)
    if not min < bed_temp < max:
      raise BrainScanTestFailure("Bed temp failure")

  def testBedTemp(self, target):
    try:
      target._target.EnableAnalogReporting(BW_PIN_B_TEMP)

      self._harness.pinMode(PIN_B_POT_HIGH, INPUT)
      self._harness.pinMode(PIN_B_POT_LOW, INPUT)
      self.testBedTempSet(target, 0.99, 1)

      self._harness.pinMode(PIN_B_POT_HIGH, OUTPUT)
      self._harness.pinMode(PIN_B_POT_LOW, INPUT)
      self.testBedTempSet(target, 0.70, 0.71)

      self._harness.pinMode(PIN_B_POT_HIGH, INPUT)
      self._harness.pinMode(PIN_B_POT_LOW, OUTPUT)
      self.testBedTempSet(target, 0.53, 0.54)

      self._harness.pinMode(PIN_B_POT_HIGH, OUTPUT)
      self._harness.pinMode(PIN_B_POT_LOW, OUTPUT)
      self.testBedTempSet(target, 0.43, 0.44)
    finally:
      target._target.DisableAnalogReporting(BW_PIN_B_TEMP)

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
      for step in range(start, start + 4):
        print "step %s: (%s, %s) = %s" % (step, coil_a, coil_b, phases[step % 4])
        if (math.copysign(1, coil_a), math.copysign(1, coil_b)) != phases[step % 4]:
          errors += 1
        target.stepAxis(axis, CW)
        time.sleep(0.25)
        (coil_a, coil_b) = self.readAxisCurrent(axis)
        if ( (coil_a > 0.7) or (coil_b > 0.7) ):
          raise BrainScanTestFailure("%s axis current too high! %s %s" % axis[NAME], coil_a, coil_b)
      print "Errors: %s" % errors
      if errors > 0:
        raise BrainScanTestFailure("%s axis failed step test" % axis[NAME])

      start = phases[(math.copysign(1, coil_a), math.copysign(1, coil_b))]
      for step in range(start, start - 4, -1):
        print "step %s: (%s, %s) = %s" % (step, coil_a, coil_b, phases[step % 4])
        if (math.copysign(1, coil_a), math.copysign(1, coil_b)) != phases[step % 4]:
          errors += 1
        target.stepAxis(axis, CCW)
        time.sleep(0.25)
        (coil_a, coil_b) = self.readAxisCurrent(axis)
        if ( (coil_a > 0.7) or (coil_b > 0.7) ):
          raise BrainScanTestFailure("%s axis current too high! %s %s" % axis[NAME], coil_a, coil_b)
      print "Errors: %s" % errors

      target.enableAxis(axis, attenuate=True)
      time.sleep(0.25)
      start = phases[(math.copysign(1, coil_a), math.copysign(1, coil_b))]
      for step in range(start, start + 4):
        print "step %s: (%s, %s) = %s" % (step, coil_a, coil_b, phases[step % 4])
        if (math.copysign(1, coil_a), math.copysign(1, coil_b)) != phases[step % 4]:
          errors += 1
        target.stepAxis(axis, CW)
        time.sleep(0.25)
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
    start_time = time.time()
    while time.time() < start_time + 1:
      fan_current = self.readTargetCurrent() - idle_current
      if fan_current > 0.2:
        raise BrainScanTestFailure("fan current too high: %s" % fan_current)
      time.sleep(0.1)
    target.assertFan(False)
    if fan_current < 0.08:
      raise BrainScanTestFailure("fan current too low: %s" % fan_current)
    print "fan current: %s" % fan_current

    target.assertExtruderHeat(True)
    start_time = time.time()
    while time.time() < start_time + 1:
      extruder_current = self.readTargetCurrent() - idle_current
      if extruder_current > 2.5:
        raise BrainScanTestFailure("extruder current too high: %s" % extruder_current)
    target.assertExtruderHeat(False)
    if extruder_current < 1.5:
      raise BrainScanTestFailure("extruder current too low: %s" % extruder_current)
    print "extruder current: %s" % extruder_current

    target.assertBedHeat(True)
    start_time = time.time()
    while time.time() < start_time + 1:
      bed_current = self.readTargetCurrent() - idle_current
      if bed_current > 13:
        raise BrainScanTestFailure("bed current too high: %s" % bed_current)
      time.sleep(0.1)
    target.assertBedHeat(False)
    if bed_current < 10:
      raise BrainScanTestFailure("bed current too low: %s" % bed_current)
    print "bed current: %s" % bed_current

    time.sleep(0.25)
    self.testAxis(target, BW_X_AXIS)
    time.sleep(0.25)
    self.testAxis(target, BW_Y_AXIS)
    time.sleep(0.25)
    self.testAxis(target, BW_Z_AXIS)
    time.sleep(0.25)
    self.testAxis(target, BW_E_AXIS)

    self.testEndstop(target, BW_X_AXIS)
    self.testEndstop(target, BW_Y_AXIS)
    self.testEndstop(target, BW_Z_AXIS)

    self.testBedTemp(target)
    self.testExtruderTemp(target)


class Brainwave(object):
  def __init__(self, port):
    self._target = firmata.FirmataInit(port, 57600, '/tmp/brainwave_log')

    self._target.EnableDigitalReporting(0)
    self._target.EnableDigitalReporting(1)
    self._target.EnableDigitalReporting(2)
    self._target.EnableDigitalReporting(3)
    self._target.EnableDigitalReporting(4)
    self._target.EnableDigitalReporting(5)
    
    self._target.SetSamplingInterval(25)

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
    if axis[ENDSTOP] != -1:
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
    #time.sleep(0.1)  # do we need this sleep?
    self._target.digitalWrite(axis[STEP], 1)

  def readEndstop(self, axis):
    return self._target.digitalRead(axis[ENDSTOP])

  def readBedTemp(self):
    ret = self._target.analogRead(BW_PIN_B_TEMP) / 1024.0
    return ret

  def readExtruderTemp(self):
    ret = self._target.analogRead(BW_PIN_E_TEMP) / 1024.0
    return ret


#lcd = Adafruit_CharLCD()
#lcd.clear()
scanner = None
quit = False

#lcd.message("   Brainscan\n Initializing")

while not quit:
  try:
    print "Connecting to test harness via Firmata protocol..."
    scanner = BrainScan(PORT)
    while not quit:
      try:
        scanner.setLEDColor(0xFFFFFF)
        #lcd.clear()
        #lcd.message(" Starting Test")
        print "Starting test"
        
        code.interact('Place board and press ^D', local=locals())
        scanner.powerTargetOn()

        print "Flashing target board with Firmata firmware..."
        # Perform chip erase
        subprocess.check_call(AVRDUDE + ['-e'])
        # Set fuses for flashing
        subprocess.check_call(AVRDUDE + FUSES)
        # Write firmata
        subprocess.check_call(AVRDUDE + ['-D', '-U', 'flash:w:BrainwaveFirmata.cpp.hex:i'])

        print "Waiting for serial device to become available..."
        time.sleep(2)
        print "Connecting to target via Firmata protocol..."
        target = Brainwave("/dev/ttyACM0")

        def test():
          scanner.runTestSuite(target)
        #scanner.runTestSuite(target)
        code.interact('type \'test()\' to run a test', local=locals())
        # TODO: we don't let go of the target's port between runs so we can't do more than one
        #quit = True
        target._target.StopCommunications()

        # write bootloader
        subprocess.check_call(AVRDUDE + ['-U', 'flash:w:BrainwaveBootloaderCDC.hex:i'])
        # Reset fuses and writelock bootloader area
        subprocess.check_call(AVRDUDE + FUSES + ['-U', 'lock:w:0x2f:m'])
        scanner.activateBootloader()
        code.interact('Press PROGRAM and RESET buttons, type ^D to continue', local=locals())
        #time.sleep(2) # wait for linux to find the device
        # This hangs:
        #subprocess.check_call(AVRDUDEBOOT + ['-U', 'flash:w:Sprinter.cpp.hex:i'])
        #scanner.resetTarget()

        print "Powering target down..."
        scanner.powerTargetDown()
        scanner.setLEDColor(0x00FF00)
      except BrainScanTestFailure as e:
        print "Test failure"
        scanner.powerTargetDown()
        scanner.setLEDColor(0xFF0000)
        #lcd.clear()
        #lcd.message(" Test Failure\n%s" % e.msg)
        print e
        time.sleep(10)
    scanner._harness.StopCommunications()
  except subprocess.CalledProcessError:
    scanner.setLEDColor(0xFF0000)
    time.sleep(120)
    raise BrainScanTestFailure("avrdude failure")
  finally:
    #lcd.clear()
    if scanner:
      scanner.powerTargetDown()
