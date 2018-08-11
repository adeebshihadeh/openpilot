from selfdrive.car.mercedes.values import CAR, DBC
from selfdrive.can.parser import CANParser
from selfdrive.config import Conversions as CV
from common.kalman.simple_kalman import KF1D
import numpy as np

def parse_gear_shifter(can_gear):
  if can_gear == 0x8:
    return "park"
  elif can_gear == 0x7:
    return "reverse"
  elif can_gear == 0x6:
    return "neutral"
  elif can_gear == 0x5:
    return "drive"

  return "unknown"


def get_can_parser(CP):
  signals = [
    # sig_name, sig_address, default
    ("GEAR", "GEAR_PACKET", 0),
    ("DRIVER_BRAKE", "BRAKE_MODULE", 0),
    ("BRAKE_POSITION", "BRAKE_MODULE", 0),
    ("COMBINED_GAS", "GAS_PEDAL", 0),
    ("GAS_PEDAL", "GAS_PEDAL", 0),
    ("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),
    ("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),
    ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),
    ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),
    ("DOOR_OPEN_FL", "DOOR_SENSORS", 1),
    ("DOOR_OPEN_FR", "DOOR_SENSORS", 1),
    ("DOOR_OPEN_RL", "DOOR_SENSORS", 1),
    ("DOOR_OPEN_RR", "DOOR_SENSORS", 1),
    ("SEATBELT_DRIVER_LATCHED", "SEATBELT_SENSORS", 1),
    ("STEER_ANGLE", "STEER_SENSOR", 0),
    ("STEER_RATE", "STEER_SENSOR", 0),
    ("CRUISE_DISABLED", "CRUISE_CONTROL3", 0),
    ("CRUISE_SET_SPEED", "CRUISE_CONTROL3", 0),
    ("LEFT_BLINKER", "DRIVER_CONTROLS", 0),
    ("RIGHT_BLINKER", "DRIVER_CONTROLS", 0),
    ("HIGHBEAM_TOGGLE", "DRIVER_CONTROLS", 0),
  ]

  checks = []

  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)


class CarState(object):
  def __init__(self, CP):
    self.CP = CP
    self.left_blinker_on = 0
    self.right_blinker_on = 0

    # initialize can parser
    self.car_fingerprint = CP.carFingerprint

    # vEgo kalman filter
    dt = 0.01
    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    self.v_ego_kf = KF1D(x0=np.matrix([[0.0], [0.0]]),
                         A=np.matrix([[1.0, dt], [0.0, 1.0]]),
                         C=np.matrix([1.0, 0.0]),
                         K=np.matrix([[0.12287673], [0.29666309]]))
    self.v_ego = 0.0

  def update(self, cp):
    # copy can_valid
    self.can_valid = cp.can_valid

    # update prevs, update must run once per loop
    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on

    self.door_all_closed = not any([cp.vl["DOOR_SENSORS"]['DOOR_OPEN_FL'], cp.vl["DOOR_SENSORS"]['DOOR_OPEN_FR'],
                                    cp.vl["DOOR_SENSORS"]['DOOR_OPEN_RL'], cp.vl["DOOR_SENSORS"]['DOOR_OPEN_RR']])
    self.seatbelt = cp.vl["SEATBELT_SENSORS"]['SEATBELT_DRIVER_LATCHED']

    can_gear = cp.vl["GEAR_PACKET"]['GEAR']
    self.gear_shifter = parse_gear_shifter(can_gear)

    self.brake_pressed = cp.vl["BRAKE_MODULE"]['DRIVER_BRAKE']
    brake_position = cp.vl["BRAKE_MODULE"]['BRAKE_POSITION'] / 1024.
    self.user_brake = brake_position if self.brake_pressed else 0
    self.brake_lights = brake_position > 0

    self.pedal_gas = cp.vl["GAS_PEDAL"]['GAS_PEDAL'] / 256.0
    self.car_gas = cp.vl["GAS_PEDAL"]['COMBINED_GAS'] / 256.0
    self.gas_pressed = self.pedal_gas > 0

    # calc best v_ego estimate, by averaging two opposite corners
    self.v_wheel_fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.MPH_TO_MS
    self.v_wheel_fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.MPH_TO_MS
    self.v_wheel_rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.MPH_TO_MS
    self.v_wheel_rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.MPH_TO_MS

    self.v_wheel = float(np.mean([self.v_wheel_fl, self.v_wheel_fr, self.v_wheel_rl, self.v_wheel_rr]))

    # Kalman filter
    if abs(self.v_wheel - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_x = np.matrix([[self.v_wheel], [0.0]])

    self.v_ego_raw = self.v_wheel
    v_ego_x = self.v_ego_kf.update(self.v_wheel)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])
    self.standstill = not self.v_wheel > 0.001

    self.angle_steers = cp.vl["STEER_SENSOR"]['STEER_ANGLE']
    self.angle_steers_rate = cp.vl["STEER_SENSOR"]['STEER_RATE']
    
    self.main_on = 1
    self.left_blinker_on = cp.vl["DRIVER_CONTROLS"]['LEFT_BLINKER']
    self.right_blinker_on = cp.vl["DRIVER_CONTROLS"]['RIGHT_BLINKER']

    self.v_cruise = cp.vl["CRUISE_CONTROL3"]['CRUISE_SET_SPEED'] * CV.MPH_TO_MS
    self.cruise_enabled = not cp.vl["CRUISE_CONTROL3"]['CRUISE_DISABLED']
    # self.pcm_acc_status = cp.vl["PCM_CRUISE"]['CRUISE_STATE']]
    
    
    self.generic_toggle = bool(cp.vl["DRIVER_CONTROLS"]['HIGHBEAM_TOGGLE'])
