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
    ("GAS_PEDAL", "GAS_PEDAL", 0),
    # ("WHEEL_ENCODER_FL", "WHEEL_ENCODERS", 0),
    # ("WHEEL_ENCODER_FR", "WHEEL_ENCODERS", 0),
    # ("WHEEL_ENCODER_RL", "WHEEL_ENCODERS", 0),
    # ("WHEEL_ENCODER_RR", "WHEEL_ENCODERS", 0),
    ("DOOR_OPEN_FL", "DOOR_SENSORS", 1),
    ("DOOR_OPEN_FR", "DOOR_SENSORS", 1),
    ("DOOR_OPEN_RL", "DOOR_SENSORS", 1),
    ("DOOR_OPEN_RR", "DOOR_SENSORS", 1),
    ("SEATBELT_DRIVER_LATCHED", "SEATBELT_SENSORS", 1),
    # ("TC_DISABLED", "ESP_CONTROL", 1),
    ("STEER_ANGLE", "STEER_SENSOR", 0),
    ("STEER_RATE", "STEER_SENSOR", 0),
    # ("GAS_RELEASED", "PCM_CRUISE", 0),
    # ("CRUISE_STATE", "PCM_CRUISE", 0),
    # ("MAIN_ON", "PCM_CRUISE_2", 0),
    ("CRUISE_SET_SPEED", "CRUISE_CONTROL3", 0),
    # ("LOW_SPEED_LOCKOUT", "PCM_CRUISE_2", 0),
    # ("STEER_TORQUE_DRIVER", "STEER_TORQUE_SENSOR", 0),
    # ("STEER_TORQUE_EPS", "STEER_TORQUE_SENSOR", 0),
    ("LEFT_BLINKER", "DRIVER_CONTROLS", 0),
    ("RIGHT_BLINKER", "DRIVER_CONTROLS", 0),
    # ("IPAS_STATE", "EPS_STATUS", 1),
    # ("BRAKE_LIGHTS_ACC", "ESP_CONTROL", 0),
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

    # TODOO: stop using gps. find wheel speed on can
    self.gps = messaging.sub_sock(context, service_list['gpsLocation'].port)
    self.speed = 0

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
    self.brake_pressed = cp.vl["BRAKE_MODULE"]['DRIVER_BRAKE']
    self.pedal_gas = cp.vl["GAS_PEDAL"]['GAS_PEDAL']
    self.car_gas = self.pedal_gas
    # self.esp_disabled = cp.vl["ESP_CONTROL"]['TC_DISABLED']

    # TODOO: find wheel speed signals, not wheel encoders
    # calc best v_ego estimate, by averaging two opposite corners
    # self.v_wheel_fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS
    # self.v_wheel_fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS
    # self.v_wheel_rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS
    # self.v_wheel_rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS

    gps = messaging.recv_sock(self.gps)
    if gps is not None:
      self.speed = gps.gpsLocation.speed

    self.v_wheel_fl = self.speed
    self.v_wheel_fr = self.speed
    self.v_wheel_rl = self.speed
    self.v_wheel_rr = self.speed

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
    self.gear_shifter = parse_gear_shifter(can_gear)
    # self.main_on = cp.vl["PCM_CRUISE_2"]['MAIN_ON']
    self.left_blinker_on = cp.vl["DRIVER_CONTROLS"]['LEFT_BLINKER']
    self.right_blinker_on = cp.vl["DRIVER_CONTROLS"]['RIGHT_BLINKER']

    self.brake_error = 0
    # self.steer_torque_driver = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_DRIVER']
    # self.steer_torque_motor = cp.vl["STEER_TORQUE_SENSOR"]['STEER_TORQUE_EPS']

    self.user_brake = 0
    self.v_cruise = cp.vl["CRUISE_CONTROL3"]['CRUISE_SET_SPEED'] * CV.MPH_TO_MS
    # self.pcm_acc_status = cp.vl["PCM_CRUISE"]['CRUISE_STATE']
    # self.gas_pressed = not cp.vl["PCM_CRUISE"]['GAS_RELEASED']
    self.gas_pressed = True
    self.brake_lights = self.brake_pressed > 0
    self.generic_toggle = bool(cp.vl["DRIVER_CONTROLS"]['HIGHBEAM_TOGGLE'])
