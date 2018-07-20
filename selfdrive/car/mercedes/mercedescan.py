import struct


# *** Mercedes specific ***

def calc_checksum(data):
  checksum = 0xFF
  temp_chk = 0
  bit_sum = 0

  for byte in data:
    shift = 0x80
    iterate = 8
    while iterate:
      iterate -= 1
      bit_sum = byte & shift
      temp_chk = checksum & 0x80
      if bit_sum != 0:
        bit_sum = 0x1C
        if (temp_chk != 0):
          bit_sum = 1
        checksum = checksum << 1
        temp_chk = checksum | 1
        bit_sum ^= temp_chk
      else:
        if temp_chk != 0:
          bit_sum = 0x1D
        checksum = checksum << 1
        bit_sum ^= checksum
      checksum = bit_sum
      shift = shift >> 1
  return ~checksum & 0xFF


def make_can_msg(addr, dat, alt, cks=False):
  if cks:
    dat.append(calc_checksum(dat))
  return [addr, 0, dat, alt]





# ********** unimplemented stuff taken from toyotacan.py **********



def create_ipas_steer_command(packer, steer, enabled, apgs_enabled):
  # """Creates a CAN message for the Toyota Steer Command."""
  # if steer < 0:
  #   direction = 3
  # elif steer > 0:
  #   direction = 1
  # else:
  #   direction = 2

  # mode = 3 if enabled else 1

  # values = {
  #   "STATE": mode,
  #   "DIRECTION_CMD": direction,
  #   "ANGLE": steer,
  #   "SET_ME_X10": 0x10,
  #   "SET_ME_X40": 0x40
  # }
  # if apgs_enabled:
  #   return packer.make_can_msg("STEERING_IPAS", 0, values)
  # else:
  #   return packer.make_can_msg("STEERING_IPAS_COMMA", 0, values)
  return None


def create_steer_command(packer, steer, raw_cnt):
  # """Creates a CAN message for the Toyota Steer Command."""

  # values = {
  #   "STEER_REQUEST": abs(steer) > 0.001,
  #   "STEER_TORQUE_CMD": steer,
  #   "COUNTER": raw_cnt,
  #   "SET_ME_1": 1,
  # }
  # return packer.make_can_msg("STEERING_LKA", 0, values)
  return None


def create_accel_command(packer, accel, pcm_cancel, standstill_req):
  # TODO: find the exact canceling bit
  values = {
    "ACCEL_CMD": accel,
    "SET_ME_X63": 0x63,
    "SET_ME_1": 1,
    "RELEASE_STANDSTILL": not standstill_req,
    "CANCEL_REQ": pcm_cancel,
  }
  return packer.make_can_msg("ACC_CONTROL", 0, values)


def create_ui_command(packer, steer, sound1, sound2):
  # values = {
  #   "RIGHT_LINE": 1,
  #   "LEFT_LINE": 1,
  #   "SET_ME_X0C": 0x0c,
  #   "SET_ME_X2C": 0x2c,
  #   "SET_ME_X38": 0x38,
  #   "SET_ME_X02": 0x02,
  #   "SET_ME_X01": 1,
  #   "SET_ME_X01_2": 1,
  #   "REPEATED_BEEPS": sound1,
  #   "TWO_BEEPS": sound2,
  #   "LDA_ALERT": steer,
  # }
  # return packer.make_can_msg("LKAS_HUD", 0, values)
  return None

def fix(msg, addr):
  checksum = 0
  idh = (addr & 0xff00) >> 8
  idl = (addr & 0xff)

  checksum = idh + idl + len(msg) + 1
  for d_byte in msg:
    checksum += ord(d_byte)

  #return msg + chr(checksum & 0xFF)
  return msg + struct.pack("B", checksum & 0xFF)