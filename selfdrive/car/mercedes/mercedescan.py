
# *** Mercedes specific ***

def calc_checksum(data):
  checksum = 0xFF
  temp_chk = 0
  bit_sum = 0

  for byte in data:
    shift = 0x80
    for i in range(8, 0, -1):
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


# use cruise control stalk to adjust speed
def create_accel_change_command(change, cnt):
  addr = 0x45
  bus = 0
  dat = [0x00, 0xff, 0x00, 0x00, 0x00, 0x00, cnt]

  if change >= 5:
    dat[0] = 0x4
  elif change >= 1:
    dat[0] = 0x8
  elif change <= -5:
    dat[0] = 0x10
  elif change <= -1:
    dat[0] = 0x20

  dat.append(calc_checksum(dat))

  # [addr, 0, msg, bus]
  return [addr, 0, dat, 0]



if __name__ == "__main__":
  print "accel cmds"
  print "+5 mph", repr(create_accel_change_command(5, 0x10))
  print "-5 mph", repr(create_accel_change_command(-5, 0x20))
  print "+1 mph", repr(create_accel_change_command(1, 0x30))
  print "-1 mph", repr(create_accel_change_command(-1, 0x40))
