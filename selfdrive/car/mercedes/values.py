from selfdrive.car import dbc_dict

class CAR:
  ECLASS = "Mercedes Benz E350 2010"


#class ECU:
  # CAM = 0 # camera
  # DSU = 1 # driving support unit
  # APGS = 2 # advanced parking guidance system


# addr: (ecu, cars, bus, 1/freq*100, vl)
#STATIC_MSGS = [(0x141, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1,   2, '\x00\x00\x00\x46'),
#               (0x128, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1,   3, '\xf4\x01\x90\x83\x00\x37'),
#
#               (0x292, ECU.APGS, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0,   3, '\x00\x00\x00\x00\x00\x00\x00\x9e'),
#               (0x283, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0,   3, '\x00\x00\x00\x00\x00\x00\x8c'),
#               (0x2E6, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 0,   3, '\xff\xf8\x00\x08\x7f\xe0\x00\x4e'),
#               (0x2E7, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 0,   3, '\xa8\x9c\x31\x9c\x00\x00\x00\x02'),
#
#               (0x240, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1,   5, '\x00\x10\x01\x00\x10\x01\x00'),
#               (0x241, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1,   5, '\x00\x10\x01\x00\x10\x01\x00'),
#               (0x244, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1,   5, '\x00\x10\x01\x00\x10\x01\x00'),
#               (0x245, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1,   5, '\x00\x10\x01\x00\x10\x01\x00'),
#               (0x248, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1,   5, '\x00\x00\x00\x00\x00\x00\x01'),
#               (0x344, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0,   5, '\x00\x00\x01\x00\x00\x00\x00\x50'),
#
#               (0x160, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1,   7, '\x00\x00\x08\x12\x01\x31\x9c\x51'),
#               (0x161, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1,   7, '\x00\x1e\x00\x00\x00\x80\x07'),
#
#               (0x32E, ECU.APGS, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0,  20, '\x00\x00\x00\x00\x00\x00\x00\x00'),
#               (0x33E, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 0,  20, '\x0f\xff\x26\x40\x00\x1f\x00'),
#               (0x365, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 0,  20, '\x00\x00\x00\x80\x03\x00\x08'),
#               (0x365, ECU.DSU, (CAR.RAV4, CAR.COROLLA), 0,  20, '\x00\x00\x00\x80\xfc\x00\x08'),
#               (0x366, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 0,  20, '\x00\x00\x4d\x82\x40\x02\x00'),
#               (0x366, ECU.DSU, (CAR.RAV4, CAR.COROLLA), 0,  20, '\x00\x72\x07\xff\x09\xfe\x00'),
#
#               (0x367, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0,  40, '\x06\x00'),
#
#               (0x414, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100, '\x00\x00\x00\x00\x00\x00\x17\x00'),
#               (0x489, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100, '\x00\x00\x00\x00\x00\x00\x00'),
#               (0x48a, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100, '\x00\x00\x00\x00\x00\x00\x00'),
#               (0x48b, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100, '\x66\x06\x08\x0a\x02\x00\x00\x00'),
#               (0x4d3, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100, '\x1C\x00\x00\x01\x00\x00\x00\x00'),
#               (0x130, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 1, 100, '\x00\x00\x00\x00\x00\x00\x38'),
#               (0x466, ECU.CAM, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4), 1, 100, '\x20\x20\xAD'),
#               (0x466, ECU.CAM, (CAR.COROLLA), 1, 100, '\x24\x20\xB1'),
#               (0x396, ECU.APGS, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100, '\xBD\x00\x00\x00\x60\x0F\x02\x00'),
#               (0x43A, ECU.APGS, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100, '\x84\x00\x00\x00\x00\x00\x00\x00'),
#               (0x43B, ECU.APGS, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100, '\x00\x00\x00\x00\x00\x00\x00\x00'),
#               (0x497, ECU.APGS, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100, '\x00\x00\x00\x00\x00\x00\x00\x00'),
#               (0x4CC, ECU.APGS, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100, '\x0D\x00\x00\x00\x00\x00\x00\x00'),
#               (0x4CB, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.RAV4, CAR.COROLLA), 0, 100, '\x0c\x00\x00\x00\x00\x00\x00\x00'),
#               (0x470, ECU.DSU, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 1, 100, '\x00\x00\x02\x7a'),
#              ]

STATIC_MSGS = []


def check_ecu_msgs(fingerprint, candidate, ecu):
  # return True if fingerprint contains messages normally sent by a given ecu
  ecu_msgs = [x[0] for x in STATIC_MSGS if (x[1] == ecu and
                                            candidate in x[2] and
                                            x[3] == 0)]

  return any(msg for msg in fingerprint if msg in ecu_msgs)


FINGERPRINTS = {
  CAR.ECLASS: [{
    1: 8, 2: 5, 3: 8, 4: 5, 5: 8, 14: 8, 21: 4, 41: 3, 69: 8, 95: 8, 105: 2, 109: 4, 115: 8, 213: 8, 222: 8, 243: 8, 249: 8, 254: 8, 255: 8, 257: 8, 260: 8, 261: 8, 301: 8, 307: 4, 327: 1, 331: 8, 373: 8, 376: 4, 379: 2, 384: 2, 385: 2, 415: 8, 421: 5, 449: 4, 461: 8, 512: 1, 513: 8, 514: 4, 515: 8, 516: 8, 517: 8, 518: 8, 519: 8, 520: 8, 581: 8, 583: 8, 643: 8, 645: 8, 747: 8, 750: 8, 752: 8, 753: 8, 755: 8, 757: 8, 759: 6, 760: 3, 767: 8, 773: 4, 774: 8, 781: 8, 782: 8, 783: 8, 793: 1, 815: 8, 822: 8, 829: 4, 841: 8, 845: 2, 846: 8, 851: 8, 855: 8, 881: 8, 883: 8, 885: 8, 887: 8, 888: 8, 890: 7, 892: 8, 893: 8, 894: 8, 902: 2, 909: 8, 910: 8, 925: 8, 927: 8, 928: 2, 931: 3, 934: 2, 936: 2, 939: 8, 943: 8, 949: 2, 965: 8, 980: 8, 985: 7, 993: 8, 997: 8, 999: 8, 1024: 8, 1025: 8, 1026: 8, 1028: 8, 1030: 8, 1032: 8, 1033: 8, 1047: 8, 1048: 8, 1065: 8, 1073: 8, 1441: 8
  }]
}

DBC = {
  CAR.ECLASS: dbc_dict('mercedes_benz_e350_2010.dbc', None)
}
