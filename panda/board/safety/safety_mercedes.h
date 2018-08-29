uint16_t mercedes_ign = 0;

static void mercerdes_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int bus_number = (to_push->RDTR >> 4) & 0xFF;
  uint32_t addr = to_push->RIR >> 21;

  controls_allowed = 1;

  // first 6 bytes in this message are 0xff when ignition is off
  if (addr == 0x245 && bus_number == 0) {
    mercedes_ign = (mercedes_ign << 1) + (to_push->RDLR != 0xffffffff);
  }

  // // enter controls on rising edge of ACC, exit controls on ACC off
  // if ((addr == 0x370) && (bus_number == 0)) {
  //   int cruise_engaged = to_push->RDLR & 0x800000;  // bit 23
  //   if (cruise_engaged && !cadillac_cruise_engaged_last) {
  //     controls_allowed = 1;
  //   } else if (!cruise_engaged) {
  //     controls_allowed = 0;
  //   }
  //   cadillac_cruise_engaged_last = cruise_engaged;
  // }
}

static int mercedes_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  // uint32_t addr = to_send->RIR >> 21;
  return true;
}

static void mercedes_init(int16_t param) {
  controls_allowed = 0;
  mercedes_ign = 0;
}

static int mercedes_ign_hook() {
  return mercedes_ign > 0;
}

const safety_hooks mercedes_hooks = {
  .init = mercedes_init,
  .rx = mercerdes_rx_hook,
  .tx = mercedes_tx_hook,
  .tx_lin = alloutput_tx_lin_hook,
  .ignition = mercedes_ign_hook,
  .fwd = alloutput_fwd_hook,
};
