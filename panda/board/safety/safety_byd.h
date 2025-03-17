const CanMsg BYD_TX_MSGS[] = {{0x250, 0, 8}};

RxCheck byd_rx_checks[] = {
  //{.msg = {{0x35F, 0, 8, .frequency = 20U}, { 0 }, { 0 }}},
};

static void byd_rx_hook(const CANPacket_t *to_push) {
  vehicle_moving = true;
  controls_allowed = true;
  UNUSED(to_push);
}

static bool byd_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);
  int len = GET_LEN(to_send);
  UNUSED(addr);
  UNUSED(len);

  return tx;
}

static int byd_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  if (bus_num == 0) {
    bus_fwd = 2;
  }

  if (bus_num == 2) {
    bool is_lkas_msg = ((addr == 482) || (addr == 790));
    bool is_acc_msg = (addr == 814);
    bool block_msg = is_lkas_msg || is_acc_msg;
    if (!block_msg) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

static safety_config byd_init(uint16_t param) {
  UNUSED(param);
  return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
}


const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
};
