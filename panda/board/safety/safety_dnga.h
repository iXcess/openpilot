// CAN messages
#define DNGA_GAS_SENSOR         0x201
#define DNGA_BRAKE              0x0A1
#define DNGA_STEERING_MODULE    0x0A4
#define DNGA_GAS_PEDAL          0x18E
#define DNGA_GAS_PEDAL_2        0x18F
#define DNGA_BUTTONS            0x1AB
#define DNGA_EPS_SHAFT_TORQUE   0x1C0
#define DNGA_PCM_BUTTONS_HYBRID 0x207
#define DNGA_PCM_BUTTONS        0x208
#define DNGA_GAS_COMMAND        0x200
#define DNGA_TORQUE_COMMAND     0x202
#define DNGA_STEERING_LKAS      0x1D0
#define DNGA_ACC_BRAKE          0x271
#define DNGA_ACC_CMD_HUD        0x273
#define DNGA_LKAS_HUD           0x274

const CanMsg DNGA_TX_MSGS[] = {{0x250, 0, 8}, {0x250, 0, 6}, {0x251, 0, 5},  // dnga
                               {0x350, 0, 8}, {0x350, 0, 6}, {0x351, 0, 5},  // knee
                               {0x1, 0, 8}}; // CAN flasher

RxCheck dnga_rx_checks[] = {
  {.msg = {{0x201, 0, 8, .check_checksum = false, .max_counter = 0U, .frequency = 100U}, { 0 }, { 0 }}},
};

static void dnga_rx_hook(const CANPacket_t *to_push) {
  // dnga is never at standstill
  vehicle_moving = true;

  if (GET_ADDR(to_push) == 0x201U) {
    controls_allowed = true;
  }
}

static bool dnga_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = GET_ADDR(to_send);
  int len = GET_LEN(to_send);

  if (!controls_allowed && (addr != 0x1)) {
    tx = false;
  }

  // Allow going into CAN flashing mode for base & knee even if controls are not allowed
  bool flash_msg = ((addr == 0x250) || (addr == 0x350)) && (len == 8);
  if (!controls_allowed && (GET_BYTES(to_send, 0, 4) == 0xdeadfaceU) && (GET_BYTES(to_send, 4, 4) == 0x0ab00b1eU) && flash_msg) {
    tx = true;
  }

  return tx;
}

static safety_config dnga_init(uint16_t param) {
  UNUSED(param);
  return BUILD_SAFETY_CFG(dnga_rx_checks, DNGA_TX_MSGS);
}

const safety_hooks dnga_hooks = {
  .init = dnga_init,
  .rx = dnga_rx_hook,
  .tx = dnga_tx_hook,
  .fwd = default_fwd_hook,
};
