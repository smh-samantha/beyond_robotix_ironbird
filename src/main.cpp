// main.cpp
// Minimal MCP23008 CS test + DroneCAN boilerplate

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X08.h>
#include <IWatchdog.h>
#include <app.h>
#include <dronecan.h>
#include <simple_dronecanmessages.h>
#include <stdio.h>
#include <vector>

// -------------------- User config --------------------
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint8_t MCP_I2C_ADDR = 0x20;

static const uint8_t CS_PINS[] = { 0, 1, 2, 3 }; // MCP GPIO0..3
static constexpr uint8_t CS_COUNT = sizeof(CS_PINS) / sizeof(CS_PINS[0]);

// -------------------- MCP23008 --------------------
static Adafruit_MCP23X08 mcp;

// MCP23008 registers
static constexpr uint8_t MCP_REG_IODIR = 0x00;
static constexpr uint8_t MCP_REG_GPPU  = 0x06;
static constexpr uint8_t MCP_REG_GPIO  = 0x09;
static constexpr uint8_t MCP_REG_OLAT  = 0x0A;

static uint8_t mcp_olat = 0xFF;

// --- Force PA9/PA10 to I2C1 AF and bind Wire to them ---
static void init_i2c_pa9_pa10() {
  pin_function(digitalPinToPinName(PA9),  STM_PIN_DATA(STM_MODE_AF_OD, GPIO_PULLUP, GPIO_AF4_I2C1));  // SCL
  pin_function(digitalPinToPinName(PA10), STM_PIN_DATA(STM_MODE_AF_OD, GPIO_PULLUP, GPIO_AF4_I2C1));  // SDA

  Wire.setSCL(PA9);
  Wire.setSDA(PA10);

  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeout(10);
}

static bool mcp_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MCP_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

static bool mcp_read_reg(uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(MCP_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MCP_I2C_ADDR, 1, (int)true) != 1) return false;
  out = Wire.read();
  return true;
}

static bool init_mcp() {
  if (!mcp.begin_I2C(MCP_I2C_ADDR, &Wire)) return false;

  // GP0..3 outputs, GP4..7 inputs
  if (!mcp_write_reg(MCP_REG_IODIR, 0xF0)) return false;
  // Disable pull-ups (inputs will float unless externally pulled)
  if (!mcp_write_reg(MCP_REG_GPPU, 0x00)) return false;
  // Default: drive GP0..3 LOW (for test)
  mcp_olat = 0xF0;
  if (!mcp_write_reg(MCP_REG_OLAT, mcp_olat)) return false;

  return true;
}

static void scan_i2c_bus() {
  char line[64];
  size_t idx = 0;
  idx += (size_t)snprintf(line + idx, sizeof(line) - idx, "I2C scan:");
  for (uint8_t addr = 0x20; addr <= 0x27; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission(true);
    if (err == 0) {
      idx += (size_t)snprintf(
        line + idx,
        sizeof(line) - idx,
        " 0x%02X",
        addr
      );
    }
    IWatchdog.reload();
  }
  if (Serial.availableForWrite() >= (int)(idx + 2)) {
    Serial.println(line);
  }
}

static void print_mcp_regs(uint32_t heartbeat_count, uint8_t cs_index) {
  uint8_t iodir = 0xFF;
  uint8_t gpio  = 0xFF;
  uint8_t olat  = 0xFF;
  (void)mcp_read_reg(MCP_REG_IODIR, iodir);
  (void)mcp_read_reg(MCP_REG_GPIO, gpio);
  (void)mcp_read_reg(MCP_REG_OLAT, olat);

  char line[96];
  int len = snprintf(
    line,
    sizeof(line),
    "ALIVE %lu CS=%u IODIR=0x%02X GPIO=0x%02X OLAT=0x%02X",
    (unsigned long)heartbeat_count,
    (unsigned int)cs_index,
    (unsigned int)iodir,
    (unsigned int)gpio,
    (unsigned int)olat
  );
  if (len > 0 && Serial.availableForWrite() >= len + 2) {
    Serial.println(line);
  }
}

static void cs_hold_low(uint8_t pin, uint32_t hold_ms) {
  mcp_olat &= (uint8_t)~(1u << pin);
  (void)mcp_write_reg(MCP_REG_OLAT, mcp_olat);
  const uint32_t start = millis();
  while (millis() - start < hold_ms) {
    IWatchdog.reload();
  }
  mcp_olat |= (uint8_t)(1u << pin);
  (void)mcp_write_reg(MCP_REG_OLAT, mcp_olat);
}

// -------------------- DroneCAN boilerplate --------------------
std::vector<DroneCAN::parameter> custom_parameters = {
  { "NODEID", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 100,  0, 127 },
  { "PARM_1", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
  { "PARM_2", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
  { "PARM_3", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
  { "PARM_4", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
  { "PARM_5", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
  { "PARM_6", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
  { "PARM_7", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE,   0.0f, 0.0f, 100.0f },
};

static DroneCAN dronecan;

static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer) {
  switch (transfer->data_type_id) {
    case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID: {
      uavcan_equipment_ahrs_MagneticFieldStrength pkt{};
      uavcan_equipment_ahrs_MagneticFieldStrength_decode(transfer, &pkt);
      break;
    }
  }
  DroneCANonTransferReceived(dronecan, ins, transfer);
}

static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id) {
  if (transfer_type == CanardTransferTypeBroadcast) {
    switch (data_type_id) {
      case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID: {
        *out_data_type_signature = UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_SIGNATURE;
        return true;
      }
    }
  }
  return false || DroneCANshoudlAcceptTransfer(ins, out_data_type_signature, data_type_id, transfer_type, source_node_id);
}
 
void setup() {
  app_setup();
  IWatchdog.begin(2000000);
  Serial.begin(SERIAL_BAUD);
  Serial.println("Starting MCP23008 CS test...");

  init_i2c_pa9_pa10();

  if (!init_mcp()) {
    Serial.println("ERROR: MCP23008 not found on I2C");
  } else {
    Serial.println("MCP23008 OK");
  }

  dronecan.version_major = 1;
  dronecan.version_minor = 0;
  dronecan.init(
    onTransferReceived,
    shouldAcceptTransfer,
    custom_parameters,
    "Beyond Robotix Node"
  );

  uint8_t cs_index = 0;
  uint32_t last_pulse_ms = 0;
  uint32_t last_diag_ms = 0;
  uint32_t last_scan_ms = 0;
  uint32_t heartbeat_ms = 0;
  uint32_t heartbeat_count = 0;

  while (true) {
    const uint32_t now = millis();

    if (now - last_diag_ms >= 1000) {
      last_diag_ms = now;
      print_mcp_regs(heartbeat_count, cs_index);
    }

    if (now - last_scan_ms >= 10000) {
      last_scan_ms = now;
      scan_i2c_bus();
    }

    if (now - heartbeat_ms >= 1000) {
      heartbeat_ms = now;
      heartbeat_count++;
    }

    if (now - last_pulse_ms >= 500) {
      last_pulse_ms = now;
      const uint8_t pin = CS_PINS[cs_index];
      cs_hold_low(pin, 50);
      cs_index = (uint8_t)((cs_index + 1) % CS_COUNT);
    }

    dronecan.cycle();
    IWatchdog.reload();
  }
}

void loop() {
  // Use while(true) in setup.
}
