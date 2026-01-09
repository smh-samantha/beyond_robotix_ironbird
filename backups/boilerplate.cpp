// main.cpp  (MicroNode: MCP23008 raw I2C LED bring-up, wedge-proof)
// - Uses setup() + while(true) pattern (NO custom main())
// - Keeps app_setup() + watchdog
// - I2C on PA9(SCL) + PA10(SDA) using default Wire object but forced pin mux
// - Auto-detect MCP23008 addr 0x20..0x27
// - Configures GP0/GP1 once
// - LED cycle uses ONLY write (no readback) to avoid HAL lockups
// - 1 Hz diagnostics includes readback (OLAT + GPIO) + bus recovery if wedged

#include <Arduino.h>
#include <Wire.h>
#include <IWatchdog.h>
#include <app.h>

// MCP23008 registers
static constexpr uint8_t REG_IODIR = 0x00;
static constexpr uint8_t REG_GPIO  = 0x09;
static constexpr uint8_t REG_OLAT  = 0x0A;

static constexpr uint8_t GP0_BIT = 0;
static constexpr uint8_t GP1_BIT = 1;

static uint8_t  mcp_addr   = 0;
static bool     mcp_found  = false;
static bool     mcp_ready  = false;

static uint32_t t_diag  = 0;
static uint32_t t_cycle = 0;
static uint8_t  cycle_state = 0;

static void print_hex_u8(uint8_t v) {
  if (v < 16) Serial.print('0');
  Serial.print(v, HEX);
}

// --- Force PA9/PA10 to I2C1 AF and bind Wire to them ---
static void init_i2c_pa9_pa10() {
  pin_function(digitalPinToPinName(PA9),  STM_PIN_DATA(STM_MODE_AF_OD, GPIO_PULLUP, GPIO_AF4_I2C1));  // SCL
  pin_function(digitalPinToPinName(PA10), STM_PIN_DATA(STM_MODE_AF_OD, GPIO_PULLUP, GPIO_AF4_I2C1));  // SDA

  Wire.setSCL(PA9);
  Wire.setSDA(PA10);

  Wire.begin();
  Wire.setClock(50000);
  Wire.setTimeout(10); // may not fully protect from HAL waits, but keep it
}

// --- Bus recovery: toggle SCL to free SDA if device holds SDA low ---
static void i2c_bus_recover_pa9_pa10() {
  Serial.println("I2C RECOVER: toggling SCL to free bus...");

  // Deinit Wire and take over pins as GPIO open-drain
  Wire.end();

  pinMode(PA9, OUTPUT_OPEN_DRAIN);   // SCL
  pinMode(PA10, INPUT_PULLUP);       // SDA (let it float high)

  digitalWrite(PA9, HIGH);
  delay(2);

  // Pulse SCL ~16 times
  for (int i = 0; i < 16; i++) {
    IWatchdog.reload();
    digitalWrite(PA9, LOW);
    delayMicroseconds(10);
    digitalWrite(PA9, HIGH);
    delayMicroseconds(10);
  }

  // Optional STOP condition: SDA low->high while SCL high
  pinMode(PA10, OUTPUT_OPEN_DRAIN);
  digitalWrite(PA10, LOW);
  delayMicroseconds(10);
  digitalWrite(PA9, HIGH);
  delayMicroseconds(10);
  digitalWrite(PA10, HIGH);
  delayMicroseconds(10);

  // Re-init I2C
  init_i2c_pa9_pa10();
}

// --- I2C primitives ---
static bool i2c_probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission(true) == 0);
}

static bool mcp_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission(true) == 0);
}

static bool mcp_read_reg(uint8_t addr, uint8_t reg, uint8_t &out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 1, (int)true) != 1) return false;
  out = Wire.read();
  return true;
}

// --- Discovery ---
static void find_mcp_addr() {
  mcp_found = false;
  mcp_addr  = 0;

  for (uint8_t addr = 0x20; addr <= 0x27; addr++) {
    IWatchdog.reload();
    if (i2c_probe(addr)) {
      mcp_found = true;
      mcp_addr  = addr;
      break;
    }
  }

  Serial.print("MCP23008 addr: ");
  if (mcp_found) {
    Serial.print("0x"); print_hex_u8(mcp_addr);
    Serial.println(" (FOUND)");
  } else {
    Serial.println("NOT FOUND (0x20..0x27)");
  }
}

static bool mcp_config_gp0_gp1_outputs() {
  // GP0/GP1 outputs => IODIR bits 0,1 = 0; others inputs = 1 => 0xFC
  if (!mcp_write_reg(mcp_addr, REG_IODIR, 0xFC)) return false;

  uint8_t iodir = 0xFF;
  if (!mcp_read_reg(mcp_addr, REG_IODIR, iodir)) return false;

  Serial.print("IODIR readback: 0x");
  print_hex_u8(iodir);
  Serial.println();

  return (iodir == 0xFC);
}

// --- LED control (FAST PATH: write only, no reads) ---
static bool mcp_set_leds_active_low_fast(bool gp0_on, bool gp1_on) {
  // ACTIVE-LOW: ON => 0, OFF => 1
  uint8_t olat = 0xFF;

  // Maintain OLAT locally (avoid readback here)
  // Start from "both off" (bits high) plus preserve other bits as high.
  // We only care about bits 0 and 1.
  olat = 0xFF;
  if (gp0_on) olat &= ~(1u << GP0_BIT); else olat |= (1u << GP0_BIT);
  if (gp1_on) olat &= ~(1u << GP1_BIT); else olat |= (1u << GP1_BIT);

  const bool ok = mcp_write_reg(mcp_addr, REG_OLAT, olat);

  Serial.print("LEDs GP0=");
  Serial.print(gp0_on ? "ON" : "OFF");
  Serial.print(" GP1=");
  Serial.print(gp1_on ? "ON" : "OFF");
  Serial.print("  OLAT=0x");
  print_hex_u8(olat);
  Serial.print(" write=");
  Serial.println(ok ? "OK" : "FAIL");

  return ok;
}

// --- 1 Hz diagnostics: safe-ish readbacks + recovery ---
static void mcp_diag_readback_and_health() {
  if (!mcp_found) return;

  // Probe first (cheap)
  bool alive = i2c_probe(mcp_addr);
  Serial.print("HEALTH: alive=");
  Serial.println(alive ? "Y" : "N");

  if (!alive) {
    i2c_bus_recover_pa9_pa10();
    alive = i2c_probe(mcp_addr);
    Serial.print("HEALTH(after recover): alive=");
    Serial.println(alive ? "Y" : "N");
    if (!alive) {
      mcp_ready = false;
      mcp_found = false;
      return;
    }
  }

  // Readbacks (these are the most likely to wedge)
  uint8_t olat = 0xFF, gpio = 0xFF;
  bool ok_olat = mcp_read_reg(mcp_addr, REG_OLAT, olat);
  bool ok_gpio = mcp_read_reg(mcp_addr, REG_GPIO, gpio);

  if (!ok_olat || !ok_gpio) {
    Serial.println("DIAG: readback FAIL -> attempting bus recover");
    i2c_bus_recover_pa9_pa10();
    ok_olat = mcp_read_reg(mcp_addr, REG_OLAT, olat);
    ok_gpio = mcp_read_reg(mcp_addr, REG_GPIO, gpio);
  }

  Serial.print("READBACK: OLAT=");
  if (ok_olat) { Serial.print("0x"); print_hex_u8(olat); }
  else { Serial.print("FAIL"); }

  Serial.print(" GPIO=");
  if (ok_gpio) { Serial.print("0x"); print_hex_u8(gpio); }
  else { Serial.print("FAIL"); }

  Serial.println();
}

void setup() {
  app_setup();
  IWatchdog.begin(2000000);

  Serial.begin(115200);
  delay(300);

  Serial.println("Starting RAW MCP23008 LED test (wedge-proof)...");
  Serial.println("I2C = PA9(SCL), PA10(SDA). Searching 0x20..0x27.");

  init_i2c_pa9_pa10();

  find_mcp_addr();
  if (mcp_found) {
    mcp_ready = mcp_config_gp0_gp1_outputs();
    Serial.print("MCP config GP0/GP1 outputs: ");
    Serial.println(mcp_ready ? "OK" : "FAIL");
    if (mcp_ready) {
      (void)mcp_set_leds_active_low_fast(false, false);
      cycle_state = 0;
      t_cycle = millis();
    }
  }

  while (true) {
    const uint32_t now = millis();

    // 1 Hz diagnostics (includes readback + recovery)
    if (now - t_diag >= 1000) {
      t_diag = now;

      if (!mcp_found) {
        find_mcp_addr();
        if (mcp_found) {
          mcp_ready = mcp_config_gp0_gp1_outputs();
          Serial.print("MCP config GP0/GP1 outputs: ");
          Serial.println(mcp_ready ? "OK" : "FAIL");
          if (mcp_ready) {
            (void)mcp_set_leds_active_low_fast(false, false);
            cycle_state = 0;
            t_cycle = now;
          }
        }
      } else {
        mcp_diag_readback_and_health();
      }

      Serial.print("STATE: found=");
      Serial.print(mcp_found ? "Y" : "N");
      Serial.print(" ready=");
      Serial.println(mcp_ready ? "Y" : "N");
    }

    // 500 ms LED cycle (WRITE-ONLY; if a write fails, recover)
    if (mcp_found && mcp_ready && (now - t_cycle >= 500)) {
      t_cycle = now;

      bool ok = true;
      switch (cycle_state) {
        case 0: ok = mcp_set_leds_active_low_fast(true,  false); break;
        case 1: ok = mcp_set_leds_active_low_fast(false, true ); break;
        case 2: ok = mcp_set_leds_active_low_fast(true,  true ); break;
        default:ok = mcp_set_leds_active_low_fast(false, false); break;
      }
      cycle_state = (cycle_state + 1) & 0x03;

      if (!ok) {
        Serial.println("LED write FAIL -> bus recover + re-init MCP");
        i2c_bus_recover_pa9_pa10();
        // Reconfigure expander after recovery
        mcp_ready = mcp_config_gp0_gp1_outputs();
        Serial.print("Reconfig after recover: ");
        Serial.println(mcp_ready ? "OK" : "FAIL");
      }
    }

    IWatchdog.reload();
  }
}

void loop() {
  // unused
}
