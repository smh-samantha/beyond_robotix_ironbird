// main.cpp  (MicroNode: MCP23008 provides CS for 2x BMP5xx; RAW SPI chip-id test)
// I2C: PA9(SCL), PA10(SDA)
// SPI: PA5(SCK), PA6(MISO), PA7(MOSI)
// MCP23008 @ 0x20..0x27, GP0/GP1 used as ACTIVE-LOW CS lines

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <IWatchdog.h>
#include <app.h>

// ---- MCP23008 regs ----
static constexpr uint8_t REG_IODIR = 0x00;
static constexpr uint8_t REG_GPIO  = 0x09;
static constexpr uint8_t REG_OLAT  = 0x0A;

static constexpr uint8_t CS0_BIT = 0; // GP0
static constexpr uint8_t CS1_BIT = 1; // GP1

static uint8_t mcp_addr = 0;
static bool mcp_ok = false;

static void print_hex_u8(uint8_t v) {
  if (v < 16) Serial.print('0');
  Serial.print(v, HEX);
}

static void init_i2c_pa9_pa10() {
  pin_function(digitalPinToPinName(PA9),  STM_PIN_DATA(STM_MODE_AF_OD, GPIO_PULLUP, GPIO_AF4_I2C1));  // SCL
  pin_function(digitalPinToPinName(PA10), STM_PIN_DATA(STM_MODE_AF_OD, GPIO_PULLUP, GPIO_AF4_I2C1)); // SDA
  Wire.setSCL(PA9);
  Wire.setSDA(PA10);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setTimeout(10);
}

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

static bool find_mcp(uint8_t &out_addr) {
  for (uint8_t a = 0x20; a <= 0x27; a++) {
    IWatchdog.reload();
    if (i2c_probe(a)) { out_addr = a; return true; }
  }
  return false;
}

// Set GP0/GP1 outputs, default HIGH (deselected)
static bool mcp_init_cs() {
  if (!find_mcp(mcp_addr)) return false;

  Serial.print("MCP found @ 0x");
  print_hex_u8(mcp_addr);
  Serial.println();

  // GP0/GP1 outputs, others inputs => 0b11111100 = 0xFC
  if (!mcp_write_reg(mcp_addr, REG_IODIR, 0xFC)) {
    Serial.println("IODIR write FAIL");
    return false;
  }

  // deselect both CS: GP0=1 GP1=1
  uint8_t olat = 0xFF;
  olat |= (1u << CS0_BIT);
  olat |= (1u << CS1_BIT);
  if (!mcp_write_reg(mcp_addr, REG_OLAT, olat)) {
    Serial.println("OLAT write FAIL");
    return false;
  }

  // Readback diagnostics
  uint8_t iodir_rb = 0xAA, olat_rb = 0xAA, gpio_rb = 0xAA;
  bool ok1 = mcp_read_reg(mcp_addr, REG_IODIR, iodir_rb);
  bool ok2 = mcp_read_reg(mcp_addr, REG_OLAT,  olat_rb);
  bool ok3 = mcp_read_reg(mcp_addr, REG_GPIO,  gpio_rb);

  Serial.print("IODIR_rb("); Serial.print(ok1 ? "OK" : "FAIL"); Serial.print(")=0x"); print_hex_u8(iodir_rb);
  Serial.print("  OLAT_rb("); Serial.print(ok2 ? "OK" : "FAIL"); Serial.print(")=0x"); print_hex_u8(olat_rb);
  Serial.print("  GPIO_rb("); Serial.print(ok3 ? "OK" : "FAIL"); Serial.print(")=0x"); print_hex_u8(gpio_rb);
  Serial.println();

  // If this is not 0xFC, stop: we are not controlling pins the way we think.
  if (!ok1 || iodir_rb != 0xFC) {
    Serial.println("WARNING: IODIR readback is not 0xFC. MCP type/register-map may not match.");
    // still return true so you can see CS toggling behavior, but flag it
  }

  return true;
}

// Active-low select
static void mcp_set_cs(bool cs0_active, bool cs1_active) {
  uint8_t olat = 0xFF;
  if (cs0_active) olat &= ~(1u << CS0_BIT); else olat |= (1u << CS0_BIT);
  if (cs1_active) olat &= ~(1u << CS1_BIT); else olat |= (1u << CS1_BIT);

  (void)mcp_write_reg(mcp_addr, REG_OLAT, olat);

  // Optional readback each time (useful if you suspect it freezes)
  // uint8_t rb=0; mcp_read_reg(mcp_addr, REG_OLAT, rb);
}

static void init_spi_pa5_pa6_pa7() {
  SPI.setSCLK(PA5);
  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.begin();
}

// BMP5xx: chip id at reg 0x01, read is 0x80 | reg
static uint8_t bmp_read_chip_id_via_cs(bool use_cs0) {
  // Ensure only one selected
  mcp_set_cs(use_cs0, !use_cs0);
  delayMicroseconds(50);

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  uint8_t id = 0;

  // send read command + reg
  SPI.transfer(0x80 | 0x01);
  id = SPI.transfer(0x00);

  SPI.endTransaction();

  // deselect both
  mcp_set_cs(false, false);
  delayMicroseconds(50);

  return id;
}

void setup() {
  app_setup();
  IWatchdog.begin(2000000);

  Serial.begin(115200);
  delay(300);

  Serial.println("RAW SPI BMP5xx chip-id test using MCP CS (GP0/GP1)");
  Serial.println("I2C: PA9/PA10  SPI: PA5/PA6/PA7");

  init_i2c_pa9_pa10();
  mcp_ok = mcp_init_cs();
  Serial.print("MCP ok: "); Serial.println(mcp_ok ? "YES" : "NO");

  init_spi_pa5_pa6_pa7();
  Serial.println("SPI ready.");

  uint32_t t = 0;

  while (true) {
    const uint32_t now = millis();
    if (now - t >= 500) {
      t = now;

      if (!mcp_ok) {
        Serial.println("No MCP; cannot test CS.");
      } else {
        uint8_t id0 = bmp_read_chip_id_via_cs(true);
        uint8_t id1 = bmp_read_chip_id_via_cs(false);

        Serial.print("CS0 chip-id: 0x"); print_hex_u8(id0);
        Serial.print("    CS1 chip-id: 0x"); print_hex_u8(id1);

        // BMP580 is commonly 0x50. If you see 0x00 or 0xFF, SPI/CS isn't working.
        if (id0 == 0x00 || id0 == 0xFF) Serial.print("  (CS0 suspect)");
        if (id1 == 0x00 || id1 == 0xFF) Serial.print("  (CS1 suspect)");
        Serial.println();
      }
    }

    IWatchdog.reload();
  }
}

void loop() {}
