#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <app.h>
#include <vector>
#include <simple_dronecanmessages.h>
#include <SPI.h>
#include "Adafruit_BMP5xx.h"
#include <ardupilotmega/mavlink.h>

// DroneCAN Parameters (Unchanged)
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

// --- DUAL SENSOR CONFIGURATION ---
static constexpr uint8_t BMP_CS_1 = PB0;
static constexpr uint8_t BMP_CS_2 = PB1;

Adafruit_BMP5xx bmp1, bmp2;
bool s1_ready = false, s2_ready = false;

// SPI Hardware Debug Function - Expanded for 4 Sensors
void runRawSPITest() {
    auto probe = [](uint8_t cs, const char* label) {
        digitalWrite(cs, LOW);
        SPI.transfer(0x81); // Read Chip ID command
        uint8_t id = SPI.transfer(0x00);
        digitalWrite(cs, HIGH);

        Serial.print("RAW SPI TEST ("); Serial.print(label); Serial.print(") -> ID: 0x");
        Serial.println(id, HEX);
        
        if (id == 0x50) {
            Serial.print(">>> "); Serial.print(label); Serial.println(" SUCCESS!");
        } else {
            Serial.print(">>> "); Serial.print(label); Serial.println(" FAILURE: Check soldering.");
        }
    };

    probe(BMP_CS_1, "Sensor 1");
    probe(BMP_CS_2, "Sensor 2");
}

DroneCAN dronecan;
uint32_t looptime = 0;

#ifndef MAVLINK_SERIAL
#define MAVLINK_SERIAL Serial1
#endif

#ifndef MAVLINK_BAUD
#define MAVLINK_BAUD 115200
#endif

#ifndef MAVLINK_SYS_ID
#define MAVLINK_SYS_ID 1
#endif

#ifndef MAVLINK_COMP_ID
#define MAVLINK_COMP_ID 200
#endif

#ifndef DEBUG_NAMED_VALUES
#define DEBUG_NAMED_VALUES 0
#endif

static constexpr uint16_t RAW_MSG_ID_STATIC_PRESSURE = 1028;
static constexpr uint16_t RAW_MSG_ID_TEMPERATURE = 1110;

static uint8_t getRawNodeId() {
    const uint8_t node_id = canardGetLocalNodeID(&dronecan.canard);
    if (node_id >= 1 && node_id <= 127) {
        return node_id;
    }
    const float param = dronecan.getParameter("NODEID");
    if (param >= 1.0f && param <= 127.0f) {
        return static_cast<uint8_t>(param);
    }
    return 1;
}

static void sendRawCanFloat(uint16_t msg_id, uint8_t node_id, uint8_t sensor_id, float value) {
    CanardCANFrame frame{};
    frame.id = (static_cast<uint32_t>(msg_id) << 8) | (node_id & 0x7F);
    frame.data_len = 5;
    frame.data[0] = sensor_id;
    union {
        float f;
        uint8_t b[4];
    } u{};
    u.f = value;
    frame.data[1] = u.b[0];
    frame.data[2] = u.b[1];
    frame.data[3] = u.b[2];
    frame.data[4] = u.b[3];
    CANSend(&frame);
}

static void mavlinkSendMessage(const mavlink_message_t &msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    MAVLINK_SERIAL.write(buffer, len);
}

static void mavlinkSendBaro(uint8_t sensor_index, float pressure_hpa, float temp_c) {
    mavlink_message_t msg{};
    char name[10] = {0};

    snprintf(name, sizeof(name), "BARO580P%u", sensor_index);
    mavlink_msg_named_value_float_pack(
        MAVLINK_SYS_ID,
        MAVLINK_COMP_ID,
        &msg,
        millis(),
        name,
        pressure_hpa
    );
    mavlinkSendMessage(msg);

    snprintf(name, sizeof(name), "BARO580T%u", sensor_index);
    mavlink_msg_named_value_float_pack(
        MAVLINK_SYS_ID,
        MAVLINK_COMP_ID,
        &msg,
        millis(),
        name,
        temp_c
    );
    mavlinkSendMessage(msg);

#if DEBUG_NAMED_VALUES
    Serial.print("NVF BARO580P");
    Serial.print(sensor_index);
    Serial.print("=");
    Serial.print(pressure_hpa, 1);
    Serial.print(" BARO580T");
    Serial.print(sensor_index);
    Serial.print("=");
    Serial.println(temp_c, 1);
#endif
}

static void sendMagHiRes(uint8_t sensor_id, float pressure_pa, float temp_c) {
    // Repurpose MagneticFieldStrengthHiRes to carry pressure/temperature as float32s.
    dronecan_sensors_magnetometer_MagneticFieldStrengthHiRes msg{};
    msg.sensor_id = sensor_id;
    msg.magnetic_field_ga[0] = pressure_pa;
    msg.magnetic_field_ga[1] = temp_c;
    msg.magnetic_field_ga[2] = 0.0f;
    sendUavcanMsg(dronecan.canard, msg);
}

static void formatSensorLine(char *out, size_t out_size, bool has, float pressure_hpa, float temp_c) {
    if (!has) {
        snprintf(out, out_size, "OFF");
        return;
    }

    char pbuf[8];
    char tbuf[8];
    dtostrf(pressure_hpa, 6, 1, pbuf);
    dtostrf(temp_c, 5, 1, tbuf);
    snprintf(out, out_size, "P=%s T=%s", pbuf, tbuf);
}

static void formatPaTempLine(char *out, size_t out_size, bool has, float pressure_pa, float temp_c) {
    if (!has) {
        snprintf(out, out_size, "OFF");
        return;
    }

    char pbuf[10];
    char tbuf[8];
    dtostrf(pressure_pa, 7, 0, pbuf);
    dtostrf(temp_c, 5, 1, tbuf);
    snprintf(out, out_size, "P=%sPa T=%s", pbuf, tbuf);
}

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

static bool shouldAcceptTransfer(const CanardInstance *ins, uint64_t *out_data_type_signature, uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id) {
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
    Serial.begin(115200);
    MAVLINK_SERIAL.begin(MAVLINK_BAUD);
    Serial.println("Starting Quad Baro Node!");
    dronecan.version_major = 1;
    dronecan.version_minor = 0;
    dronecan.init(onTransferReceived, shouldAcceptTransfer, custom_parameters, "Beyond Robotix Node");

    // --- SPI PIN SETUP (MicroNode defaults) ---
    SPI.setSCLK(PA1);
    SPI.setMISO(PA6);
    SPI.setMOSI(PA7);
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);

    // --- MANDATORY: INIT ALL CS PINS TO PREVENT BUS CONTENTION ---
    uint8_t cs_pins[] = {BMP_CS_1, BMP_CS_2};
    for(uint8_t p : cs_pins) {
        pinMode(p, OUTPUT);
        digitalWrite(p, HIGH); // De-select all sensors immediately
    }

    runRawSPITest();

    while (true) {
        const uint32_t now = millis();

        if (now - looptime > 100) {
            looptime = now;
            const uint8_t raw_node_id = getRawNodeId();
            int32_t vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_12B);
            int32_t cpu_temp = __LL_ADC_CALC_TEMPERATURE(vref, analogRead(ATEMP), LL_ADC_RESOLUTION_12B);

            // --- SENSOR 1 ---
            if (!s1_ready) s1_ready = bmp1.begin(BMP_CS_1, &SPI);
            const bool s1_has = s1_ready && bmp1.performReading();
            if (s1_has) {
                com_beyondrobotix_baro_BaroPT pkt_baro1{
                    .sensor_id = 1,
                    .pressure_pa = bmp1.pressure * 100.0f,
                    .temperature_c = bmp1.temperature
                };
                sendUavcanMsg(dronecan.canard, pkt_baro1);
                sendMagHiRes(1, bmp1.pressure * 100.0f, bmp1.temperature);
                mavlinkSendBaro(1, bmp1.pressure, bmp1.temperature);
                sendRawCanFloat(RAW_MSG_ID_STATIC_PRESSURE, raw_node_id, 1, bmp1.pressure * 100.0f);
                sendRawCanFloat(RAW_MSG_ID_TEMPERATURE, raw_node_id, 1, bmp1.temperature);
            }

            // --- SENSOR 2 ---
            if (!s2_ready) s2_ready = bmp2.begin(BMP_CS_2, &SPI);
            const bool s2_has = s2_ready && bmp2.performReading();
            if (s2_has) {
                com_beyondrobotix_baro_BaroPT pkt_baro2{
                    .sensor_id = 2,
                    .pressure_pa = bmp2.pressure * 100.0f,
                    .temperature_c = bmp2.temperature
                };
                sendUavcanMsg(dronecan.canard, pkt_baro2);
                sendMagHiRes(2, bmp2.pressure * 100.0f, bmp2.temperature);
                mavlinkSendBaro(2, bmp2.pressure, bmp2.temperature);
                sendRawCanFloat(RAW_MSG_ID_STATIC_PRESSURE, raw_node_id, 2, bmp2.pressure * 100.0f);
                sendRawCanFloat(RAW_MSG_ID_TEMPERATURE, raw_node_id, 2, bmp2.temperature);
            }

            char line[256];
            char s1buf[16], s2buf[16];
            char dc1buf[24], dc2buf[24];
            formatSensorLine(s1buf, sizeof(s1buf), s1_has, bmp1.pressure, bmp1.temperature);
            formatSensorLine(s2buf, sizeof(s2buf), s2_has, bmp2.pressure, bmp2.temperature);
            formatPaTempLine(dc1buf, sizeof(dc1buf), s1_has, bmp1.pressure * 100.0f, bmp1.temperature);
            formatPaTempLine(dc2buf, sizeof(dc2buf), s2_has, bmp2.pressure * 100.0f, bmp2.temperature);
            snprintf(
                line,
                sizeof(line),
                "S1:%-16s | S2:%-16s | NVF1:%-16s | NVF2:%-16s | DC1:%-20s | DC2:%-20s | CPU:%5ldC",
                s1buf,
                s2buf,
                s1buf,
                s2buf,
                dc1buf,
                dc2buf,
                (long)cpu_temp
            );
            Serial.println(line);

        }

        dronecan.cycle();
        IWatchdog.reload();
    }
}

void loop() {}
