#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <app.h>
#include <vector>
#include <simple_dronecanmessages.h>
#include <SPI.h>
#include "Adafruit_BMP5xx.h"

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
static constexpr uint8_t BMP_CS_1 = PA4; // Sensor 1
static constexpr uint8_t BMP_CS_2 = PB0; // Sensor 2

Adafruit_BMP5xx bmp1;
Adafruit_BMP5xx bmp2;

bool s1_ready = false;
bool s2_ready = false;

// SPI Hardware Debug Function - Updated for Dual Sensors
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
    Serial.println("Starting Dual Baro Node!");
    dronecan.version_major = 1;
    dronecan.version_minor = 0;
    dronecan.init(onTransferReceived, shouldAcceptTransfer, custom_parameters, "Beyond Robotix Node");

    // --- CRITICAL SPI PIN FIX ---
    SPI.setSCLK(PA5); 
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);

    // Init both Chip Select pins
    pinMode(BMP_CS_1, OUTPUT);
    pinMode(BMP_CS_2, OUTPUT);
    digitalWrite(BMP_CS_1, HIGH);
    digitalWrite(BMP_CS_2, HIGH);

    runRawSPITest();

    while (true) {
        const uint32_t now = millis();

        if (now - looptime > 100) {
            looptime = now;
            int32_t vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_12B);
            int32_t cpu_temp = __LL_ADC_CALC_TEMPERATURE(vref, analogRead(ATEMP), LL_ADC_RESOLUTION_12B);

            // --- SENSOR 1 PROCESSING ---
            if (!s1_ready) {
                if (bmp1.begin(BMP_CS_1, &SPI)) {
                    Serial.println(">>> Sensor 1 Found!");
                    bmp1.setPowerMode((bmp5xx_powermode_t)BMP5XX_POWERMODE_NORMAL);
                    bmp1.setOutputDataRate((bmp5xx_odr_t)BMP5XX_ODR_50_HZ);
                    s1_ready = true;
                }
            } else if (bmp1.performReading()) {
                // Publish S1 Data to DroneCAN
                uavcan_equipment_air_data_StaticPressure pkt_p1{};
                pkt_p1.static_pressure = bmp1.pressure * 100.0f; 
                sendUavcanMsg(dronecan.canard, pkt_p1);

                uavcan_equipment_device_Temperature pkt_t1{};
                pkt_t1.temperature = bmp1.temperature + 273.15f; 
                sendUavcanMsg(dronecan.canard, pkt_t1);
                
                // Terminal Print S1
                Serial.print("S1: "); Serial.print(bmp1.pressure); Serial.print(" hPa, ");
                Serial.print(bmp1.temperature); Serial.print(" C | ");
            }

            // --- SENSOR 2 PROCESSING ---
            if (!s2_ready) {
                if (bmp2.begin(BMP_CS_2, &SPI)) {
                    Serial.println(">>> Sensor 2 Found!");
                    bmp2.setPowerMode((bmp5xx_powermode_t)BMP5XX_POWERMODE_NORMAL);
                    bmp2.setOutputDataRate((bmp5xx_odr_t)BMP5XX_ODR_50_HZ);
                    s2_ready = true;
                }
            } else if (bmp2.performReading()) {
                // Publish S2 Data to DroneCAN
                uavcan_equipment_air_data_StaticPressure pkt_p2{};
                pkt_p2.static_pressure = bmp2.pressure * 100.0f; 
                sendUavcanMsg(dronecan.canard, pkt_p2);

                uavcan_equipment_device_Temperature pkt_t2{};
                pkt_t2.temperature = bmp2.temperature + 273.15f; 
                sendUavcanMsg(dronecan.canard, pkt_t2);
                
                // Terminal Print S2
                Serial.print("S2: "); Serial.print(bmp2.pressure); Serial.print(" hPa, ");
                Serial.print(bmp2.temperature); Serial.print(" C | ");
            }

            // Summary Info
            Serial.print("CPU: "); Serial.print(cpu_temp); Serial.println("C");

            // Battery/Health Broadcast
            uavcan_equipment_power_BatteryInfo pkt_bat{};
            pkt_bat.voltage = analogRead(PA1);
            pkt_bat.current = analogRead(PA0);
            pkt_bat.temperature = cpu_temp;
            sendUavcanMsg(dronecan.canard, pkt_bat);
        }

        dronecan.cycle();
        IWatchdog.reload();
    }
}

void loop() {}