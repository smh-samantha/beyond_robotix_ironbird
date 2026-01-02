#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <app.h>
#include <vector>
#include <simple_dronecanmessages.h>
#include <SPI.h>
#include "Adafruit_BMP5xx.h"


// set up your parameters here with default values. NODEID should be kept
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

// --- SENSOR CONFIGURATION ---
static constexpr uint8_t BMP_CS = PA4; 
Adafruit_BMP5xx bmp;
bool sensor_ready = false; 

// SPI Hardware Debug Function
// This manually pings the BMP580 "CHIP_ID" register (0x01)
void runRawSPITest() {
    digitalWrite(BMP_CS, LOW);
    // 0x81 is the read command for register 0x01 on BMP580
    SPI.transfer(0x81); 
    uint8_t chip_id = SPI.transfer(0x00); 
    digitalWrite(BMP_CS, HIGH);

    Serial.print("RAW SPI TEST -> Register 0x01 (Chip ID): 0x");
    Serial.println(chip_id, HEX);
    
    if (chip_id == 0x50) {
        Serial.println(">>> HARDWARE SUCCESS: STM32 SPI1 is talking to BMP580!");
    } else {
        Serial.println(">>> HARDWARE FAILURE: SCK (PA5) or MISO (PA6) signal is missing.");
    }
}

DroneCAN dronecan;

uint32_t looptime = 0;

/*
This function is called when we receive a CAN message, and it's accepted by the shouldAcceptTransfer function.
We need to do boiler plate code in here to handle parameter updates and so on, but you can also write code to interact with sent messages here.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    switch (transfer->data_type_id)
    {
    case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID:
    {
        uavcan_equipment_ahrs_MagneticFieldStrength pkt{};
        uavcan_equipment_ahrs_MagneticFieldStrength_decode(transfer, &pkt);
        break;
    }
    }
    DroneCANonTransferReceived(dronecan, ins, transfer);
}

/*
For this function, we need to make sure any messages we want to receive follow the following format with
UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID as an example
 */
static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    if (transfer_type == CanardTransferTypeBroadcast)
    {
        switch (data_type_id)
        {
        case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID:
        {
            *out_data_type_signature = UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_SIGNATURE;
            return true;
        }
        }
    }
    return false || DroneCANshoudlAcceptTransfer(ins, out_data_type_signature, data_type_id, transfer_type, source_node_id);
}

void setup()
{   
    app_setup();
    IWatchdog.begin(2000000);
    Serial.begin(115200);
    Serial.println("Starting Node!");
    dronecan.version_major = 1;
    dronecan.version_minor = 0;
    dronecan.init(
        onTransferReceived, 
        shouldAcceptTransfer, 
        custom_parameters,
        "Beyond Robotix Node"
    );

    // --- CRITICAL SPI PIN FIX ---
    // Override the PA1 default from variant.h and force PA5 as Clock
    SPI.setSCLK(PA5); 
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16); // 5MHz for stable soldered signals

    pinMode(BMP_CS, OUTPUT);
    digitalWrite(BMP_CS, HIGH);

    // Run the Hardware Probe
    runRawSPITest();

    while (true)
    {
        const uint32_t now = millis();

        if (now - looptime > 100)
        {
            looptime = now;

            int32_t vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_12B);
            int32_t cpu_temp = __LL_ADC_CALC_TEMPERATURE(vref, analogRead(ATEMP), LL_ADC_RESOLUTION_12B);

            // --- SENSOR PROCESSING ---
            if (!sensor_ready) {
                if (bmp.begin(BMP_CS, &SPI)) {
                    Serial.println(">>> BMP580 Found and Initialized!");
                    bmp.setPowerMode((bmp5xx_powermode_t)BMP5XX_POWERMODE_NORMAL);
                    bmp.setPressureOversampling((bmp5xx_oversampling_t)BMP5XX_OVERSAMPLING_16X);
                    bmp.setTemperatureOversampling((bmp5xx_oversampling_t)BMP5XX_OVERSAMPLING_2X);
                    bmp.setIIRFilterCoeff((bmp5xx_iir_filter_t)BMP5XX_IIR_FILTER_COEFF_3);
                    bmp.setOutputDataRate((bmp5xx_odr_t)BMP5XX_ODR_50_HZ);
                    sensor_ready = true;
                } else {
                    runRawSPITest();
                }
            } else if (bmp.performReading()) {
                // Static Pressure
                uavcan_equipment_air_data_StaticPressure press_pkt{};
                press_pkt.static_pressure = bmp.pressure * 100.0f; // Pa
                sendUavcanMsg(dronecan.canard, press_pkt);

                // Temperature
                uavcan_equipment_device_Temperature temp_pkt{};
                temp_pkt.temperature = bmp.temperature + 273.15f; // Kelvin
                sendUavcanMsg(dronecan.canard, temp_pkt);
                
                Serial.print("CPU: "); Serial.print(cpu_temp);
                Serial.print("C | BMP Pressure: "); Serial.print(bmp.pressure);
                Serial.print(" hPa | BMP Temp: "); Serial.println(bmp.temperature);
            }

            uavcan_equipment_power_BatteryInfo pkt{};
            pkt.voltage = analogRead(PA1);
            pkt.current = analogRead(PA0);
            pkt.temperature = cpu_temp;
            sendUavcanMsg(dronecan.canard, pkt);
        }

        dronecan.cycle();
        IWatchdog.reload();
    }
}

void loop() {}