#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <Wire.h>
#include <AllSensors_AUAV.h>

AllSensors_AUAV pressureSensor(&Wire, AllSensors_AUAV::SensorPressureRange::L10D);

DroneCAN dronecan;

unsigned long looptime1 = 0;
unsigned long looptime2 = 0;
unsigned long now = 0;
unsigned long last_msg_time_diff = 0;
unsigned long last_msg_time_abs = 0;

/*
This function is called when we receive a CAN message, and it's accepted by the shouldAcceptTransfer function.
We need to do boiler plate code in here to handle parameter updates and so on, but you can also write code to interact with sent messages here.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{

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

    return false || DroneCANshoudlAcceptTransfer(ins, out_data_type_signature, data_type_id, transfer_type, source_node_id);
}

uint8_t readSensor(AllSensors_AUAV::SensorType type)
{

    if (!pressureSensor.isBusy(type))
    {
        uint8_t status = pressureSensor.readData(type);

        // Restart measurement
        pressureSensor.startMeasurement(type, AllSensors_AUAV::MeasurementType::AVERAGE8);

        switch (type)
        {
        case AllSensors_AUAV::SensorType::DIFFERENTIAL:
            last_msg_time_diff = pressureSensor.last_message_diff;
            pressureSensor.last_message_diff = now;
        break;
        case AllSensors_AUAV::SensorType::ABSOLUTE:
            last_msg_time_abs = pressureSensor.last_message_abs;
            pressureSensor.last_message_abs = now;
        break;
        default:
            break;
        }
        return status;
    }

    // If we haven't received a message in the last 100ms, restart the measurement
    if (now - last_msg_time_diff > 150)
    {
        pressureSensor.startMeasurement(AllSensors_AUAV::SensorType::DIFFERENTIAL, AllSensors_AUAV::MeasurementType::AVERAGE8);
        dronecan.debug("No Diff Reading", 0);
    }
    // If we haven't received a message in the last 100ms, restart the measurement
    if (now - last_msg_time_abs > 150)
    {
        pressureSensor.startMeasurement(AllSensors_AUAV::SensorType::ABSOLUTE, AllSensors_AUAV::MeasurementType::AVERAGE4);
        dronecan.debug("No Abs Reading", 0);
    }
    return 0;

}

void setup()
{
    Serial.begin(115200);
    Serial.println("Node Start");
    delay(100);

    Wire.setSCL(PIN_WIRE_SCL);
    Wire.setSDA(PIN_WIRE_SDA);
    Wire.begin();
  
    delay(20);

    pressureSensor.setPressureUnit(AllSensors_AUAV::PressureUnit::PASCAL);
    pressureSensor.setTemperatureUnit(AllSensors_AUAV::TemperatureUnit::KELVIN);

    pressureSensor.readStatus(AllSensors_AUAV::SensorType::DIFFERENTIAL);
    delay(100);
    pressureSensor.readStatus(AllSensors_AUAV::SensorType::ABSOLUTE);

    delay(100);

    pressureSensor.readData(AllSensors_AUAV::SensorType::DIFFERENTIAL);
    delay(100);
    pressureSensor.readData(AllSensors_AUAV::SensorType::ABSOLUTE);

    delay(100);

    pressureSensor.startMeasurement(AllSensors_AUAV::SensorType::DIFFERENTIAL, AllSensors_AUAV::MeasurementType::AVERAGE8);
    delay(100);
    pressureSensor.startMeasurement(AllSensors_AUAV::SensorType::ABSOLUTE, AllSensors_AUAV::MeasurementType::AVERAGE4);
    
    delay(100);

    dronecan.init(onTransferReceived, shouldAcceptTransfer);

    IWatchdog.begin(2000000); // if the loop takes longer than 2 seconds, reset the system
}

void loop()
{
    now = millis();
    char error_msg[50];

    // send our battery message at 10Hz
    if (now - looptime2 > 100)
    {
        readSensor(AllSensors_AUAV::SensorType::DIFFERENTIAL);
    }

    if (now - looptime1 > 100)
    {
        looptime1 = millis();
        looptime2 = millis() + 50;

        readSensor(AllSensors_AUAV::SensorType::ABSOLUTE);
        
        // send air data message
        uavcan_equipment_air_data_RawAirData air_data{};
        air_data.differential_pressure = pressureSensor.pressure_d; // in Pascals
        air_data.differential_pressure_sensor_temperature = pressureSensor.temperature_d; // Kelvin
        air_data.static_pressure = pressureSensor.pressure_a; // in Pascals
        air_data.static_pressure_sensor_temperature = pressureSensor.temperature_a; // Kelvin

        uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_MAX_SIZE];  // this is the maximum size of the message
        uint32_t len = uavcan_equipment_air_data_RawAirData_encode(&air_data, buffer);
        static uint8_t transfer_id_air_data;
        canardBroadcast(&dronecan.canard,
                        UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID,
                        &transfer_id_air_data,
                        CANARD_TRANSFER_PRIORITY_HIGH,
                        buffer,
                        len);

        // Send Barometer pressure message
        uavcan_equipment_air_data_StaticPressure barometer_data{};
        barometer_data.static_pressure = pressureSensor.pressure_a; // in Pascals
        static uint8_t transfer_id_static_pressure;

        len = uavcan_equipment_air_data_StaticPressure_encode(&barometer_data, buffer);
        canardBroadcast(&dronecan.canard,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_ID,
                        &transfer_id_static_pressure,
                        CANARD_TRANSFER_PRIORITY_HIGH,
                        buffer,
                        len);

        // Send Barometer temperature message
        uavcan_equipment_air_data_StaticTemperature temperature_data{};
        temperature_data.static_temperature = pressureSensor.temperature_a; // in Kelvin    
        static uint8_t transfer_id_static_temperature;

        len = uavcan_equipment_air_data_StaticTemperature_encode(&temperature_data, buffer);
        canardBroadcast(&dronecan.canard,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_ID,
                        &transfer_id_static_temperature,
                        CANARD_TRANSFER_PRIORITY_HIGH,
                        buffer,
                        len);
    }

    dronecan.cycle();
    IWatchdog.reload();
}
