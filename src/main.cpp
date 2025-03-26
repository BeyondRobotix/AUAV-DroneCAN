#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <Wire.h>
#include <AllSensors_AUAV.h>

AllSensors_AUAV pressureSensor(&Wire, AllSensors_AUAV::SensorPressureRange::L10D);

DroneCAN dronecan;

unsigned long looptime = 0;
unsigned long now = 0;

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

void readSensor(AllSensors_AUAV::SensorType type)
{
    if (!pressureSensor.isBusy(type))
    {
        pressureSensor.readData(type);

        // Restart measurement
        pressureSensor.startMeasurement(type, AllSensors_AUAV::MeasurementType::AVERAGE16);
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Node Start");

    Wire.setSCL(PIN_WIRE_SCL);
    Wire.setSDA(PIN_WIRE_SDA);
    Wire.begin();
  
    delay(20);

    pressureSensor.setPressureUnit(AllSensors_AUAV::PressureUnit::PASCAL);
    pressureSensor.setTemperatureUnit(AllSensors_AUAV::TemperatureUnit::KELVIN);

    dronecan.init(onTransferReceived, shouldAcceptTransfer);

    IWatchdog.begin(2000000); // if the loop takes longer than 2 seconds, reset the system
}

void loop()
{
    now = millis();
    readSensor(AllSensors_AUAV::SensorType::DIFFERENTIAL);
    readSensor(AllSensors_AUAV::SensorType::ABSOLUTE);

    // send our battery message at 10Hz
    if (now - looptime > 50)
    {
        looptime = millis();

        // send air data message
        uavcan_equipment_air_data_RawAirData air_data{};
        air_data.differential_pressure = pressureSensor.pressure_d; // in Pascals
        air_data.differential_pressure_sensor_temperature = pressureSensor.temperature_d; // Kelvin
        air_data.static_pressure = pressureSensor.pressure_a; // in Pascals
        air_data.static_air_temperature = pressureSensor.temperature_a; // Kelvin

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
