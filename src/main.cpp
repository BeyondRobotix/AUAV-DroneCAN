#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <app.h>
#include <vector>
#include <simple_dronecanmessages.h>
#include <AllSensors_AUAV.h>

AllSensors_AUAV pressureSensor(&Wire);

// set up your parameters here with default values. NODEID should be kept
std::vector<DroneCAN::parameter> custom_parameters = {
    {"NODEID", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 50, 0, 127},
    {"SENSOR_PRESSURE", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 10, 0, 30},
};


DroneCAN dronecan;

uint32_t looptime = 0;

unsigned long lastLoopAbs = 0;
unsigned long lastLoopDiff = 0;
unsigned long now = 0;
unsigned long last_msg_time_diff = 0;
unsigned long last_msg_time_abs = 0;
unsigned long last_msg_time_can = 0;

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
    if (now - last_msg_time_diff > 300)
    {
        pressureSensor.startMeasurement(AllSensors_AUAV::SensorType::DIFFERENTIAL, AllSensors_AUAV::MeasurementType::AVERAGE8);
        dronecan.debug("No Diff Reading", 0);
        last_msg_time_diff = now;
    }
    // If we haven't received a message in the last 100ms, restart the measurement
    if (now - last_msg_time_abs > 300)
    {
        pressureSensor.startMeasurement(AllSensors_AUAV::SensorType::ABSOLUTE, AllSensors_AUAV::MeasurementType::AVERAGE4);
        dronecan.debug("No Abs Reading", 0);
        last_msg_time_abs = now;
    }
    return 0;
}

void setup()
{
    // to use debugging tools, remove app_setup and set FLASH start from 0x800A000 to 0x8000000 in ldscript.ld
    // this will over-write the bootloader. To use the bootloader again, reflash it and change above back.
    app_setup(); // needed for coming from a bootloader, needs to be first in setup
    Serial.begin(115200);

    dronecan.version_major = 1;
    dronecan.version_minor = 0;
    dronecan.init(
        onTransferReceived,
        shouldAcceptTransfer,
        custom_parameters,
        "com.beyondrobotix.airdata");

    IWatchdog.begin(2000000); // if the loop takes longer than 2 seconds, reset the system

    delay(100);

    Wire.setSCL(PIN_WIRE_SCL);
    Wire.setSDA(PIN_WIRE_SDA);
    Wire.begin();

    delay(20);

    pressureSensor.setPressureDiffUnit(AllSensors_AUAV::PressureUnit::PASCAL);
    pressureSensor.setPressureAbsUnit(AllSensors_AUAV::PressureUnit::PASCAL);
    pressureSensor.setTemperatureUnit(AllSensors_AUAV::TemperatureUnit::KELVIN);

    pressureSensor.setPressureRange(dronecan.getParameter("SENSOR_PRESSURE") * 2); // Multiplied by 2 for the sensor range, as it is 2x the value in inH2O

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

    while(canardGetLocalNodeID(&dronecan.canard) == CANARD_BROADCAST_NODE_ID)
    {
        dronecan.cycle();
        IWatchdog.reload();
    }



    while (true)
    {
        const uint32_t now = millis();

        dronecan.cycle();
        IWatchdog.reload();

        if (now - lastLoopDiff > 100)
        {
            readSensor(AllSensors_AUAV::SensorType::DIFFERENTIAL);
            lastLoopDiff = millis();
        }

        if (now - lastLoopAbs > 100)
        {
            readSensor(AllSensors_AUAV::SensorType::ABSOLUTE);
            lastLoopAbs = millis();
        }

        // Send messages every 100ms
        if (now - last_msg_time_can > 200)
        {
            // send air data message
            uavcan_equipment_air_data_RawAirData air_data{};
            air_data.differential_pressure = pressureSensor.pressure_d;                       // in Pascals
            air_data.differential_pressure_sensor_temperature = pressureSensor.temperature_d; // Kelvin
            air_data.static_pressure = pressureSensor.pressure_a;                             // in Pascals
            air_data.static_pressure_sensor_temperature = pressureSensor.temperature_a;       // Kelvin
            sendUavcanMsg(dronecan.canard, air_data, CANARD_TRANSFER_PRIORITY_HIGH);

            // // Send Barometer pressure message
            uavcan_equipment_air_data_StaticPressure barometer_data{};
            barometer_data.static_pressure = pressureSensor.pressure_a; // in Pascals
            sendUavcanMsg(dronecan.canard, barometer_data, CANARD_TRANSFER_PRIORITY_HIGH);

            // // Send Barometer temperature message
            uavcan_equipment_air_data_StaticTemperature temperature_data{};
            temperature_data.static_temperature = pressureSensor.temperature_a; // in Kelvin
            sendUavcanMsg(dronecan.canard, temperature_data, CANARD_TRANSFER_PRIORITY_HIGH);

            last_msg_time_can = millis();
        }


    }
}

void loop()
{
    // Doesn't work coming from bootloader ? use while loop in setup
}