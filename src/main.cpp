#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <Wire.h>
#include <AllSensors_DLHR.h>

AllSensors_DLHR pressureSensor(&Wire, AllSensors_DLHR::SensorType::DIFFERENTIAL, AllSensors_DLHR::SensorResolution::RESOLUTION_18_BITS, 10.0);

DroneCAN dronecan;

uint32_t looptime = 0;


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

void readSensor()
{
    pressureSensor.readData(true);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Node Start");

    Wire.setSCL(PB_13);
    Wire.setSDA(PB_14);
    Wire.begin();
  
    delay(20);

    pressureSensor.setPressureUnit(AllSensors_DLHR::PressureUnit::PASCAL);
    pressureSensor.startMeasurement(AllSensors_DLHR::MeasurementType::AVERAGE16);

    dronecan.init(onTransferReceived, shouldAcceptTransfer);

    IWatchdog.begin(2000000); // if the loop takes longer than 2 seconds, reset the system
}

void loop()
{
    const uint32_t now = millis();

    // send our battery message at 10Hz
    if (now - looptime > 50)
    {
        looptime = millis();
        
        readSensor();
        pressureSensor.startMeasurement(AllSensors_DLHR::MeasurementType::AVERAGE16);

        // send air data message
        uavcan_equipment_air_data_RawAirData air_data{};
        air_data.differential_pressure = pressureSensor.pressure;
        air_data.differential_pressure_sensor_temperature = pressureSensor.temperature;

        // boilerplate to send a message
        uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_MAX_SIZE];  // this is the maximum size of the message
        uint32_t len = uavcan_equipment_air_data_RawAirData_encode(&air_data, buffer);
        static uint8_t transfer_id;
        canardBroadcast(&dronecan.canard,
                        UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_HIGH,
                        buffer,
                        len);
    }

    dronecan.cycle();
    IWatchdog.reload();
}
