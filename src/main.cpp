#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>

DroneCAN dronecan;

uint32_t looptime = 0;

/*
This function is called when we receive a CAN message, and it's accepted by the shouldAcceptTransfer function.
We need to do boiler plate code in here to handle parameter updates and so on, but you can also write code to interact with sent messages here.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{

    // switch on data type ID to pass to the right handler function
    // if (transfer->transfer_type == CanardTransferTypeRequest)
    // check if we want to handle a specific service request
    switch (transfer->data_type_id)
    {

    case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID:
    {
        uavcan_equipment_ahrs_MagneticFieldStrength pkt{};
        uavcan_equipment_ahrs_MagneticFieldStrength_decode(transfer, &pkt);
        Serial.print(pkt.magnetic_field_ga[0], 4);
        Serial.print(" ");
        Serial.print(pkt.magnetic_field_ga[1], 4);
        Serial.print(" ");
        Serial.print(pkt.magnetic_field_ga[2], 4);
        Serial.print(" ");
        Serial.println();
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
        // Check if we want to handle a specific broadcast packet
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
    Serial.begin(115200);
    Serial.println("Node Start");

    dronecan.init(onTransferReceived, shouldAcceptTransfer);

    IWatchdog.begin(2000000); // if the loop takes longer than 2 seconds, reset the system
}

void loop()
{
    const uint32_t now = millis();

    // send our battery message at 10Hz
    if (now - looptime > 100)
    {
        looptime = millis();

        // collect MCU core temperature data
        int32_t vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_12B);
        int32_t cpu_temp = __LL_ADC_CALC_TEMPERATURE(vref, analogRead(ATEMP), LL_ADC_RESOLUTION_12B);

        // construct dronecan packet
        uavcan_equipment_power_BatteryInfo pkt{};
        pkt.voltage = now / 10000;
        pkt.temperature = cpu_temp;

        // boilerplate to send a message
        uint8_t buffer[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE];
        uint32_t len = uavcan_equipment_power_BatteryInfo_encode(&pkt, buffer);
        static uint8_t transfer_id;
        canardBroadcast(&dronecan.canard,
                        UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
                        UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        buffer,
                        len);
    }

    dronecan.cycle();
    IWatchdog.reload();
}
