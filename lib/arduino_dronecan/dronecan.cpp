#include <dronecan.h>

void DroneCAN::init(CanardOnTransferReception onTransferReceived,
                    CanardShouldAcceptTransfer shouldAcceptTransfer)
{
    CANInit(CAN_1000KBPS, 2);

    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               NULL);

    if (node_id > 0)
    {
        canardSetLocalNodeID(&canard, node_id);
    }
    else
    {
        Serial.println("Waiting for DNA node allocation\n");
    }

    // initialise the internal LED
    pinMode(19, OUTPUT);

    // get the parameters in the EEPROM
    this->read_parameter_memory();
}

uint8_t DroneCAN::get_preferred_node_id()
{
    if (parameters[0].value > 0 && parameters[0].value < 128)
    {
        return (uint8_t)parameters[0].value;
    }
    else
    {
        return PREFERRED_NODE_ID;
    }
}

/*
Processes any DroneCAN actions required. Call as quickly as practical !
*/
void DroneCAN::cycle()
{
    const uint32_t now = millis();

    if (now - this->looptime > 1000)
    {
        this->looptime = millis();
        this->process1HzTasks(this->micros64());
        digitalWrite(19, this->led_state);
        this->led_state = !this->led_state;
    }

    this->processRx();
    this->processTx();
    this->request_DNA();
}

uint64_t DroneCAN::micros64()
{
    return (uint64_t)micros();
}

void DroneCAN::getUniqueID(uint8_t uniqueId[16])
{
    memset(uniqueId, 0, 16);

    uint32_t cpuid0 = HAL_GetUIDw0();
    uint32_t cpuid1 = HAL_GetUIDw1();
    uint32_t cpuid2 = HAL_GetUIDw2();

    uniqueId[0] = (uint8_t)(cpuid0 >> 24);
    uniqueId[1] = (uint8_t)(cpuid0 >> 16);
    uniqueId[2] = (uint8_t)(cpuid0 >> 8);
    uniqueId[3] = (uint8_t)(cpuid0);
    uniqueId[4] = (uint8_t)(cpuid1 >> 24);
    uniqueId[5] = (uint8_t)(cpuid1 >> 16);
    uniqueId[6] = (uint8_t)(cpuid1 >> 8);
    uniqueId[7] = (uint8_t)(cpuid1);
    uniqueId[8] = (uint8_t)(cpuid2 >> 24);
    uniqueId[9] = (uint8_t)(cpuid2 >> 16);
    uniqueId[10] = (uint8_t)(cpuid2 >> 8);
    uniqueId[11] = (uint8_t)(cpuid2);
    uniqueId[12] = 0;
    uniqueId[13] = 0;
    uniqueId[14] = 0;
    uniqueId[15] = 0;
}

void DroneCAN::handle_GetNodeInfo(CanardRxTransfer *transfer)
{
    Serial.print("GetNodeInfo request from");
    Serial.println(transfer->source_node_id);

    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;

    memset(&pkt, 0, sizeof(pkt));

    node_status.uptime_sec = micros64() / 1000000ULL;
    pkt.status = node_status;

    // fill in your major and minor firmware version
    pkt.software_version.major = 1;
    pkt.software_version.minor = 2;
    pkt.software_version.optional_field_flags = 0;
    pkt.software_version.vcs_commit = 0; // should put git hash in here

    // should fill in hardware version
    pkt.hardware_version.major = 2;
    pkt.hardware_version.minor = 3;

    getUniqueID(pkt.hardware_version.unique_id);

    strncpy((char *)pkt.name.data, "Beyond Robotix Node", sizeof(pkt.name.data));
    pkt.name.len = strnlen((char *)pkt.name.data, sizeof(pkt.name.data));

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(&canard,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle parameter GetSet request
 */
void DroneCAN::handle_param_GetSet(CanardRxTransfer *transfer)
{
    struct uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req))
    {
        return;
    }

    struct parameter *p = NULL;
    if (req.name.len != 0)
    {
        for (uint16_t i = 0; i < ARRAY_SIZE(parameters); i++)
        {
            if (req.name.len == strlen(parameters[i].name) &&
                strncmp((const char *)req.name.data, parameters[i].name, req.name.len) == 0)
            {
                p = &parameters[i];
                req.index = i;
                break;
            }
        }
    }
    else if (req.index < ARRAY_SIZE(parameters))
    {
        p = &parameters[req.index];
    }
    if (p != NULL && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY)
    {
        /*
          this is a parameter set command. The implementation can
          either choose to store the value in a persistent manner
          immediately or can instead store it in memory and save to permanent storage on a
         */
        switch (p->type)
        {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            p->value = req.value.integer_value;
            EEPROM.put(req.index * 4, p->value);
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
            p->value = req.value.real_value;
            EEPROM.put(req.index * 4, p->value);
            break;
        default:
            return;
        }
    }

    /*
      for both set and get we reply with the current value
     */
    struct uavcan_protocol_param_GetSetResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    if (p != NULL)
    {
        pkt.value.union_tag = p->type;
        switch (p->type)
        {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            pkt.value.integer_value = p->value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
            pkt.value.real_value = p->value;
            break;
        default:
            return;
        }
        pkt.name.len = strlen(p->name);
        strcpy((char *)pkt.name.data, p->name);
    }

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&pkt, buffer);

    canardRequestOrRespond(&canard,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle parameter executeopcode request
 */
void DroneCAN::handle_param_ExecuteOpcode(CanardRxTransfer *transfer)
{
    struct uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req))
    {
        return;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE)
    {
        // here is where you would reset all parameters to defaults
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE)
    {
        // here is where you would save all the changed parameters to permanent storage
    }

    struct uavcan_protocol_param_ExecuteOpcodeResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.ok = true;

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer);

    canardRequestOrRespond(&canard,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
Read the EEPROM parameter storage and set the current parameter list to the read values
*/
void DroneCAN::read_parameter_memory()
{
    struct parameter *p = NULL;
    float p_val = 0.0;

    for (uint16_t i = 0; i < ARRAY_SIZE(parameters); i++)
    {
        EEPROM.get(i * 4, p_val);
        p = &parameters[i];
        p->value = p_val;
    }
}

/*
  handle a DNA allocation packet
 */
int DroneCAN::handle_DNA_Allocation(CanardRxTransfer *transfer)
{
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID)
    {
        // already allocated
        return 0;
    }

    // Rule C - updating the randomized time interval
    DNA.send_next_node_id_allocation_request_at_ms =
        millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        Serial.println("Allocation request from another allocatee\n");
        DNA.node_id_allocation_unique_id_offset = 0;
        return 0;
    }

    // Copying the unique ID from the message
    struct uavcan_protocol_dynamic_node_id_Allocation msg;

    uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg);

    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    getUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0)
    {
        Serial.println("Mismatching allocation response\n");
        DNA.node_id_allocation_unique_id_offset = 0;
        // No match, return
        return 0;
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data))

    {
        // The allocator has confirmed part of unique ID, switching to
        // the next stage and updating the timeout.
        DNA.node_id_allocation_unique_id_offset = msg.unique_id.len;
        DNA.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

        Serial.print("Matching allocation response: ");
        Serial.println(msg.unique_id.len);
    }
    else
    {
        // Allocation complete - copying the allocated node ID from the message
        canardSetLocalNodeID(&canard, msg.node_id);
        Serial.print("Node ID allocated: ");
        Serial.println(msg.node_id);
    }
    return 0;
}

/*
  ask for a dynamic node allocation
 */
void DroneCAN::request_DNA()
{

    // see if we are still doing DNA
    if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID)
    {
        return;
    }

    // we're still waiting for a DNA allocation of our node ID
    if (millis() < DNA.send_next_node_id_allocation_request_at_ms)
    {
        return;
    }

    const uint32_t now = millis();
    static uint8_t node_id_allocation_transfer_id = 0;

    DNA.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (random() % UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(this->get_preferred_node_id() << 1U);

    if (DNA.node_id_allocation_unique_id_offset == 0)
    {
        allocation_request[0] |= 1; // First part of unique ID
    }

    uint8_t my_unique_id[16];
    getUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(16 - DNA.node_id_allocation_unique_id_offset);

    if (uid_size > MaxLenOfUniqueIDInRequest)
    {
        uid_size = MaxLenOfUniqueIDInRequest;
    }
    if (uid_size + DNA.node_id_allocation_unique_id_offset > 16)
    {
        uid_size = 16 - DNA.node_id_allocation_unique_id_offset;
    }

    memmove(&allocation_request[1], &my_unique_id[DNA.node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    const int16_t bcast_res = canardBroadcast(&canard,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                                              UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                                              &node_id_allocation_transfer_id,
                                              CANARD_TRANSFER_PRIORITY_LOW,
                                              &allocation_request[0],
                                              (uint16_t)(uid_size + 1));
    if (bcast_res < 0)
    {
        Serial.print("Could not broadcast ID allocation req; error");
        Serial.println(bcast_res);
    }

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    DNA.node_id_allocation_unique_id_offset = 0;
}

/*
  handle a BeginFirmwareUpdate request from a management tool like DroneCAN GUI tool or MissionPlanner

  There are multiple ways to handle firmware update over DroneCAN:

    1) on BeginFirmwareUpdate reboot to the bootloader, and implement
       the firmware upudate process in the bootloader. This is good on
       boards with smaller amounts of flash

    2) if you have enough flash for 2 copies of your firmware then you
       can use an A/B scheme, where the new firmware is saved to the
       inactive flash region and a tag is used to indicate which
       firmware to boot next time

    3) you could write the firmware to secondary storage (such as a
       microSD) and the bootloader would flash it on next boot

    In this example firmware we will write it to a file
    newfirmware.bin, which is option 3

    Note that you cannot rely on the form of the filename. The client
    may hash the filename before sending
 */
void DroneCAN::handle_begin_firmware_update(CanardRxTransfer *transfer)
{
}

/*
  send a read for a firmware update. This asks the client (firmware
  server) for a piece of the new firmware
 */
void DroneCAN::send_firmware_read(void)
{
    uint32_t now = millis();
    if (now - fwupdate.last_read_ms < 750)
    {
        // the server may still be responding
        return;
    }
    fwupdate.last_read_ms = now;

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_READ_REQUEST_MAX_SIZE];

    struct uavcan_protocol_file_ReadRequest pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.path.path.len = strlen((const char *)fwupdate.path);
    pkt.offset = fwupdate.offset;
    memcpy(pkt.path.path.data, fwupdate.path, pkt.path.path.len);

    uint16_t total_size = uavcan_protocol_file_ReadRequest_encode(&pkt, buffer);

    canardRequestOrRespond(&canard,
                           fwupdate.node_id,
                           UAVCAN_PROTOCOL_FILE_READ_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_READ_ID,
                           &fwupdate.transfer_id,
                           CANARD_TRANSFER_PRIORITY_HIGH,
                           CanardRequest,
                           &buffer[0],
                           total_size);
}

/*
  handle response to send_firmware_read()
 */
void DroneCAN::handle_file_read_response(CanardRxTransfer *transfer)
{
    if ((transfer->transfer_id + 1) % 32 != fwupdate.transfer_id ||
        transfer->source_node_id != fwupdate.node_id)
    {
        /* not for us */
        Serial.println("Firmware update: not for us");
        return;
    }
    struct uavcan_protocol_file_ReadResponse pkt;
    if (uavcan_protocol_file_ReadResponse_decode(transfer, &pkt))
    {
        /* bad packet */
        Serial.println("Firmware update: bad packet\n");
        return;
    }
    if (pkt.error.value != UAVCAN_PROTOCOL_FILE_ERROR_OK)
    {
        /* read failed */
        fwupdate.node_id = 0;
        Serial.println("Firmware update read failure\n");
        return;
    }

    // write(fwupdate.fd, pkt.data.data, pkt.data.len);
    // if (pkt.data.len < 256)
    // {
    //     /* firmware updare done */
    //     close(fwupdate.fd);
    //     Serial.println("Firmwate update complete\n");
    //     fwupdate.node_id = 0;
    //     return;
    // }
    fwupdate.offset += pkt.data.len;

    /* trigger a new read */
    fwupdate.last_read_ms = 0;
}

/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
 */
void DroneCAN::send_NodeStatus(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    node_status.uptime_sec = micros64() / 1000000ULL;
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;
    // put whatever you like in here for display in GUI
    node_status.vendor_specific_status_code = 0;

    /*
      when doing a firmware update put the size in kbytes in VSSC so
      the user can see how far it has reached
     */
    if (fwupdate.node_id != 0)
    {
        node_status.vendor_specific_status_code = fwupdate.offset / 1024;
        node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_SOFTWARE_UPDATE;
    }

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

void DroneCAN::process1HzTasks(uint64_t timestamp_usec)
{
    /*
      Purge transfers that are no longer transmitted. This can free up some memory
    */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
      Transmit the node status message
    */
    send_NodeStatus();
}

void DroneCAN::processTx()
{
    for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;)
    {
        CANSend(txf);
        canardPopTxQueue(&canard); // fuck it we ball
    }
}

void DroneCAN::processRx()
{
    const uint64_t timestamp = micros();
    if (CANMsgAvail())
    {
        CANReceive(&CAN_rx_msg);
        int ret = canardHandleRxFrame(&canard, &CAN_rx_msg, timestamp);
        if (ret < 0)
        {
            Serial.print("Canard RX fail");
            Serial.println(ret);
        }
    }
}

void DroneCANonTransferReceived(DroneCAN &dronecan, CanardInstance *ins, CanardRxTransfer *transfer)
{
    if (transfer->transfer_type == CanardTransferTypeBroadcast)
    {
        // check if we want to handle a specific broadcast message
        switch (transfer->data_type_id)
        {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
        {
            dronecan.handle_DNA_Allocation(transfer);
            break;
        }
        }
    }
    // switch on data type ID to pass to the right handler function
    else if (transfer->transfer_type == CanardTransferTypeRequest)
    {
        // check if we want to handle a specific service request
        switch (transfer->data_type_id)
        {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        {
            dronecan.handle_GetNodeInfo(transfer);
            break;
        }
        case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        {
            while (1)
            {
            }; // force the watchdog to reset
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        {
            dronecan.handle_param_GetSet(transfer);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
        {
            dronecan.handle_param_ExecuteOpcode(transfer);
            break;
        }
        case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        {
            dronecan.handle_begin_firmware_update(transfer);
            break;
        }
        }
    }
}

bool DroneCANshoudlAcceptTransfer(const CanardInstance *ins,
                                  uint64_t *out_data_type_signature,
                                  uint16_t data_type_id,
                                  CanardTransferType transfer_type,
                                  uint8_t source_node_id)
{
    if (transfer_type == CanardTransferTypeRequest)
    {
        // Check if we want to handle a specific service request
        switch (data_type_id)
        {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_FILE_READ_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_FILE_READ_SIGNATURE;
            return true;
        }
        }
    }

    if (transfer_type == CanardTransferTypeResponse)
    {
        // Check if we want to handle a specific service response
        switch (data_type_id)
        {
        case UAVCAN_PROTOCOL_FILE_READ_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_FILE_READ_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
            return true;
        }
        }
    }

    if (transfer_type == CanardTransferTypeBroadcast)
    {
        // Check if we want to handle a specific broadcast packet
        switch (data_type_id)
        {
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID:
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE;
            return true;
        }
        }
    }
}
