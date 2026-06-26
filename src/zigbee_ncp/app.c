/***************************************************************************//**
 * @file app.c
 * @brief Callbacks implementation and application specific code.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * Modified from NabuCasa original to add Green Power MultiRail support.
 * When a bidirectional GP frame (rxAfterTx=1) is detected at MAC level,
 * the NCP automatically schedules a pre-queued response (0xF0 Commissioning
 * Reply) via a second RAIL instance at exactly GP_RX_OFFSET_USEC after the
 * incoming frame — entirely on the NCP, no host round-trip required.
 *
 * Drop-in replacement for:
 *   src/zigbee_ncp/app.c
 * in a fork of https://github.com/NabuCasa/silabs-firmware-builder
 *
 ******************************************************************************/

#include PLATFORM_HEADER
#include "ember.h"

//----------------------
// Implemented Callbacks

/** @brief
 *
 * Application framework equivalent of ::emberRadioNeedsCalibratingHandler
 */
void emberAfRadioNeedsCalibratingCallback(void)
{
  sl_mac_calibrate_current_channel();
}

// ============================================================================
// Green Power MultiRail support
// Requires component zigbee_multirail_demo to be present in the build.
// The host (bellows) pre-queues the 0xF0 Commissioning Reply via dGpSend
// (EZSP 0xC6) before the user presses the sensor button.  When the GPD sends
// 0xE0 (with rxAfterTx=1), the packet-handoff callback below fires at MAC
// level, finds the pre-queued frame and schedules hardware-timed RAIL2
// transmission at exactly GP_RX_OFFSET_USEC.
// ============================================================================

#if defined(SL_CATALOG_ZIGBEE_MULTIRAIL_DEMO_PRESENT)
#include "multirail-demo.h"
#include "stack/include/gp-types.h"

// Time (µs) between GPD Tx and its receive-window opening.
// Per ZGP spec the GPD opens its window at dGPD_RX_OFFSET = 20 ms.
// 20500 µs gives a small safety margin.
#ifndef GP_RX_OFFSET_USEC
#define GP_RX_OFFSET_USEC 20500
#endif

// Internal SiLabs stack helpers (not in public headers, but stable).
extern uint8_t sl_mac_lower_mac_get_radio_channel(uint8_t mac_index);
extern EmberMessageBuffer sli_zigbee_gpdf_make_header(bool useCca,
                                                      EmberGpAddress *src,
                                                      EmberGpAddress *dst);

// Stack-internal global: RAIL timer value (µs) at the time the current MAC
// frame was received.  Set by sli_zigbee_application_process_incoming()
// before it invokes the packet-handoff callback, so it is always valid when
// our sli_zigbee_af_packet_handoff_incoming_callback() fires.
// Used to compensate for the main-loop dispatch latency so that RAIL2
// schedules the GP response at exactly GP_RX_OFFSET_USEC after MAC reception.
extern uint32_t sli_zigbee_current_mac_timestamp;

// Forward declaration
static void appGpScheduleOutgoingGpdf(EmberZigbeePacketType packetType,
                                      int8u *packetData,
                                      int8u size_p);

/** @brief Application init — initialise the second RAIL instance.
 *
 * The second RAIL handle is used to send hardware-timed GP frames.
 * Parameters mirror the primary Zigbee radio so no additional calibration
 * is needed.
 */
void emberAfMainInitCallback(void)
{
  emberAfPluginMultirailDemoInit(
    NULL,                                         // use default RAIL config
    NULL,                                         // copy TX power from RAIL 1
    true,                                         // PA auto-mode
    RAIL_GetTxPowerDbm(emberGetRailHandle()),      // same TX power as RAIL 1
    NULL,                                         // use default 128-byte FIFO
    0,
    0xFFFF,                                       // no PAN filter
    NULL                                          // no IEEE address filter
    );
}

/** @brief RAIL2 event callback — nothing to do for GP scheduling. */
void emberAfPluginMultirailDemoRailEventCallback(RAIL_Handle_t handle,
                                                 RAIL_Events_t events)
{
  (void)handle;
  (void)events;
}

/** @brief Retrieve and serialise one entry from the dGpSend TX queue for the
 *  given GPD address.
 *
 *  The caller must free the returned entry's asdu buffer after use.
 *  Returns NULL if no entry is found or RAIL2 is not yet initialised.
 */
static EmberGpTxQueueEntry *get_gp_stub_tx_queue(EmberGpAddress *addr)
{
  EmberGpTxQueueEntry sli_zigbee_gp_tx_queue;
  MEMCOPY(&sli_zigbee_gp_tx_queue.addr, addr, sizeof(EmberGpAddress));

  uint8_t data[128];
  uint16_t dataLength;

  if (emberAfPluginMultirailDemoGetHandle()
      && emberGpGetTxQueueEntryFromQueue(&sli_zigbee_gp_tx_queue,
                                         data,
                                         &dataLength,
                                         128) != EMBER_NULL_MESSAGE_BUFFER) {
    // Build a MAC frame for this GP TX queue entry
    EmberMessageBuffer header = sli_zigbee_gpdf_make_header(
      true, NULL, &(sli_zigbee_gp_tx_queue.addr));

    // Append the GP command ID
    uint8_t len = emberMessageBufferLength(header) + 1;
    emberAppendToLinkedBuffers(header,
                               &(sli_zigbee_gp_tx_queue.gpdCommandId), 1);

    // Append the GP command payload
    emberSetLinkedBuffersLength(header,
                                emberMessageBufferLength(header) + dataLength);
    emberCopyToLinkedBuffers(data, header, len, dataLength);

    // Remove from the stub queue — we are taking ownership
    emberGpRemoveFromTxQueue(&sli_zigbee_gp_tx_queue);

    // Serialise into a flat RAIL frame:
    // [Total length (excl. itself) | MAC frame bytes | 2-byte CRC placeholder]
    uint8_t outPktLength = emberMessageBufferLength(header);
    uint8_t outPkt[128];
    outPkt[0] = outPktLength + 2;
    emberCopyFromLinkedBuffers(header, 0, &outPkt[1], outPktLength);
    emberReleaseMessageBuffer(header);

    // Return a static copy (one entry at a time is fine for GP commissioning)
    static EmberGpTxQueueEntry copyOfGpStubTxQueue;
    copyOfGpStubTxQueue.inUse = true;
    copyOfGpStubTxQueue.asdu = emberFillLinkedBuffers(outPkt, (outPkt[0] + 1));
    MEMCOPY(&(copyOfGpStubTxQueue.addr), addr, sizeof(EmberGpAddress));
    return &copyOfGpStubTxQueue;
  }
  return NULL;
}

/** @brief Packet handoff — called for every incoming raw MAC frame.
 *
 *  Passes the frame to appGpScheduleOutgoingGpdf and always accepts it
 *  so normal stack processing continues.
 */
EmberPacketAction sli_zigbee_af_packet_handoff_incoming_callback(
  EmberZigbeePacketType packetType,
  EmberMessageBuffer packetBuffer,
  uint8_t index,
  void *data)
{
  (void)data;
  uint8_t size_p = emberMessageBufferLength(packetBuffer) - index;
  uint8_t packetData[128];
  emberCopyFromLinkedBuffers(packetBuffer, index, packetData, size_p);
  appGpScheduleOutgoingGpdf(packetType, packetData, size_p);
  return EMBER_ACCEPT_PACKET;
}

/** @brief Schedule a pre-queued GP response via RAIL2 if the incoming frame
 *  is a bidirectional GP frame (rxAfterTx = 1) with enough time remaining
 *  before the GPD's receive window.
 *
 *  MAC frame layout (GP broadcast, no source address):
 *    [0-1]  FC (Frame Control)
 *    [2]    Sequence number
 *    [3-4]  Destination PAN ID
 *    [5-6]  Destination address (0xFFFF broadcast)
 *    [7]    NWK FC byte
 *    [8]    NWK ExtFC byte  (if ExtFC-present bit set in [7])
 *    [9-12] GPD Source ID   (if AppId = 0 in ExtFC)
 *
 *  NWK FC bits of interest:
 *    b7    = ExtFC present
 *    b6    = Auto-Commissioning (AC)
 *    b5-b2 = Protocol Version  (must be 0b0011 = 3 for GP)
 *    b1-b0 = Frame Type        (0=Data, 1=Maintenance)
 *
 *  NWK ExtFC bits:
 *    b7    = Direction (0 = from GPD)
 *    b6    = RxAfterTx
 *    b2-b0 = AppId    (0 = SrcID-based)
 *
 *  Timing:
 *    sli_zigbee_current_mac_timestamp holds the RAIL timer value (µs) at
 *    the time the MAC frame was received, set by the stack before calling
 *    this callback.  We subtract it from RAIL_GetTime() to get the exact
 *    main-loop dispatch latency and schedule RAIL2 TX to fire at precisely
 *    GP_RX_OFFSET_USEC after MAC reception regardless of that latency.
 */
static void appGpScheduleOutgoingGpdf(EmberZigbeePacketType packetType,
                                      int8u *packetData,
                                      int8u size_p)
{
  if (packetType != EMBER_ZIGBEE_PACKET_TYPE_RAW_MAC || size_p <= 9) {
    return;
  }

  uint8_t nwkFc  = packetData[7];
  uint8_t nwkEfc = packetData[8];

  // Only GP frames (Protocol Version = 3, bits b5-b2 = 0b0011)
  if ((nwkFc & 0x3C) != 0x0C) {
    return;
  }

  bool isMaintenance = ((nwkFc & 0xC3) == 0x01);  // FT=1, ExtFC=0, AC=0
  bool isDataRxAfterTx = ((nwkFc & 0xC3) == 0x80) // FT=0, ExtFC=1, AC=0
                         && ((nwkEfc & 0xC0) == 0x40); // Dir=0, rxAfterTx=1

  if (!isMaintenance && !isDataRxAfterTx) {
    return;
  }

  // Compute latency since MAC reception.
  // sli_zigbee_current_mac_timestamp is a RAIL µs value written by the stack
  // in sli_zigbee_application_process_incoming() before our callback fires.
  uint32_t elapsed = RAIL_GetTime() - sli_zigbee_current_mac_timestamp;
  if (elapsed >= GP_RX_OFFSET_USEC) {
    // Too late — the GPD receive window has already closed.
    return;
  }

  // Extract GPD Source ID from the NWK frame (AppId = 0 assumed)
  EmberGpAddress gpdAddr;
  gpdAddr.applicationId = EMBER_GP_APPLICATION_SOURCE_ID;
  gpdAddr.id.sourceId = 0;

  if (isDataRxAfterTx
      && ((nwkEfc & 0x07) == EMBER_GP_APPLICATION_SOURCE_ID)) {
    (void)memcpy(&gpdAddr.id.sourceId, &packetData[9],
                 sizeof(EmberGpSourceId));
  }

  // Look for a pre-queued response for this GPD
  EmberGpTxQueueEntry *entry = get_gp_stub_tx_queue(&gpdAddr);
  if (!entry) {
    return;
  }

  // Serialise the queued MAC frame
  uint8_t outPktLength = emberMessageBufferLength(entry->asdu);
  uint8_t outPkt[128];
  emberCopyFromLinkedBuffers(entry->asdu, 0, outPkt, outPktLength);

  // Schedule RAIL2 transmission to hit the GPD receive window exactly.
  // when = GP_RX_OFFSET_USEC - elapsed fires at MAC_RX_time + GP_RX_OFFSET_USEC,
  // compensating for the main-loop dispatch latency measured above.
  RAIL_SchedulerInfo_t schedulerInfo = {
    .priority        = 50,
    .slipTime        = 2000,
    .transactionTime = 5000,
  };
  RAIL_ScheduleTxConfig_t scheduledTxConfig = {
    .mode = RAIL_TIME_DELAY,
    .when = GP_RX_OFFSET_USEC - elapsed,
  };

  (void)emberAfPluginMultirailDemoSend(
    outPkt,
    outPktLength,
    sl_mac_lower_mac_get_radio_channel(0),
    &scheduledTxConfig,
    &schedulerInfo);

  emberGpRemoveFromTxQueue(entry);
}

#endif // SL_CATALOG_ZIGBEE_MULTIRAIL_DEMO_PRESENT
