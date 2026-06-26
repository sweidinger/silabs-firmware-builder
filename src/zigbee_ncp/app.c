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
 * the NCP schedules a pre-queued response (0xF0 Commissioning Reply) via
 * a second RAIL instance at exactly GP_RX_OFFSET_USEC after the incoming
 * frame.  If RAIL2 scheduling fails the queue entry is left intact so the
 * NCP native GP stub can still attempt the send.
 *
 * Drop-in replacement for:
 *   src/zigbee_ncp/app.c
 * in a fork of https://github.com/NabuCasa/silabs-firmware-builder
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
// Requires component zigbee_multirail_demo in the build.
//
// Host (bellows) pre-queues the 0xF0 Commissioning Reply via dGpSend (EZSP
// 0xC6) before the sensor button is pressed.  When the GPD sends 0xE0 (with
// rxAfterTx=1), sli_zigbee_af_packet_handoff_incoming_callback fires at MAC
// level, finds the pre-queued frame and schedules hardware-timed RAIL2
// transmission at exactly GP_RX_OFFSET_USEC.
//
// IMPORTANT: the queue entry is removed from the GP stub TX queue ONLY after
// RAIL2 successfully schedules the TX (RAIL_STATUS_NO_ERROR).  If RAIL2
// scheduling fails, the entry is left in the queue so the NCP native GP stub
// can still attempt a send on the next occasion.
// ============================================================================

#if defined(SL_CATALOG_ZIGBEE_MULTIRAIL_DEMO_PRESENT)
#include "multirail-demo.h"
#include "stack/include/gp-types.h"
#include "rail.h"

// Time (us) between GPD Tx end and its receive-window opening.
// Per ZGP spec the GPD opens its window at dGPD_RX_OFFSET = 20 ms.
// We use 20000 us (not 20500) to err on the early side.
#ifndef GP_RX_OFFSET_USEC
#define GP_RX_OFFSET_USEC 20000
#endif

// Internal SiLabs stack helpers (stable internal symbols).
extern uint8_t sl_mac_lower_mac_get_radio_channel(uint8_t mac_index);
extern EmberMessageBuffer sli_zigbee_gpdf_make_header(bool useCca,
                                                      EmberGpAddress *src,
                                                      EmberGpAddress *dst);

// Stack-internal global: RAIL timer value (us) at the time the current MAC
// frame was received.  Set before our packet-handoff callback fires.
extern uint32_t sli_zigbee_current_mac_timestamp;

// Diagnostic counters (readable via emberGetCounters indirectly; mainly for
// debug builds).  Reset on each commissioning attempt.
static uint16_t gp_multirail_filter_pass  = 0;
static uint16_t gp_multirail_queue_found  = 0;
static uint16_t gp_multirail_rail2_ok     = 0;
static uint16_t gp_multirail_rail2_fail   = 0;

// Forward declaration
static void appGpScheduleOutgoingGpdf(EmberZigbeePacketType packetType,
                                      uint8_t *packetData,
                                      uint8_t size_p);

/** @brief Application init -- initialise the second RAIL instance. */
void emberAfMainInitCallback(void)
{
  emberAfPluginMultirailDemoInit(
    NULL,                                         // default RAIL config
    NULL,                                         // copy TX power from RAIL 1
    true,                                         // PA auto-mode
    RAIL_GetTxPowerDbm(emberGetRailHandle()),      // same TX power as RAIL 1
    NULL,                                         // default 128-byte FIFO
    0,                                            // txFifoSize (ignored when NULL)
    0xFFFF,                                       // no PAN filter
    NULL                                          // no IEEE address filter
    );
}

/** @brief RAIL2 event callback. */
void emberAfPluginMultirailDemoRailEventCallback(RAIL_Handle_t handle,
                                                 RAIL_Events_t events)
{
  (void)handle;
  (void)events;
}

/** @brief Get and serialise one TX queue entry for the given GPD address.
 *
 *  Fills *outPkt with a RAIL-ready packet (length byte + MAC frame + 2 CRC
 *  placeholder bytes) and returns the total byte count, or 0 on failure.
 *
 *  NOTE: does NOT remove the entry from the queue.  The caller must call
 *  emberGpRemoveFromTxQueue() explicitly after a successful RAIL2 send.
 */
static uint8_t get_gp_stub_tx_packet(EmberGpAddress *addr,
                                     uint8_t *outPkt,
                                     uint8_t maxLen)
{
  if (!emberAfPluginMultirailDemoGetHandle()) {
    return 0;  // RAIL2 not initialised
  }

  EmberGpTxQueueEntry queueEntry;
  MEMSET(&queueEntry, 0, sizeof(queueEntry));
  MEMCOPY(&queueEntry.addr, addr, sizeof(EmberGpAddress));

  uint8_t asduData[128];
  uint16_t asduLen = 0;

  if (emberGpGetTxQueueEntryFromQueue(&queueEntry, asduData, &asduLen, 128)
      == EMBER_NULL_MESSAGE_BUFFER) {
    return 0;  // no matching entry in queue
  }

  // Build the GP MAC frame: header + commandId + ASDU payload
  EmberMessageBuffer header = sli_zigbee_gpdf_make_header(
    false, NULL, &queueEntry.addr);   // useCca=false for GP response
  if (header == EMBER_NULL_MESSAGE_BUFFER) {
    return 0;
  }

  // Append GP command ID and ASDU
  emberAppendToLinkedBuffers(header, &queueEntry.gpdCommandId, 1);
  if (asduLen > 0) {
    uint8_t hdrLen = emberMessageBufferLength(header);
    emberSetLinkedBuffersLength(header, hdrLen + asduLen);
    emberCopyToLinkedBuffers(asduData, header, hdrLen, asduLen);
  }

  // Serialise: [total_len_incl_2crc | mac_frame_bytes | crc_placeholder x2]
  uint8_t frameLen = emberMessageBufferLength(header);
  uint8_t totalLen = 1 + frameLen + 2;  // length byte + frame + 2 CRC
  if (totalLen > maxLen) {
    emberReleaseMessageBuffer(header);
    return 0;
  }

  outPkt[0] = frameLen + 2;  // PHY length field (frame + CRC)
  emberCopyFromLinkedBuffers(header, 0, &outPkt[1], frameLen);
  outPkt[1 + frameLen]     = 0x00;  // CRC placeholder (radio fills in)
  outPkt[1 + frameLen + 1] = 0x00;
  emberReleaseMessageBuffer(header);

  return totalLen;
}

/** @brief Packet handoff -- called for every incoming raw MAC frame. */
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

/** @brief Schedule a pre-queued GP response via RAIL2 for the bidirectional
 *  GP frame receive window.
 *
 *  MAC frame layout (GP broadcast, short dst, no src address):
 *    [0-1]  Frame Control
 *    [2]    Sequence number
 *    [3-4]  Destination PAN ID
 *    [5-6]  Destination address (0xFFFF)
 *    [7]    NWK FC byte
 *    [8]    NWK ExtFC byte  (when ExtFC-present bit set in [7])
 *    [9-12] GPD Source ID   (when AppId = 0 in ExtFC)
 */
static void appGpScheduleOutgoingGpdf(EmberZigbeePacketType packetType,
                                      uint8_t *packetData,
                                      uint8_t size_p)
{
  if (packetType != EMBER_ZIGBEE_PACKET_TYPE_RAW_MAC || size_p < 13) {
    return;
  }

  uint8_t nwkFc  = packetData[7];
  uint8_t nwkEfc = packetData[8];

  // Only GP frames (Protocol Version = 3, bits b5-b2 = 0b0011 = 0x0C)
  if ((nwkFc & 0x3C) != 0x0C) {
    return;
  }

  // We only care about data frames with rxAfterTx=1:
  //   nwkFc bits: b7=ExtFC-present=1, b1-b0=FT=0 (data), AC bit ignored
  //   nwkEfc bits: b7=Dir=0 (from GPD), b6=rxAfterTx=1
  bool isDataRxAfterTx = ((nwkFc  & 0x83) == 0x80)   // FT=data, ExtFC=1
                         && ((nwkEfc & 0xC0) == 0x40); // Dir=0, rxAfterTx=1

  if (!isDataRxAfterTx) {
    return;
  }

  // Only SrcID mode (AppId = 0b000)
  if ((nwkEfc & 0x07) != EMBER_GP_APPLICATION_SOURCE_ID) {
    return;
  }

  gp_multirail_filter_pass++;

  // Compute latency since MAC reception.
  uint32_t elapsed = RAIL_GetTime() - sli_zigbee_current_mac_timestamp;
  if (elapsed >= GP_RX_OFFSET_USEC) {
    return;  // too late
  }

  // Extract GPD Source ID from MAC payload
  EmberGpAddress gpdAddr;
  MEMSET(&gpdAddr, 0, sizeof(gpdAddr));
  gpdAddr.applicationId = EMBER_GP_APPLICATION_SOURCE_ID;
  MEMCOPY(&gpdAddr.id.sourceId, &packetData[9], sizeof(EmberGpSourceId));

  // Build the RAIL TX packet from the pre-queued GP TX stub entry.
  // NOTE: entry remains in queue until we explicitly call emberGpRemoveFromTxQueue.
  uint8_t outPkt[128];
  uint8_t outPktLen = get_gp_stub_tx_packet(&gpdAddr, outPkt, sizeof(outPkt));
  if (outPktLen == 0) {
    return;  // no entry or RAIL2 not ready
  }

  gp_multirail_queue_found++;

  RAIL_SchedulerInfo_t schedulerInfo = {
    .priority        = 50,
    .slipTime        = 2000,
    .transactionTime = 5000,
  };
  RAIL_ScheduleTxConfig_t scheduledTxConfig = {
    .mode = RAIL_TIME_DELAY,
    .when = GP_RX_OFFSET_USEC - elapsed,
  };

  RAIL_Status_t status = emberAfPluginMultirailDemoSend(
    outPkt,
    outPktLen,
    sl_mac_lower_mac_get_radio_channel(0),
    &scheduledTxConfig,
    &schedulerInfo);

  if (status == RAIL_STATUS_NO_ERROR) {
    // RAIL2 TX scheduled successfully.
    // Remove from the GP stub queue so the native stub does not double-send.
    gp_multirail_rail2_ok++;
    EmberGpTxQueueEntry removeEntry;
    MEMSET(&removeEntry, 0, sizeof(removeEntry));
    MEMCOPY(&removeEntry.addr, &gpdAddr, sizeof(EmberGpAddress));
    emberGpRemoveFromTxQueue(&removeEntry);
  } else {
    // RAIL2 scheduling failed.
    // Leave the entry in the queue so the NCP native GP stub can still
    // attempt to send it (may be slightly late but better than nothing).
    gp_multirail_rail2_fail++;
  }
}

#endif // SL_CATALOG_ZIGBEE_MULTIRAIL_DEMO_PRESENT