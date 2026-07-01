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

// Forward declaration
static void appGpScheduleOutgoingGpdf(EmberZigbeePacketType packetType,
                                      int8u *packetData,
                                      int8u size_p);

// Microseconds elapsed since MAC received the packet.
// NOTE: Only valid when macTimeStamp is a reliable per-packet RAIL timestamp.
// For GP commissioning frames (0xE0, 0xE3) it is typically stale — see below.
#define macToAppDelay(macTimeStamp) \
  ((RAIL_GetTime() & 0x00FFFFFF) - (macTimeStamp))

// ============================================================================
// Debug tracking — written from packet-handoff callback (MAC-level),
// reported from emberAfMainTickCallback (main-loop, safe for debug output).
// ============================================================================

// Result codes stored in g_gp_dbg.result:
#define DBG_RESULT_NONE          0   // no GP frame seen yet
#define DBG_RESULT_NOT_GP        1   // NWK FC did not match GP protocol version
#define DBG_RESULT_NO_MATCH      2   // frame type not isMaintenance / isDataRxAfterTx
#define DBG_RESULT_QUEUE_MISS    3   // no pre-queued entry for this SrcID
#define DBG_RESULT_RAIL2_SCHED   4   // RAIL2 transmission successfully scheduled
#define DBG_RESULT_RAIL2_FAIL    5   // RAIL2 scheduling returned error

static volatile struct {
  uint8_t  last_nwk_fc;         // NWK FC byte of the last GP-candidate frame
  uint8_t  last_nwk_efc;        // NWK ExtFC byte (0 for maintenance frames)
  uint32_t last_src_id;         // SrcID extracted from the last frame
  uint8_t  last_queue_cmd_id;   // GP command ID from the queue entry (0xF0 / 0xF3)
  uint8_t  result;              // DBG_RESULT_* code from last GP frame
  RAIL_Status_t last_rail_status; // RAIL_StartScheduledTx return value
  uint32_t rail2_tx_sent;       // RAIL_EVENT_TX_PACKET_SENT count
  uint32_t rail2_tx_error;      // RAIL_EVENT_SCHEDULER_STATUS non-success count
  RAIL_SchedulerStatus_t last_sched_status; // last scheduler error code
  uint32_t call_count;          // total times appGpScheduleOutgoingGpdf was called
  bool dirty;                   // true when state changed since last log output
} g_gp_dbg;

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

/** @brief RAIL2 event callback — track TX outcomes for debug reporting.
 *
 *  Counters are visible via g_gp_dbg (accessible from a debugger or via
 *  custom EZSP query).  On success, RAIL_EVENT_TX_PACKET_SENT fires;
 *  on failure, RAIL_EVENT_SCHEDULER_STATUS fires with a non-zero status.
 *
 *  RAIL_EVENT_TX_PACKET_SENT:    frame physically transmitted OTA ✓
 *  RAIL_EVENT_SCHEDULER_STATUS:  multi-protocol scheduler result
 *                                (RAIL_SCHEDULER_STATUS_NO_ERROR = OK)
 */
void emberAfPluginMultirailDemoRailEventCallback(RAIL_Handle_t handle,
                                                 RAIL_Events_t events)
{
  if (events & RAIL_EVENTS_TX_COMPLETION) {
    if (events & RAIL_EVENT_TX_PACKET_SENT) {
      g_gp_dbg.rail2_tx_sent++;
    } else {
      g_gp_dbg.rail2_tx_error++;
    }
    g_gp_dbg.dirty = true;
  }

  if (events & RAIL_EVENT_SCHEDULER_STATUS) {
    RAIL_SchedulerStatus_t s = RAIL_GetSchedulerStatus(handle);
    g_gp_dbg.last_sched_status = s;
    if (s != RAIL_SCHEDULER_STATUS_NO_ERROR) {
      g_gp_dbg.rail2_tx_error++;
      g_gp_dbg.dirty = true;
    }
  }
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

    // Store the command ID for debug logging
    g_gp_dbg.last_queue_cmd_id = sli_zigbee_gp_tx_queue.gpdCommandId;

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
 *    ...
 *    [size_p-3..size_p-1]  MAC timestamp (appended by stack, big-endian 24-bit)
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

  // Track every GP frame that reaches this point
  g_gp_dbg.last_nwk_fc  = nwkFc;
  g_gp_dbg.last_nwk_efc = nwkEfc;
  g_gp_dbg.call_count++;
  g_gp_dbg.result = DBG_RESULT_NOT_GP;
  g_gp_dbg.dirty  = true;

  bool isMaintenance = ((nwkFc & 0xC3) == 0x01);  // FT=1, NFCE=0, AC=don't-care
  // FIX 2026-06-30: CL110 sets AC=1 (Auto Commissioning / button-press) in its 0xE0.
  // Old mask 0xC3 checked bit6=AC=0, failing for CL110 (nwkFc=0xCC → 0xC0 ≠ 0x80).
  // New mask 0x83 ignores bit6 so AC=0 and AC=1 devices both match.
  // RxAfterTx is bit7 of nwkEfc; CL110 sends nwkEfc=0x9C (RxAfterTx=1, KeyType=1, SecLvl=3).
  // Old condition (nwkEfc & 0xC0)==0x40 required bit7=0,bit6=1 → never true for CL110.
  // New condition (nwkEfc & 0x80)==0x80 directly tests the RxAfterTx bit.
  bool isDataRxAfterTx = ((nwkFc & 0x83) == 0x80) // FT=0 (Data), NFCE=1, AC=don't-care
                         && ((nwkEfc & 0x80) == 0x80); // RxAfterTx=1 (bit7)

  if (!isMaintenance && !isDataRxAfterTx) {
    g_gp_dbg.result = DBG_RESULT_NO_MATCH;
    return;
  }

  // Always schedule at GP_RX_OFFSET_USEC from "now" — do NOT use
  // macToAppDelay() for commissioning frames.
  //
  // The timestamp appended to 0xE0/0xE3 frames in the packet buffer is sourced
  // from the GP-proxy global sli_zigbee_current_mac_timestamp, which is only
  // refreshed when the GP proxy subsystem processes an 0xE3 Channel Request.
  //
  // For 0xE0 Commissioning Data frames the GP proxy does NOT update that
  // global, so it still holds the timestamp from the previous 0xE3 (seconds
  // ago).  macToAppDelay() then returns a huge elapsed value >> GP_RX_OFFSET_USEC,
  // triggering an early return BEFORE the queue lookup — RAIL2 never fires.
  //
  // The packet-handoff callback is invoked within ~200 µs of MAC frame receipt
  // (synchronous with the MAC interrupt chain), so scheduling GP_RX_OFFSET_USEC
  // (20.5 ms) from "now" is accurate enough to hit the GPD receive window for
  // both 0xE3 Maintenance frames and 0xE0 Data frames.
  const uint32_t scheduleWhen = GP_RX_OFFSET_USEC;

  // Extract GPD Source ID from the NWK frame (AppId = 0 assumed)
  EmberGpAddress gpdAddr;
  gpdAddr.applicationId = EMBER_GP_APPLICATION_SOURCE_ID;
  gpdAddr.id.sourceId = 0;

  if (isDataRxAfterTx
      && ((nwkEfc & 0x03) == EMBER_GP_APPLICATION_SOURCE_ID)) {
    // 0xE0 Commissioning: GP Data frame with ExtFC byte present.
    // AppID is bits 0-1 only (mask 0x03); bit2 is LSB of Security Level.
    // Old mask 0x07 included bit2, so (0x9C & 0x07)=0x04 ≠ 0 → SrcID never extracted.
    // Layout: [7]=NWK FC, [8]=NWK ExtFC, [9..12]=SrcID
    (void)memcpy(&gpdAddr.id.sourceId, &packetData[9],
                 sizeof(EmberGpSourceId));
  } else if (isMaintenance && size_p > 12) {
    // 0xE3 Channel Request: GP Maintenance frame, no ExtFC byte.
    // Layout: [7]=NWK FC, [8..11]=SrcID (0x00000000 = broadcast for 0xE3)
    (void)memcpy(&gpdAddr.id.sourceId, &packetData[8],
                 sizeof(EmberGpSourceId));
  }

  g_gp_dbg.last_src_id = gpdAddr.id.sourceId;

  // Look for a pre-queued response for this GPD
  EmberGpTxQueueEntry *entry = get_gp_stub_tx_queue(&gpdAddr);
  if (!entry) {
    g_gp_dbg.result = DBG_RESULT_QUEUE_MISS;
    return;
  }

  // Serialise the queued MAC frame
  uint8_t outPktLength = emberMessageBufferLength(entry->asdu);
  uint8_t outPkt[128];
  emberCopyFromLinkedBuffers(entry->asdu, 0, outPkt, outPktLength);

  // NOTE: FC patching removed (2026-07-01).
  //
  // The original block attempted to patch the Security Frame Counter in the
  // pre-queued 0xF0 ASDU by scanning outPkt for the 0xF0 command byte and
  // overwriting bytes at offset +2 with gpd_security_fc.  This was wrong for
  // two independent reasons:
  //
  //   1. WRONG ASDU FORMAT ASSUMPTION.
  //      The block assumed: options(1) | FC(4) | enc_key(16) | MIC(4).
  //      Our host builds:  options(1) | enc_key(16) | MIC(4) | FC(4)  (ZGP spec A.3.3.5.2).
  //      Offset fi+2 therefore hits enc_key[0:4], not the FC field.
  //      → The first 4 bytes of the encrypted NWK key were silently trashed.
  //
  //   2. NONCE IS FC-DEPENDENT.
  //      The block's comment claimed "nonce is SrcID||SrcID||SrcID||0x05 —
  //      FC-independent, so MIC stays valid after patching FC."  That nonce
  //      formula is wrong.  The actual GSDK TC-LK nonce is:
  //          0x00000000 || SrcID_LE || FC_LE || 0x05
  //      which IS FC-dependent.  Even if the FC were patched at the correct
  //      ASDU offset, the MIC would be invalid for the patched FC value.
  //
  // The host already ensures the 0xF0 ASDU contains the correct FC via
  // periodic re-queuing (~1 s interval, FC incremented by 256 each call).
  // At the time the 0xE0 arrives the queued ASDU has FC = raw_counter + 1,
  // which satisfies the GPD's FC check (FC_received > FC_stored).
  // The enc_key and MIC were computed with that same FC in the nonce, so
  // the CL110 can verify and accept the commissioning reply.
  //
  // Schedule RAIL2 transmission to hit the GPD receive window exactly
  RAIL_SchedulerInfo_t schedulerInfo = {
    .priority        = 50,
    .slipTime        = 2000,
    .transactionTime = 5000,
  };
  RAIL_ScheduleTxConfig_t scheduledTxConfig = {
    .mode = RAIL_TIME_DELAY,
    .when = scheduleWhen,
  };

  RAIL_Status_t railStatus = emberAfPluginMultirailDemoSend(
    outPkt,
    outPktLength,
    sl_mac_lower_mac_get_radio_channel(0),
    &scheduledTxConfig,
    &schedulerInfo);

  g_gp_dbg.last_rail_status = railStatus;
  g_gp_dbg.result = (railStatus == RAIL_STATUS_NO_ERROR)
                    ? DBG_RESULT_RAIL2_SCHED
                    : DBG_RESULT_RAIL2_FAIL;

  emberGpRemoveFromTxQueue(entry);
}

#endif // SL_CATALOG_ZIGBEE_MULTIRAIL_DEMO_PRESENT
