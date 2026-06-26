/***************************************************************************//**
 * @file app.c
 * @brief Callbacks implementation and application specific code.
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * Modified from NabuCasa original to add Green Power MultiRail support.
 *
 ******************************************************************************/

#include PLATFORM_HEADER
#include "ember.h"

void emberAfRadioNeedsCalibratingCallback(void)
{
  sl_mac_calibrate_current_channel();
}

#if defined(SL_CATALOG_ZIGBEE_MULTIRAIL_DEMO_PRESENT)
#include "multirail-demo.h"
#include "stack/include/gp-types.h"

#ifndef GP_RX_OFFSET_USEC
#define GP_RX_OFFSET_USEC 20500
#endif

extern uint8_t sl_mac_lower_mac_get_radio_channel(uint8_t mac_index);
extern EmberMessageBuffer sli_zigbee_gpdf_make_header(bool useCca,
                                                      EmberGpAddress *src,
                                                      EmberGpAddress *dst);
// May be stale for GP data frames (0xE0) — see comment below.
extern uint32_t sli_zigbee_current_mac_timestamp;

static void appGpScheduleOutgoingGpdf(EmberZigbeePacketType packetType,
                                      int8u *packetData,
                                      int8u size_p);

void emberAfMainInitCallback(void)
{
  emberAfPluginMultirailDemoInit(
    NULL, NULL, true,
    RAIL_GetTxPowerDbm(emberGetRailHandle()),
    NULL, 0,
    0xFFFF, NULL);
}

void emberAfPluginMultirailDemoRailEventCallback(RAIL_Handle_t handle,
                                                 RAIL_Events_t events)
{
  (void)handle;
  (void)events;
}

static EmberGpTxQueueEntry *get_gp_stub_tx_queue(EmberGpAddress *addr)
{
  EmberGpTxQueueEntry sli_zigbee_gp_tx_queue;
  MEMCOPY(&sli_zigbee_gp_tx_queue.addr, addr, sizeof(EmberGpAddress));

  uint8_t data[128];
  uint16_t dataLength;

  if (emberAfPluginMultirailDemoGetHandle()
      && emberGpGetTxQueueEntryFromQueue(&sli_zigbee_gp_tx_queue,
                                         data, &dataLength, 128)
         != EMBER_NULL_MESSAGE_BUFFER) {
    EmberMessageBuffer header = sli_zigbee_gpdf_make_header(
      true, NULL, &(sli_zigbee_gp_tx_queue.addr));

    uint8_t len = emberMessageBufferLength(header) + 1;
    emberAppendToLinkedBuffers(header,
                               &(sli_zigbee_gp_tx_queue.gpdCommandId), 1);
    emberSetLinkedBuffersLength(header,
                                emberMessageBufferLength(header) + dataLength);
    emberCopyToLinkedBuffers(data, header, len, dataLength);
    emberGpRemoveFromTxQueue(&sli_zigbee_gp_tx_queue);

    uint8_t outPktLength = emberMessageBufferLength(header);
    uint8_t outPkt[128];
    outPkt[0] = outPktLength + 2;
    emberCopyFromLinkedBuffers(header, 0, &outPkt[1], outPktLength);
    emberReleaseMessageBuffer(header);

    static EmberGpTxQueueEntry copyOfGpStubTxQueue;
    copyOfGpStubTxQueue.inUse = true;
    copyOfGpStubTxQueue.asdu = emberFillLinkedBuffers(outPkt, (outPkt[0] + 1));
    MEMCOPY(&(copyOfGpStubTxQueue.addr), addr, sizeof(EmberGpAddress));
    return &copyOfGpStubTxQueue;
  }
  return NULL;
}

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

/** @brief Schedule a pre-queued GP response via RAIL2.
 *
 *  Supports two incoming packet types:
 *
 *  RAW_MAC (frame_type=1, GP Maintenance frames like 0xE3):
 *    The callback receives the full 802.15.4 MAC frame.  `index`=0, so
 *    packetData[0..6] = MAC header, packetData[7] = NWK FC.
 *    Layout: [FC(2)][Seq(1)][DestPAN(2)][DestAddr(2)][NWK FC][NWK ExtFC][SrcID(4)]
 *
 *  NWK / NWK_ENCRYPTED (frame_type=0, GP Data frames like 0xE0):
 *    The Ember stack classifies GP data frames (frame_type=0) as NWK-layer
 *    packets and strips the MAC header before calling this hook.  `index`
 *    points to the NWK layer, so packetData[0] = NWK FC directly.
 *    Layout: [NWK FC][NWK ExtFC][SrcID(4)][SecFC(4)][Cmd][Payload]
 *
 *  NWK FC bits:
 *    b7    = ExtFC present
 *    b6    = Auto-Commissioning (AC)  [b5-b2 = proto ver = 3]
 *    b1-b0 = Frame Type (0=Data, 1=Maintenance)
 *
 *  NWK ExtFC bits:
 *    b7    = Direction (0 = from GPD)
 *    b6    = RxAfterTx
 *    b2-b0 = AppId (0 = SrcID)
 *
 *  Timing note:
 *    sli_zigbee_current_mac_timestamp is only reliably set for RAW_MAC frames.
 *    For NWK-type frames it may hold a stale value from the previous RAW_MAC
 *    frame (seconds old).  We guard isDataRxAfterTx against this by falling
 *    back to elapsed=0 when elapsed >= GP_RX_OFFSET_USEC, scheduling RAIL2 TX
 *    at GP_RX_OFFSET_USEC from now (jitter = main-loop dispatch latency < 5ms).
 */
static void appGpScheduleOutgoingGpdf(EmberZigbeePacketType packetType,
                                      int8u *packetData,
                                      int8u size_p)
{
  // Determine NWK header offset in packetData.
  //   RAW_MAC / RAW_MAC_ENCRYPTED: NWK starts at byte 7 (after MAC header)
  //   NWK / NWK_ENCRYPTED:         NWK starts at byte 0 (MAC already stripped)
  uint8_t nwkOffset;
  if (packetType == EMBER_ZIGBEE_PACKET_TYPE_RAW_MAC
      || packetType == EMBER_ZIGBEE_PACKET_TYPE_RAW_MAC_ENCRYPTED) {
    if (size_p < 10) return;
    nwkOffset = 7;
  } else if (packetType == EMBER_ZIGBEE_PACKET_TYPE_NWK
             || packetType == EMBER_ZIGBEE_PACKET_TYPE_NWK_ENCRYPTED) {
    if (size_p < 3) return;
    nwkOffset = 0;
  } else {
    return;
  }

  uint8_t nwkFc  = packetData[nwkOffset];
  uint8_t nwkEfc = packetData[nwkOffset + 1];

  // Only GP frames (Protocol Version = 3, bits b5-b2 = 0b0011 = 0x0C)
  if ((nwkFc & 0x3C) != 0x0C) {
    return;
  }

  // GP Maintenance frame (0xE3 Channel Request): FT=1, ExtFC=0, AC=0
  bool isMaintenance = ((nwkFc & 0xC3) == 0x01);
  // GP Data frame with rxAfterTx=1 (0xE0 Commissioning): FT=0, ExtFC=1, AC ignored
  bool isDataRxAfterTx = ((nwkFc & 0x83) == 0x80)
                         && ((nwkEfc & 0xC0) == 0x40); // Dir=0, RxAfterTx=1

  if (!isMaintenance && !isDataRxAfterTx) {
    return;
  }

  // Compute dispatch latency since MAC reception.
  // For NWK-type packets, sli_zigbee_current_mac_timestamp may be stale (set
  // only for RAW_MAC frames in some SDK builds).  Fall back to elapsed=0 for
  // isDataRxAfterTx so we still schedule RAIL2 TX at GP_RX_OFFSET_USEC.
  uint32_t elapsed = RAIL_GetTime() - sli_zigbee_current_mac_timestamp;
  if (elapsed >= GP_RX_OFFSET_USEC) {
    if (isDataRxAfterTx) {
      elapsed = 0; // stale timestamp — schedule from now
    } else {
      return; // maintenance: window genuinely closed
    }
  }

  // Extract GPD Source ID (AppId=0 assumed)
  EmberGpAddress gpdAddr;
  gpdAddr.applicationId = EMBER_GP_APPLICATION_SOURCE_ID;
  gpdAddr.id.sourceId = 0;

  if (isDataRxAfterTx
      && ((nwkEfc & 0x07) == EMBER_GP_APPLICATION_SOURCE_ID)
      && size_p >= (uint8_t)(nwkOffset + 6)) {
    (void)memcpy(&gpdAddr.id.sourceId,
                 &packetData[nwkOffset + 2],
                 sizeof(EmberGpSourceId));
  }

  EmberGpTxQueueEntry *entry = get_gp_stub_tx_queue(&gpdAddr);
  if (!entry) {
    return;
  }

  uint8_t outPktLength = emberMessageBufferLength(entry->asdu);
  uint8_t outPkt[128];
  emberCopyFromLinkedBuffers(entry->asdu, 0, outPkt, outPktLength);

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
    outPkt, outPktLength,
    sl_mac_lower_mac_get_radio_channel(0),
    &scheduledTxConfig, &schedulerInfo);

  emberGpRemoveFromTxQueue(entry);
}

#endif // SL_CATALOG_ZIGBEE_MULTIRAIL_DEMO_PRESENT
