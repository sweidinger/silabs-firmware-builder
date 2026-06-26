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
 * GP commissioning frame handling:
 *   - 0xE3 (Channel Request, frame_type=1 Maintenance): arrives as
 *     EMBER_ZIGBEE_PACKET_TYPE_RAW_MAC with full MAC header; NWK FC at [7].
 *   - 0xE0 (Commissioning, frame_type=0 Data): arrives as a non-RAW_MAC type
 *     (NWK-layer) with the MAC header already stripped by the Ember stack;
 *     NWK FC at [0].
 *   We handle both by branching on EMBER_ZIGBEE_PACKET_TYPE_RAW_MAC.
 *   Any non-GP frame is quickly rejected by the GP protocol version check.
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
// Only reliably updated for RAW_MAC frames (GP maintenance); may be stale
// for NWK-layer frames (GP data).  See elapsed fallback below.
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

static void appGpScheduleOutgoingGpdf(EmberZigbeePacketType packetType,
                                      int8u *packetData,
                                      int8u size_p)
{
  // Choose NWK header offset based on packet type.
  //
  // EMBER_ZIGBEE_PACKET_TYPE_RAW_MAC:
  //   Full 802.15.4 frame: [FC(2)][Seq(1)][DestPAN(2)][DestAddr(2)][NWK...]
  //   NWK FC is at packetData[7].
  //
  // All other types (NWK-layer, etc.):
  //   Ember has already stripped the MAC header; the callback receives only
  //   the NWK payload.  NWK FC is at packetData[0].
  //   GP data frames (0xE0, frame_type=0) fall into this category.
  //
  // Any non-GP frame is rejected by the GP protocol version check below.
  uint8_t nwkOffset;
  if (packetType == EMBER_ZIGBEE_PACKET_TYPE_RAW_MAC) {
    if (size_p < 10) {
      return;
    }
    nwkOffset = 7;
  } else {
    if (size_p < 6) {
      return;
    }
    nwkOffset = 0;
  }

  uint8_t nwkFc  = packetData[nwkOffset];
  uint8_t nwkEfc = packetData[nwkOffset + 1];

  // Reject non-GP frames: GP protocol version = 3, bits[5:2] = 0b0011 = 0x0C
  if ((nwkFc & 0x3C) != 0x0C) {
    return;
  }

  // GP Maintenance frame (0xE3 Channel Request): FT=1, ExtFC=0, AC=0
  bool isMaintenance = ((nwkFc & 0xC3) == 0x01);

  // GP Data commissioning frame (0xE0) with rxAfterTx=1:
  //   NWK FC: FT=0, ExtFC present, AC bit ignored
  //   ExtFC:  Direction=0 (from GPD), RxAfterTx=1
  bool isDataRxAfterTx = ((nwkFc & 0x83) == 0x80)
                         && ((nwkEfc & 0xC0) == 0x40);

  if (!isMaintenance && !isDataRxAfterTx) {
    return;
  }

  // Compute dispatch latency since MAC reception.
  //
  // sli_zigbee_current_mac_timestamp is only updated for RAW_MAC frames (GP
  // maintenance).  For NWK-layer frames (GP data, 0xE0) it retains the value
  // from the last maintenance frame — potentially seconds old — making elapsed
  // >> GP_RX_OFFSET_USEC.  Fall back to elapsed=0 so we still schedule RAIL2
  // TX at GP_RX_OFFSET_USEC from now; the extra jitter (main-loop dispatch
  // latency < 5 ms) is well within the GPD's 20 ms receive window.
  uint32_t elapsed = RAIL_GetTime() - sli_zigbee_current_mac_timestamp;
  if (elapsed >= GP_RX_OFFSET_USEC) {
    if (isDataRxAfterTx) {
      elapsed = 0;
    } else {
      return;
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

  // Look for a pre-queued response frame in the GP TX stub queue
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
