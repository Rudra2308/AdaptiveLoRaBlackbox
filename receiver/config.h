#pragma once

// Keep these radio values identical to transmitter/config.h.
#define SERIAL_BAUD 115200
#define RADIO_SCK 18
#define RADIO_MISO 19
#define RADIO_MOSI 23
#define RADIO_CS 5
#define RADIO_RST 13
#define RADIO_DIO0 2
#define RADIO_FREQ_MHZ 433.0
#define RADIO_BW_KHZ 125.0
#define RADIO_SF 7
#define RADIO_CR 5
#define RADIO_SYNC_WORD 0x12
#define RADIO_TX_DBM 17
#define RADIO_PREAMBLE 8

#define ACK_TIMEOUT_MS 1000
#define COMMAND_ATTEMPTS 3
#define BER_PAYLOAD_BYTES 16
#define BER_PAYLOAD_BITS (BER_PAYLOAD_BYTES * 8)

// Start with RSSI/SNR/PDR evidence over multiple received frames. These are
// conservative adaptation thresholds, not SX1278 sensitivity specifications.
#define LINK_WINDOW_FRAMES 4
#define LINK_BAD_WINDOWS_TO_DOWNGRADE 2
#define LINK_GOOD_WINDOWS_TO_UPGRADE 4
#define LINK_STALE_MS 12000
#define LINK_GOOD_RSSI_DBM -88.0f
#define LINK_GOOD_SNR_DB 8.0f
#define LINK_GOOD_LOSS_PCT 1.0f
#define LINK_GOOD_RETRY_PCT 3.0f
#define LINK_FAIR_RSSI_DBM -98.0f
#define LINK_FAIR_SNR_DB 1.0f
#define LINK_FAIR_LOSS_PCT 6.0f
#define LINK_FAIR_RETRY_PCT 15.0f
#define LINK_WEAK_RSSI_DBM -109.0f
#define LINK_WEAK_SNR_DB -7.0f
#define LINK_WEAK_LOSS_PCT 20.0f
#define LINK_WEAK_RETRY_PCT 35.0f
#define LINK_POWER_GOOD_DBM 10
#define LINK_POWER_FAIR_DBM 12
#define LINK_POWER_WEAK_DBM 15
#define LINK_POWER_CRITICAL_DBM 17
#define LINK_CHUNK_GOOD 4
#define LINK_CHUNK_FAIR 4
#define LINK_CHUNK_WEAK 2
#define LINK_CHUNK_CRITICAL 1
