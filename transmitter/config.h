#pragma once

// Keep these values identical to receiver/config.h.
#define SERIAL_BAUD 115200
#define I2C_SDA 21
#define I2C_SCL 22
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

#define DEVICE_ID 1
#define SAMPLE_HZ 50
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_HZ)
#define PRE_SECONDS 2
#define POST_SECONDS 2
#define EVENT_CAPACITY (SAMPLE_HZ * (PRE_SECONDS + POST_SECONDS))
#define EVENT_CHUNK_SAMPLES 4
#define HEARTBEAT_INTERVAL_MS 3000
#define ACK_TIMEOUT_MS 1000
#define MAX_RETRIES 2
#define BER_INTERVAL_MS 1000
#define BER_PAYLOAD_BYTES 16
#define BER_PAYLOAD_BITS (BER_PAYLOAD_BYTES * 8)

// Adaptive power stays inside this board/configured cap. SF/BW/CR remain
// fixed so the receiver can never lose synchronization during adaptation.
// RadioLib SX127x supports low output-power settings; allow the receiver's
// configured cap to be below the normal 10 dBm target when bench testing.
#define ADAPTIVE_MIN_TX_DBM 2
#define ADAPTIVE_MAX_TX_DBM RADIO_TX_DBM
#define ADAPTIVE_DEFAULT_CHUNK_SAMPLES 4
#define ADAPTIVE_MIN_CHUNK_SAMPLES 1
#define ADAPTIVE_WEAK_LEVEL 2

struct DetectionConfig {
  float impact_mps2;
  float gyro_rad_s;
  float tilt_deg;
  uint32_t tilt_duration_ms;
  uint32_t repeated_window_ms;
  uint8_t repeated_count;
  uint32_t cooldown_ms;
  float pre_seconds;
  float post_seconds;
  float impact_weight;
  float rotation_weight;
  float repeated_weight;
  float decay_per_hour;
};

static DetectionConfig detection = {
  25.0f, 4.0f, 35.0f, 500, 10000, 3, 1500,
  2.0f, 2.0f, 0.5f, 0.3f, 0.2f, 0.0f
};
