# Fresh build radio and serial protocol

## Radio settings

Both ESP32 radios must use the same SX1278 modulation settings: 433.0 MHz, 125 kHz bandwidth, SF7, coding rate 4/5, sync word `0x12`, preamble 8, CRC enabled. These remain fixed in V2 to prevent a one-sided SF/BW change from losing synchronization. Transmitter output power is adapted by receiver feedback but remains capped by `RADIO_TX_DBM`; event chunk size is adapted as a software reliability/load control.

The transmitter assigns a 32-bit sequence to each frame. The receiver suppresses duplicate payload processing and still ACKs duplicate reliable frames. Heartbeats and event metadata/sample chunks use `ACK,<seq>`, up to three total attempts, and a 1000 ms ACK timeout. BER test payloads are one-shot and counted as packet loss if not received. The BER completion frame is ACKed.

## Transmitter radio frames

- `V1,HB,seq,device,uptime_ms,config_revision,sample_count,accel_magnitude,gyro_magnitude,event_queue_drops,logical,acked,failed,attempts,retries,ack_latency_ms,ack_count,actual_tx_dbm,link_level,samples_per_frame` every 3 seconds. Heartbeats are sent by the radio task, independently of the sensor sampler. The final three fields report transmitter profile and actual power, including local fallback adjustments.
- `V1,EVT,seq,event_id,trigger_ms,reason,peak_accel_mps2,peak_gyro_rad_s,event_risk,cumulative_risk,total_sample_count,config_revision,pre_s,post_s,gyro_bias_x,gyro_bias_y,gyro_bias_z,phase,start_index` marks an incident. `phase` is `PRE` when the cached samples are sent immediately and `POST` when the later samples and final risk values are sent. The capture windows are fixed at two seconds before and two seconds after the trigger.
- `V1,SMP,seq,event_id,chunk_index,chunk_count,start_index,offset_ms:ax:ay:az:gx:gy:gz;...` sends up to four samples per ACKed chunk. The samples contain raw physical-unit MPU6050 readings, and time offsets are relative to the trigger. The pre-trigger phase is sent first; the post-trigger phase starts after two seconds have elapsed.
- `V1,TEST,seq,run_id,index,hex_payload` carries a deterministic 16-byte payload at one packet per second. Seed: `run_id ^ (index * 0x9e3779b9) ^ 0xa5c39e17`, followed by xorshift32 for each byte.
- `V1,BEND,seq,run_id,expected_count` closes a BER test and is ACKed.

## Link adaptation

The receiver evaluates four decoded V1 frames per window using average RSSI/SNR, sequence gaps/duplicates, and deltas in transmitter heartbeat retry/failure counters. Two consecutively worse windows step down one level; four better windows step up one level. The first valid window establishes the initial state. Offline is reported after 12 seconds without a valid frame.

The receiver sends `LINK,id,level,target_power_dbm,samples_per_event_frame` as a normal point-to-point command. The transmitter validates level 0–3, the configured power cap, and chunk size 1–4; it applies `setOutputPower()` and the chunk setting, then responds with `RSP,id,LINK_APPLIED,power`. BER is stopped/rejected at levels 2–3. SF/BW/CR never change dynamically.

| Level | Name | Default power target | Samples per event frame | BER |
|---:|---|---:|---:|---|
| 0 | GOOD | 10 dBm | 4 | Allowed |
| 1 | FAIR | 12 dBm | 4 | Allowed |
| 2 | WEAK | 15 dBm | 2 | Blocked |
| 3 | CRITICAL | 17 dBm or configured cap | 1 | Blocked |

The numerical RSSI/SNR/loss thresholds are conservative starting values in `receiver/config.h`, not module sensitivity guarantees. Calibrate them with the actual RA-02 boards, antennas, mounting, and environment.

After two consecutive failed reliable frames, the transmitter locally raises output power by 2 dB per pair of failures up to `ADAPTIVE_MAX_TX_DBM`; this provides a recovery path when receiver feedback cannot cross the link. On reconnect, the receiver resets stale sequence and heartbeat-counter baselines but retains the last applied level. RSSI is normalized against reported actual TX power while evaluating that level, and a stable profile is reapplied after four good windows if local fallback had raised power.

## Receiver commands to transmitter

The browser writes one newline-terminated command to the receiver USB serial port. The receiver forwards it over LoRa and reports the transmitter response to the browser as `CMD_ACK,<id>,<result>,<value>`.

- `SET,id,revision,impact_mps2,gyro_rad_s,tilt_deg,tilt_duration_ms,repeated_window_ms,repeated_count,cooldown_ms,pre_s,post_s,impact_weight,rotation_weight,repeated_weight,decay_per_hour`
- `BER,id,run_id,packet_count`
- `BER_STOP,id`

The transmitter replies over LoRa with `RSP,id,<result>[,<value>]`. Settings are range-checked and saved in ESP32 Preferences before `SET_OK` is returned. The receiver matches the reply to the command ID and retries up to three times. It processes any transmitter data frame received while awaiting the command response.

## Receiver USB serial records

The receiver prints `RADIO_RX,<frame>` and `RADIO_TX,<frame>` for radio traffic, plus structured records the host viewer consumes:

- `HEARTBEAT,...`
- `RADIO,millis,rssi_dbm,snr_db,seq,duplicate,missing_sequences`
- `EVENT_META,...`, `EVENT_SAMPLE,...`, `EVENT_CHUNK,...`
- `BER_SAMPLE,run,index,seq,bits,errors,rssi_dbm,snr_db`
- `BER_DONE,run,expected,received,missing`
- `LINK_STATE,level,name,mean_rssi_dbm,mean_snr_db,loss_percent,retry_percent,power_dbm,samples_per_frame,ber_allowed,reason`
- The host app turns `LINK_STATE` rows with reason `adaptive` into a transition history showing changes to power, event chunk size, BER availability, and live setting access. Offline/recovery transitions are shown as well; modem settings remain fixed.
- `LINK_CONTROL_FAIL,level,reason`
- `BER_ABORT,LINK_WEAK,level`
- `CMD_ACK,...` and `CMD_ERROR,...`

## Metric definitions and limits

- PDR is ACKed logical reliable frames divided by reliable frames attempted, taken from transmitter heartbeat counters.
- PER is reliable frames exhausting their bounded retries divided by reliable frames attempted.
- Retry rate is extra attempts divided by logical reliable frames. ACK latency is the mean elapsed time from first transmission to ACK.
- RSSI/SNR come from successfully decoded frames. Sequence gaps are only known once a later sequence arrives.
- Conditional payload BER is mismatched bits among successfully decoded test payloads. CRC-rejected frames are packet loss, not guessed bit errors.
- If an event chunk exhausts retries, the app shows the received/expected sample count. This build has no post-transfer chunk replay or nonvolatile event storage.
