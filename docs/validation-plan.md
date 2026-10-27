# Drone Flight Black Box V2 validation

Record Arduino ESP32 package and RadioLib versions, selected board, COM ports, radio cap, antenna type/orientation, and each observed link-state transition.

## Bench startup

1. Confirm common ground, stable 3.3 V power, and antennas installed on both SX1278 boards. Confirm transmitter IMU wiring on GPIO 21/22 and radio wiring on GPIO 18/19/23/5/13/2.
2. Upload both V2 sketches. Confirm `READY,TX` and `READY,RX` at 115200 baud.
3. Verify the radios have identical frequency/BW/SF/CR/sync/CRC. Confirm a heartbeat and its ACK exchange.
4. Keep the drone unpowered and secured for event tests. Confirm calibration, gravity magnitude, and stationary gyro bias are plausible.

## Link adaptation progression

1. Start at a short, clear line-of-sight distance with both antennas upright. Wait for the receiver to accumulate four valid frames and report `LINK_STATE`.
2. Confirm the receiver logs `RADIO_TX,LINK,...`, then `LINK_STATE`; the transmitter logs `RADIO_RX,LINK,...`, `LINK_APPLIED,...`, and the `RSP` frame. Verify power and samples/frame match the selected state.
3. Increase distance gradually or introduce ordinary indoor obstructions. Hold each condition until a transition is reported. Confirm degradation advances one state after two poor four-frame windows, rather than jumping directly to critical.
4. At WEAK verify BER is blocked/stopped and event chunks use two samples per frame. At CRITICAL verify one sample per frame, configured maximum power, and disabled live settings/BER. Verify local event export/replay remains enabled.
5. Move closer and hold the clear condition. Confirm recovery moves one state at a time after four good windows. Repeat several cycles and record RSSI/SNR/loss/retries/power and state-change latency.
6. Leave the transmitter running but move beyond reception range in a controlled bench setup. Confirm OFFLINE appears after 12 seconds. Restore range and verify the link is measured again and adaptation resumes.
7. During the outage, verify transmitter power rises locally after repeated ACK failures (shown as actual power in app heartbeats). On return, verify stale sequence gaps do not hold the link in CRITICAL, the profile progresses gradually, and power is restored only after stable good windows.

Never remove antennas or wrap a transmitting radio in foil. Keep test powers within applicable radio limits. Distance and obstructions are not calibrated RF attenuation; they are a repeatable qualitative check, not a range certification.

## Flight-event capture and feature behavior

1. Use a gentle, repeatable bench motion on the secured, unpowered drone. Confirm an event appears immediately with the pre-trigger cache, then gains the post-trigger samples after two seconds.
2. Check the event's trigger indicators, sample count, trigger-relative offsets, six axes, gyro bias, and event JSON export.
3. Run a short BER test in GOOD/FAIR. Confirm valid compared bits are shown separately from missing packets. Move to WEAK and confirm BER stops or is rejected while event evidence remains prioritized.
4. Apply detector settings in GOOD/FAIR and confirm `CMD_ACK,...,SET_OK`. Confirm live settings are blocked in CRITICAL but event history and local export remain available.

## Recovery and records

- Disconnect/reconnect the browser's receiver serial port and confirm reconnect works without resetting either ESP32.
- Export the receiver serial log and event JSON. Mark events incomplete if a frame exhausts retries; do not infer missing samples.
- Do not describe a target sample rate, zero BER, attitude estimate, or severity score as hardware-validated until the assembled boards have been measured.
