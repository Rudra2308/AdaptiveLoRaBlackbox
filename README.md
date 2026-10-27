# Drone Flight Black Box V2 — Link Aware LoRa

> College project prototype: a receiver-led, link-aware LoRa experiment for
> recording drone motion events. This is not a certified flight recorder or a
> flight-control link.

## Repository contents

- [`transmitter/`](transmitter/) — ESP32 + MPU6050 sensing and LoRa transmitter firmware.
- [`receiver/`](receiver/) — ESP32 LoRa receiver and link monitor firmware.
- [`host/`](host/) — local browser dashboard and zero-dependency Node.js server.
- [`docs/protocol.md`](docs/protocol.md) — radio frames and USB serial records.
- [`docs/validation-plan.md`](docs/validation-plan.md) — bench validation checklist.
- [`MAIN_doc.md`](MAIN_doc.md) — project report source, presentation notes, and glossary.

## Quick start

1. Install the Arduino IDE, ESP32 board support, and libraries listed under [Hardware and setup](#hardware-and-setup).
2. Upload `transmitter/transmitter.ino` to the transmitter ESP32 and `receiver/receiver.ino` to the receiver ESP32.
3. From this repository folder, run `node host/server.js` and open `http://localhost:8765` in Chrome or Edge.
4. Follow [How to test link adaptation](#how-to-test-link-adaptation) and [`docs/validation-plan.md`](docs/validation-plan.md) for bench checks.

This repository contains the V2 baseline. The V3 experiment with negotiated SF/BW/CR profiles is maintained separately.

V2 branches from the earlier drone motion logger and adds a receiver-led link controller. V2 keeps its LoRa modem settings matched and adapts transmit power, event frame size, and optional features in stages as link evidence changes.

## What V2 adapts

The receiver evaluates rolling windows of four decoded frames using packet RSSI, packet SNR, sequence gaps/duplicates, and transmitter retry/failure counters from heartbeats. It requires sustained evidence before changing state: two poor windows to step down one state, and four good windows to step back up one state. This hysteresis avoids toggling when measurements hover near a threshold.

| Link state | Approximate evidence | Drone transmitter response | App behavior |
|---|---|---|---|
| Good | RSSI at least -88 dBm, SNR at least 8 dB, low loss/retries | 10 dBm target; up to 4 event samples per frame | All features enabled |
| Fair | RSSI at least -98 dBm, SNR at least 1 dB, moderate loss/retries | 12 dBm target; up to 4 samples per frame | Warning/quality shown; features remain available |
| Weak | RSSI at least -109 dBm, SNR at least -7 dB, or elevated loss/retries | 15 dBm target; 2 samples per event frame; BER is stopped/blocked | BER blocked to preserve event traffic |
| Critical | Worse than the weak thresholds | Capped maximum power target; 1 sample per event frame | BER and live settings are blocked; event history, local replay, and export remain available |
| Offline | No valid LoRa frame for 12 seconds | No control command can reach the transmitter; radio continues listening | Radio-dependent commands are blocked; local records remain usable |

The power targets are bounded by `RADIO_TX_DBM` in `receiver/config.h` and `ADAPTIVE_MAX_TX_DBM` in `transmitter/config.h`. Keep the caps matched and set them no higher than the module, antenna, and local rules permit. The normal targets are 10–17 dBm; the transmitter accepts a lower configured cap down to 2 dBm. Actual received signal varies with antenna, placement, enclosure, interference, and the board; the table thresholds are starting values to calibrate on your hardware, not range guarantees.

Every confirmed profile transition appears in the app's **Link changes** history on Overview and LoRa Analysis. It lists the old and new values for transmit power, samples per event frame, BER availability, and live setting availability. Offline and recovery transitions are reported too. Frequency, bandwidth, spreading factor, coding rate, sync word, and CRC remain fixed; those are explicitly identified as unchanged.

V2 changes **transmit power and application behavior**, but keeps frequency, bandwidth, spreading factor, coding rate, sync word, and CRC fixed and matched. RadioLib supports runtime SX127x modulation and output-power changes, but changing spreading factor or bandwidth on only one side would lose packets. Coordinated modulation changes need a confirmed rendezvous and recovery scheme; this first adaptive build avoids that desynchronization risk. Smaller event frames reduce the time and evidence exposed to one failed packet; they do not change the modem's sensitivity.

## Features and evidence preserved

If ACKs disappear, the transmitter raises its own power by 2 dB after every two failed reliable frames, up to its configured cap. This local fallback can restore a link when the receiver cannot send a power command. Heartbeats report actual TX power, and the app distinguishes it from the receiver profile target. On reconnect the receiver discards the outage's sequence/counter gaps, keeps the last applied level, and resumes gradual state progression. It adjusts RSSI for fallback power and reapplies the selected profile after four stable windows when needed.

- 50 Hz target six-axis MPU6050 samples and gyro calibration.
- Two-second pre-trigger cache sent when an inertial flight event is detected, followed by the trigger sample and two seconds of post-trigger samples under the same event ID.
- Raw acceleration/angular-rate samples, trigger-relative timestamps, trigger indicators, gyro bias, peak values, and event severity data for attitude replay.
- Heartbeats, reliable ACK/retry counters, receiver RSSI/SNR, BER testing, live detector settings, and browser event export.
- Sensor sampling remains on the Arduino loop while the radio task owns LoRa and handles retries.
- Event evidence continues to use reliable delivery at weak states; queued events are not deliberately discarded just because the link is weak. A sustained outage can still overflow the small volatile event queue, which the heartbeat reports.

This remains a standalone MPU6050 motion logger. It does not read GPS, altitude, battery, motors/ESC, RC input, or flight-controller state. Its attitude is estimated from six-axis IMU data; yaw drifts and there is no absolute position reconstruction. It is not a certified flight recorder.

## Hardware and setup

Use two ESP32 DevKit V1 / ESP32-WROOM-32 boards, two SX1278 RA-02 radios, and one GY-521 MPU6050 on the drone-side transmitter. Use a stable 3.3 V radio supply, common grounds, and suitable antennas on both radios. Never power an RA-02 from 5 V.

| Signal | ESP32 pin | Connects to |
|---|---:|---|
| MPU6050 SDA / SCL | GPIO 21 / 22 | Transmitter IMU |
| LoRa SCK / MISO / MOSI | GPIO 18 / 19 / 23 | Both radios |
| LoRa NSS / RESET / DIO0 | GPIO 5 / 13 / 2 | Both radios |

1. Install Arduino IDE 2.x and the **esp32 by Espressif Systems** board package. Boards Manager URL: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`.
2. Install **RadioLib**, **Adafruit MPU6050**, **Adafruit Unified Sensor**, and **Adafruit BusIO** from Library Manager.
3. Open and upload `transmitter/transmitter.ino`, then open and upload `receiver/receiver.ino`. Select **ESP32 Dev Module** for both.
4. Keep `transmitter/config.h` and `receiver/config.h` radio settings identical. Defaults are 433 MHz, BW 125 kHz, SF7, CR 4/5, sync `0x12`, preamble 8, CRC enabled.
5. Keep the IMU still during transmitter startup calibration. Confirm `READY,TX` and `READY,RX` on USB serial at 115200 baud.
6. In PowerShell from this V2 folder run:

```powershell
node host/server.js
```

Open `http://localhost:8765` in Chrome or Edge, click **Connect receiver**, and choose the receiver COM port. Close Arduino Serial Monitor while the browser owns that port.

## How to test link adaptation

Start with a **bench test**, not an airborne test. Keep both antennas attached and upright, use the same radio settings, and place the units a few metres apart in a clear line of sight. Start the app and watch **Overview → Adaptive LoRa link** or **LoRa Analysis**.

1. Wait for the receiver to collect link windows. The receiver serial log should show `LINK_STATE,<level>,<name>,...`; the transmitter should show `RADIO_RX,LINK,...`, `LINK_APPLIED,...`, and `RADIO_TX,RSP,...`. In a strong setup it should settle at **GOOD** and step the transmitter power down. With heartbeat-only traffic, expect roughly 12 seconds for the first four-frame assessment; event traffic adds more frames and can make this faster.
2. Slowly increase separation or introduce ordinary indoor obstructions while keeping both antennas installed. Hold each position until the state changes. The receiver should step through **FAIR**, **WEAK**, and **CRITICAL** only after repeated measurements. Do not wrap a transmitting radio in foil or operate it without an antenna.
   Check **Link changes** in the app after each transition; it should name each changed value and show fixed modem settings as unchanged.
3. At **WEAK**, verify BER is stopped/blocked and event frames use two samples each. At **CRITICAL**, verify one sample per event frame, maximum configured power, and disabled live settings/BER. Existing local flight-event replay and export should still work.
4. Move the receiver back closer. Recovery should climb one state at a time only after four good windows (about 48 seconds with heartbeat-only traffic; event traffic changes the timing). Make a gentle, safe bench motion and confirm the event capture is still recorded and transferred. If the transmitter raised power during the fade, the app should show actual power separately from the profile target and report any power restoration in **Link changes**.
5. Save the receiver serial log and exported JSON. Record the observed RSSI/SNR, loss/retry values, power steps, state transition times, and any incomplete events. Re-run at several safe separations to tune the thresholds for your antennas and mounting.

The controller needs decoded packets to measure quality. If the link is completely lost, it cannot send a power command; the receiver reports **OFFLINE** after 12 seconds and resumes adaptation after packets return. Do not treat a successful short bench test as proof of safe control range.

## Radio adaptation design references

- Semtech explains that changing LoRa bandwidth/spreading factor trades data rate, link budget, and time-on-air, and describes link-margin-based ADR from accumulated measurements: [Understanding LoRa Adaptive Data Rate](https://lora-developers.semtech.com/uploads/documents/files/Understanding_LoRa_Adaptive_Data_Rate_Downloadable.pdf).
- Semtech's FAQ describes ADR as coordinated adjustment of data rate, TX power, and retransmissions; its network-server model is different from this direct two-radio link: [Semtech LoRaWAN ADR FAQ](https://www.semtech.com/design-support/faq/faq-lorawan).
- RadioLib's SX127x example demonstrates runtime `setBandwidth`, `setSpreadingFactor`, `setCodingRate`, and `setOutputPower` calls and valid return-code handling: [SX127x Settings example](https://github.com/jgromes/RadioLib/blob/master/examples/SX127x/SX127x_Settings/SX127x_Settings.ino).

## Limits

- The sketches need compilation with the Arduino package/library versions installed on your machine and hardware validation. No ESP32 compile or radio test is claimed here.
- Thresholds are initial defaults. Tune them from receiver logs in your real antenna/mounting environment.
- Event queue and transmitter sequence state are volatile; browser storage retains up to 30 events. Export JSON for a backup.
- If a reliable event frame exhausts retries, the app marks the event incomplete; there is no later chunk replay request.
- BER compares valid decoded test payload bytes only. CRC-rejected packets are counted as missing packets, not guessed bit errors.

See [`docs/protocol.md`](docs/protocol.md) for frame fields and [`docs/validation-plan.md`](docs/validation-plan.md) for the test checklist.
