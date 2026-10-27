# Drone Flight Black Box V2
## Master project document for the report, presentation, and team speeches

**Project type:** Electronics, sensors, embedded systems, and LoRa communication  
**Project status:** Working prototype under experimental validation  
**Build:** `drone_black_box_v2`  
**Intended use:** Source material for the college project report, presentation slides, demonstrations, and individual team-member speeches

> Treat this as the project's source of truth. The current system is a prototype. Do not present planned benefits or unmeasured performance as proven results. Add measured values, photos, plots, and test conditions after completing validation.

## 1. Project in one minute

Small drones can experience impacts, unusual rotation, or prolonged tilt. Ordinary flight telemetry is useful while the drone is operating, but an incident may interrupt that link or leave the cause unclear. This project explores a separate, low-cost sensor logger that records motion around a detected event and sends the evidence over a point-to-point LoRa radio link to a receiver application.

An ESP32 reads a six-axis MPU6050 inertial sensor at a target rate of 50 samples per second. A rolling memory buffer preserves the recent motion history. When a configurable threshold rule detects an event, the transmitter sends approximately two seconds of pre-event data and two seconds of post-event data, together with event indicators and summary measurements. The receiver acknowledges reliable packets, records the measurements, displays radio link quality, and provides an attitude replay view.

The V2 experiment adds a gradual link policy. The receiver evaluates received signal strength, signal-to-noise ratio, packet gaps, and retry information. It asks the transmitter to change output power, reduce samples per event packet, and pause optional BER testing as conditions worsen. If acknowledgements disappear entirely, the transmitter can raise its own power in bounded steps to help restore the connection.

The project is intended to study whether an adaptive, observable LoRa link can transfer more useful incident evidence than a fixed-profile link in a small experimental setup. That benefit still needs to be measured.

## 2. Problem and motivation

A drone's motion can change quickly during an impact or loss-of-control event. A short summary such as “impact detected” does not preserve the motion that led up to the event. A compact pre-event and post-event record can help a student or engineer inspect sensor readings around the trigger.

The radio path can also become unreliable because of separation, obstacles, antenna orientation, interference, or the drone's body. A fixed transmitter setting may waste power when the link is clear, while a weak link may lose event packets. The project therefore investigates both sides of the problem:

1. **Capture evidence locally:** Keep a short rolling motion history and preserve samples around a trigger.
2. **Deliver evidence over a changing link:** Measure delivery quality and adjust the transmission profile in controlled steps.
3. **Make behavior visible:** Show events, missing data, signal measurements, state changes, and power changes in a receiver application.

LoRa is a reasonable technology to investigate for a low-rate, long-range telemetry or incident-data path. It is not a high-throughput link: it is not intended to replace the drone's primary flight-control link, carry video, or stream the full sensor history continuously. LoRa's data rate, transmit power, and airtime are related trade-offs; LoRaWAN also has adaptive data-rate mechanisms, although this prototype uses a direct two-radio link rather than a LoRaWAN network. [LoRaWAN 1.1 specification](https://lora-alliance.org/wp-content/uploads/2020/11/lorawantm_specification_-v1.1.pdf)

## 3. Project aim, research question, and hypothesis

### Aim

Build and experimentally evaluate a low-cost drone motion logger that records sensor data around detected incidents and transfers it through a link-aware LoRa connection.

### Research question

Under controlled bench conditions, does a gradual link policy using RSSI, SNR, packet delivery evidence, and transmitter retry information improve event-data delivery and outage recovery compared with a fixed LoRa transmission profile?

### Working hypothesis

The adaptive profile may improve the chance of receiving complete event records during moderate link degradation by increasing transmitter power and reducing event packet size, while avoiding optional BER traffic at weak link states. A local transmitter fallback may help restore communication after acknowledgements stop. These are hypotheses to test, not results already established.

### Objectives

- Read acceleration and angular-rate measurements from an MPU6050 using an ESP32.
- Apply configurable, understandable event-trigger rules to the sensor readings.
- Retain pre-trigger samples and collect post-trigger samples for each event.
- Send event metadata and samples in acknowledged LoRa packets.
- Measure link indicators available from the radio and packet protocol.
- Apply gradual, bounded changes to transmit power, event chunk size, and optional BER behavior.
- Present the event record, link state, and confirmed setting changes in a receiver app.
- Validate the prototype with repeatable bench tests and document its limits.

## 4. System overview

```text
Drone-side prototype                         Ground-side prototype
┌───────────────────────┐                    ┌────────────────────────┐
│ ESP32 + MPU6050       │                    │ ESP32 receiver         │
│                       │     LoRa packets   │                        │
│ 50 Hz sample loop     ├───────────────────►│ ACK, RSSI/SNR, counters│
│ rolling event buffer  │◄───────────────────┤ link-state controller  │
│ event trigger rules   │                    └───────────┬────────────┘
│ acknowledged sender   │                                │ USB serial
│ bounded power fallback│                                ▼
└───────────────────────┘                    ┌────────────────────────┐
                                              │ Browser receiver app   │
                                              │ event log and plots    │
                                              │ attitude replay        │
                                              │ link analysis/export   │
                                              └────────────────────────┘
```

### Prototype hardware

- Two ESP32 DevKit V1 / ESP32-WROOM-32 boards.
- Two SX1278 RA-02 LoRa radio modules, one at each endpoint.
- One GY-521 board containing an MPU6050 accelerometer and gyroscope on the transmitter.
- A computer connected by USB to the receiver ESP32, running the local browser app.

The current radio defaults in the firmware are 433 MHz, 125 kHz bandwidth, spreading factor 7, coding rate 4/5, sync word `0x12`, preamble length 8, and CRC enabled. Both radio endpoints must use matching modem settings. The selected frequency and transmit power must be appropriate for the test location and radio hardware.

## 5. How an event is detected and recorded

### Sensor measurements

The MPU6050 supplies three acceleration axes and three angular-rate axes. The firmware targets 50 samples per second. At startup it estimates gyro bias and a gravity reference while the sensor is held still. The measurements are stored with device timestamps.

### Trigger rules

An event can be triggered by one or more configured conditions:

- acceleration magnitude above a threshold;
- bias-corrected angular-rate magnitude above a threshold;
- tilt from the startup gravity reference above a threshold for a configured duration;
- repeated triggers within a configured time window.

The application lets the operator adjust supported thresholds and trigger timing while the link permits. These are threshold-based rules, not a trained machine-learning classifier. The recorded labels (`IMPACT`, `ROTATION`, `TILT`, and `REPEATED`) describe which conditions fired; they do not establish the physical root cause of a crash.

### Pre-event and post-event buffer

At the 50 Hz target, the transmitter maintains about four seconds of samples in a rolling memory buffer. On a trigger it queues up to two seconds of samples from before the event. It then collects the trigger sample and approximately two seconds of later data. The pre- and post-event portions share an event identifier so the receiver can assemble them as one record.

The transmitted record contains raw acceleration and angular-rate samples, sample indices, times relative to the trigger, trigger indicators, event summary values, and gyro-bias information. This gives the application more evidence to display than a single event flag.

### Severity estimate

The firmware computes a configurable weighted score from normalized peak acceleration, normalized peak angular rate, and repeated-event count. It is a project-defined severity indicator on a 0–100 scale, not a probability of a crash, medical injury, structural damage, or a validated aviation risk score. The formula and weights should be reported alongside any score shown in a presentation.

## 6. What “3D reconstruction” means in this project

The app plots the three acceleration axes and three angular-rate axes and animates a simple 3D box to illustrate estimated roll, pitch, and yaw around an event. Roll and pitch use accelerometer gravity direction with gyroscope integration; yaw is integrated from the gyroscope.

This is an **attitude replay**, not a full 3D reconstruction of the drone's path through space. A six-axis IMU alone does not provide a stable absolute position or heading. Yaw drifts over time, and integrating acceleration to position also accumulates error. The current prototype does not include GPS, a magnetometer, barometer, optical flow, or flight-controller state. The visualization is useful for inspecting short motion sequences, but it must not be presented as a metrically accurate reconstruction of a flight trajectory.

For a future version, a stronger reconstruction could combine the IMU with GPS, barometric altitude, magnetometer, and flight-controller logs, then synchronize their clocks and validate estimates against a known reference. That is future work, not a current feature.

## 7. Link-aware LoRa behavior in V2

### Measurements used

The receiver collects RSSI and SNR from successfully decoded packets, sequence gaps and duplicates, and changes in reliable-frame retry/failure counters reported in heartbeats. It evaluates windows of four decoded frames. The thresholds in `receiver/config.h` are initial experiment settings and should be calibrated for the actual boards, antennas, enclosure, and test environment.

### Gradual profiles

| State | Initial classification thresholds | Transmitter profile | Features |
|---|---|---|---|
| Good | RSSI ≥ −88 dBm, SNR ≥ 8 dB, loss ≤ 1%, retries ≤ 3% | 10 dBm target; up to 4 samples per event frame | BER and live settings available |
| Fair | RSSI ≥ −98 dBm, SNR ≥ 1 dB, loss ≤ 6%, retries ≤ 15% | 12 dBm target; up to 4 samples per frame | BER and live settings available |
| Weak | RSSI ≥ −109 dBm, SNR ≥ −7 dB, loss ≤ 20%, retries ≤ 35% | 15 dBm target; 2 samples per frame | BER stopped or blocked to reserve airtime for events |
| Critical | Below the weak criteria | Up to the configured power cap; 1 sample per frame | BER and live detector settings blocked; event history and export remain available |
| Offline | No valid LoRa frame for 12 seconds | Receiver cannot send commands during the outage | App reports the outage; local app records remain usable |

Classification also uses retry and loss evidence. A good or fair state requires the corresponding conditions together; it is not based on RSSI alone. The receiver requires two poor windows before stepping down one state and four good windows before stepping back up one state. This delay is called hysteresis; it reduces rapid state changes around a threshold but makes transitions slower.

### Power recovery when feedback is lost

The receiver cannot command the transmitter while no packets can get through. To provide a recovery path, the transmitter raises its output power locally by 2 dB after every two consecutive failed reliable frames, up to its configured cap. Heartbeats report actual transmitter power as well as the receiver-selected profile. On reconnect, the receiver clears stale outage-related sequence and counter gaps, retains its last applied link state, evaluates signal strength relative to the reported power, and reapplies a selected profile after stable good windows if needed.

This fallback is experimental. It does not guarantee recovery from a complete obstruction, a damaged antenna, interference, or a receiver that is out of range. The transmitter power cap must match the module and comply with local radio rules.

### Why keep spreading factor and bandwidth fixed?

The prototype changes output power and application behavior, but keeps frequency, bandwidth, spreading factor, coding rate, sync word, and CRC fixed at both endpoints. A one-sided change to a modem parameter can make the two radios unable to decode each other. Coordinated modulation changes would need a tested negotiation and fallback method. Fixing those parameters keeps this first experiment simpler and reduces the chance that adaptation itself disconnects the radios.

### What smaller event chunks do and do not do

At weak states, the transmitter puts fewer sensor samples in each application packet. This can make an individual packet shorter and reduces how much event data is contained in one failed packet. It also creates more packets and ACK exchanges for the same event, so it does **not automatically improve total delivery or reduce total airtime**. The experiment should measure event completion and transfer time at each chunk size.

## 8. Receiver app and displayed evidence

The browser app connects to the receiver ESP32 by USB serial. It is designed to show:

- transmitter heartbeat and sample counters;
- the latest and windowed RSSI/SNR and packet link counters;
- current link state, actual transmitter power, and profile target;
- a history of confirmed link-profile changes, including the changed parameters;
- detected events, trigger indicators, sample completeness, plots, and attitude replay;
- BER comparison results, where allowed by the link state;
- event JSON and sample CSV exports.

The local app makes the radio's behavior observable to a student during testing. App display is not a link-quality guarantee. If packets are missing, the receiver cannot reconstruct measurements it never received.

## 9. What this project is—and is not

### It is

- a college-level electronics and embedded-systems prototype;
- an experiment in six-axis motion sensing and threshold-based event logging;
- a direct point-to-point LoRa data link with acknowledgements and a receiver-led adaptive profile;
- an investigation of whether link-aware transmission can improve incident-record delivery under selected test conditions;
- a short-window attitude visualization from IMU data;
- a platform whose current implementation needs systematic hardware validation.

### It is not

- a certified aviation black box or a safety-certified flight recorder;
- a replacement for a drone's primary flight-control or command-and-control radio;
- a video, high-rate telemetry, or continuous full-flight data link;
- a complete redundant dual-radio system integrated with another flight link—the current V2 firmware is one LoRa transmitter-to-receiver link;
- a full 3D flight-path reconstruction or a source of absolute position;
- an AI model that identifies the true cause of a crash;
- nonvolatile onboard event storage: the event queue is in RAM, so power loss can erase untransmitted records;
- proof that a particular range, packet-delivery rate, BER, recovery time, or safety level has been achieved.

Published work already includes LoRa-based UAV data loggers, UAV-assisted LoRa networking and parameter optimization, and redundant radio systems for small UAVs. Therefore, the project should not claim that “LoRa on a drone,” “adaptive transmit power,” or “redundant drone radios” is itself new. A reasonable student contribution is the integrated incident-capture workflow and a measured comparison of its link-aware event delivery against a fixed-profile baseline. [LoRa UAV data logger (IEEE, 2022)](https://ieeexplore.ieee.org/abstract/document/9942751), [FlyingLoRa (2023)](https://www.sciencedirect.com/science/article/pii/S138912862200545X), [redundant sUAS communications (IEEE, 2024)](https://ieeexplore.ieee.org/abstract/document/10556834).

## 10. Experimental validation plan

### Stage A: electronics and sensor checks

1. Verify 3.3 V radio power, common grounds, pin wiring, and antennas before powering the radios.
2. Keep the IMU stationary during startup calibration. Record the reported gyro bias and gravity reference.
3. Check that stationary acceleration is near one g in magnitude and gyro readings are near the calibrated bias. Record deviations rather than assuming the sensor is perfect.
4. Confirm the two radios have matching frequency, bandwidth, spreading factor, coding rate, sync word, preamble, and CRC settings.

### Stage B: event capture check

1. Keep the drone unpowered and physically secured; use a gentle repeatable hand motion or a controlled bench fixture.
2. Trigger a test event and confirm the record contains pre-event data and approximately two seconds of post-event data.
3. Compare received sample count with expected count. Save incomplete events as incomplete; do not fill gaps by interpolation without labeling it.
4. Repeat each trigger type separately where practical: impact threshold, angular-rate threshold, tilt duration, and repeated events.

### Stage C: controlled link test

1. Start close to the receiver with antennas installed and unobstructed. Record the baseline profile, RSSI, SNR, packet counters, and transfer time.
2. Change one condition at a time: separation, ordinary obstruction, antenna orientation, or transmit power cap. Do not remove antennas from transmitting radios.
3. Observe state changes and confirm the app names the settings that actually changed. Save receiver logs and exports.
4. Restore a stronger link. Record time to receive again, time to return through profile states, event completion rate, and actual transmit power.
5. Repeat multiple trials at each condition. Report trial count, hardware setup, and spread of results, not only the best run.

### Suggested comparison

Compare at least two modes under the same setup:

- **Baseline:** fixed transmit power and fixed event chunk size;
- **V2 adaptive:** receiver-led profile changes with local power fallback.

Suggested measurements:

| Metric | Measurement method |
|---|---|
| Event completeness | Received unique samples divided by expected samples per event |
| Event delivery success | Number of complete event records divided by number of triggered records |
| Event transfer time | Trigger time to receipt of the final expected sample |
| Reliable packet delivery | ACKed logical packets divided by logical packets sent |
| Retry and failure rates | Transmitter counters reported in heartbeats |
| Outage recovery time | Time from link restoration condition to first valid packet and to stable profile |
| Transmit power behavior | Profile target and actual power reported in heartbeats |
| BER test | Bit mismatches among successfully decoded known-payload test packets; also report missing packets separately |
| Energy/power draw | Measure current and voltage at the transmitter supply for each mode if suitable equipment is available |

The current BER mode compares payload bits only after successful decoding. CRC-rejected packets are missing packets; they are not counted as known bit errors. Avoid reporting a zero observed BER as proof that the physical channel has zero errors.

### Variables and controls

- **Independent variables:** link profile mode, distance, obstruction, antenna placement, and whether an event is being transferred.
- **Dependent variables:** event completeness, packet delivery, retries, transfer delay, recovery delay, RSSI/SNR, and energy use.
- **Controls:** same radio settings, same antennas, same event motion, same sample configuration, and repeatable placement for baseline and adaptive trials.

### Safety and scope of testing

Start on a bench. Do not fly until the electronics, logging, link behavior, and power integrity are validated and the institution's safety process permits it. Do not attach this prototype to an aircraft as a control system. Keep antennas connected and use radio settings permitted for the test region. A bench link test does not establish safe operating range.

## 11. Expected challenges and engineering trade-offs

- **RSSI/SNR fluctuate:** Use windows and hysteresis; tune thresholds from measured data.
- **No feedback during an outage:** Receiver commands cannot reach a disconnected transmitter, so a bounded local fallback is included.
- **Power and flight endurance:** Higher TX power can increase current draw; measure it instead of assuming the cost is negligible.
- **Chunk size trade-off:** Smaller packets may fail less often individually but increase packet and ACK overhead.
- **Loss of volatile evidence:** RAM buffering is not crash-proof storage. An SD card or other nonvolatile storage is future work.
- **Limited sensor observability:** A six-axis IMU cannot identify every failure cause or provide reliable absolute position.
- **Configuration mismatch:** Frequency, bandwidth, spreading factor, coding rate, sync word, and CRC must match at both radios.
- **Reproducibility:** Indoor distance and walls are not calibrated attenuation. Record the environment and repeat trials.
- **Regional radio rules:** Allowed frequencies, output power, duty cycle, and antenna conditions depend on location and hardware; verify before RF testing.

## 12. Suggested presentation structure

Use this as a slide outline; add actual photographs and plots from the team's build.

1. **Title and team** — project name, members, department, guide.
2. **Problem** — why motion evidence around a drone incident is useful.
3. **Aim and research question** — make clear this is an experiment, not a certified product.
4. **System architecture** — sensor, ESP32 transmitter, LoRa pair, ESP32 receiver, browser app.
5. **Sensor and event logic** — six-axis sensing, trigger rules, pre/post-event buffer.
6. **LoRa link design** — radio settings, ACKs, link metrics, gradual states, local fallback.
7. **Attitude replay** — show what the 3D box represents and its limitations.
8. **Validation method** — bench tests, baseline versus adaptive, variables and metrics.
9. **Results** — insert measured data only; include trial count and setup.
10. **Limitations and future work** — nonvolatile storage, richer flight-controller sensors, independent secondary radio integration, more robust reconstruction.
11. **Conclusion** — what was demonstrated, what remains unverified, and the next experiment.

### Possible division of speaking roles

Assign names after the team is known; these roles divide the technical story without inventing member details.

- **Speaker A — Problem and objective:** incident evidence, project motivation, research question.
- **Speaker B — Electronics and sensors:** ESP32, MPU6050, calibration, sampling, event trigger and buffer.
- **Speaker C — LoRa and adaptation:** packet acknowledgements, RSSI/SNR, retry evidence, profiles, local fallback.
- **Speaker D — App and reconstruction:** event plots, attitude replay, data interpretation, what “3D” does and does not mean.
- **Speaker E — Validation and conclusion:** experimental method, results, limitations, future work.

For a smaller team, combine adjacent roles. Each speaker should use measured results from the final validation, not planned values as if already achieved.

## 13. Short sample project speech

“Our project is an experimental drone motion logger that studies how a link-aware LoRa connection can transfer incident data. An ESP32 reads acceleration and angular rate from an MPU6050. When configurable motion thresholds are crossed, the system sends sensor samples from about two seconds before and two seconds after the event to a receiver. The receiver measures RSSI, SNR, packet gaps, and retry information. Based on these measurements, the prototype changes transmitter power, event packet size, and access to optional BER testing in gradual steps. The application displays the event record, radio state, and attitude replay. The 3D view shows estimated roll, pitch, and yaw; it does not reconstruct absolute position or identify the true cause of a crash. We are validating the system on a bench by comparing event delivery and recovery against a fixed-profile baseline. It is a college experiment, not a certified flight recorder or a replacement for flight-control radio.”

## 14. Technical glossary

| Term | Simple technical meaning |
|---|---|
| **ACK (acknowledgement)** | A reply confirming that a packet was received. If it does not arrive, the sender may retry. |
| **Adaptive link** | A communication link that changes selected settings or behavior in response to measured link quality. |
| **Attitude** | The drone's orientation, commonly described by roll, pitch, and yaw. |
| **Bandwidth (BW)** | The frequency width used by the radio signal. It affects data rate, noise, and link behavior. |
| **BER** | Bit error rate: the fraction of compared bits that differ in a known test payload. This build only compares successfully decoded payloads. |
| **Buffer / rolling buffer** | Memory that continually holds recent sensor readings, replacing the oldest readings as new ones arrive. |
| **Coding rate (CR)** | LoRa's forward-error-correction setting. The V2 prototype fixes it at 4/5. |
| **CRC** | A check value used to detect corrupted packets. A failed CRC means the packet is rejected; it does not reveal which bits were wrong. |
| **dB / dBm** | dB expresses a ratio; dBm expresses radio power relative to 1 milliwatt. RSSI is a received-power estimate, while TX power is transmitter output. |
| **Duty cycle** | The fraction of time a radio is permitted to transmit within a period under some regional rules or operating constraints. |
| **Event chunk** | One radio packet containing a portion of an event's sensor samples. |
| **Hysteresis** | Requiring sustained evidence before changing state, which avoids rapid switching when readings hover near a threshold. |
| **IMU** | Inertial measurement unit. The MPU6050 used here combines a three-axis accelerometer and a three-axis gyroscope. |
| **Link margin** | The amount by which a received signal exceeds the minimum level needed for reliable decoding. A single RSSI reading does not fully describe it. |
| **LoRa** | A low-power, long-range radio modulation used for small data messages. It generally trades throughput for range and robustness. |
| **LoRaWAN** | A networking protocol built around LoRa radios, usually with gateways and a network server. This prototype is direct point-to-point LoRa, not LoRaWAN. |
| **Packet loss** | Data packets that were sent but not successfully received. Sequence gaps and failed acknowledgements can reveal some losses. |
| **PDR** | Packet delivery ratio: the fraction of logical packets that were successfully delivered or acknowledged, according to the chosen definition. |
| **PER** | Packet error rate: the fraction of transmitted packets that fail under a specified measurement method. |
| **Preamble** | A known radio pattern at the start of a transmission that helps a receiver detect and synchronize with a packet. |
| **RSSI** | Received signal strength indicator, an estimate of received radio power, reported here in dBm. It is not by itself a probability of packet success. |
| **SNR** | Signal-to-noise ratio: signal strength compared with background noise, reported here in dB. LoRa can decode some signals with negative SNR, depending on configuration. |
| **Spreading factor (SF)** | LoRa modulation setting that influences airtime, data rate, and sensitivity. V2 keeps it fixed at SF7 on both ends. |
| **Sync word** | A configured pattern that helps radios distinguish packets belonging to the same link. It must match at both ends. |
| **Telemetry** | Measurements and status data sent from the drone-side device to the receiver. |
| **TX / RX** | Transmitter / receiver. TX sends radio packets; RX listens for and decodes them. |
| **Volatile memory** | Memory such as RAM that loses its contents when power is removed. Current event buffering is volatile. |
| **Yaw drift** | Accumulating heading error when yaw is estimated by integrating gyroscope readings without an absolute heading reference. |

## 15. References and project files

### Background reading

- LoRa Alliance, [LoRaWAN 1.1 specification](https://lora-alliance.org/wp-content/uploads/2020/11/lorawantm_specification_-v1.1.pdf).
- IEEE, [Low-Cost, Long-Range UAV Data Logger Using LoRa Module (2022)](https://ieeexplore.ieee.org/abstract/document/9942751).
- Xiong et al., [FlyingLoRa: Towards energy efficient data collection in UAV-assisted LoRa networks (2023)](https://www.sciencedirect.com/science/article/pii/S138912862200545X).
- IEEE, [sUAS Command-and-Redundant-Control Communications System with ROS 2 Data Bridge (2024)](https://ieeexplore.ieee.org/abstract/document/10556834).
- Semtech, [Understanding LoRa Adaptive Data Rate](https://lora-developers.semtech.com/uploads/documents/files/Understanding_LoRa_Adaptive_Data_Rate_Downloadable.pdf).

### Build and validation material

- [V2 implementation and bench setup README](README.md)
- [Radio and serial protocol](docs/protocol.md)
- [Validation checklist](docs/validation-plan.md)
- V1 preserved build: maintained as a separate project folder, not included in this V2 repository.

