# Contributing

This repository is a college engineering experiment. Contributions that improve repeatability, data quality, reliability, documentation, or accessibility are welcome.

## Before opening a change

- Keep transmitter and receiver radio settings matched.
- Describe the board, radio module, antenna, power supply, and software versions used.
- For behavior changes, include the bench setup and observed serial output.
- Do not include personal data, Wi-Fi credentials, or unreviewed flight logs.
- Keep V2's modem parameters fixed; coordinated SF/BW/CR adaptation belongs in the separate V3 project.

## Validation notes

Use the procedures in [`docs/validation-plan.md`](docs/validation-plan.md). State clearly whether a change was compiled, tested on a bench, or tested in flight. Bench results do not establish a safe flight-control link.
