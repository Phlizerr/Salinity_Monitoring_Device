# Changelog

All notable changes to the Salinity Monitoring Device firmware are recorded here.
Versions follow [Semantic Versioning](https://semver.org/): MAJOR for changes that break
stored calibration, the uplink payload or the CSV format; MINOR for new features;
PATCH for fixes. Each version is tagged `vX.Y.Z` in git, and the running version is
shown on the device's boot screen, serial log, `help` output and web console.

## [1.0.0] - 2026-10-14

First public release.

- Salinity (conductivity probe, quadratic or power calibration curve) and DS18B20
  temperature measurement, with temperature compensation to 25 °C.
- Guided calibration over Bluetooth, USB serial and the Wi-Fi web console.
- FIELD and LOGGER operating modes, deep sleep between awake windows.
- SSD1306 OLED UI, RGB fault-annunciator LED, battery and charger monitoring.
- Reading history with CSV export from the web console.
- Optional LoRaWAN (TTN OTAA) uplink; keys kept in a gitignored `secrets.h`.

[1.0.0]: https://github.com/Phlizerr/Salinity_Monitoring_Device/releases/tag/v1.0.0
