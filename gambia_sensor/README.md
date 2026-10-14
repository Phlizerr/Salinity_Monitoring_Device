# Gambia Water Salinity Monitor

Firmware for an **ESP32**-based field device that measures **water temperature** and
**salinity**, shows them on a small OLED display, and (when enabled) transmits them over
**LoRaWAN** to **The Things Network (TTN)**. Built as part of a TFM (Master's thesis)
project for water monitoring in The Gambia.

Source: [`gambia_sensor.ino`](gambia_sensor.ino) · Command reference: [`COMMANDS.md`](COMMANDS.md)

---

## 1. What it does, in one paragraph

On power-up the device decides whether it still needs to be **calibrated**. If it has
never been calibrated, it opens a **Bluetooth** *and* a **USB serial** console so an
operator can calibrate the salinity model — either by **guided auto-calibration**
(dip the probe in solutions of known salinity and let the device fit the curve) or by
typing the coefficients in directly (`pcoef` for the quadratic curve, `cal` for the
power one) — and then start normal operation.

Once calibrated it runs as one of **two different instruments**, chosen with `mode` and
remembered in NVS:

- **Field mode** (the default) — an operator is holding the probe. Salinity is sampled
  four times a second so the number on the glass is live, the firmware watches it
  settle, and **one averaged observation is committed per dip**, with the spread that
  justifies it. The uplink fires on that commit, not on a timer. There is no button to
  press: immersion and settling are both inferred from the probe itself.
- **Logger mode** — nobody is there. A reading every 15 s inside a 5-minute awake
  window, an uplink per reading, then a 1-minute **deep sleep**.

Either way the salinity is **normalised to 25 °C** using the temperature measured at
the same instant, so readings from different sites, seasons and times of day are
comparable. A device can be sent back to calibration at any time with the `recal`
command (over USB, Bluetooth, or a LoRa downlink) — see [`COMMANDS.md`](COMMANDS.md).

---

## 2. Hardware

| Function | Component | ESP32 pin(s) | Notes |
|---|---|---|---|
All pins below are the **Salinity Device V2** board, verified against the fabricated
`Salinity Device V2.kicad_pcb` netlist.

| Function | Component | ESP32 pin(s) | Board net | Notes |
|---|---|---|---|---|
| Temperature | DS18B20 (1-Wire) | `GPIO16` | `TempProbe` | `R19` (4.7k) pulls the bus up on-board |
| Salinity — excitation | PWM square wave | `GPIO13` | `G0` | 25 kHz, 8-bit, 50 % duty → `U1` DG636E A0/A1, test point `TP2` |
| Salinity — sense | ADC input | `GPIO34` | `ADC` | 12-bit, 11 dB atten.; input-only pin |
| Display | SSD1306 OLED 128×64 (I²C) | `SDA=GPIO21`, `SCL=GPIO22` | `SDA` / `SCL` | `DiplayConn1`, addr `0x3C`, 400 kHz |
| Status LED | RGB LED `LED1`, **common anode** | `R=GPIO4`, `G=GPIO15`, `B=GPIO17` | `RGB2/RGB3/RGB1` | 150 Ω each (`R23/R22/R24`), anode on `VSYS_SW` — **active LOW**. The KiCad net names lie: the fitted part has pads 2 and 4 swapped versus the `LED_BRAG` symbol, so red is `RGB2`, not `RGB3` |
| Battery sense | `R20`/`R21` 1M divider + `C16` | `GPIO33` | — | pin sees **VBAT/2**; input-only |
| Charger status | MCP73833 `IC3` | `STAT1=GPIO35`, `STAT2=GPIO36`, `PG=GPIO39` | `STAT1/STAT2/PG` | open-drain, **active LOW**, 10k pull-ups `R17/R15/R16` |
| LoRa radio (SX127x, SPI) | NSS `5`, RST `27`, DIO `32/25/14` | `NSS/RESET/DIO0-2` | `SCK=18`, `MISO=19`, `MOSI=23` | disabled by default; **not populated on V2** |
| Power button | `Button1` → `R_base1` → Q1/Q2 latch | *none* | `ButtonA` | hardware power latch — never reaches a GPIO |
| Calibration link | Bluetooth Classic (SPP) | — | — | device name `ESP32_Gambia_Calibrator` |

### Migrated from V1
The firmware previously targeted the V1 board plus a hand-rewire. Five things moved:
excitation `GPIO2 → GPIO13`, OLED `18/19 → 21/22`, LoRa `RST 14 → 27` and
`DIO {26,33,32} → {32,25,14}`. The GPIO26 indicator LED and the GPIO13 bench push
button were **deleted** — GPIO26 is unconnected on V2 and the real button is a
hardware latch with no GPIO. The RGB LED replaces the indicator.

`DIO1` is worth calling out: it used to be `GPIO33`, which on V2 is the battery
divider. Enabling LoRa with the old map would have attached a radio interrupt to
`R20`/`R21`/`C16`.

### Status LED — a diagnostic annunciator
The LED is a **fault annunciator**, not a mode display. Healthy operation is
deliberately almost invisible (a ~2 % duty heartbeat) so that anything else stands
out, and every fault the firmware can detect has its own code rather than sharing a
colour with something unrelated.

**Why blink codes and not colours.** `R24` starves the blue die — its forward
voltage (~3.0 V) sits almost at the `VSYS_SW` rail, so 150 Ω passes a fraction of a
milliamp. Blue, cyan and magenta are therefore not reliably distinguishable on this
board. Only **red, amber, green and off** render honestly, so the design puts the
information in *motion* and *flash count* instead:

| Motion | Category |
|---|---|
| short blip | healthy and running |
| breathe | waiting for a human |
| counted flashes + long gap | a fault code — colour is the class, count is the fault |
| solid | a boot stage (or hung in one) |
| one long flash | a field observation was just committed |

#### Field mode — the LED *is* the user interface
V2 has no button, no buzzer, and in the field the operator is looking at water, not at
the OLED. So the whole dip is designed to be followed from the LED alone, as one
sequence rather than five separate colours:

| Pattern | Meaning | What to do |
|---|---|---|
| Green breathe, slow (3.6 s) | Probe is dry | Dip it |
| Amber blips, **quickening** | Settling — the blip *rate* is the convergence, tightening from ~1.2 s to ~0.26 s as the reading closes in | Hold still |
| Green blips, fast (0.3 s) | Inside the settle band, counting down 3 s to commit | Keep holding |
| **One long green flash** (0.8 s) | Committed — that dip is recorded and uplinked | Done |
| Green double every 4 s | Locked | Lift the probe to arm the next one |

Lifting the probe is what re-arms the device. That is deliberate: five samples taken
without moving the probe measure the firmware's averaging, not the site — only a real
re-dip makes a repeat reading mean anything.

#### Other normal states

| Pattern | Meaning |
|---|---|
| Green blip every 3 s | Logger mode, healthy. **Doubles** on the tick a clean reading lands, so the 15 s cadence is readable from across the room |
| Amber blip every 2 s | Charge mode: USB-C plugged in, charging |
| Green blip every 2 s | Charge mode: charge complete |
| White breathe, slow (4 s) | Calibration mode, no Bluetooth client |
| White breathe, fast (1.2 s) | Calibration mode, Bluetooth client attached |
| White double pulse | Guided auto-calibration collecting points |
| Solid red → amber → green | `setup()` stages 1 → 2 → 3. Normally a flicker; **whichever colour it stops on is the stage that hung** (1 = before the peripherals, 2 = I²C/OLED and 1-Wire, 3 = NVS and the mode decision) |
| Off | About to deep-sleep, or `rgb off` |

> Note that "white" on this board is really amber with a faint blue tinge, because
> `R24` starves the blue die. White and amber patterns are told apart by **motion**
> (breathe vs blip), never by hue.

#### Fault codes
`N` flashes, then a pause, then repeat. **Red = the device is not producing
trustworthy data. Amber = still running, but degraded.** With more than one fault up
the codes are flashed **in turn, one per cycle** — watch for ten seconds and you get
the whole list, not just the worst one.

| Code | Condition | Where to look |
|---|---|---|
| 🔴 ×1 | Power sense dead | ESP32 not seated in the V2 carrier, or the pack is shorted |
| 🔴 ×2 | Neither sensor answered | Payload carries `0x7FFF` for both fields |
| 🔴 ×3 | Charger FAULT | MCP73833 timer, thermal, or bad cell |
| 🔴 ×4 | Battery critical | Pack below 3.40 V |
| 🔴 ×5 | Measurement loop stalled | No reading completed in 3 periods — a wedged 1-Wire bus or a conversion chain that never returned |
| 🟠 ×1 | Temperature failing | DS18B20 off the bus, or CRC errors |
| 🟠 ×2 | Salinity probe reads ~0 V | Probe disconnected, or the excitation is dead. **Field mode only raises this while the probe is believed to be *in* water** — see below |
| 🟠 ×3 | OLED did not initialise | `DiplayConn1` I²C wiring |
| 🟠 ×4 | Battery low | Below 20 % |
| 🟠 ×5 | Input power good, charger in standby | No cell connected? |
| 🟠 ×6 | Salinity **not** temperature-corrected | Either no valid temperature to correct with, or a temperature outside what the current `tempco` alpha can correct — the number carries whatever the water was at and is not comparable with other sites. Chase 🟠 ×1 first, then the `[tempcomp]` line in the log |

**How 🔴 ×5 is evaluated, and what it cannot see.** Both modes test the age of
`lastMeasOkMs`, but the test has to run *before* the code that restamps it. Logger mode
gets this for free — the stamp lands in `finishMeasurement()`. Field mode stamps on every
live sample, so `fieldTick()` evaluates the fault as its **first** action, against the
stamp left by an earlier pass; evaluated after the sampling block it could never fire at
all, because the age tested would be at most one 250 ms sample interval instead of the
45 s threshold. That ordering is the detector, not an accident of layout.

The honest limit: this catches a loop that wedges and later recovers — a 1-Wire bus that
holds the line low for a minute, a conversion chain that comes back late. **A loop that
never returns cannot be caught from inside the loop**, and nothing here substitutes for
the hardware task watchdog.

**A limit worth stating.** A disconnected probe and a probe in air are the same open
circuit, so the firmware cannot tell them apart from the voltage alone. In logger mode
the probe is meant to be submerged, so ~0 V is a fault. In field mode a dry probe is
the normal resting state between sites, so amber ×2 would cry wolf on every walk — it
is therefore only raised once the probe is believed to be in water and the signal
*still* collapses. If a probe never seems to go wet, check the immersion threshold
(`immersion`) before suspecting the hardware.

Faults **latch for 30 s** after the condition clears, so a single bad reading cannot
be erased by the next cycle starting — the old scheme lost faults exactly that way.
Every raise and clear is logged, so `[fault] cleared` in the log is how you confirm a
repair worked.

**Never count blinks when you can ask.** `diag` prints the same fault set in words,
with the code next to each one, over USB or in the web console — the LED is for
noticing, the command is for confirming. The web console shows the same set as chips
under the tiles. All three read one `faults` bitmask in the firmware, so they cannot
disagree.

The LED auto-dims to ~35 % on battery and returns to full brightness when USB-C is
plugged in; `rgb <R> <G> <B>` and `rgb test` are exempt so a wiring check is always at
full output. Because the LED is common anode, a floating pin means "off" — so the
pads going high-Z in deep sleep leaves it dark with no extra work. `RGB_ENABLED 0`
leaves `GPIO4/15/17` completely untouched; `RGB_COMMON_ANODE 0` flips the polarity
for a common-cathode part.

### Battery & charger
`bat` reports pack voltage and charge state over USB/Bluetooth, and every periodic
reading appends them to the serial log. The `MCP73833` encodes its state across two
active-low pins:

| STAT1 | STAT2 | Meaning |
|---|---|---|
| L | H | Charge in progress |
| H | L | Charge complete |
| L | L | Fault (timer, thermal, bad cell) |
| H | H | Not charging — see below |

`PG` is separate and also active LOW: asserted while valid input power is present.
Voltage uses `analogReadMilliVolts()` so the ESP32's per-chip eFuse ADC calibration
applies — the raw-count path is fine for the ratiometric salinity reading, but a
battery gauge needs the absolute value to be right.

**Percentage.** `batteryPercentFromVoltage()` interpolates a piecewise LiPo discharge
curve (`BAT_CURVE`); the cell curve is nearly flat between 3.7 and 3.9 V, so a straight
line across 3.3–4.2 V would be wrong for most of the pack's life. `batteryTick()` caches
the result — charger pins at 5 Hz, pack voltage at 1 Hz with an EMA on top — because
`readBatteryVoltage()` averages 64 ADC reads and must never run at frame rate. Everything
else (display, loop, commands) reads the cache: `batVoltage`, `batPercent`, `chargeState`,
`chargerPresent`.

While charging, `GPIO33` sees the charger's output rather than the resting cell voltage,
so the percentage reads optimistically high; `charge complete` is authoritative and is
pinned to 100 %.

> **There is no charge indicator LED on this board.** Not a missing part — no footprint
> exists. `STAT1`/`STAT2`/`PG` only reach the ESP32, and their 10 kΩ pull-ups sit on the
> 3V3 rail, which only appears once `Q2` is on. A working charger is therefore completely
> silent while the device is switched off. The `power` command dumps the three pins raw
> with a verdict; it is the only charging diagnostic the hardware can support, and
> `COMMANDS.md` carries the full decode table and the fault tree.
>
> The other half of that trap: **the dev-kit's own micro-USB does not charge anything.**
> `5VIN` is unconnected on V2, so only the board's USB-C jack reaches the MCP73833.

**Power-path sanity check.** `checkPowerPath()` runs from `batteryTick()` at 5 Hz and
decides whether the charger pins mean anything at all. `GPIO35`/`36`/`39` are input-only
with no internal pull-ups, so an ESP32 running outside the V2 carrier floats all three LOW
— which decodes as "input power present, charger FAULT" and parks the firmware in charge
mode forever, never taking a reading. The guard is that the trio is electrically impossible
on a connected board: a powered MCP73833 holds `BAT` well above 1 V even into a flat cell,
so *power present + FAULT + a dead-flat divider* means the divider is not connected. When it
trips, `chargerPresent` is forced false (charge mode cannot engage), `batPercent` goes to
`-1`, `bat`/`power` say why, and `/api/status` reports `powerOk:false` so the console shows a
banner. A genuinely shorted pack sits in precondition (`STAT1` LOW, `STAT2` HIGH) rather than
FAULT, so a real battery fault still reports normally.

### Reading history
`finishMeasurement()` appends each reading to a static 120-entry ring (`hist[]`,
`histPush()`), about half an hour at the 15 s default for ~2.4 KB. Static rather than
heap — the WiFi stack is the only allocator on this chip allowed to be greedy. The ring
backs three things: the `hist` command, the console's trend chart via `/api/history`, and
a one-click spreadsheet export via `/api/csv`. It lives in RAM only, so a reset clears it;
with LoRa off it is the device's only data export.

`/api/history` returns parallel arrays (`t`, `temp`, `salt`, `volts`, `batV`, `f`) rather
than an array of objects — 120 readings would otherwise repeat five key names 120 times,
and the whole response is built in a `String` on a chip with ~130 KB of heap free once the
radio is up. `f` is a per-reading bitmask (bit 0 temperature valid, bit 1 salinity valid,
**bit 2 temperature compensation applied**) so the chart can drop failed points instead of
plotting a zero, and the CSV can mark uncorrected rows rather than mixing them in.

### Observations — the operator's data

`hist[]` is a raw sample *trace*: what the probe said, every cycle, useful for watching a
value settle and for debugging. It is not what a hydrologist wants to open.

Field mode keeps a second, separate ring — `recs[]`, 48 entries, `recPush()` — with **one
row per committed dip**: the mean over the stable window, the standard deviation that
justifies it, the sample count, the temperature, and an operator-set site label. This is
what gets uplinked and what `/api/records.csv` exports. Keeping the two apart is the
point: averaging a trace after the fact cannot tell which samples were taken while the
probe was settling, and this ring only ever receives windows the firmware already judged
stable.

`recIndex()` derives the oldest entry from *head minus count* rather than from a
has-it-wrapped test, because `recs undo` walks the head backwards — the wrapped form
reports the wrong oldest entry after an undo on a full ring.

Each row carries its **own** sequence number (`FieldRecord::seq`), taken from `recSeq` at
commit. `recSeq` is monotonic and `recs undo` does *not* rewind it: a number that has
already appeared in the serial transcript, on the OLED corner or in a LoRa uplink must
never name a second, different measurement. Storing the number per row rather than pairing
the global counter with whichever row happens to be newest is what keeps the two
consistent after an undo.

The committed row's flags are the **AND** of the per-sample flags across the window
(`stabFlags()`), not a snapshot of the live state at the instant of the commit. "This
observation is temperature-compensated" is a claim about all twenty samples averaged into
it; reading the live flag instead would let a DS18B20 that dropped out a quarter-second
before the commit condemn a whole window of good samples, and a sensor that recovered on
the last sample mark nineteen raw ones corrected. Samples are only admitted to the window
while `liveValid` — on a dropout the previous salinity is still in `liveSalt` while the
probe voltage has collapsed to ~0 V, so pushing it would insert a duplicate salinity
(shrinking the standard deviation toward a premature commit) beside a probe voltage that
never produced it.

`/api/records.csv` carries a `temp_valid` column and leaves `temp_c` **blank** when the
DS18B20 never answered. `temp_compensated` cannot stand in for it — that is also 0
whenever compensation is simply switched off — and a flat `0.00` in a temperature column
is indistinguishable from real 0 °C water to anyone opening the sheet later.

Site labels are sanitised on entry (`,`, `"`, newlines → `_`): the label goes into a CSV
column verbatim, and replacing rather than rejecting keeps the operator moving.

### WiFi web console
The ESP32-WROOM-32E has 2.4 GHz WiFi on the module, so the console costs no hardware. It
is a **local** console, not telemetry — LoRa is the long-range path, and V2 has no
footprint for the radio (every LoRa net is a one-pad net touching only the ESP32 socket).

`wifiTick()` owns the radio lifecycle and is called from the top of `loop()`, before
anything that can return early, so the console keeps answering while charge mode or
calibration mode owns the loop. Policy defaults to `auto`: up only while USB-C power is
present, which keeps WiFi's ~100 mA entirely out of the field power budget. `wifi on`
forces it up and suppresses deep sleep (sleeping would drop the session).

AP mode is the default — SSID `Gambia-XXXX` from the eFuse MAC, password `salinity`,
`http://192.168.4.1`. `wifi sta <ssid> <pass>` stores credentials in NVS (namespace
`wifi`) and joins an existing network instead, falling back to AP after a 12 s timeout.

Policy is **persisted to NVS** (`wifiSavePolicy()`, key `pol` in namespace `wifi`). It used
to be RAM-only like `sleep` and `exc`, which meant every reset — and every reflash — quietly
put it back to `auto`; and under `auto` on battery there is no AP broadcasting at all, which
from the outside is indistinguishable from a console that does not work.

**The page.** A single self-contained document in `PAGE_HTML` (PROGMEM, ~24 KB) — the
device has no internet, so nothing may be pulled from a CDN. It leads with the salinity
reading, then a trend chart that switches between salinity, temperature and battery; the
tiles below cover battery (with a bar that turns amber under 35 % and red under 15 %),
charger, countdown and how many readings are stored. There is a dark/light toggle stored in
`localStorage` — light mode exists because a phone screen in direct sun is unreadable
otherwise, which is the actual deployment condition. The log pane has a filter box, and
pause/clear/copy; the command box keeps a history on ↑/↓ and the chips below it cover the
common commands.

**The unit label is load-bearing.** The headline reads `g/L @25°C` only when the
correction is actually being applied; it says `g/L uncorrected` when there is no
temperature, and `g/L raw` when compensation is switched off — with a banner explaining
what that costs. A number whose comparability is in doubt should never look like one
that is not.

**The field panel** appears only in field mode, because a settle bar and an observation
table mean nothing when nobody is holding the probe. It mirrors the glass: state, a
wet/dry pill, the settling bar (which switches to showing the commit countdown once the
reading is inside the band, so the thing filling the bar is always the thing you are
waiting for), and the live sd/band/n. Below it, the observation table and buttons for
*Record now*, auto-record, site naming, undo and clear. Every button posts to
`/api/cmd` — the same path a typed command takes — so a press and a command are the same
event and both land in the one transcript.

It polls `/api/status` and `/api/log` once a second and posts to `/api/cmd`, which runs the
**same** `processCommand()` as the serial terminal. `/api/history` is refetched only when
`seq` changes, and `/api/records` only when `recSeq/recN` changes — keyed on both, because
`recs clear` empties the ring without touching `recSeq` and watching the sequence alone
would leave a stale table. See `COMMANDS.md` for the endpoint table.

**The log tee.** `Log` is a `Stream` that writes to the USB serial port *and* a 4 KB ring
buffer. Every `print` in the firmware goes through it (`Serial.` was mechanically replaced
by `Log.`; only `Serial.begin()` still touches the UART directly), and `processCommand()`
is handed `Log` rather than `Serial`, so replies are captured too. `/api/log?from=N`
returns everything since byte offset `N` with the new offset in `X-Log-Next` — plain text,
so the log never needs JSON escaping.

> **Build hazard.** The WiFi/log block sits *below* the `RgbStatus` and `ChargeState`
> enums on purpose. It defines `logPush()` and the stream classes, which makes it the
> sketch's first function definition — and that is exactly where the Arduino preprocessor
> injects its generated prototypes. Move it above the enums and every prototype naming
> those types stops compiling.

### Charge mode (USB-C plugged in)
The board's power path is load-sharing: `Q1` (P-FET, gate on `VBUS_5V`) disconnects the
battery from `VSYS` as soon as USB-C power arrives, so the system runs off the cable
through `D1` while the `MCP73833` charges the cell at the full ~196 mA set by `R11`
(5.1 kΩ on `PROG`). Charging therefore works whether the device is switched on or not —
and when it is off, nothing on the board indicates it, because there is no dedicated
charge LED.

When the firmware is running, plugging in USB-C puts it into **charge mode**
(`handleChargeMode()`): any conversion chain in flight is abandoned, no new measurement
starts, no LoRa uplink goes out, deep sleep is suspended, and the OLED switches to the
charging screen. Detection is `PG` (`GPIO39`), debounced over three 200 ms samples.
Unplugging resumes measuring immediately. `charge ignore` disables the pause for bench
work; see `COMMANDS.md`.

The excitation square wave is deliberately **not** forced off — charge mode calls
`excitationOff()`, which honours the `exc on` latch. The wave is what keeps the probe
drive AC; parking `GPIO13` static would hold a DC bias across the electrodes for the
whole charge and plate them. Send `exc off` if you want the drive genuinely dead.

### The old OLED/LoRa pin conflict is gone
On V1 the hand-wired OLED sat on `GPIO18/19`, which the LoRa SPI bus also needs for
SCK and MISO — you could have the display or the radio, not both. V2 gives the display
its own I²C pads (`GPIO21/22`), so that constraint no longer exists. LoRa still ships
**off** simply because no radio module is populated on the V2 board; `DIO0/1/2`,
`RESET`, `NSS`, `SCK`, `MISO` and `MOSI` terminate at the ESP32 with nothing on the
far end.

---

## 3. Build / flash configuration

These settings are required (documented from the working uploads in this project):

- **Board / FQBN:** `esp32:esp32:esp32` (ESP32 Dev Module)
- **Partition scheme:** `huge_app` (3 MB app). The default 1.3 MB partition overflows —
  WiFi + BluetoothSerial + LoRa push the binary to ~1.66 MB.
- **Upload speed:** `115200` (the 921600 default drops the connection mid-write on this board).
- **Manual bootloader:** this board's auto-reset doesn't work — hold **BOOT**, tap
  **EN/RST**, keep holding **BOOT** until writing starts.

Required libraries: `MCCI LoRaWAN LMIC`, `Adafruit GFX`, `Adafruit SSD1306`,
`Adafruit BusIO`, `OneWire` (plus the ESP32 core's `WiFi`, `BluetoothSerial`,
`Preferences`, `SPI`, `Wire`).

---

## 4. Configuration flags (top of the sketch)

| Symbol | Default | Meaning |
|---|---|---|
| `LORA_ENABLED` | `0` | `0` = display/sensor test only (no LoRa/SPI). `1` = full firmware with LoRa + deep sleep. |
| `WIFI_ENABLED` | `1` | `0` leaves the radio and web console out entirely. |
| `WIFI_AP_PASS` | `"salinity"` | AP password (WPA2 needs ≥ 8 characters). |
| `WIFI_STA_TIMEOUT_MS` | 12 s | How long a stored network gets before falling back to AP. |
| `WIFI_MIN_HEAP` | 60 KB | Below this the console refuses to start (most likely cause: Bluetooth Classic is also up). |
| `LOG_BUF_SIZE` | 4096 | Ring buffer the web console reads the log from. |
| `SLEEP_INTERVAL_US` | 60 s | Deep-sleep duration between awake windows. |
| `AWAKE_WINDOW_MS` | 5 min | How long the device stays awake taking readings before sleeping. |
| `SEND_PERIOD_MS` | 60 s | Interval between measurements/transmissions. |
| `P1`, `P2`, `P3` | `-0.0003`, `0.0437`, `0.0990` | Coefficients of the **default** salinity curve `V = P1·S² + P2·S + P3`, inverted at runtime (overwritten by stored values once calibrated). |
| `a`, `b`, `c` | `12.2315`, `1.8950`, `0.0` | Coefficients of the legacy power curve `S = a·V^b + c`, selected with `model power` (overwritten by stored values once calibrated). |
| `curveModel` | `CURVE_QUAD` | Which of the two curves is live. Persisted; an already-calibrated device upgrading from an older build keeps `CURVE_POWER`. |

TTN OTAA credentials (`DEVEUI`, `APPEUI`, `APPKEY`) live in `secrets.h`, which is gitignored. Copy `secrets.example.h` to `secrets.h` and fill in the values from the TTN console; without it the sketch still builds, using all-zero placeholder keys (with a compiler warning).

---

## 5. Operating modes

The firmware has three modes. Whether it needs calibrating is one NVS flag; which of
the two *measuring* modes it runs is another.

- **`MODE_CALIBRATION`** — Bluetooth **and** USB serial console. No measuring, no sleep.
  Used the first time the device is powered on, or any time after a `recal` command.
- **`MODE_FIELD`** *(default)* — operator-present. Continuous 4 Hz salinity, free-running
  temperature EMA, settle detection, one committed observation per dip, uplink on the
  commit. No deep sleep: the power button is the off switch.
- **`MODE_LOGGER`** — unattended. A reading every `SEND_PERIOD_MS` inside
  `AWAKE_WINDOW_MS`, an uplink per reading, then `SLEEP_INTERVAL_US` of deep sleep.

Both measuring modes keep the USB serial console live for the global commands.

**Why the split.** These are two genuinely different instruments that used to be one
blurred mode, with defaults (`sleepEnabled = false`, 15 s pacing) that landed on
neither. The 15 s period was never a power-saving mechanism — `SEND_PERIOD_MS` is named
*send*, not *measure*, and it existed to pace uplinks while dragging the measurement
along with it. In the always-on default the CPU never slept and the excitation never
stopped, so the period gated only the two cheapest things in the device: the DS18B20
chain and a ~200 ms ADC burst. Measurement rate and uplink rate are now separate.

**Why the mode is persisted.** `Button1` drives the AO3401A power latch directly and
never reaches a GPIO, so the device cannot be asked which instrument it should be on
the way up. It has to remember. `mode field` / `mode logger` writes NVS key `mmode`.

### Boot flow

```
setup()
 ├─ Serial 115200, WiFi OFF, CPU 80 MHz, ADC 12-bit / 11 dB
 ├─ Init OLED, show splash
 ├─ Read NVS "calib" → done?
 ├─ Enumerate DS18B20 ROM once (tempInit)
 ├─ Load stored a/b/c from NVS (defaults if unset)
 ├─ Read NVS "calib" → done?
 │     ├─ done == false → MODE_CALIBRATION
 │     │        └─ start Bluetooth "ESP32_Gambia_Calibrator" + USB console
 │     └─ done == true  → mmode == field ? fieldBegin() : loggerBegin()
 │                        (Bluetooth off; USB still listens for recal)
 └─ enter loop()
```

The "already calibrated?" decision is stored in **NVS** (namespace `calib`, key `done`),
so it survives power cycles and deep sleep. The `recal` command clears `done` and
reboots, sending the device back to calibration mode without wiping anything else.

Everything else in namespace `calib` is a property of *how this device is deployed*,
and since the power latch means it cannot be told any of it at boot, all of it
persists:

| Key | Set by | Default |
|---|---|---|
| `done` | `start` / `recal` | false |
| `model` | `model quad` / `model power` | quad (an already-calibrated device upgrading from an older build keeps power) |
| `p1`, `p2`, `p3` | `pcoef`, `fit`, `start` | -0.0003 / 0.0437 / 0.0990 |
| `a`, `b`, `c` | `cal`, `fit`, `start` | 12.2315 / 1.8950 / 0 |
| `mmode` | `mode field` / `mode logger` | field |
| `tcomp` | `tempcomp on` / `off` | on |
| `tempco` | `tempco <alpha>` | 0.020 /°C |
| `stab` | `stability <g/L>` | 0.05 g/L |
| `immv` | `immersion <V>` | 0.110 V |
| `autorec` | `autorec on` / `off` | on |

---

## 6. Calibration & commands

Calibration is driven by **line-based, case-insensitive commands** over **USB serial**
(115200 baud), **Bluetooth** (`ESP32_Gambia_Calibrator`), or — for `recal` only — a **LoRa
downlink**. The full reference lives in **[`COMMANDS.md`](COMMANDS.md)**; the essentials:

- **Which curve:** `model` selects `quad` (the default, `V = P1·S² + P2·S + P3` solved
  for `S`) or `power` (the legacy `S = a·V^b + c`). Persisted. Everything below fits or
  edits **whichever one is live**.
- **Auto-calibration (recommended):** `autocal`, then dip the probe in each known
  solution and send `sample <S>` (e.g. `sample 15` for 15 g/L). Send `fit` to compute the
  live curve's coefficients and see the R², then `start` to save and run. The quadratic
  needs **≥3 points at three different salinities** (a parabola through two points is a
  whole family); the power curve needs 2. `points` / `undo` manage the recorded points.
- **Manual entry:** `pcoef <P1> <P2> <P3>` types the quadratic in on one line, in any
  mode. `cal` prints the current `a`/`b`/`c` and walks you through typing new values.
- **`start`** saves both curves' coefficients, the `model` choice and `done=true` to NVS,
  stops Bluetooth, and switches to whichever measuring mode `mmode` names.
- **`recal`** (USB, Bluetooth, or LoRa downlink `0xC1`) reboots the device back into
  calibration mode at any time.

The calibration OLED shows the live curve's coefficients and commands; during `autocal` a
dedicated **AUTO-CAL screen** shows the point count, the leading coefficients, and the
last fit R².

> **Each `sample` also records the bath temperature**, and the standard is converted to
> what it reads *at that temperature* before being fitted — see §7. If the DS18B20 does
> not answer, the point is fitted as if the bath were at 25 °C and `sample` says so
> loudly, because a silent assumption there puts a permanent error in `a` and `b`.
> Changing `tempco` after calibrating invalidates that conversion; re-run `autocal` if
> you move it far.

---

## 7. Measurement & data pipeline

### Temperature compensation — read this before trusting a number

The conductivity of natural water moves about **2 % per °C**. Uncorrected, the same
water measured at 22 °C in the morning and 35 °C in the afternoon reads **~28 % apart**
— larger than most of the differences this instrument exists to detect, and enough to
make readings from different sites, seasons or times of day meaningless to compare.

Everything is therefore normalised to 25 °C using the temperature measured in the same
cycle:

```
S25 = S_measured / (1 + alpha · (T − 25))          alpha = 0.020 /°C by default
```

`alpha` is adjustable (`tempco`) because the true coefficient depends on what is
dissolved: ~0.019 for a NaCl-dominated estuary, ~0.023 for carbonate-rich water. A
device deployed on one river can be trimmed to it.

**The same correction runs in reverse during calibration.** Standards are quoted at
25 °C, so a point taken in a 30 °C bath is converted to what that standard *actually
reads at 30 °C* before it is fitted. Otherwise the bath temperature is silently baked
into the fitted coefficients, and the runtime correction is then applied on top of a
curve that already contains it — double-counting. `points` shows both the nominal and the fitted
value so this is visible rather than implied.

**When there is no temperature**, the correction cannot be applied. The firmware does
*not* quietly fall back to raw: it reports the raw number, marks the reading
uncompensated in the history/CSV flags and in the payload path, and raises **amber ×6**.
The OLED shows `raw` instead of `@25`, and the web console says so in words.

Every one of those indicators reads the **applied** flag — `liveComp` in field mode, the
last reading's `flags` bit 2 in logger mode — and not `tempCompEnabled && tempAddrValid`.
Those differ: `tempAddrValid` only records that the DS18B20 answered the enumeration at
boot, so a probe pulled since, or one returning CRC-valid garbage, left the LED saying
amber ×6 while the web console printed `g/L @25°C` and suppressed the warning banner.

The second reason they differ is the **divisor guard**. The correction divides by
`1 + alpha·(T − 25)` and is refused once that falls below `TEMPCOMP_MIN_DIVISOR` (0.25),
where the linear model is degenerate. Where that lands in degrees is *not* fixed — it is
`25 − 0.75/alpha`, so −12.5 °C at the default alpha but **+10 °C at `TEMPCO_MAX`**. A
device trimmed toward the carbonate end and deployed on a cold river can therefore be
pushed into the guard by perfectly real water. Widening the floor would not help (at
alpha 0.05 and 0 °C the divisor is already negative and the "correction" would flip the
sign), so the guard stays and is instead made audible: `tempcomp` and `tempco` print the
cut-off for the current alpha, and `salinityAt25()` logs the first reading it suppresses.

### Reading cycle

**Field mode** samples continuously: a ~4 Hz salinity read (`fieldSalSamples`, a quarter
of the logger count, so ~51 ms) against a free-running DS18B20 chain feeding a
temperature EMA. A rolling 20-sample (5 s) window carries the mean and standard
deviation; when the sd sits inside `max(stabAbs, 1 % of the mean)` for a full window and
stays there 3 s, the window is committed as one observation. Falling out of the band at
any point drops back to settling, so a disturbed reading can never be committed on the
strength of an earlier calm.

**Logger mode** runs the original **non-blocking two-phase state machine**, so the
DS18B20's 750 ms conversion never stalls the LoRa/LMIC loop:

**Phase 1 — `startMeasurement()`**
1. **Kick off the temperature conversion** on the DS18B20 (`0x44`) and return
   immediately — no `delay(750)`.
2. **Turn on salinity excitation** — a 25 kHz, 50 % PWM square wave on `GPIO2` — so the
   probe settles during the conversion window.

**Phase 2 — `finishMeasurement()`** (once ≥750 ms have elapsed)
3. **Read the ADC** on `GPIO34`, averaged over `SAL_SAMPLES` samples; convert to voltage:
   `voltage = (adc / 4095) × 3.3 V`, then stop the excitation.
4. **Convert to salinity** with the calibrated curve. By default that is the quadratic,
   stored the way the probe was characterised — voltage as a function of salinity — and
   inverted here with the quadratic formula, taking the `+` branch:

   ```
   V = P1·S² + P2·S + P3
   S = ( -P2 + √(P2² - 4·P1·(P3 - V)) ) / (2·P1)     // result in g/L
   ```

   Below `P3` (the zero-salinity intercept) and above the parabola's vertex the result is
   **clamped** to 0 g/L and to the vertex salinity respectively — see `model` in
   `COMMANDS.md`. `model power` selects the legacy `salinity = a · voltage^b + c` instead.
5. **Read the temperature** scratchpad (`0xBE`, CRC-checked, raw / 16 → °C). On any read
   error the previous value is kept, a serial `ERROR:` line is logged, and that field is
   transmitted as the `0x7FFF` sentinel.
6. **Update the OLED** and **encode & transmit** (see below).

### Payload encoding (7 bytes)

Temperature and salinity are each multiplied by 10, rounded, and packed as **big-endian
signed `int16`** (two's complement, so sub-zero temperatures survive the trip). A field
whose reading failed is sent as the sentinel **`0x7FFF` (32767)** so the backend can flag
it as missing instead of treating garbage as a value.

**The first four bytes are unchanged**, so an existing TTN decoder keeps working
untouched. Three more are appended: the standard deviation of the committed window and
the sample count, which is what turns a number into a number with a confidence attached.

| Byte | Contents |
|---|---|
| `0` | `(temperature×10) >> 8`  *(int16 high byte, or `0x7F` if invalid)* |
| `1` | `(temperature×10) & 0xFF` *(int16 low byte, or `0xFF` if invalid)* |
| `2` | `(salinity×10) >> 8`  *(int16 high byte, or `0x7F` if invalid)* — **at 25 °C** |
| `3` | `(salinity×10) & 0xFF` *(int16 low byte, or `0xFF` if invalid)* |
| `4` | `(sd×100) >> 8`  *(uint16 high byte; 0 in logger mode)* |
| `5` | `(sd×100) & 0xFF` *(uint16 low byte)* |
| `6` | salinity samples averaged into the observation *(0 in logger mode)* |

All seven bytes are always transmitted, so packet length alone does not tell the two modes
apart — **`n = 0` is the marker**. Logger mode reads the ADC once and has neither a spread
nor a sample count, so it sends `0` for both rather than a number that would decode as a
perfectly repeatable single reading.

**TTN decoder (JavaScript):**

```js
function decodeUplink(input) {
  const b = input.bytes;
  const ERR = 0x7FFF;                 // sentinel: reading failed
  const s16 = (hi, lo) => {           // big-endian signed 16-bit
    const v = (hi << 8) | lo;
    return v & 0x8000 ? v - 0x10000 : v;
  };
  const tRaw = s16(b[0], b[1]);
  const sRaw = s16(b[2], b[3]);
  const out = {
    temperature: tRaw === ERR ? null : tRaw / 10, // °C  (null = read error)
    salinity:    sRaw === ERR ? null : sRaw / 10  // g/L at 25 °C (null = read error)
  };
  // n === 0 means logger mode: one unaveraged ADC read, so there is no spread
  // and no sample count. Test n, not b.length - all seven bytes are always sent.
  if (b.length >= 7 && b[6] > 0) {    // field-mode observation
    out.sd = ((b[4] << 8) | b[5]) / 100;   // g/L, spread of the averaged window
    out.n  = b[6];                         // samples behind the mean
  }
  return { data: out };
}
```

The packet is sent on **port 1** via `LMIC_setTxData2` (unconfirmed). Join is OTAA using
the keys defined in the sketch. When `LORA_ENABLED` is `0`, the same payload is only
printed to the serial log (`[LoRa disabled] Payload HEX: …`) and nothing is transmitted.

**Uplink cadence, and why it matters.** Logger mode sends one packet per reading — every
15 s. Field mode sends one per *committed observation*, so roughly one per site: about
8–10 a day instead of 5760. That is not only politeness. EU868 allows a **1 % duty
cycle**, and a 7-byte payload at SF12 is ~1.3 s of airtime — about 130 s of minimum
spacing. A 15 s uplink at a high spreading factor is not legal and LMIC will defer it
anyway. Committing per dip leaves the budget free enough to run **SF12 for maximum
range**, which matters far more in rural Gambia than uplink frequency does — and what
reaches TTN is then the readings the operator vouched for, not 300 readings of a probe
swinging in the air between sites.

---

## 8. Timing & power management

### Logger mode

```
                 ┌──────────── awake window (5 min) ───────────┐
 wake ─▶ read ─▶ every 15 s: measure + show + (LoRa TX) ─▶ ... ─┘ ─▶ deep sleep 60 s ─▶ wake …
```

- The first reading is taken **immediately** on entering the mode.
- A new measurement is taken every `SEND_PERIOD_MS` (15 s) during the awake window.
- After `AWAKE_WINDOW_MS` (5 min), with `sleep on` and `LORA_ENABLED 1`, the device
  **deep-sleeps** for `SLEEP_INTERVAL_US` (60 s) with the display off, then reboots and
  repeats. With `sleep off`, or with no LoRa hardware, it simply restarts the awake
  window so the bench stays live.

### Field mode

Field mode does not sleep at all, and that is the right answer rather than an oversight.

The device is **always on** in this mode — CPU, OLED, excitation, ~4 Hz sampling — which
costs roughly 70 mA. But the duty cycle is set by the operator, not the firmware: about
3 minutes per site, maybe 8 sites a day, so ~25 min/day ≈ **30 mAh/day**. On a 2000 mAh
cell that is around two months of fieldwork per charge. And `Button1` drives a hardware
power latch, so "off" between sites is genuinely **zero** drain, not a sleep current.

Going continuous costs almost nothing over the old 15 s pacing, because in the shipped
configuration the expensive things already ran continuously: `sleepEnabled = false` kept
the CPU awake, `excAlwaysOn = true` kept the excitation running, and the OLED was lit and
redrawn at 5 Hz. The 15 s period gated only the DS18B20 chain (1.3 mA at 20 % duty) and a
~200 ms ADC burst. Sampling flat out moves the DS18B20 to 100 % duty: about **+1 mA on a
~70 mA baseline**, under 2 %.

| Load | Draw | Duty |
|---|---|---|
| ESP32 active @240 MHz, radios off | ~45 mA | always |
| OLED | ~10–15 mA | always |
| DG636E + probe drive | few mA | always (latched) |
| RGB LED (auto-dimmed to 35 % on battery) | ~1–3 mA | always |
| DS18B20 | 1.3 mA | 20 % → 100 % in field mode |

*(datasheet typicals, not measured on this board)*

**Continuous is also the electrode-safe state.** The 25 kHz wave does not drive the probe
directly — it drives the `DG636E` mux select lines, and that switching is what keeps the
probe drive AC. Parking `GPIO13` static leaves the mux one way and holds a DC bias across
the electrodes, which plates them. `fieldBegin()` therefore latches `excAlwaysOn` on.
Pulsed excitation is the option with a real cost here, and it is an electrochemical one,
not a power one.

---

## 9. The display (OLED UI)

There is **no idle animation** — the screen is always functional and shows live data.

**Shared header** — every screen draws `drawHeader()`: title, the pulsing ~1 Hz "alive"
heartbeat dot, then the battery percentage and an 11×7 battery glyph hard right. The
battery block owns the right ~50 px, so **titles must stay ≤ 13 characters** (this is why
the monitoring title is `GAMBIA`, not `GAMBIA MONITOR`). The glyph's fill is proportional
to charge; while charging it sweeps up to full and back over 2 s.

**Field screen** (`displayField`, field mode) — two layouts, chosen with `screen simple`
(default) / `screen detail`, persisted as NVS key `fscreen`.

The **simple** screen is for an operator standing in water. It draws one screen per
moment of the dip and nothing else. There is no site label, because on a standalone
device nobody can set it. The battery percentage appears only below 20 %; above that
only the glyph is shown.
```
 dry                        measuring                  saved / locked
 ● [▮▮ ]                    ● [▮▮ ]                    ● [▮▮ ]
 ──────────────────         ──────────────────         ──────────────────
    DIP PROBE                 18.2        g/L            18.06       g/L
                            [▮▮▮▮▮▮▮▮░░░░░░░]          [▮▮▮▮▮▮▮▮▮▮▮▮▮▮▮]
  Last: 18.06 g/L           29.4C                      29.4C     SAVED #3
 ──────────────────         ──────────────────         ──────────────────
 Put probe in water         Hold still                 Lift probe for next
```
- While settling, the live value shows **one decimal**. It gains the second once it is
  inside the band, so the extra digit is itself the "settled" signal.
- After a commit the big number is the **committed** mean, frozen. The live value keeps
  drifting while the probe is held or lifted, and the operator must copy down the saved
  number, not the live one.
- `RAW` appears next to the temperature only when the value is *not* corrected to 25 °C.
  The normal case shows no marker.
- A probe that reads something but sits under the immersion threshold shows
  `LOW SIGNAL` / `Check probe in water` instead of asking for a dip that already happened.
- **Faults appear in words** (current faults, not the LED's 30 s latched set). A red
  fault replaces the number with `ERROR` + the cause + `Readings not valid`. An amber
  fault takes turns with the footer instruction every 1.5 s (`! Temp sensor lost`).
  Several faults rotate, 2 s each.

The **detail** screen is the bench view: the live value at two decimals, `@25`/`raw`, the
settle bar, the window's spread and sample count (or the probe voltage), the last
committed observation in the corner, and the footer walking `Dip the probe` →
`Settling - hold still` → `Stable - recording N.N` → `RECORDED` → `Lift probe to re-arm`.

**Logger screen**
```
GAMBIA          ● 87%[▮▮ ]  ← title + heartbeat + battery percentage and glyph
────────────────────────
Temp          23.4°C      ← small label left, large right-aligned value + unit
Salt          31.2 g/L
────────────────────────
Next reading: 45s         ← context status / countdown footer
```
The footer is state-aware: `Measuring...` during a read, `Reading updated` just after, and
a `Next reading: Ns` countdown between samples. Before the very first reading it shows
placeholder dashes (`--.-`), and the percentage shows `--` until the first battery sample.

**Charging screen** (`displayCharging`, shown whenever charge mode is active)
```
CHARGING        ● 92%[▮▮▮]
────────────────────────
┌────────────┐
│▮▮▮▮▮▮▮▮    │┤   92%     ← big battery with the charging sweep + large percentage
└────────────┘
4.06 V         charging   ← pack voltage + the MCP73833's own verdict
────────────────────────
Unplug USB to measure
```
The state word is `charging`, `full`, `FAULT`, `no batt` or `idle`.

Both STAT pins high only means "not charging", which covers two unrelated situations, so
`chargeStateName()` and `chargeStateShort()` consult `PG` before naming it. With input power
present it really is standby and the battery connector is probably off (`no batt`). With no
input power the MCP73833 is simply in shutdown with its open-drain pins released, so the
word is `idle` / `idle (no input power)` — reporting "no battery" there was misleading when
the pack is sitting at 3.9 V.

**Calibration screen** — shows `Bluetooth: linked/waiting`, the current `a`/`b`/`c`, and
the command hint `'autocal' or 'help'`.

**Auto-cal screen** (`displayAutoCal`, shown during `autocal`) — `Points: N/8`, the current
`a`/`b`, the last fit `R2` (or `fit: pending`), and a short status line echoing the last
action (`pt3 V=0.80 S=35.0`, `fit R2=0.998`, …).

**WiFi card** (`displayWifiSplash`, six seconds when the console comes up)
```
WIFI READY      ● 92%[▮▮▮]
────────────────────────
AP  Gambia-3F1A
pass salinity

http://192.168.4.1
────────────────────────
browse to control it
```
It returns `true` when it draws, so each of the three screen call sites (charge,
calibration, monitoring) simply skips its own render that pass.

**Splash / sleep** — a boxed "GAMBIA / Water salinity monitor" splash at boot; a
"Sleeping 1 min…" status before deep sleep (then the panel is switched off).

---

## 10. Quick reference — key symbols in the code

| Function | Role |
|---|---|
| `setup()` | Init peripherals, enumerate DS18B20, pick mode from NVS |
| `loop()` | Dispatches to calibration, field or logger behaviour |
| `fieldBegin()` / `loggerBegin()` | Enter a measuring mode, clearing whatever the other left behind |
| `fieldTick()` | The field state machine: dry → settling → stable → recorded → locked |
| `fieldSampleSalinity()` / `fieldTempTick()` / `fieldImmersionTick()` | Live sampling, the temperature EMA, and debounced immersion |
| `stabPush()` / `stabIsStable()` / `stabBand()` / `stabProgress()` | Rolling window, the settle test, and the convergence the bar and LED show |
| `fieldCommit()` / `recPush()` / `recIndex()` | Commit one observation and the ring that holds them |
| `salinityAt25()` / `salinityAtTemp()` | Temperature normalisation, and its inverse for calibration |
| `sendUplink()` | Shared encode + transmit for both measuring modes |
| `processCommand()` | Shared USB/BT command parser (globals + calibration + auto-cal) |
| `handleBluetoothCommands()` / `handleSerialCommands()` | Feed lines to `processCommand()` |
| `triggerRecalibration()` | Clear NVS `done` and reboot into calibration mode |
| `startMeasurement()` / `finishMeasurement()` | Non-blocking measure → encode → display → transmit |
| `tempInit()` / `tempStartConversion()` / `tempReadResult()` | Async DS18B20 read (no `delay(750)`) |
| `readAdcVoltageAveraged()` / `measureVoltageBlocking()` | Averaged salinity ADC read |
| `voltageToSalinity()` | The live curve — quadratic formula on `P1/P2/P3`, or `a·V^b + c` — in the probe's own temperature domain |
| `salinityToVoltage()` / `quadCeiling()` | The forward quadratic, and the vertex where it stops being invertible |
| `fitCalibration()` | Log–log least-squares fit → `a`, `b`, R² |
| `fitCalibrationQuad()` | Forward least-squares fit of `V` against `S` → `P1`, `P2`, `P3`, R² (in volts) |
| `goToDeepSleep()` | Timer-based deep sleep |
| `drawMainScreen()` / `drawValueRow()` / `drawHeader()` | Logger UI |
| `displayField()` | Field UI: live value, settle bar, last observation, what to do next |
| `drawBatteryGlyph()` / `displayCharging()` | Battery glyph (any size) and the charging screen |
| `batteryTick()` / `batteryPrime()` / `batteryPercentFromVoltage()` | Cached pack voltage, charger state, percentage |
| `handleChargeMode()` | Suspends measuring while USB-C power is present |
| `wifiTick()` / `wifiBringUp()` / `wifiStartAp()` / `wifiTakeDown()` | Web console radio lifecycle and policy |
| `wifiHandleRoot/Status/Cmd/Log/History/Csv/Records/RecordsCsv()` | The HTTP endpoints |
| `LogStream` / `StringStream` | Serial+ring tee, and command-reply capture for HTTP |
| `displayWifiSplash()` | OLED card with SSID / password / URL |
| `displayCalibration()` / `displayAutoCal()` | Calibration & auto-cal UI |
| `printHelp()` | Prints the command list |
| `onEvent()` | LMIC LoRaWAN event handler (incl. `recal` downlink) |
