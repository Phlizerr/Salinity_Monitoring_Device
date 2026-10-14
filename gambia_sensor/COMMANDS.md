# Terminal / Command Reference — Gambia Water Salinity Monitor

Every command below is a **line of text** (newline-terminated), **case-insensitive**,
and can be sent over any of the interfaces in the table. Replies always go back on the
same interface you sent the command from.

## Interfaces

| Interface | How to reach it | When it's live |
|---|---|---|
| **USB serial** | Serial monitor / VSCode terminal at **115200 baud** | Always (both calibration and Measuring modes) |
| **Bluetooth (SPP)** | Connect to `ESP32_Gambia_Calibrator` with a BT serial terminal | Calibration mode only (BT is off in Measuring mode) |
| **WiFi (web console)** | Browse to the device and use the command box, or `GET /api/cmd?c=<command>` | Whenever the radio is up — see `wifi` below (default: while USB-C power is present) |
| **LoRa downlink** | TTN downlink, first payload byte `0xC1` | Measuring mode only (delivered in the RX window after an uplink) |

> **macOS note:** macOS cannot open a Bluetooth-Classic SPP port, so on a Mac use the
> **USB serial** terminal. `recal` also works from USB while the device is running.

---

## Command summary

| Command | Mode | What it does |
|---|---|---|
| `help` | any | Prints the full command list |
| `coef` | any | Prints the coefficients of whichever curve is live, and the last fit R² |
| `model` / `model quad` / `model power` | any | Which curve turns probe volts into salinity. Persisted |
| `pcoef <P1> <P2> <P3>` | any | Types the quadratic coefficients in directly. Persisted |
| `recal` | any | Reboots into calibration mode (see below) |
| `adc [N]` | any | Raw ADC diagnostic on GPIO34: counts + voltage, excitation OFF vs ON (`N` bursts ~1 s apart) |
| `exc on` / `exc off` | any | Latch the excitation square wave permanently ON (bench measurement) / restore per-reading pulsing |
| `samples [N]` | any | Show, or set to `N`, the number of salinity ADC samples averaged into one reading (range 8–4096) |
| `tsamples [N]` | any | Show, or set to `N`, the number of DS18B20 temperature conversions averaged into one reading (range 1–6) |
| `sleep on` / `sleep off` | any | Allow deep sleep between readings / stay always on. **Logger mode only** |
| `mode` / `mode field` / `mode logger` | any | Report or switch the measuring mode. Persisted to NVS |
| `tempcomp` / `tempcomp on` / `off` | any | Normalise salinity to 25 °C (**default on**). Persisted |
| `tempco <alpha>` | any | The temperature coefficient itself, per °C (default `0.020`, range 0–0.05). Persisted |
| `site <name>` / `site` | any | Set, or clear, the label stamped on every observation (max 11 chars) |
| `rec` | field | Commit the current window as one observation, settled or not |
| `screen` / `screen simple` / `screen detail` | any | Field-mode OLED layout: operator view (default) or bench view with spread, sample count and probe voltage. Persisted to NVS |
| `recs` / `recs clear` / `recs undo` | any | List, empty, or drop the newest committed observation |
| `autorec` / `autorec on` / `off` | any | Commit automatically once the reading settles (**default on**). Persisted |
| `stability [g/L]` | any | Show or set the absolute floor of the settle band (default `0.05`). Persisted |
| `immersion [V]` | any | Show or set the probe voltage above which the probe counts as in water (default `0.110`). Persisted |
| `diag` | any | Full self-check: mode, uptime, peripherals, every active fault **with the blink code the LED is showing for it** |
| `rgb` | any | Status LED: report state, `on`/`off`/`auto`/`test`, `bright <0-255>`, or a held `<R> <G> <B>` colour |
| `bat` | any | Battery voltage, percentage, charger state (`MCP73833`) and whether input power is present |
| `CALSINGLE <V>` | any | Calibrate the battery divider from one multimeter reading (gain only) |
| `CAL1 <V>` / `CAL2 <V>` | any | Two-point battery calibration (gain **and** offset) |
| `CALSHOW` / `CALRESET` | any | Print the stored battery `GAIN`/`OFFSET`, or restore 1.0 / 0.0 |
| `power` / `usb` | any | Raw `PG`/`STAT1`/`STAT2` pin levels plus a plain-language verdict — the USB-C charging troubleshooter |
| `hist` | any | The stored readings as a table (`/api/csv` downloads the same data) |
| `charge` / `charge auto` / `charge ignore` | any | Report the charger, and choose whether plugging in USB-C **pauses** measurements (`auto`, **default**) or not (`ignore`) |
| `wifi` / `wifi on` / `off` / `auto` / `ap` / `sta <ssid> <pass>` | any | The web console: report it, force the radio up or down, or store a network to join |
| `cal` | calibration | Manual edit of the raw coefficients `a`, `b`, `c` (the **power** curve only) |
| `autocal` | calibration | Starts guided auto-calibration; clears previous points |
| `sample <S>` | calibration | Records a point at known salinity `S` (g/L) |
| `points` | calibration | Lists the recorded `(V, S)` points |
| `undo` | calibration | Removes the last recorded point |
| `fit` | calibration | Fits **whichever curve is live** to the points — `P1,P2,P3` (needs ≥3 points at 3 different salinities) or `a,b` with `c` set to 0 (needs ≥2) |
| `cancel` | calibration | Leaves auto-calibration (keeps current coefficients) |
| `start` | calibration | Saves both curves' coefficients and the model choice to NVS, then switches to Measuring mode |

*"Mode"* = which operating mode accepts the command. In **Measuring mode** only the global
commands (`help`, `coef`, `model`, `pcoef`, `recal`, `adc`, `exc on/off`, `samples`, `tsamples`,
`sleep on/off`, `mode`, `tempcomp`, `tempco`, `site`, `rec`, `recs`, `autorec`,
`stability`, `immersion`, `bat`, `CAL*`, `charge`, `wifi`, `rgb`) do anything; the
calibration commands reply *"In Measuring mode…"*. `rec` additionally requires field mode.

**Measuring mode** is the umbrella term for the run state after `start`. It is one of two
instruments, chosen with `mode` and remembered in NVS:

- **Field** (**default**) — an operator is holding the probe. Salinity is sampled four
  times a second so the number is live, the firmware watches it settle, and **one
  averaged observation is committed per dip** with the spread that justifies it. The
  uplink fires on that commit. No deep sleep — the power button is the off switch.
- **Logger** — unattended. A reading every **15 s** inside a 5-minute awake window, an
  uplink per reading, then deep sleep (`sleep on`; `sleep off` is the bench default and
  keeps it always on).

Either way salinity is normalised to 25 °C — see **Measurement quality** below.

---

## Hardware diagnostics (`adc`, `exc on/off`)

These help you verify the salinity analog front-end (excitation → probe → filter → ADC)
without going through the full salinity pipeline. They work in **any mode**.

### `adc [N]` — read the ADC pin directly

Reads **GPIO34** (12-bit, 11 dB attenuation, `0–4095` counts ⇒ `0–3.3 V`) and prints raw
counts and voltage for **two conditions per line**: excitation **OFF** (probe not driven)
and excitation **ON** (25 kHz square wave driving the probe). `adc` does one burst;
`adc 20` does 20 bursts ~1 s apart so you can watch the value while you dip the probe.

```
adc
> ADC diagnostic on GPIO34 (12-bit, 11 dB, 0-4095 => 0-3.3 V)
>   OFF: 12.0 cts / 0.010 V   |   ON: 1850.0 cts / 1.491 V
```

The OFF column always shows a true baseline: even when the excitation is latched on (see
below), `adc` briefly detaches the drive to sample the baseline, then restores the latch.

### `exc on` / `exc off` — latch the excitation square wave

`exc on` turns the 25 kHz square wave **on and leaves it on** so the probe is continuously
driven — useful for probing the front-end with a multimeter/oscilloscope. While latched,
regular measurements and the `adc` command can no longer switch it off. `exc off` clears
the latch and returns to normal per-reading pulsing.

> **`exc off` is refused in field mode.** Field mode has no per-reading pulsing to fall
> back to — it samples continuously and relies entirely on the latch. Clearing it would
> detach the excitation pin for good, leaving every sample reading an undriven probe, so
> the reading would sit on `Dip the probe` forever with no fault raised. Send
> `mode logger` first if you genuinely want the drive dead.

> The latch lives in RAM, so a **power-cycle or EN/RST reset** drops it back to off — just
> resend `exc on`. With `LORA_ENABLED 0` (the default) there is no deep sleep, so it stays
> on indefinitely otherwise.

---

### `bat` — battery and charger

```
> bat
Battery: 3.921 V  (1.961 V at GPIO33, x2.0 divider)
Level:   66 %  (filtered 3.919 V -> 66 % on screen)
Charger: charging   |   Input power: present
```

Voltage comes from the `R20`/`R21` 1M divider on `GPIO33`, so the pin sees half the
pack and the reading is doubled. Charger state is decoded from the `MCP73833`'s
active-low `STAT1`/`STAT2` pair — see the truth table in `README.md` §2. The same two
figures are appended to every periodic measurement line in the serial log.

The percentage comes from a piecewise LiPo discharge curve (`BAT_CURVE` in the
sketch), interpolated between points: the cell curve is almost flat between 3.7 and
3.9 V, so a straight 3.3→4.2 V line would read badly wrong for most of the pack's
life. `bat` prints two numbers — the instantaneous one and the EMA-filtered one the
OLED shows, which is deliberately slow so the figure does not jitter while you watch it.

> **While charging the percentage reads high.** `GPIO33` sits on the `BAT` node, which
> during charge is the *charger's output*: the `MCP73833` holds ~4.2 V in constant-voltage
> long before the pack is actually full. `bat` says so explicitly when it sees that state.
> `charge complete` is authoritative, and is pinned to 100 %.

---

### `CALSINGLE` / `CAL1` + `CAL2` — per-device battery calibration

**Every board needs this once.** `analogReadMilliVolts()` already applies the ESP32's own
eFuse calibration, so the voltage *at the pin* is as good as the silicon gets — but
`BAT_DIVIDER = 2.0` assumes `R20` and `R21` are exactly equal, and at **1 % each** they are
not. The first board measured read **3.93 V against 3.881 V** on a multimeter: 1.25 % high,
entirely inside the resistors' own tolerance. No constant in the firmware can fix that for
every board, so it is measured per device and stored in NVS.

```
V_corrected = V_measured * GAIN + OFFSET
```

Defaults are `GAIN = 1.0`, `OFFSET = 0.0`, so an uncalibrated board behaves exactly as it
did before. They live in their own NVS namespace (`batcal`), so **`recal` does not clear
them** — that command only touches the salinity namespace (`calib`).

#### One-point (what you normally want)

A divider tolerance is a *scale* error, not a shift, so one point is the honest fit:

```
> CALSINGLE 3.881
Measured : 3.9301 V (uncorrected)
Multimeter: 3.8810 V
Saved to NVS.
GAIN   : 0.98750
OFFSET : 0.00000 V
Measured : 3.9301 V (uncorrected)
Corrected: 3.8810 V
```

Measure at the **pack terminals**, not at `GPIO33`, and with **USB unplugged** — charge
current lifts the `BAT` node 50–100 mV and would calibrate the error straight in.

#### Two-point (only if you suspect a fixed offset)

`CAL1` records a point, then you change the pack voltage and `CAL2` fits the line through
both and saves `GAIN` and `OFFSET` together:

```
> CAL1 4.102      # full-ish pack
> ... run the device down for a while ...
> CAL2 3.741
```

The two points must be **≥ 0.10 V apart** or the fit is refused — closer than that and ADC
noise sets the slope. `CAL1` survives until `CAL2`, a `CALRESET`, or a reboot.

#### Checking and undoing

`CALSHOW` prints the stored constants plus the current measured/corrected pair; `CALRESET`
puts back `1.0` / `0.0`. `bat` also shows the raw uncorrected volts and the active
constants on every call.

A fit outside `GAIN` 0.80–1.25 or `|OFFSET|` > 0.5 V is **refused and not saved** — that
range cannot come from a divider tolerance, so it means a typo or a probe on the wrong
node. Calibrating with the pin below 1 V is refused too: that is a missing pack, or an
ESP32 that is not seated in the carrier.

> **Expect the percentage to drop.** Correcting a 1.25 % gain error at 3.93 V moves the
> reading to 3.881 V, which on the discharge curve is **69 % → 62 %**. The gauge was
> reading optimistically before, not accurately now.

---

### `power` — is USB-C actually delivering anything?

**There is no charge indicator LED anywhere on the V2 board.** Nothing lights up when you
plug in USB-C. `Q1` also isolates the battery the instant VBUS appears, so the pack voltage
stops being evidence of anything. The MCP73833's three open-drain pins — `PG` on GPIO39,
`STAT1` on GPIO35, `STAT2` on GPIO36, all active LOW — are the only report the board can
give, and `power` dumps them raw alongside a verdict:

```
---- MCP73833 raw pins (open drain, active LOW, 10k to 3V3) ----
  PG    GPIO39 = LOW   -> input power PRESENT
  STAT1 GPIO35 = LOW
  STAT2 GPIO36 = HIGH
  decoded:      charging
  battery:      4.061 V at BAT (via the 1M/1M divider on GPIO33)

---- verdict ----
  USB-C power IS reaching the charger.
  The cell is charging right now. This is the normal working state.
```

`PG` is the single most useful bit. It answers "is 5 V reaching the charger chip", which
separates a cable/connector/source problem from a charger problem:

| `PG` | `STAT1` | `STAT2` | Meaning |
|---|---|---|---|
| HIGH | — | — | **No input power.** Nothing is reaching the MCP73833 — cable, source, jack, or D1 |
| LOW | LOW | HIGH | Charging. The normal, working state |
| LOW | HIGH | LOW | Charge complete — the cell is full, so no current flows |
| LOW | LOW | LOW | Fault: safety timer, thermal, or a dead cell |
| LOW | HIGH | HIGH | Standby — power is in, but no battery is detected on `BAT` |
| HIGH | HIGH | HIGH | Idle — no input power, so the charger is in shutdown with its pins released. **Not** a battery fault; this is the normal state on battery |

**Running the module outside the carrier board.** `GPIO35`/`36`/`39` are input-only with
**no internal pull-ups** — the carrier's 10 kΩ pull-ups to 3V3 are the only thing holding
them high. Put the ESP32 on a bench on its own and all three float LOW, which decodes as
*"input power present, charger in FAULT"* and is enough to drop the firmware into charge
mode permanently, pausing every measurement on a board that has no charger at all.

`checkPowerPath()` catches it: that trio cannot coexist with a flat `BAT` divider, because a
powered MCP73833 holds `BAT` well above 1 V even into a dead cell. When it fires, charger
monitoring is disabled, measurements keep running, the console shows a banner, and `power`
says so instead of printing a verdict it cannot support:

```
---- verdict ----
  These readings are NOT trustworthy: all three pins are low while
  BAT reads flat, which cannot happen on a connected V2 board. ...
```

A genuinely shorted pack sits in *precondition* (`STAT1` LOW, `STAT2` HIGH), not FAULT, so a
real battery fault does not trip this guard and is still reported normally. The check re-runs
at 5 Hz, so seating the module back in the carrier clears it without a reset.

Two further traps this command exists to catch:

- **The dev-kit's own micro-USB does not charge anything.** `5VIN` is unconnected on V2, so
  the only path to the charger is the board's USB-C jack. Powering the ESP32 over the
  dev-kit cable and wondering why nothing charges is the expected result, not a fault.
- **The STAT/PG pull-ups sit on the 3V3 rail**, which only exists once `Q2` is on. With the
  device switched off these readings mean nothing — and a healthy charger is completely
  silent, because no LED exists to show it. **Switch the device on before judging.**

If `PG` stays HIGH with a cable in, work through it in this order:

1. Is the cable in the **board's** USB-C jack rather than the dev-kit's micro-USB?
2. Try a **USB-A → USB-C** cable from a plain phone charger. A C-to-C source only switches
   VBUS on when it sees the 5.1 kΩ CC pulldowns (`R29`/`R30`); if one is unfitted, cracked
   or dry, C-to-C stays dead while A-to-C keeps working. **That asymmetry is the tell.**
3. Meter it: 5 V between the `D1` anode and GND with the cable in. 5 V present but `PG`
   still HIGH points at the MCP73833 or its VDD joint; no 5 V there points at the cable,
   the source, or the jack's solder pads.

---

### `hist` — stored readings

The device keeps the last **120 readings** in a RAM ring (about half an hour at the 15 s
default; ~2.4 KB). `hist` prints them as a table, newest last, with the age of each entry
and a marker on any sensor that failed:

```
Last 14 reading(s) of 14 taken (ring holds 120):
   age     temp     salinity   probe    batt
   195s   21.44 C     1.62 g/L  0.4131 V  4.061 V
   180s   21.44 C     1.61 g/L  0.4126 V  4.060 V
```

The same data backs the console's trend chart and downloads as a spreadsheet from
`http://<ip>/api/csv`. The ring lives in RAM only — a reset clears it.

---

### `wifi` — the web console

```
> wifi
Console: AP Gambia-3F1A  ->  http://192.168.4.1
Password: salinity
Clients: 1
Policy:  auto - up only while USB-C power is present
Network: own AP (no stored network)
Free heap: 187364
```

The ESP32-WROOM-32E already has WiFi, so this costs no hardware. It serves a single
self-contained page (no CDN — the device has no internet) with live tiles, a scrolling
log and a command box wired to the **same** `processCommand()` the serial terminal uses:
anything you can type over USB works there.

| Form | Effect |
|---|---|
| `wifi` | Report SSID, URL, client count/RSSI, policy, stored network and free heap |
| `wifi auto` | Radio up **only while USB-C power is present** (**default**) |
| `wifi on` | Force it up, battery or not. Deep sleep is suppressed while it is up |
| `wifi off` | Force it down |
| `wifi ap` | Forget the stored network and serve the device's own AP |
| `wifi sta <ssid> <pass>` | Store credentials in NVS and join that network instead (SSID may not contain spaces) |

**AP mode (default)** needs no infrastructure: join `Gambia-XXXX` (`XXXX` from the eFuse
MAC, so two boards never clash) with the password above and browse to `192.168.4.1`. The
OLED shows a **WIFI READY** card with the SSID, password and URL for six seconds when the
console comes up, and the charging screen alternates its bottom line with the URL.

**STA mode** joins an existing network, which is nicer from a laptop because you keep your
internet connection. If the join does not complete within 12 s it falls back to AP, so a
wrong password never leaves you locked out.

Why `auto` is the default: WiFi costs roughly **100 mA** while associated, against LoRa's
~120 mA for a few hundred milliseconds per uplink. The whole deep-sleep design exists to
avoid that, so the radio only runs when the device is on mains power — which is also
exactly when you would be servicing it. The policy lives in RAM (like `sleep` and `exc`),
so a reset returns to `auto`; the stored STA credentials **are** persistent.

#### HTTP API

| Endpoint | Returns |
|---|---|
| `GET /` | The console page |
| `GET /api/status` | JSON: mode, temp, salinity, probe voltage, battery V/%, charger state, charge mode, the curve model (`model`) and both sets of coefficients (`a`/`b`/`c`, `p1`/`p2`/`p3`), sample counts, countdown, WiFi info, free heap, uptime, the compensation state (`tcomp`/`tcompNow`/`tempco`) and a `field` object (state, wet, settle window, band, progress, record counts, site) |
| `GET /api/cmd?c=<command>` | Runs the command, replies with its output as `text/plain` |
| `GET /api/log?from=<byte offset>` | Everything printed since that offset; the new offset comes back in the `X-Log-Next` header |
| `GET /api/history` | JSON: the stored readings as parallel arrays (`t`, `temp`, `salt`, `volts`, `batV`, `f`) — what the trend chart draws |
| `GET /api/csv` | The same readings as a downloadable CSV (`gambia-readings.csv`) |
| `GET /api/records` | JSON: the committed observations as whole rows (mean, sd, n, temp, site, compensation flag) |
| `GET /api/records.csv` | The observations as a downloadable CSV (`gambia-observations.csv`) — **this is the field data**; `/api/csv` is the raw sample trace |

So it scripts cleanly without the page: `curl "http://192.168.4.1/api/cmd?c=bat"`.

The log is a 4 KB ring buffer fed by `Log`, a `Stream` that tees to the USB serial port.
Every `print` in the firmware goes through it, and so do command replies from **all**
interfaces — the web page, USB and Bluetooth share one transcript.

> `recal` reboots inside the request, so the page will show a dropped connection rather
> than a reply. That is expected; the device comes back in calibration mode.

---

### `charge` — charge mode and its policy

```
> charge
Charger: PLUGGED IN  (charging, 4.06 V, 92 %)
Policy:  auto   - measurements PAUSE while USB-C is plugged in
Probe:   excitation latched ON (AC drive, no DC on the electrodes)
Change with 'charge auto' or 'charge ignore'.
```

With the default `auto` policy, plugging the USB-C charger in puts the device into
**charge mode**: any conversion chain in flight is abandoned, no new measurement
starts, no LoRa uplink goes out, deep sleep is suspended, and the OLED switches to the
charging screen. Unplugging resumes measuring immediately (the next reading is taken
straight away rather than waiting out the 15 s period).

| Form | Effect |
|---|---|
| `charge` | Report charger presence, state, voltage, percentage and the current policy |
| `charge auto` | Plugging in USB-C pauses measurements (**default**) |
| `charge ignore` | Keep measuring with a cable in — the bench/development override |

Detection is the `MCP73833`'s `PG` pin (`GPIO39`), debounced over three 200 ms samples
so a bouncing connector cannot flap the device in and out of the mode.

`adc` and `sample` are refused while charge mode is active, and `exc on` sets the latch
without restarting the wave. Serial (and, in calibration mode, Bluetooth) command
handling stays live throughout — a device on the bench with a cable in it still answers.

**The excitation is deliberately left alone.** Charge mode calls `excitationOff()`,
which honours the `exc on` latch, so by default the 25 kHz square wave keeps running.
That wave is what keeps the probe drive AC; parking `GPIO13` static would leave the
`DG636E` switched one way and hold a DC bias across the electrodes for the whole
charge, which plates and corrodes them. Send `exc off` first if you want the drive
genuinely dead while charging.

---

### `rgb` — status LED control

`LED1` is a common-anode RGB LED (red `GPIO4`, green `GPIO15`, blue `GPIO17`, 150 Ω
per channel). By default it reports firmware state and flashes fault codes — see
the pattern and code tables in `README.md` §2. `diag` names whatever it is
flashing, which beats counting blinks.

| Form | Effect |
|---|---|
| `rgb` | Report on/off, auto/manual, brightness, driver (PWM or on/off) and the wiring |
| `rgb off` | Blank the LED. Status tracking continues underneath |
| `rgb on` | Un-blank it |
| `rgb auto` | Drop a held colour and resume automatic status reporting |
| `rgb test` | Wiring check: red, green, blue, white, 400 ms each (blocking) |
| `rgb bright <0-255>` | Master brightness (default `180`) |
| `rgb <R> <G> <B>` | Hold a fixed colour (0–255 each) until `rgb auto` |

`rgb test` is the quickest way to confirm the board is wired as the firmware thinks:
if the colours come out in the wrong order the `rgbPinR/G/B` mapping is wrong, and if
every colour is inverted (lit when it should be dark) flip `RGB_COMMON_ANODE` to `0`.

With no argument, `rgb` also reports **what the LED is currently showing** — the
active fault code and its meaning, or "no faults". A held colour (`rgb <R> <G> <B>`)
and `rgb test` run at full brightness even on battery; automatic patterns dim to
~35 % when USB-C is unplugged.

Colours are gamma-corrected and pass through a per-channel trim (`RGB_GAIN_R/G/B`)
that exists because all three resistors are 150 Ω while the red die has the lowest
forward voltage — without it, "white" comes out pink. Tune those constants by eye
against the real LED. Blue stays at maximum: `R24` already starves it, which is why
the status vocabulary carries meaning in motion and flash count rather than hue.

---

### `diag` — full self-check

Prints everything the firmware knows about its own health in one reply: mode and
uptime, how long ago the last reading completed, battery and charger state, whether
each peripheral answered (OLED, DS18B20, power sense, LEDC, WiFi), and then every
active fault **with the exact blink code the LED is flashing for it**.

```
=== DIAGNOSTICS ===
Mode      : measuring
Uptime    : 412 s
Readings  : 27 taken, last completed 4 s ago
Battery   : 3.91 V, 62 %,  charger: idle (no input power)
Peripherals:
  OLED      : ok
  DS18B20   : NOT FOUND
  Power sense: ok
  RGB driver: LEDC PWM
  WiFi      : down
Faults (what the LED is flashing):
  AMBER x1  temperature failing - DS18B20 off the 1-Wire bus, or CRC errors
LED       : automatic
```

Use it instead of counting flashes whenever you can reach a terminal or the web
console. The LED, `diag` and the console's fault chips all render the same `faults`
bitmask, so they cannot drift apart. Faults stay listed for 30 s after they clear,
marked `[cleared, still held]`, which is how you catch an intermittent one.

---

## Reading cadence & averaging (`coef`, `samples`, `sleep on/off`)

In **logger mode** the device takes one reading every **15 seconds**. Each reading is
the average of `samples` raw ADC bursts on the salinity front-end, so a larger sample
count trades a slightly longer (blocking) read for a quieter, more stable number.

**Field mode ignores `samples`** and uses its own, smaller count (128) so a live sample
takes ~51 ms and the display can update four times a second. Averaging depth there is not
this knob — it is the 20-sample settle window, which averages 5 s of history *and* reports
the spread. See `stability`.

### `model` — which curve turns volts into salinity

There are two, and `model` picks between them. Persisted to NVS, so it survives the power
latch like every other deployment setting.

| Model | Equation | Fitted by |
|---|---|---|
| **`quad`** (default) | `V = P1·S² + P2·S + P3`, solved for `S` | `fit` (≥3 points) or `pcoef` |
| `power` | `S = a·V^b + c` | `fit` (≥2 points) or `cal` |

The quadratic is stored the way the probe was characterised on the bench — **voltage as a
function of salinity**, the direction the measurements were actually taken in. Runtime
needs the other direction, so the firmware inverts it with the quadratic formula:

```
S = ( -P2 + √(P2² - 4·P1·(P3 - V)) ) / (2·P1)
```

taking the **+** branch of the square root. With `P1 < 0` the parabola bends over and the
two roots straddle its vertex; dividing by a negative `2·P1` flips their order, so the
`+` root is the *lower* salinity — the one on the rising flank the probe works on. The
`−` root is its mirror image past the turnover and would read river water as ~100 g/L.

Two ends where the algebra stops matching the hardware, both **clamped**:

- **`V < P3`** — `P3` is the intercept, the voltage the curve calls zero salinity
  (`0.0990 V` as shipped). Below it the `+` root goes negative, so it reads **0 g/L**. The
  real probe reads **~0.090 V in air**, just under `P3`, so every lift-out lands here —
  but only by 9 mV, so a probe whose dry offset drifted up slightly would report tenths of
  a g/L in air instead of 0. The in-water/in-air decision is **not** made here: it is a
  separate voltage test against the `immersion` threshold (`0.110 V` as shipped), set
  *above* the dry reading rather than below `P3`, so air cannot pass as very fresh water.
- **`V` above the vertex** — the discriminant goes negative and there is no real root.
  The reading is pinned at the curve's maximum and the log says so, **once**:

  ```
  [curve] probe reads 1.7200 V, above the 1.6904 V ceiling of the quadratic curve.
          Reporting its maximum, 72.83 g/L.
          The reading is CLAMPED, not measured. Re-fit the curve, or
          send 'model power' if this water is outside what it covers.
  ```

  The ceiling moves with the coefficients — `coef` prints where it lands for whichever
  ones are loaded. As shipped it is **72.83 g/L at 1.6904 V**, well clear of seawater at
  ~35 g/L, but a real limit worth knowing about.

```
model
> Salinity curve: quad   V = P1*S^2 + P2*S + P3, solved for S
>   probe now: 0.4153 V -> 5.000 g/L (uncompensated)
```

Switching models **clears the displayed R²** — it belonged to the other curve's fit.
Existing coefficients for both curves are kept, so switching back and forth is free.

> **Upgrading an already-calibrated device.** A device whose NVS says `done` but carries
> no stored model was calibrated before the quadratic existed, so its `a`/`b` *are* its
> calibration. It keeps the **power** curve on first boot and says so in the log, rather
> than silently swapping in a curve that was never fitted to that probe. `model quad`
> moves it over — then re-run `autocal`, or `pcoef` the bench coefficients in.

### `pcoef <P1> <P2> <P3>` — type the quadratic coefficients in

For a curve characterised on the bench rather than fitted on the device. Three numbers,
space-separated, dots not commas. Allowed in **any mode** — the point of a bench curve is
being able to load it into a device that is already deployed. Persisted immediately.

```
pcoef -0.0003 0.0437 0.0990
> Saved to NVS.
> Quadratic curve  V = P1*S^2 + P2*S + P3   (salinity by the quadratic formula)
>   P1 = -0.000300
>   P2 = 0.043700
>   P3 = 0.099000
>   valid 0.0990 V .. 1.6904 V  ->  0 .. 72.83 g/L
```

Bare `pcoef` just prints them. Each field goes through the strict number parser, so
`pcoef -0,0006 …` is **refused** rather than read as zero — a zero `P1` is a legal-looking
quadratic that is secretly a straight line.

> **NVS wins over the source, silently.** The `p1`/`p2`/`p3` keys are written by `pcoef`
> and by `start`. Once they exist, editing the defaults at the top of the sketch and
> re-flashing **changes nothing** — `setup()` loads the stored values over them, and `coef`
> keeps printing the old numbers with nothing to say why. Same for `a`/`b`/`c`, `immv`,
> `tempco` and the rest of namespace `calib`. On a device that has never had them set, the
> source defaults do apply, which is why a first flash looks like it works. If a re-flash
> seems to be ignored, check `coef` against the source: if they disagree, NVS is the
> reason — use `pcoef` to set the values directly, or `recal` to start over.

### `coef` — print the active coefficients

Prints the coefficients of whichever curve is live, plus the last fit R² if a fit has run
this session. Works in any mode, so you can check what the device is applying without
re-entering calibration.

```
coef
> Calibration coefficients (model: V = P1*S^2 + P2*S + P3, inverted):
>   P1 = -0.000300
>   P2 = 0.043700
>   P3 = 0.099000
>   zero-salinity intercept: 0.0990 V  (anything below reads 0 g/L)
>   ceiling: 72.83 g/L at 1.6904 V, above which readings are CLAMPED
>   the power coefficients a/b/c are stored but idle ('model power')
```

### `samples [N]` — averaging depth

`samples` with no argument shows the current count; `samples 512` sets it (clamped to
**8–4096**). At ~400 µs per sample, 256 samples ≈ 100 ms and 4096 ≈ 1.6 s — all well
within the 15 s period. The default is **256** (up from the old 32). The setting lives in
RAM and resets to the default on reboot.

```
samples
> ADC samples per reading: 256  (range 8-4096, ~400 us/sample)
samples 1024
> ADC samples per reading set to 1024
```

> `samples` averages the **salinity ADC**. Temperature averaging is separate — see
> `tsamples` below.

### `tsamples [N]` — temperature averaging depth

`tsamples` shows or sets how many DS18B20 conversions are averaged into one temperature
reading (clamped to **1–6**, default **4**). The conversions are chained non-blocking —
each takes ~750 ms — and accumulated, so a reading with `tsamples 4` spends ~3 s on
temperature. Invalid conversions (CRC errors) are dropped and the rest are still averaged.
Like the others, the setting lives in RAM and resets on reboot.

```
tsamples
> Temp conversions per reading: 4  (range 1-6, ~750 ms/conversion)
tsamples 6
> Temp conversions per reading set to 6
```

> Keep `tsamples × 750 ms` plus the salinity read within the 15 s period — the 1–6 clamp
> already guarantees this. The serial log shows how many conversions actually succeeded,
> e.g. `Temp: 21.44 C (avg of 4/4 conv)`.

### `sleep on` / `sleep off` — continuous vs low-power

**Logger mode only.** `sleep off` (the **default**) keeps the device always on, reading
every 15 s forever — ideal for lab/bench work. `sleep on` restores the unattended cycle:
measure during a 5-minute awake window, then deep-sleep between windows (deep sleep only
actually happens when LoRa hardware is enabled). The flag lives in RAM and resets to `off`
on reboot.

In field mode this does nothing, and that is deliberate — an operator is standing over the
device and the power button is a hardware latch, so sleeping between dips would only lose
the reading. See `mode`.

---

## Field work

The field-mode commands exist because **V2 has no user input at all**: `Button1` drives
the power latch directly and never reaches a GPIO, and `GPIO26` is unconnected. Every
gesture is therefore either inferred from the probe (immersion, settling) or typed here —
over USB, or from the buttons in the web console, which post to this same parser.

### `mode` — which instrument this is

```
mode
> Measuring mode: FIELD
>   field  - operator holds the probe: continuous read, the firmware
>            watches it settle, one averaged observation per dip,
>            uplink on the commit. No sleep (the power switch is off).
>   logger - unattended: a reading every 15 s inside a 5 min window,
>            uplink each one, then deep-sleep. 'sleep on' to enable.
mode logger
```

Persisted to NVS (`mmode`), because the power latch means the device cannot be asked
which one it should be on the way up. Sent from calibration mode it is stored and takes
effect on `start`; sent while measuring it switches immediately. `start` also writes the
mode, so a freshly calibrated device always has an explicit one rather than inheriting a
compile-time default.

> **Upgrading a device flashed before the field/logger split.** Such a device is already
> calibrated but has no `mmode` key, and the only behaviour it ever had was timed logging.
> It is therefore kept in **LOGGER** mode on the first boot after the update — silently
> promoting it to a field instrument would stop it reporting (field mode transmits only on
> a committed dip, and there is nobody there to dip it) and drain the pack. The boot log
> says so:
>
> ```
> No stored measuring mode: this device was calibrated by a build that
> predates the field/logger split, so it is kept in LOGGER mode.
> Send 'mode field' if it is meant to be a handheld instrument.
> ```

### The dip, end to end

With auto-record on (the default) the whole workflow is physical:

1. **Dip the probe.** The LED breathes green until it goes in; immersion is detected from
   the probe voltage and debounced over 1.2 s.
2. **Hold still.** Amber blips *quicken* as the reading converges. The OLED bar fills.
3. **It commits itself.** Once the rolling 20-sample (5 s) window sits inside the settle
   band for 3 s, the mean is recorded with its standard deviation and uplinked — one long
   green flash.
4. **Lift the probe.** Green double-blips until you do; lifting re-arms the next dip.

Lifting is what re-arms, deliberately: five samples without moving the probe measure the
firmware's averaging, not the site.

### `site <name>` — label the observations

```
site Bintang
> Site label set to 'Bintang'. It is stamped on every observation from now on.
site
> Site label cleared.
```

Max 11 characters. Commas, quotes and newlines become `_`, because the label goes into a
CSV column verbatim. The label also becomes the OLED title, so you can see which site the
device thinks it is at.

### `rec` / `recs` — the observations

`rec` commits the current window by hand — useful with `autorec off`, or when a site's
water simply will not settle. It warns and records anyway if the window is outside the
band, so you always know what you got:

```
rec
> WARNING: not settled (sd 0.2140 > band 0.1820 g/L). Recording anyway.
> Recorded #4 @Bintang: 18.09 +/- 0.214 g/L @25C over n=20
```

```
recs
> Last 3 observation(s) of 3 (ring holds 48):
>     #    age    site        salinity@25   sd      temp     n
>     2   572s  Bintang        17.98 g/L   0.041  29.10 C   20
>     3   307s  Bintang        18.14 g/L   0.050  29.30 C   20
>     4    22s  -              18.06 g/L   0.039     -- C   20  [raw, uncompensated]
> Download from http://<ip>/api/records.csv
```

The **`#` column is the observation's own number**, assigned when it was committed and
never reused. A `-- C` temperature means the DS18B20 did not answer for that dip; the
CSV leaves that cell blank and sets `temp_valid` to 0 rather than reporting 0.00 °C.

`recs undo` drops the newest; `recs clear` empties the ring. **Neither rewinds the running
count** — after an undo the next dip gets a fresh number, not the discarded one, so a
number that has already appeared in the log or gone out over LoRa never names two
different measurements.

The ring holds 48 observations in RAM only — **download the CSV before powering off.**

### `autorec`, `stability`, `immersion` — tuning the settle detection

| Command | What it controls |
|---|---|
| `autorec on` / `off` | Commit automatically after 3 s inside the band, or wait for `rec` |
| `stability [g/L]` | The **absolute floor** of the band. The band actually used is `max(this, 1 % of the reading)` — a fixed band alone is too tight at 35 g/L, a relative one never converges near zero |
| `immersion [V]` | The probe voltage above which the probe counts as wet |

```
stability
> Settle band: +/-0.050 g/L, or 1.0 % of the reading, whichever is larger.
>   window 20 samples at 250 ms = 5.0 s of history.
>   now: sd 0.0412 over n=20  -> STABLE
```

```
immersion
> Immersion threshold: 0.110 V  (~0.252 g/L on the current curve)
>   probe now: 0.0901 V -> dry
> Set it between the dry reading and the freshest water you expect.
> 'adc' shows the dry reading with the excitation on.
```

> **If the device seems dead in the field, check this first.** A threshold set for
> brackish water silently refuses to do anything at a fresh site — the probe is in the
> river and the screen still says `Dip the probe`. The firmware detects this case
> specifically: the OLED switches to `Below immers. thresh` and the web console offers a
> corrected value to paste. Use `adc` to see the dry reading and set the threshold
> between it and the freshest water you expect.

---

## Measurement quality

### `tempcomp` / `tempco` — temperature compensation

Conductivity moves about **2 % per °C**. Uncorrected, the same water reads ~28 % apart
between a 22 °C morning and a 35 °C afternoon. Everything is therefore normalised to
25 °C using the temperature measured in the same cycle:

```
S25 = S_measured / (1 + alpha · (T − 25))
```

```
tempcomp
> Temperature compensation: ON
>   model  : S25 = S / (1 + alpha * (T - 25))
>   alpha  : 0.0200 per degC   ('tempco <alpha>' to trim)
>   status : applied
>   limit  : water colder than -12.5 C cannot be corrected at this alpha
```

`tempco <alpha>` trims the coefficient (range 0–0.05). Typical values: `0.019` for a
NaCl-dominated estuary, `0.020` general, `0.023` carbonate-rich. Both settings persist.

The value must be a **number with a dot**, not a comma — `tempco 0,023` is rejected
rather than silently read as `0`. (Alpha 0 is technically in range but makes the
correction a no-op while `tempcomp` still reports ON, so the command warns about it.)

**When there is no temperature** the correction cannot be applied. The firmware does not
quietly fall back to raw — it reports the raw number, marks it uncompensated in the CSV
and the observation table, and raises **amber ×6**. The OLED shows `raw` instead of `@25`.

> **The `limit` line is not decoration.** The correction divides by `1 + alpha·(T − 25)`,
> and the firmware refuses to apply it once that divisor drops below 0.25, where the
> linear model is degenerate and the result explodes. Where that lands in degrees moves
> with alpha:
>
> | alpha | coldest water it can correct |
> |---|---|
> | 0.019 | −14.5 °C |
> | 0.020 | −12.5 °C |
> | 0.023 | −7.6 °C |
> | 0.030 | 0.0 °C |
> | 0.040 | +6.3 °C |
> | 0.050 | +10.0 °C |
>
> At the default alpha this only ever trips on garbage. Trimmed hard toward the carbonate
> end **on a cold river it can trip on real water** — and when it does, the log says so by
> name rather than leaving you with an unexplained amber ×6:
>
> ```
> [tempcomp] 8.10 C is outside the range alpha=0.0450 can correct (limit 8.3 C).
>            Reporting RAW salinity.
>            Lower alpha with 'tempco <a>' if the water really is this cold.
> ```

> **`tempco` interacts with calibration — but no longer invalidates it.** Each `sample`
> records the standard's nominal value *and* the bath temperature. The conversion into the
> bath's temperature domain is redone **at `fit` time**, from the settings in force then,
> so changing `alpha` or toggling `tempcomp` between sampling and fitting no longer leaves
> the curve in a different temperature domain from the runtime correction that undoes it.
> There is no need to re-sample; `points` shows what the stored points now fit to and says
> when the settings have moved:
>
> ```
> points
> > Points: 2
> >    #   probe V   standard@25   bath C   fitted@bath
> >    1    0.8021         15.00    30.10     16.76  [settings changed since]
> >    2    1.8314         35.00    30.00     39.03  [settings changed since]
> > The fit uses 'fitted@bath': the standard corrected to the bath it was
> > measured in, so the curve carries no temperature of its own.
> > NOTE: alpha or 'tempcomp' has changed since some of these were taken.
> >       They are shown, and will be fitted, under the settings in force
> >       NOW - which is what runtime will undo. No need to re-sample.
> ```

---

## Auto-calibration (recommended)

Auto-calibration measures the probe in **solutions of known salinity that you type in**
and fits **whichever curve `model` says is live**. `fit` never fits the other one — that
would print coefficients nothing reads, which looks exactly like a successful calibration
from the terminal and changes not one reading.

| Live model | What `fit` computes | Minimum points | R² is in |
|---|---|---|---|
| **`quad`** (default) | `P1`, `P2`, `P3` of `V = P1·S² + P2·S + P3` | **3**, at three *different* salinities | volts |
| `power` | `a`, `b` of `S = a·V^b` (fixes `c = 0`) | 2, with varying V | log–log |

The quadratic is fitted **forward** — voltage against salinity, by normal equations — and
that is deliberate. The error least squares should be minimising lives in `V`: that is the
measured quantity, with the ADC noise on it, while `S` is the standard, known to whatever
the bottle says. Fitting `S` against `V` would put the residuals on the axis that has no
error in it, and would have to invert every point through a curve that does not exist yet.

Its R² is therefore in volts, answering *"how well does this curve predict the voltage
these standards produce"*. **It is not comparable with the power model's log–log R²** —
don't read one against the other.

Three points is a hard floor for the quadratic, not a recommendation: two points pin down
a line, and a parabola through two points is a whole family. Three points at only *two*
distinct salinities is the same problem wearing a disguise, and `fit` refuses it too.

### Step by step

1. Put the device in calibration mode (first boot, or send `recal`).
2. Send `autocal` — clears any previous points and prints instructions.
3. Dip the probe in your first known solution, wait for it to stabilise, then send
   e.g. `sample 5` (5 g/L). The device drives the excitation, averages the ADC and
   stores the `(voltage, 5.0)` point.
4. Repeat for the other solutions, e.g. `sample 15`, `sample 35`.
5. Send `points` any time to review what you've recorded; `undo` drops the last one
   if a sample was bad.
6. Send `fit` — prints the new coefficients and the **R²**. R² close to `1.0` is a good
   fit; a low value means a point is off (re-`sample` it) or the solutions were too close.
   On the quadratic it also prints the curve's ceiling, and warns if a recorded point sits
   *above* it — a curve that cannot reproduce its own calibration needs more points.
7. Send `start` to save the coefficients to NVS and begin normal Measuring mode.

### Example session (USB terminal)

```
autocal
> Auto-calibration started. Any previous points cleared.
sample 5
> Recorded point 1: V=0.3120 V, S=5.00 g/L
sample 15
> Recorded point 2: V=0.5450 V, S=15.00 g/L
sample 35
> Recorded point 3: V=0.8010 V, standard 35.00 g/L @25C, bath 30.10 C -> fitting 38.57 g/L
fit
> Fit OK (model: V = P1*S^2 + P2*S + P3):
>   P1=-0.000300
>   P2=0.043700
>   P3=0.099000
>   R2=0.9998
>   (R2 is in VOLTS - not comparable with the power model's log-log R2)
>   covers up to 72.83 g/L / 1.6904 V, then the curve turns over
start
> Exiting calibration mode. Stopping Bluetooth and starting FIELD mode.
```

The **AUTO-CAL screen** on the OLED mirrors this: point count (`Points: N/8`), the live
curve's leading coefficients, the last fit `R²` (or `fit: pending`), and a status line.

> Use solutions with **S > 0**. Under the power curve a 0 g/L reference can't be
> represented at all while `c = 0`; under the quadratic it is representable — `P3` is
> exactly that intercept — but a fresh-water point is the least informative one you can
> take, so spend your standards on the range you actually measure.

---

## Manual calibration (`cal`)

For when you already know the **power** coefficients (e.g. from a lab fit) and just want
to type them in. The quadratic equivalent is `pcoef`, documented above.

> `cal` only ever edits `a`/`b`/`c`. If the quadratic curve is live it says so up front,
> because otherwise the walk-through reads as a successful calibration while editing
> coefficients nothing uses.

```
cal
> Current values: a=..., b=..., c=...
> Do you want to change them? (yes/no)
yes
> Enter the new value for 'a':
12.2315
> Enter the new value for 'b':
1.8950
> Enter the new value for 'c':
0.0
> Calibration finished! Send 'start' to save & run.
start
```

Sending `no` (or anything that isn't `yes`/`y`) cancels without changing anything.

---

## Recalibration at any time (`recal`)

`recal` lets you re-calibrate a device that's already deployed, **without erasing NVS by
hand**. It clears the "calibrated" flag and **reboots** into calibration mode (rebooting
is what cleanly releases the LoRa SPI bus and LMIC state). Your previous `a`/`b`/`c`
remain stored and are shown as the starting point.

You can trigger it from **all three interfaces**:

- **USB:** type `recal` in the serial monitor (works even during normal Measuring operation).
- **Bluetooth:** send `recal` while connected (calibration mode).
- **LoRa:** queue a **downlink** to the device whose **first byte is `0xC1`** (any FPort).
  It is applied in the RX window right after the next uplink, then the device reboots.

After the reboot the device is in calibration mode again: Bluetooth `ESP32_Gambia_Calibrator`
is back up and the USB terminal accepts `autocal` / `cal` / `start`.

---

## Notes on readings & the payload

- Temperature is read from the DS18B20 **without blocking** the main loop: the `tsamples`
  conversions (~750 ms each) are chained between loop passes and averaged, so LoRa timing
  isn't disturbed.
- A failed reading is reported on the serial log as
  `ERROR: temperature read failed…` / `ERROR: salinity read failed…`, the **last good
  value is kept on the display**, and the uplink carries the sentinel `0x7FFF` for that
  field so the backend can flag it as missing.
- **Bytes 4–6 are field-mode only.** They carry the standard deviation of the committed
  window (×100, uint16 BE) and the number of salinity samples averaged into it. Logger
  mode reads the ADC once, so it has neither: it sends **0 for both**, and a decoder
  should read `n = 0` as *not measured* rather than as a perfectly repeatable single
  sample. (It previously put the DS18B20 conversion count in byte 6, which a decoder
  written from this contract read as a salinity sample count.)
- See the README for the full payload format and the TTN decoder.
