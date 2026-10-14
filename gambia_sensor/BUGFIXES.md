# Bug fix review — 2026-09-24

A correctness review of `gambia_sensor.ino` and `battery_cal.h`, covering the FIELD/LOGGER
split, temperature compensation, the observation ring, the web/CSV exports and the uplink
payload. Seventeen defects were found and fixed. Everything below is verified against the
source; the firmware builds at 57 % of `huge_app` (1.81 MB) and 23 % of RAM, with no
warnings from the changed code under `--warnings all`.

Nothing here has been run on hardware yet. The fixes are grouped by what an operator
would actually see go wrong.

---

## 1. Data that lies about itself

These are the ones that matter most, because they produce numbers a hydrologist has no
way to disbelieve.

### 1.1 Invalid samples contaminated the stability window
`fieldTick()` pushed into the window without checking `liveValid`. On a dropout sample
`fieldSampleSalinity()` sets `liveVolts` to ~0 and leaves `liveSalt` at its **previous**
value, so the push inserted a duplicate salinity — shrinking the standard deviation and
pulling a premature commit — beside a ~0 V probe voltage that never produced it. The
committed row then reported a falsely tight `sd` next to a `probe_v` and `saltRaw` that
did not correspond to its salinity, and that row was uplinked.

An intermittent probe is exactly the case that reaches this: `immersedRaw` chatters, which
restamps `immersionEdgeMs` on every flip, so `immersed` never debounces false and the
state machine stays in SETTLING/STABLE throughout.

**Fixed:** `liveValid` is now part of the admission test.

### 1.2 A record could be flagged "@25" when its samples were raw
`fieldCommit()` derived the compensation flag from `tempCompEnabled && liveTempOk`, which
was wrong twice over. It ignored the `TEMPCOMP_MIN_DIVISOR` guard, so a reading the guard
had already forced back to raw still went out flagged compensated; and it read the *last*
sample only, so a DS18B20 that dropped out a quarter-second before the commit condemned a
whole window of good compensated samples.

**Fixed:** each sample now carries its own provenance (`stabFlag[]`), and the committed
row's flags are the **AND** across the window (`stabFlags()`). "This observation is
temperature-compensated" is a claim about all twenty samples averaged into it.

### 1.3 The observations CSV reported a fabricated 0.00 °C
`liveTemp` is never invalidated when the DS18B20 is absent, so records stored its
initialiser and `gambia-observations.csv` showed a flat `0.00` in `temp_c` — indistinguishable
from real 0 °C water. `temp_compensated` could not stand in for it, since that is also 0
whenever compensation is simply switched off. The flag was in `FieldRecord::flags` all
along; nothing published it.

**Fixed:** new `temp_valid` column, `temp_c` left **blank** when invalid, `tempOk` added to
`/api/records`, and the web table and `recs` listing show `—` / `-- C` instead of `0.00`.

### 1.4 `/api/status` reported compensation that had not happened
`compNow` was derived from `tempAddrValid`, which only records that the DS18B20 answered
the enumeration at **boot**. A probe pulled since, or one returning CRC-valid garbage, left
`finishMeasurement()` raising amber ×6 while the page printed `g/L @25°C` and suppressed the
warning banner. The OLED's `@25`/`raw` marker had the same defect.

**Fixed:** both now read what actually happened — `liveComp` in field mode, the last
reading's `flags` bit 2 in logger mode.

### 1.5 Calibration points were frozen at the settings in force when taken
`fitCalibration()` read only `calS`, the temperature-converted value computed at `sample`
time. `calSRef`/`calT`/`calTOk` were stored and never used. Two failure paths:

- `sample` at alpha 0.020, then `tempco 0.023`, then `fit` — the curve is built on points
  corrected by 0.020 while runtime divides by 0.023. A standing ~2 % error in a 32 °C bath,
  which the firmware only warned about.
- `tempcomp off` while sampling, `tempcomp on` before fitting — the bath temperature is
  baked into `a` and `b` *and* divided out again at runtime. Double-corrected, ~14 % at a
  32 °C bath.

**Fixed:** new `calFitValue(i)` recomputes each point from the standard and bath temperature
at **fit** time, under the settings that will actually be used. `points` shows what the
stored points now fit to and flags rows whose settings have moved since. Re-sampling after a
`tempco` change is no longer necessary, and the command's advice was corrected to say so.

---

## 2. Silent loss of function

### 2.1 `exc off` permanently bricked field mode
`fieldSampleSalinity()` never drives the excitation itself — it relies entirely on the latch
`fieldBegin()` sets. `exc off` cleared it and detached LEDC from GPIO13 for good, so every
sample read an undriven probe: `v ≈ 0`, `liveValid` false, `immersedRaw` false. The state
machine could never leave `FIELD_DRY`, no observation could ever be committed, no fault was
raised, and the OLED sat on `Dip the probe` with nothing pointing at the cause. Only
`exc on` or `mode field` recovered. (Logger mode is unaffected: `startMeasurement()` calls
`excitationOn()` every cycle.)

**Fixed:** refused in field mode, with a message saying why and what to do instead.

### 2.2 `tempco <non-numeric>` silently disabled compensation
`String::toFloat()` returns 0.0 on garbage, and `TEMPCO_MIN` is `0.000`, so `tempco 0,023`
(comma decimal separator — the default on a Spanish or French keyboard), `tempco x` and a
bare `tempco ` all set alpha to 0, wrote it to NVS and survived reboot. The divisor became a
constant 1, so the correction was a no-op for every reading while `tempcomp` still reported
ON, `/api/status` reported `tcompNow: true`, the OLED showed `@25` and amber ×6 never fired.
Silent loss of the correction the whole feature exists for.

**Fixed:** new `parseFloatStrict()`, applied to `tempco`, `stability`, `immersion` and
`sample`. An explicit warning when alpha is deliberately set to 0.

### 2.3 The field-mode stall watchdog could never fire
`fieldTick()` evaluated `F_STALLED` **after** calling `fieldSampleSalinity()`, which restamps
`lastMeasOkMs` on the same pass — so the age tested was at most one 250 ms sample interval,
never the 45 s threshold. And if `fieldTick()` stopped running, the `faultSet()` call never
ran either. Red ×5 was unreachable in the one mode with no timed-reading watchdog behind it.

**Fixed:** the fault is evaluated as `fieldTick()`'s **first** action, against the stamp left
by an earlier pass, so a `fieldTempTick()` that blocks on a held-low 1-Wire bus is caught on
the pass after it returns. Documented alongside the honest limit: a loop that never returns
cannot be caught from inside the loop, and nothing here substitutes for the hardware task
watchdog.

### 2.4 A firmware update would have re-purposed deployed loggers
The persisted mode defaulted to `MODE_FIELD`, and `start` never wrote the key at all. A node
flashed before the FIELD/LOGGER split has `done = true` but no `mmode`, so on the first boot
after an update it would have become a field instrument: no 15 s readings, no periodic uplink
(field mode transmits only on a committed dip, and there is nobody there to dip it), no deep
sleep, and the excitation latched on permanently. The site would simply stop reporting while
the pack drained, with nothing in the log saying why.

**Fixed:** a calibrated device with no stored mode is kept in **LOGGER** and says so in the
boot log; the key is then written once so the ambiguity never recurs. `start` now persists the
mode too, so a freshly calibrated device never inherits a compile-time default.

---

## 3. Memory safety and protocol

### 3.1 `dtostrf()` into a 12-byte stack buffer
`displayField()` formatted `liveSalt` at two decimals into `char numbuf[12]`, and
`drawValueRow()` did the same at one decimal. `dtostrf()` takes **no size limit** — it writes
as many characters as the value needs. The value is not bounded by anything: the manual
calibration path accepts `a`, `b` and `c` with no validation at all (`a = command.toFloat()`),
and `voltageToSalinity()` is `a·V^b + c`. A mistyped coefficient or a degenerate auto-cal fit
puts an arbitrarily large number onto a buffer sitting directly below the `getTextBounds()`
out parameters. At two decimals it overflows a decade earlier than at one.

**Fixed:** new `fmtDisplayValue()` uses bounded `snprintf` and clamps. A 128 px screen cannot
show 1e9 g/L meaningfully anyway, and `>99999` tells the operator the curve is wrong, which a
wrapped number would not.

### 3.2 `recs undo` rewound the monotonic counter
`recSeq` is documented as "total observations ever committed" and is what names an observation
in the serial transcript, on the OLED corner and in any LoRa uplink already sent. Decrementing
it meant the next dip reused a retired number, so two different measurements shared one
identifier — precisely what a monotonic counter exists to prevent. `recIndex()` never needed
the decrement (it derives everything from head and count), and `recs clear` already kept the
count, so the two commands disagreed with each other.

**Fixed:** `recSeq` is no longer decremented. Each record now carries its **own** `seq`, so the
OLED corner, `rec`, `recs`, `/api/records` and the CSV all name the row in front of them rather
than pairing a global counter with whichever row happens to be newest.

### 3.3 Logger uplinks violated the documented payload contract
Bytes 4–6 are documented as field-mode only, byte 6 specifically as "samples averaged".
`finishMeasurement()` put `tempOkCount` there, so a decoder written from that comment read the
DS18B20 conversion count (typically 4) as a salinity sample count, and read `sd = 0.00` as
perfect repeatability rather than "not measured".

**Fixed:** logger mode sends 0 for both. The contract comment, the README byte table and the
TTN decoder all now state that `n = 0` is the marker for a logger reading — packet length
cannot serve, since all seven bytes are always transmitted.

### 3.4 `battery_cal.h` defined mutable globals without linkage control
`batGain`, `batOffset` and the three pending-calibration-point variables were plain (or
`static`) definitions in a header. The sketch links today only because the Arduino builder
concatenates every `.ino` into one translation unit; adding a single `.cpp` beside the sketch
— the natural next step as this grows past one file — would fail with *multiple definition of
batGain*. `#pragma once` guards a file against being read twice per TU, not against being read
by two TUs. `static` would link but give each TU its own copy, so a calibrated value saved
through one would be invisible to the other.

**Fixed:** `inline` (C++17+; this builds at `gnu++2b`), which gives every TU the same single
object.

### 3.5 `constrain()` re-read NVS up to three times per line
`constrain(x, a, b)` is a macro that expands its first argument up to three times. Passing
`prefs.getFloat(...)` directly performed two or three flash reads per line, nine across the
three settings lines in `setup()`. Harmless only because `getFloat()` happens to be side-effect
free — the same shape around anything that advances state is a correctness bug, and it is the
classic Arduino macro-argument trap.

**Fixed:** read into locals first, then clamp the locals.

### 3.6 Failure markers truncated out of two listings
`-Wformat-truncation` flagged three `snprintf` buffers too small for their worst case.
`snprintf` is bounded, so none of these could overflow — but each dropped its tail exactly
when the tail carried the information:

- `hist` listing, `char line[72]`: ~48 characters of numbers plus two failure markers
  totalling 34. A reading that failed **both** sensors — the one row anybody scanning the
  listing is looking for — lost `[salinity failed]` off the end. Now 112.
- `points` listing: `[no temp]` and `[settings changed since]` can both be present. Now 96.
- `autoCalMsg[26]`: `knownS` is bounded only by `> 0`, so a fat-fingered `sample 100000`
  truncated the OLED status line. Now 40.

---

## 4. Guard behaviour corrected and made visible

`TEMPCOMP_MIN_DIVISOR` refuses the correction once `1 + alpha·(T − 25)` falls below 0.25. Its
comment claimed that was about −25 °C at the default alpha. It is **−12.5 °C** — and the
threshold is not fixed at all, because it moves with alpha:

| alpha | coldest water it can correct |
|---|---|
| 0.019 | −14.5 °C |
| 0.020 | −12.5 °C |
| 0.023 | −7.6 °C |
| 0.030 | 0.0 °C |
| 0.040 | +6.3 °C |
| 0.050 | +10.0 °C |

A device trimmed toward the carbonate end and deployed on a cold river can therefore be pushed
into the guard by perfectly real water, falling back to raw and raising amber ×6 on a good
reading with no indication that the coefficient, not the sensor, was the cause.

Widening the floor is **not** the fix: at alpha 0.05 and 0 °C the divisor is already negative
and the "correction" would flip the sign of the salinity. The linear model genuinely is
degenerate there. So the guard stays, and was instead made audible:

- the comment now states the real formula and numbers;
- `tempCompCutoffC()` reports the cut-off, which `tempcomp` and `tempco` print, so the operator
  sees it at the moment they choose alpha;
- `salinityAt25()` logs the first reading it suppresses, by name, rather than leaving an
  unexplained amber ×6.

---

## Behaviour changes an operator will notice

| Change | Effect |
|---|---|
| `exc off` in field mode | Now refused, with an explanation |
| `recs undo` | No longer rewinds the running count; the next dip gets a fresh number |
| `recs` listing | Gained a leading `#` column; invalid temperatures print `-- C` |
| `tempco` / `stability` / `immersion` / `sample` | Reject non-numeric input instead of reading it as 0 |
| `tempcomp` / `tempco` output | Gained a `limit` line naming the coldest correctable water |
| `points` | Shows what the stored points fit to **now**, and flags moved settings |
| `gambia-observations.csv` | Gained `observation` and `temp_valid` columns; `temp_c` blank when invalid |
| `/api/records` | Gained `seq` and `tempOk` fields |
| Logger uplinks | Bytes 4–6 are now 0 (were `sd = 0`, `n = tempOkCount`) |
| First boot after update, on a device with no stored mode | Stays in LOGGER, and says so |

The CSV column additions and the uplink change are the two that touch anything downstream:
re-point any spreadsheet that reads columns by position rather than by name, and re-deploy the
TTN decoder from the README.
