#pragma once
// ================== PER-DEVICE BATTERY CALIBRATION ==================
//
// analogReadMilliVolts() applies the chip's own eFuse ADC calibration, so the
// voltage *at GPIO33* is already about as good as this silicon gets. What it
// cannot know is the divider: R20/R21 are 1 M 1 % parts, so the x2 ratio in
// BAT_DIVIDER is nominal only and each assembled board lands somewhere inside
// a +-2 % band. On the first board that showed up as 3.93 V displayed against
// 3.881 V on the multimeter - 1.25 % high, i.e. entirely within the resistors'
// own tolerance. No firmware constant can fix that for all boards; it has to be
// measured per device, which is what this file is for.
//
//     V_corrected = V_measured * batGain + batOffset
//
// Defaults 1.0 / 0.0 mean an uncalibrated board behaves exactly as before.
// Both live in their own NVS namespace ("batcal") so they survive `recal`,
// which only clears the salinity namespace ("calib").
//
// Deliberately free of sketch globals - the caller passes the raw measurement
// in - so this header can be included high up, before the battery code exists.

#include <Arduino.h>
#include <Preferences.h>

static const char* BATCAL_NS = "batcal";

// `inline` and not a bare definition: the Arduino builder concatenates every
// .ino into one translation unit, so a plain `float batGain = 1.0f;` links today
// by luck alone. Add one .cpp beside the sketch that includes this header and
// the build dies with "multiple definition of batGain" - #pragma once guards a
// file against being read twice per TU, not against being read by two TUs.
// `static` would link, but would then give each TU its OWN gain, so a calibrated
// value saved through one copy would not be seen by the other. `inline` is the
// one spelling that gives every TU the same single object.
inline float batGain   = 1.0f;
inline float batOffset = 0.0f;

// Sanity envelopes. A divider tolerance stack cannot plausibly leave these, so
// anything outside means corrupt NVS or a fat-fingered multimeter entry, and
// silently ignoring it beats bricking the gauge.
static const float BATCAL_GAIN_MIN   = 0.80f;
static const float BATCAL_GAIN_MAX   = 1.25f;
static const float BATCAL_OFFSET_MAX = 0.50f;   // volts, magnitude
// Two calibration points closer together than this make the slope meaningless:
// the ADC noise floor would dominate the division.
static const float BATCAL_MIN_SPAN   = 0.10f;   // volts between point 1 and 2

// Point 1 of a two-point run, held in RAM until CAL2 arrives. `inline` for the
// same reason as batGain above: these are mutable state, and a per-TU copy would
// mean CAL1 recorded through one dispatcher is invisible to the CAL2 that
// follows it.
inline bool  batCalP1Valid = false;
inline float batCalP1Meas  = 0.0f;
inline float batCalP1True  = 0.0f;

inline float batCalApply(float measured) { return measured * batGain + batOffset; }

inline bool batCalIsDefault() { return batGain == 1.0f && batOffset == 0.0f; }

inline bool batCalPlausible(float g, float o) {
  return g >= BATCAL_GAIN_MIN && g <= BATCAL_GAIN_MAX &&
         fabsf(o) <= BATCAL_OFFSET_MAX;
}

inline void batCalLoad() {
  Preferences p;
  if (p.begin(BATCAL_NS, true)) {
    float g = p.getFloat("gain", 1.0f);
    float o = p.getFloat("off",  0.0f);
    p.end();
    if (batCalPlausible(g, o)) { batGain = g; batOffset = o; }
  }
}

inline void batCalSave() {
  Preferences p;
  if (!p.begin(BATCAL_NS, false)) return;
  p.putFloat("gain", batGain);
  p.putFloat("off",  batOffset);
  p.end();
}

inline void batCalPrint(Stream& out, float measured) {
  out.print(F("GAIN   : ")); out.println(batGain, 5);
  out.print(F("OFFSET : ")); out.print(batOffset, 5); out.println(F(" V"));
  out.print(F("Measured : ")); out.print(measured, 4); out.println(F(" V (uncorrected)"));
  out.print(F("Corrected: ")); out.print(batCalApply(measured), 4); out.println(F(" V"));
  if (batCalIsDefault()) out.println(F("(defaults - this board has never been calibrated)"));
}

// Returns true if `raw` was one of the CAL* commands, false to let the sketch's
// own dispatcher carry on. `measured` must be the UNCORRECTED reading, since
// every point is recorded against the uncorrected scale - feeding a corrected
// value back in would compound the previous calibration into the new one.
// Cheap prefilter so the dispatcher can skip the 64-conversion measurement for
// every non-CAL command. Note `cal` (the salinity routine) is deliberately NOT
// one of ours, and neither is `cancel`.
inline bool batCalIsCalCommand(const String& raw) {
  String lc = raw; lc.trim(); lc.toLowerCase();
  return lc == "calshow" || lc == "calreset" ||
         lc == "cal1" || lc.startsWith("cal1 ") ||
         lc == "cal2" || lc.startsWith("cal2 ") ||
         lc == "calsingle" || lc.startsWith("calsingle ");
}

inline bool batCalCommand(const String& raw, Stream& out, float measured) {
  String c = raw; c.trim();
  String lc = c;  lc.toLowerCase();

  if (lc == "calshow") {
    batCalPrint(out, measured);
    if (batCalP1Valid) {
      out.print(F("Point 1 pending: measured ")); out.print(batCalP1Meas, 4);
      out.print(F(" V -> true ")); out.print(batCalP1True, 4);
      out.println(F(" V.  Change the pack voltage, then CAL2 <volts>."));
    }
    return true;
  }

  if (lc == "calreset") {
    batGain = 1.0f; batOffset = 0.0f; batCalP1Valid = false;
    batCalSave();
    out.println(F("Battery calibration reset to defaults (GAIN 1.0, OFFSET 0.0)."));
    batCalPrint(out, measured);
    return true;
  }

  bool isC1 = (lc == "cal1")      || lc.startsWith("cal1 ");
  bool isC2 = (lc == "cal2")      || lc.startsWith("cal2 ");
  bool isCS = (lc == "calsingle") || lc.startsWith("calsingle ");
  if (!isC1 && !isC2 && !isCS) return false;

  int sp = c.indexOf(' ');
  float truth = (sp < 0) ? 0.0f : c.substring(sp + 1).toFloat();
  // toFloat() returns 0.0 on garbage, which the range test rejects anyway.
  if (sp < 0 || truth < 2.0f || truth > 5.0f) {
    out.println(F("usage: CAL1 <volts> | CAL2 <volts> | CALSINGLE <volts>"));
    out.println(F("       <volts> is your multimeter reading at the pack, 2.0-5.0 V"));
    return true;
  }

  out.print(F("Measured : ")); out.print(measured, 4); out.println(F(" V (uncorrected)"));
  out.print(F("Multimeter: ")); out.print(truth, 4); out.println(F(" V"));

  if (measured < 1.0f) {
    out.println(F("REFUSED: the divider reads under 1 V - no pack, or this ESP32 is"));
    out.println(F("         not seated in the carrier board. Nothing saved."));
    return true;
  }

  if (isC1) {
    batCalP1Meas  = measured;
    batCalP1True  = truth;
    batCalP1Valid = true;
    out.println(F("Point 1 recorded. Now change the pack voltage by at least 0.10 V"));
    out.println(F("(charge or discharge it a while), then send CAL2 <volts>."));
    out.print(F("Corrected: ")); out.print(batCalApply(measured), 4);
    out.println(F(" V  (still on the old GAIN/OFFSET)"));
    return true;
  }

  float g, o;
  if (isCS) {
    // Single point: pure gain, offset forced to zero. Right when the error is a
    // divider ratio (it is, here) - a resistor tolerance scales, it does not
    // shift. Reach for the two-point run only if a fixed offset is suspected.
    g = truth / measured;
    o = 0.0f;
  } else {
    if (!batCalP1Valid) {
      out.println(F("No point 1. Send CAL1 <volts> first, or use CALSINGLE <volts>."));
      return true;
    }
    float dm = measured - batCalP1Meas;
    if (fabsf(dm) < BATCAL_MIN_SPAN) {
      out.print(F("REFUSED: the two points are only ")); out.print(fabsf(dm), 4);
      out.println(F(" V apart. Need 0.10 V or more,"));
      out.println(F("         otherwise ADC noise sets the slope. Point 1 kept."));
      return true;
    }
    g = (truth - batCalP1True) / dm;
    o = batCalP1True - g * batCalP1Meas;
  }

  if (!batCalPlausible(g, o)) {
    out.print(F("REFUSED: implausible fit (GAIN ")); out.print(g, 5);
    out.print(F(", OFFSET ")); out.print(o, 5); out.println(F(" V)."));
    out.println(F("         Check the multimeter value was the PACK, not the pin."));
    out.println(F("         Nothing saved."));
    return true;
  }

  batGain   = g;
  batOffset = o;
  batCalP1Valid = false;
  batCalSave();

  out.println(F("Saved to NVS."));
  batCalPrint(out, measured);
  return true;
}
