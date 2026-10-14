#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <WiFi.h>
#include <BluetoothSerial.h>
#include <math.h>
#include <OneWire.h>
#include "esp_sleep.h"
#include <Preferences.h>
#include "battery_cal.h"   // per-device battery ADC gain/offset

// ================== OLED DISPLAY I2C (SSD1306 128x64) ==================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*  The V1 board had the OLED hand-wired onto GPIO18/19, which collided with the
 *  LoRa SPI bus (SCK/MISO) and forced an either/or choice. The V2 board fixes
 *  that: `DiplayConn1` is a proper connector on its own I2C pads, so the
 *  display and the radio can now both be populated.
 */
#define LORA_ENABLED   0        // 0 = no LoRa/SPI (radio not populated on V2). 1 = full firmware.

#define DISPLAY_SDA    21       // header pin 33, net SDA -> DiplayConn1 pad 4
#define DISPLAY_SCL    22       // header pin 36, net SCL -> DiplayConn1 pad 3
#define DISPLAY_ADDR   0x3C
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOk = false;

// ================== NVS (non-volatile memory) ==================
Preferences prefs;

// ================== BLUETOOTH CALIBRATION ==================
BluetoothSerial SerialBT;

// Calibration variables (editable and persistent)
float a = 12.2315;
float b = 1.8950;
float c = 0.0;

// ---- Quadratic model (the default) ----
// The bench characterisation is a FORWARD fit - the probe voltage as a function
// of the salinity of the standard it is sitting in:
//
//     V = P1*S^2 + P2*S + P3
//
// Runtime needs the other direction, so voltageToSalinity() inverts it with the
// quadratic formula, taking the + branch of the square root (see there for why
// that is the physical one and what happens at the two ends where it is not).
//
// P1 < 0 means the response bends over: the curve has a maximum, and above the
// voltage at its vertex there is no real salinity that produces the reading. That
// ceiling moves with the coefficients, so it is computed rather than assumed -
// quadCeiling() finds it and `coef` prints where it lands for whichever numbers
// are actually loaded. As shipped it is ~1.690 V / ~72.8 g/L, well clear of
// seawater, but it is a real limit and voltageToSalinity() reports hitting it.
float p1 = -0.0003f;
float p2 =  0.0437f;
float p3 =  0.0990f;

// Edge-triggered so a probe parked past the ceiling logs once, not four times a
// second in field mode. Same shape as tempCompGuardTripped, and for the same
// reason: the operator needs to be told, once, that the number on the glass is a
// clamp and not a measurement.
bool quadCeilingTripped = false;

// Which curve turns a probe voltage into a salinity. Persisted as "model".
//
// CURVE_POWER is the original S = a*V^b + c, kept because `autocal`/`fit`, the
// manual `cal` walk-through and every already-calibrated device in the field
// speak it. CURVE_QUAD is the inverted quadratic above and is what a fresh
// device uses; `model power|quad` switches, `coef` prints whichever is live.
enum CurveModel {
  CURVE_POWER,   // S = a*V^b + c
  CURVE_QUAD     // V = P1*S^2 + P2*S + P3, inverted for S
};
CurveModel curveModel = CURVE_QUAD;

// Calibration protocol states (manual a/b/c entry)
enum ProtocolState {
  IDLE,
  AWAIT_CHANGE_CONFIRM,
  AWAIT_VALUE_A,
  AWAIT_VALUE_B,
  AWAIT_VALUE_C
};
ProtocolState currentState = IDLE;

// General operating mode
//
// MODE_FIELD and MODE_LOGGER are two genuinely different instruments that used
// to be one blurred mode, with defaults (`sleepEnabled=false`, 15 s pacing) that
// landed on neither:
//
//   FIELD  - an operator is holding the probe. Salinity is sampled continuously
//            so the number on the glass is live, the firmware watches it settle,
//            and one averaged observation is committed per dip. Deep sleep is
//            meaningless here (the hardware power latch is the off switch) and
//            the uplink fires on the commit, not on a timer.
//   LOGGER - nobody is there. The original behaviour: measure every
//            SEND_PERIOD_MS inside an awake window, uplink each reading, then
//            deep-sleep. This is what AWAKE_WINDOW_MS / SLEEP_INTERVAL_US exist
//            for.
//
// The choice is persisted in NVS ("calib"/"mode") because the V2 power button is
// a hardware latch with no GPIO behind it - the device cannot be asked which one
// it should be on the way up, so it has to remember.
enum WorkMode {
  MODE_CALIBRATION,    // Bluetooth only, no measuring or sleep
  MODE_FIELD,          // operator-present: continuous read, settle, commit per dip
  MODE_LOGGER          // unattended: timed readings + LoRa uplink + deep sleep
};

WorkMode currentMode = MODE_CALIBRATION;

// Which of the two measuring modes a calibrated device boots into. Persisted.
WorkMode measureMode = MODE_FIELD;

// NOTE: both enums below are named in function signatures, so they must be
// complete here — above the first function definition in the sketch. The Arduino
// preprocessor injects its generated prototypes at exactly that point, and a
// prototype naming a type the compiler has not seen yet will not build.

// What the LED shows when nothing is wrong. Faults are deliberately NOT in this
// enum: they live in the fault bitmask below and are rendered as blink codes
// that override whichever of these is current. Keeping the two apart is the
// whole point - the old enum mixed them, so a failed reading was erased by the
// next measurement starting and "charger fault" and "both sensors dead" shared
// one colour. Every pattern here is non-blocking and derived from millis().
// The first three MUST stay first: rgbWrite() tests `rgbStatus <= RGB_BOOT_DONE`
// to skip the battery dimming while setup() is still running.
enum RgbStatus {
  RGB_BOOT_EARLY,  // solid red     - setup() before the peripherals are up
  RGB_BOOT_IO,     // solid amber   - bringing up I2C/OLED and the 1-Wire bus
  RGB_BOOT_DONE,   // solid green   - peripherals up, reading NVS / picking a mode
  RGB_CAL_IDLE,    // white breathe - calibration mode, nobody attached
  RGB_CAL_LINKED,  // fast breathe  - calibration mode, Bluetooth client attached
  RGB_AUTOCAL,     // white pulses  - guided auto-calibration in progress
  RGB_RUN,         // green blip    - logger mode, healthy (doubles on a reading)
  RGB_FIELD_DRY,   // green breathe - field mode, probe out of the water
  RGB_FIELD_SETTLE,// amber blips   - field mode, settling (rate = convergence)
  RGB_FIELD_STABLE,// green fast    - field mode, stable, holding before commit
  RGB_FIELD_REC,   // long green    - field mode, observation committed
  RGB_FIELD_LOCK,  // green double  - field mode, lift the probe to arm the next
  RGB_CHARGING,    // amber blip    - charger plugged in, charging
  RGB_CHARGED,     // green blip    - charger plugged in, charge complete
  RGB_SLEEP        // off           - about to deep-sleep
};

// MCP73833 charge state, decoded from the active-low STAT1/STAT2 pair.
enum ChargeState { CHG_CHARGING, CHG_COMPLETE, CHG_FAULT, CHG_STANDBY };

// Where a field-mode dip has got to. Declared here rather than beside the rest
// of the field-mode block for the same reason as the two enums above: it is
// named in fieldStateName()'s signature. The full state machine is documented
// at fieldTick().
enum FieldState {
  FIELD_DRY,       // probe out of the water - waiting to be dipped
  FIELD_SETTLING,  // immersed, reading still moving
  FIELD_STABLE,    // inside the band, counting down STABLE_HOLD_MS
  FIELD_RECORDED,  // just committed - the long green flash
  FIELD_LOCKED     // committed and waiting for the probe to come out
};

// ================== FAULT BITMASK ==================
// One bit per independently diagnosable condition. Every detector in the sketch
// raises its own bit here, and the RGB LED, the `diag` command and /api/status
// all render this same word - so the blink codes cannot drift away from what the
// firmware actually knows, and no fault has to be inferred from a colour that
// was reused for something else.
//
// The bit position IS the blink code. Bits 0-7 are the red class (the device is
// not producing trustworthy data), bits 8-15 the amber class (still running, but
// degraded or suspect); within a class the flash count is the bit index plus
// one. Adding a fault means adding a bit here and a row in FAULT_TABLE.
enum FaultBit : uint32_t {
  F_POWER_PATH   = 1UL << 0,   // red 1   - power sense dead / not in the carrier
  F_SENSORS_DEAD = 1UL << 1,   // red 2   - neither sensor answered
  F_CHARGER      = 1UL << 2,   // red 3   - MCP73833 reports FAULT
  F_BAT_CRITICAL = 1UL << 3,   // red 4   - pack below BAT_CRITICAL_V
  F_STALLED      = 1UL << 4,   // red 5   - measurement loop is not cycling
  F_TEMP         = 1UL << 8,   // amber 1 - DS18B20 missing or failing
  F_SALINITY     = 1UL << 9,   // amber 2 - probe reads ~0 V
  F_OLED         = 1UL << 10,  // amber 3 - SSD1306 did not initialise
  F_BAT_LOW      = 1UL << 11,  // amber 4 - pack below BAT_LOW_PCT
  F_NO_BATTERY   = 1UL << 12,  // amber 5 - input power good, charger in standby
  F_NO_TEMPCOMP  = 1UL << 13,  // amber 6 - salinity is NOT temperature-corrected
};
const uint32_t FAULT_RED_MASK   = 0x000000FFUL;
const uint32_t FAULT_AMBER_MASK = 0x0000FF00UL;

// `faultRaw` is what is true right now; `faultLatched` is the same set held open
// for FAULT_HOLD_MS after each bit clears. The hold is what makes the LED usable
// for diagnosis at all: a reading that fails once is otherwise overwritten by
// the next cycle ~15 s later, so you would have to be looking at exactly the
// right moment to catch it.
const unsigned long FAULT_HOLD_MS = 30000;
const int           FAULT_SLOTS   = 16;
uint32_t      faultRaw     = 0;
uint32_t      faultLatched = 0;
unsigned long faultClearMs[FAULT_SLOTS] = {0};

// Thresholds behind F_BAT_LOW / F_BAT_CRITICAL. Critical is a voltage rather
// than a percentage because the bottom of BAT_CURVE is where the curve is
// steepest and the percentage means least. Both are only evaluated on battery:
// with a cable in, the divider sees the charger output, not the cell.
const float BAT_CRITICAL_V = 3.40f;
const int   BAT_LOW_PCT    = 20;

// ---- Reading history ----
// The device used to measure and forget: a reading went out over LoRa (or, with
// LoRa disabled, nowhere at all) and nothing was kept. The web console wants to
// draw a trend and export a CSV, so keep the last HIST_MAX readings in a RAM
// ring. Static, not heap - the WiFi stack is the only thing on this chip that
// gets to be greedy. 120 entries at the 15 s default is half an hour of history
// for ~2.4 KB, and the ring silently overwrites the oldest entry after that.
struct Reading {
  uint32_t t;        // seconds since boot, when the reading finished
  float    temp;     // degC
  float    salt;     // g/L, normalised to TEMP_REF_C - the number to trust
  float    saltRaw;  // g/L as the curve returned it, before compensation
  float    volts;    // probe voltage that produced `salt`
  float    batV;     // filtered pack voltage at the time
  uint8_t  flags;    // bit0 = temperature valid, bit1 = salinity valid,
                     // bit2 = temperature compensation was applied
};
const int HIST_MAX = 120;
Reading  hist[HIST_MAX];
int      histCount = 0;   // valid entries, saturates at HIST_MAX
int      histHead  = 0;   // next slot to write
uint32_t histSeq   = 0;   // total readings ever taken; the page polls this to
                          // decide whether it needs to refetch the history

// ---- Field observations ----
// Separate from `hist` on purpose. `hist` is a raw sample trace - what the probe
// said, every cycle, useful for watching a value settle and for debugging. This
// is the operator's data: one row per committed dip, each already an average
// over a stable window, with the spread that justifies it. This is what gets
// uplinked and what the CSV a hydrologist opens should contain.
struct FieldRecord {
  // The observation's own number, assigned from recSeq at commit and never
  // reused. Stored per row rather than derived from recSeq at display time,
  // because `recs undo` removes a row without rewinding the counter: after an
  // undo the newest kept row is no longer the recSeq'th one, and anything
  // pairing the global counter with a specific row's values would label that
  // row with a number belonging to a different measurement.
  uint32_t seq;
  uint32_t t;         // seconds since boot when the observation was committed
  float    salt;      // mean compensated salinity over the stable window, g/L
  float    sd;        // standard deviation across that window, g/L
  float    temp;      // mean temperature over the window, degC
  float    volts;     // mean probe voltage over the window
  float    batV;      // pack voltage at commit
  uint8_t  n;         // samples averaged
  uint8_t  flags;     // bit0 = temperature valid, bit2 = compensation applied
  char     site[12];  // operator label, set with `site <name>`; "" if unnamed
};
const int REC_MAX = 48;
FieldRecord recs[REC_MAX];
int      recCount = 0;    // valid entries, saturates at REC_MAX
int      recHead  = 0;    // next slot to write
uint32_t recSeq   = 0;    // total observations ever committed

// Label applied to observations as they are committed. There is no keyboard on
// the device, so this is set from the web console or a serial terminal before
// walking to a site - `site <name>`, or `site` alone to clear it.
char siteLabel[12] = "";

// NOTE: the WiFi console and log block below sits here, AFTER the enums above,
// for the reason spelled out in that comment: it defines logPush() and the
// LogStream/StringStream methods, which makes it the sketch's first function
// definition and therefore the point where the Arduino preprocessor injects
// its generated prototypes. Move it back above the enums and every prototype
// naming RgbStatus or ChargeState stops compiling.

// ================== WIFI CONSOLE ==================
/*  The ESP32-WROOM-32E on this board already has 2.4 GHz WiFi, so the console
 *  below costs no hardware. It is a LOCAL console, not telemetry: LoRa is the
 *  long-range path, and V2 has no footprint for the radio (see README).
 *
 *  Policy (`wifi` command, RAM-only like `sleep`/`exc`):
 *    auto (default) - radio up only while USB-C power is present, so the field
 *                     power budget is untouched. WiFi costs ~100 mA; the whole
 *                     deep-sleep design exists to avoid exactly that.
 *    on             - forced up, battery or not. Deep sleep is suppressed while
 *                     it is up, otherwise sleeping would drop your connection.
 *    off            - forced down.
 *
 *  AP mode is the default and needs no infrastructure: join `Gambia-XXXX`
 *  (XXXX from the eFuse MAC, so two boards never clash) with the password
 *  below, then browse to 192.168.4.1. `wifi sta <ssid> <pass>` stores
 *  credentials in NVS and joins an existing network instead, which is nicer
 *  from a laptop because you keep your internet connection; it falls back to
 *  AP if the join times out.
 */
#define WIFI_ENABLED 1
#include <WebServer.h>

const char*         WIFI_AP_PASS        = "salinity";   // WPA2 needs >= 8 chars
const uint16_t      WIFI_HTTP_PORT      = 80;
const unsigned long WIFI_STA_TIMEOUT_MS = 12000;        // then fall back to AP
const uint32_t      WIFI_MIN_HEAP       = 60000;        // refuse to start below this

enum WifiPolicy { WIFI_POL_OFF, WIFI_POL_AUTO, WIFI_POL_ON };
enum WifiState  { WIFI_DOWN, WIFI_STA_CONNECTING, WIFI_READY };

WifiPolicy    wifiPolicy = WIFI_POL_AUTO;
WifiState     wifiState  = WIFI_DOWN;
bool          wifiIsAp   = true;
String        wifiSsid   = "";
String        wifiIpStr  = "";
String        staSsid    = "";      // persisted in NVS namespace "wifi"
String        staPass    = "";
unsigned long wifiStaStartMs   = 0;
unsigned long wifiSplashUntil  = 0; // OLED shows the SSID/IP card until this
WebServer     server(WIFI_HTTP_PORT);

// ================== SERIAL + WEB LOG ==================
/*  Everything the firmware prints goes through `Log`, which tees to the USB
 *  serial port AND into a ring buffer the web console reads incrementally.
 *  `Log` is a Stream, so it drops straight into processCommand() where `Serial`
 *  used to go - which means command replies are captured as well, and the web
 *  page shows exactly what the serial monitor shows.
 */
const size_t LOG_BUF_SIZE = 4096;
char     logBuf[LOG_BUF_SIZE];
uint32_t logSeq  = 0;              // total bytes ever written (monotonic)

static inline void logPush(uint8_t c) {
  logBuf[logSeq % LOG_BUF_SIZE] = (char)c;
  logSeq++;
}

class LogStream : public Stream {
public:
  size_t write(uint8_t c) override { logPush(c); return Serial.write(c); }
  size_t write(const uint8_t* b, size_t n) override {
    for (size_t i = 0; i < n; i++) logPush(b[i]);
    return Serial.write(b, n);
  }
  int  available() override { return Serial.available(); }
  int  read()      override { return Serial.read(); }
  int  peek()      override { return Serial.peek(); }
  void flush()     override { Serial.flush(); }
};
LogStream Log;

// Captures a command's reply so a web handler can return it as the HTTP body.
class StringStream : public Stream {
public:
  String out;
  size_t write(uint8_t c) override { out += (char)c; return 1; }
  size_t write(const uint8_t* b, size_t n) override {
    for (size_t i = 0; i < n; i++) out += (char)b[i];
    return n;
  }
  int  available() override { return 0; }
  int  read()      override { return -1; }
  int  peek()      override { return -1; }
  void flush()     override {}
};



bool clientJustConnected = true;

// Forward declarations (called before their definition below)
void triggerRecalibration(Stream* out);
void printHelp(Stream& out);

// ================== TTN KEYS ==================
// Real keys live in secrets.h, which is gitignored. Copy secrets.example.h to
// secrets.h and fill it in; without it the build uses the all-zero template
// and the device will never join.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#warning "secrets.h not found - using placeholder TTN keys from secrets.example.h"
#include "secrets.example.h"
#endif

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// ================== SENSORS ==================
const int DS18S20_Pin = 16;   // header pin 27, net TempProbe; R19 (4.7k) pulls it up on-board
const int adcPin      = 34;   // header pin 5,  net ADC  - salinity sense, input-only pin
const int outputPin   = 13;   // header pin 15, net G0   - salinity excitation square wave.
                              // Drives U1 (DG636E) A0/A1 select lines + test point TP2.
                              // NOTE: was GPIO2 on the V1 hand-rewired board; GPIO2 is
                              // unconnected on V2, so the old value drove a dead pin.
const int frequency   = 25000;
const int resolution  = 8;

// ================== POWER BUTTON — handled in hardware on V2 ==================
// The V1 bench rig had a push-and-stay switch on GPIO13 and an indicator LED on
// GPIO26, polled by readPowerButton(). Neither exists on V2:
//   - `Button1` drives the net ButtonA -> R_base1 (10k) -> the Q1/Q2 AO3401A
//     power latch. It never reaches a GPIO, so firmware cannot read it.
//   - GPIO26 is unconnected, and GPIO13 is now the excitation net G0.
// readPowerButton() was removed accordingly; the RGB LED below took over the
// indicator job.

// ================== BATTERY & CHARGER MONITORING ==================
// V2 brings the power chain back to the ESP32, all on input-only pins:
//   - BAT is divided by R20/R21 (1M/1M) and filtered by C16 (100n) into GPIO33,
//     so the pin sees half the pack voltage -> multiply the reading by 2.
//   - The MCP73833 charger's open-drain STAT1/STAT2/PG lines land on GPIO35/36/39,
//     each pulled to 3V3 by R17/R15/R16 (10k). All three are active LOW.
const int batSensePin = 33;   // header pin 8,  net Net-(ESP32-GPIO33) - VBAT / 2
const int stat1Pin    = 35;   // header pin 6,  net STAT1 - MCP73833 STAT1
const int stat2Pin    = 36;   // header pin 3,  net STAT2 - MCP73833 STAT2
const int pgPin       = 39;   // header pin 4,  net PG    - MCP73833 power-good

const float BAT_DIVIDER = 2.0f;   // R20/R21 are both 1M -> the pin sees VBAT/2
int batSamples = 64;              // ADC reads averaged per battery measurement

// ---- Cached power state ----
// readBatteryVoltage() averages `batSamples` ADC reads, far too slow to call at
// frame rate, so batteryTick() refreshes these on a timer and everything else
// (display, loop, commands) reads the cache.
float       batVoltage     = 0.0f;         // filtered pack voltage, volts
int         batPercent     = -1;           // 0-100, or -1 before the first sample
// The percentage gets its own EMA on top of the one on batVoltage. Smoothing the
// volts alone is not enough: the curve has knees, and near one a millivolt of
// residual noise steps the percentage by a whole point. Negative = unprimed.
float       batPctEma      = -1.0f;
ChargeState chargeState    = CHG_STANDBY;  // decoded STAT1/STAT2
bool        chargerPresent = false;        // debounced PG: USB-C power is in

// False when the STAT/PG readings cannot be believed - see checkPowerPath().
// Everything that acts on charger state has to consult this first, or a bare
// dev-kit parks itself in charge mode and never takes a reading.
bool        powerPathOk    = true;

// PG debounce: plugging a connector bounces, and PG flapping would otherwise
// throw the whole device in and out of charge mode. Require three agreeing
// 200 ms samples (~600 ms) before believing a change.
static bool pgDebLast  = false;
static int  pgDebAgree = 0;

// ---- Charge mode ----
// True while the charger is plugged in AND we are honouring it: measurements
// are suspended and the OLED shows the charge screen. `pauseOnCharge` is the
// policy behind it, flipped at runtime with the `charge` command — `charge
// ignore` keeps the device measuring with a cable in, which is what you want
// when developing at a desk.
bool chargeModeActive = false;
bool pauseOnCharge    = true;

// Single-cell LiPo terminal voltage -> state of charge. The discharge curve is
// almost flat between 3.7 and 3.9 V, so a straight line from 3.3 to 4.2 V would
// read badly wrong for most of the pack's life; this is the usual light-load
// curve, interpolated between points. Tens of mA is what this board draws, so
// the sag correction is left out on purpose.
struct BatCurvePoint { float v; uint8_t pct; };
static const BatCurvePoint BAT_CURVE[] = {
  {4.20f, 100}, {4.10f, 90}, {4.00f, 80}, {3.94f, 70}, {3.87f, 60},
  {3.83f,  50}, {3.80f, 40}, {3.77f, 30}, {3.73f, 20}, {3.66f, 10},
  {3.60f,   5}, {3.30f,  0}
};

// ================== RGB STATUS LED ==================
/*
 *  LED1 on the Salinity Device V2 board is a COMMON-ANODE RGB LED (KiCad
 *  symbol Device:LED_BRAG). The shared anode sits on VSYS_SW and each cathode
 *  returns to a GPIO through its own 150 R resistor, so a channel lights when
 *  its pin is pulled LOW -> the LED is ACTIVE LOW.
 *
 *      net    GPIO   header pin   resistor   LED1 pad   colour
 *      RGB1    17        28         R24         1        blue
 *      RGB2     4        26         R23         4        RED
 *      RGB3    15        23         R22         2        GREEN
 *
 *  The colours above were confirmed by driving one channel at a time and
 *  looking at the board (2026-09-18). Do NOT trust the KiCad net names here:
 *  the schematic uses symbol Device:LED_BRAG, whose pin order is
 *  Blue/Red/Anode/Green, so the nets came out named RK/GK. The part actually
 *  fitted is Blue/GREEN/Anode/RED - pads 2 and 4 are swapped versus the
 *  symbol. Hence RGB3 ("RK") is the green die and RGB2 ("GK") is the red one.
 *
 *  Each channel gets its own LEDC PWM channel (independent of the 25 kHz
 *  salinity excitation on `outputPin`) so brightness, colour mixing and the
 *  breathing patterns work. If LEDC refuses a pin we fall back to plain on/off
 *  via digitalWrite: the LED still reports state, just without shading. Set
 *  RGB_ENABLED to 0 to leave all three pins completely untouched.
 *
 *  Boot note: GPIO15 idles HIGH at reset (internal pull-up) = green off, but
 *  GPIO4 idles LOW = red lit for the few ms before rgbInit() runs. Harmless.
 *
 *  Blue is weak by construction: the blue die's forward voltage (~3.0 V) sits
 *  almost at the VSYS_SW rail, so the 150 R R24 can only pass a fraction of a
 *  milliamp. It glows, but faintly, and no firmware gain can fix it - see
 *  RGB_GAIN_B below.
 */
#define RGB_ENABLED       1    // 0 = don't touch GPIO4/15/17 at all
#define RGB_COMMON_ANODE  1    // 1 = LOW lights a channel (this board's wiring)

#if RGB_COMMON_ANODE
  #define RGB_OFF_LEVEL HIGH
#else
  #define RGB_OFF_LEVEL LOW
#endif

const int rgbPinR = 4;    // net RGB2, header pin 26, via R23, LED1 pad 4
const int rgbPinG = 15;   // net RGB3, header pin 23, via R22, LED1 pad 2
const int rgbPinB = 17;   // net RGB1, header pin 28, via R24, LED1 pad 1

const int RGB_PWM_FREQ = 1000;   // 1 kHz: flicker-free and well clear of the excitation
const int RGB_PWM_BITS = 8;      // 0-255 duty, same scale as the colour values

// Signalling timebase. The heartbeat blips are deliberately short: in measuring
// mode the LED is dark ~98 % of the time, which makes a fault code stand out by
// contrast and costs almost nothing on battery. The fault flashes are longer
// because they have to be counted, not just noticed.
const unsigned long RGB_BLIP_ON_MS   = 60;    // length of one heartbeat pulse
const unsigned long RGB_BLIP_SLOT_MS = 220;   // pulse-to-pulse spacing in a train
const unsigned long RGB_BEAT_MS      = 3000;  // heartbeat period in measuring mode
const unsigned long RGB_FLASH_ON_MS  = 160;   // one flash of a fault code
const unsigned long RGB_FLASH_OFF_MS = 220;   // gap between flashes of one code
const unsigned long RGB_CODE_GAP_MS  = 1300;  // silence between code repetitions

// On battery the LED runs at this fraction of rgbBrightness. Nobody is watching
// the board in the field and the indicator must not be a measurable part of the
// power budget; plug USB-C in and it comes back to full by itself.
const uint8_t RGB_BATTERY_DIM = 90;   // 90/255 ~ 35 %

// Per-channel trim applied before the master brightness. All three resistors
// are 150 R, but the red die has the lowest forward voltage (so it draws the
// most current) and the eye is most sensitive to red: without this, "white"
// comes out pink. Adjust by eye against the real LED.
//
// NOTE (2026-09-18): until the rgbPinR/rgbPinG swap above was found, GAIN_R
// was being applied to the green die and GAIN_G to the red one, so these
// numbers were tuned against the wrong channels. They now sit on the dies
// their comment always described, but re-tune by eye if white looks off.
// Blue stays at maximum: R24 already starves it (see the header note), so
// there is nothing to trim back.
const uint8_t RGB_GAIN_R = 140;
const uint8_t RGB_GAIN_G = 255;
const uint8_t RGB_GAIN_B = 255;

uint8_t rgbBrightness = 180;    // master 0-255, changed with `rgb bright <N>`
bool    rgbHwPwm      = false;  // true once all three LEDC attaches succeeded
bool    rgbEnabled    = true;   // `rgb off` blanks the LED, status logic keeps running
bool    rgbManual     = false;  // true while `rgb <R> <G> <B>` holds a fixed colour

RgbStatus rgbStatus = RGB_BOOT_EARLY;  // enum declared up with the other sketch enums

unsigned long rgbPipMs     = 0;   // millis() of the last clean reading (double blip)
uint32_t      rgbCodeBit   = 0;   // fault bit currently being flashed out
unsigned long rgbCodeStart = 0;   // millis() at the head of the current code cycle

// When true the excitation square wave is latched ON continuously (bench mode)
// so the probe is always driven; excitationOff() becomes a no-op until cleared
// with the `exc off` command. Defaults ON so a reset/reflash keeps the wave
// running (bench measurement). Send `exc off`, or set this back to false, to
// return to per-reading pulsing for low-power field/LoRa operation.
bool excAlwaysOn = true;

OneWire ds(DS18S20_Pin);

const float ADC_REF_VOLTAGE = 3.3f;

// ADC samples averaged into one salinity reading. Runtime-adjustable with the
// `samples <N>` command: a larger N averages away more noise at the cost of a
// longer (blocking) read. At ~400 us/sample, 256 samples ≈ 100 ms, which fits
// comfortably inside the 15 s measuring period. Constrained to [8, 4096].
int       salSamples         = 512;  // default; was 32
const int SAL_SAMPLES_MIN    = 8;
const int SAL_SAMPLES_MAX    = 4096;
const int SAMPLE_SETTLE_MS   = 60;   // excitation settle time before a manual sample
const unsigned long DS18B20_CONV_MS = 750;  // DS18B20 12-bit conversion time

// ================== TEMPERATURE COMPENSATION ==================
/*  The conductivity of natural water moves about 2 % per degC. Without
 *  correcting for it, the same water measured at 22 degC in the morning and
 *  35 degC in the afternoon reads ~25 % apart - which is larger than most of the
 *  differences this instrument exists to detect, and it makes readings from
 *  different sites, seasons or times of day incomparable.
 *
 *  The standard fix is to report everything normalised to 25 degC:
 *
 *      S25 = S_measured / (1 + alpha * (T - 25))
 *
 *  alpha = 0.02 /degC is the usual value for natural waters and is what the
 *  literature means by "linear temperature correction". It is runtime-adjustable
 *  (`tempco <alpha>`) because the true coefficient depends on the ionic makeup
 *  of the water: it is near 0.019 for a NaCl-dominated estuary and closer to
 *  0.023 for waters heavy in carbonates. A device deployed on one river can be
 *  trimmed to it.
 *
 *  The same correction runs in reverse during calibration - see calRecord().
 *  Standards are specified at 25 degC, so a point taken in a 30 degC bath has to
 *  be told what the standard WOULD read at 30 degC before it is fitted;
 *  otherwise the bath temperature is silently baked into a and b, and the
 *  correction is then applied on top of a curve that already contains it.
 *
 *  When the DS18B20 is missing the correction cannot be applied at all. The
 *  firmware does NOT quietly fall back to raw: it reports the raw number, marks
 *  the reading as uncompensated in the history/CSV flags, and raises
 *  F_NO_TEMPCOMP (amber 6) so the LED says the data is degraded.
 */
const float TEMP_REF_C       = 25.0f;   // everything is normalised to this
float       tempCoeff        = 0.020f;  // alpha, per degC
bool        tempCompEnabled  = true;    // `tempcomp on|off`
const float TEMPCO_MIN       = 0.000f;
const float TEMPCO_MAX       = 0.050f;

// Below this the correction divisor approaches zero and the result explodes.
//
// Where that sits in degrees is NOT fixed - it moves with alpha, because the
// divisor is 1 + alpha*(T - 25):
//
//     T_cut = 25 - (1 - TEMPCOMP_MIN_DIVISOR) / alpha
//
//     alpha 0.019 -> -14.5 C      alpha 0.030 -> +0.0 C
//     alpha 0.020 -> -12.5 C      alpha 0.040 -> +6.3 C
//     alpha 0.023 ->  -7.6 C      alpha 0.050 -> +10.0 C   (TEMPCO_MAX)
//
// So at the default alpha this only ever trips on a garbage reading, but a
// device trimmed hard toward the carbonate end and deployed on a cold river CAN
// be pushed into the guard by perfectly real water - and the linear model really
// is degenerate there, so widening the floor would not help: at alpha=0.05 and
// T=0 the divisor is already negative and the "correction" would flip the sign.
// The guard is therefore kept, and instead made audible: tempCompCutoffC() below
// reports T_cut, `tempco`/`tempcomp` print it so the operator sees it at the
// moment they choose alpha, and salinityAt25() logs the first reading it
// suppresses rather than letting amber 6 stand there unexplained.
const float TEMPCOMP_MIN_DIVISOR = 0.25f;

// Coldest water this alpha can still correct. Returns a large negative number
// when alpha is 0, where the divisor is a constant 1 and the guard is unreachable.
float tempCompCutoffC() {
  if (tempCoeff <= 1e-6f) return -1000.0f;
  return TEMP_REF_C - (1.0f - TEMPCOMP_MIN_DIVISOR) / tempCoeff;
}

// Edge state for the "the guard, not the sensor, suppressed the correction" log.
// salinityAt25() runs ~4x/second in field mode, so this has to be edge-triggered.
bool tempCompGuardTripped = false;

// ================== FIELD MODE ==================
/*  The operator-present instrument. The probe goes in the water, the number on
 *  the glass settles, the firmware decides when it has settled, averages the
 *  stable window and commits one observation. No button is involved because V2
 *  has none to involve: `Button1` drives the AO3401A power latch directly and
 *  never reaches a GPIO (see the POWER BUTTON note below), so every gesture in
 *  this mode has to be inferred from the probe itself.
 *
 *  Immersion is inferred from the probe voltage. Air is effectively an open
 *  circuit, so the divider sits near 0 V; any real water pulls it up well clear.
 *  Worth stating the limit plainly: a DISCONNECTED probe and a probe in air are
 *  the same open circuit, so this cannot distinguish them - which is why field
 *  mode does not raise the "probe dead" fault on a dry reading (see
 *  fieldSampleSalinity).
 *
 *  Stability is a rolling standard deviation over STAB_WINDOW samples. The band
 *  is `max(STAB_ABS_G_L, mean * STAB_REL)`: a fixed band alone is too tight at
 *  35 g/L seawater and a relative one alone never converges near zero.
 */
// (the FieldState enum itself is declared up with the other sketch enums - it is
// named in fieldStateName()'s signature, so it has to precede the generated
// prototypes; see the NOTE beside WorkMode.)
FieldState fieldState = FIELD_DRY;

// How often a salinity sample is taken in field mode. 250 ms is fast enough to
// read as "live" and leaves the DS18B20 chain, the display and the web server
// plenty of room. `fieldSalSamples` is deliberately smaller than `salSamples`:
// at 400 us/sample, 512 would blow the whole budget on one read.
const unsigned long FIELD_SAMPLE_MS = 250;
int       fieldSalSamples      = 128;   // ~51 ms per live sample

// Rolling stability window. 20 samples at 250 ms is a 5 s view, which is long
// enough to see a drift and short enough that the operator is not kept waiting.
const int STAB_WINDOW = 20;
float     stabBuf[STAB_WINDOW];    // compensated salinity
float     stabVolt[STAB_WINDOW];   // probe voltage that produced it
float     stabTemp[STAB_WINDOW];   // temperature at the time
// Per-sample provenance, same bit layout as FieldRecord::flags (bit0 temp valid,
// bit2 compensation applied). The committed row's flags are the AND of these
// across the window, not a snapshot of the live state at the instant of the
// commit - a probe whose DS18B20 drops out on the very last sample must not mark
// twenty compensated samples raw, and one that recovers on the last sample must
// not mark nineteen raw samples compensated.
uint8_t   stabFlag[STAB_WINDOW];
int       stabCount = 0;           // samples in the window, saturates
int       stabHead  = 0;
float     stabMean  = 0.0f;        // recomputed on every push
float     stabSd    = 0.0f;

// The stability band, and how long the reading has to stay inside it.
float     stabAbs   = 0.05f;      // g/L, absolute floor of the band
const float STAB_REL = 0.01f;     // 1 % of the mean, whichever is larger
const unsigned long STABLE_HOLD_MS = 3000;
const float STAB_ABS_MIN = 0.001f;
const float STAB_ABS_MAX = 5.0f;

// Immersion threshold and its debounce. Both directions are debounced: splash
// and turbulence otherwise bounce the state machine on entry, and a momentary
// lift would commit a half-submerged reading on exit.
// This is a VOLTAGE threshold and stays one under either curve - it is asking
// "is there a circuit here at all", which is a question about the divider, not
// about salinity.
//
// It was 0.030 V for a long time, on the assumption that a probe in air is an
// open circuit sitting near 0 V. On the real V2 probe it is not: air reads about
// 0.090 V with the excitation on, so the old default was three times BELOW the
// dry reading and the device called a probe in air "immersed" permanently. In
// field mode that is not a cosmetic bug - the dry reading is rock steady, so it
// sails through the stability window and auto-record commits an observation of
// 0.00 g/L taken in air, indistinguishable in the log from real fresh water.
//
// 0.110 V is picked from that measurement: 20 mV (about 25 ADC counts) above the
// dry reading, which on the shipped quadratic is ~0.25 g/L - weak estuary water
// and up. The window to work in is narrow, because P3, the voltage the curve
// calls zero salinity, is only 9 mV above dry:
//
//     dry 0.090 V | P3 0.099 V | 0.25 g/L 0.110 V | 0.5 g/L 0.121 V
//
// So the tension is real and has no free lunch. Set too high and the device
// silently refuses to do anything at a fresh site, which looks like a dead
// device rather than a misconfigured one. Set too low and it fabricates
// observations out of air, which is worse, because those reach the log. Check
// the dry reading with `adc` on each probe and set this with `immersion <V>`;
// a probe whose dry offset differs from 0.090 V needs a different number here.
float     immersionV = 0.110f;
const float IMMERSION_MIN = 0.005f;
const float IMMERSION_MAX = 1.500f;
const unsigned long IMMERSION_DEBOUNCE_MS = 1200;

// Auto-commit. With it off the operator still sees everything live and the
// stable flag, but nothing is written until `rec` is sent - useful on the bench,
// and the escape hatch if a site's water will not settle.
bool      autoRecord = true;

// Field screen layout. The simple screen is what an operator standing in a river
// sees: one number, the settle bar, and what to do next. The detail screen keeps
// the spread, sample count and probe voltage for bench work. `screen simple` /
// `screen detail`, persisted as NVS key "fscreen".
bool      fieldDetailScreen = false;

unsigned long fieldSampleMs   = 0;   // last live salinity sample
unsigned long fieldStableMs   = 0;   // when the reading entered the band
unsigned long fieldRecMs      = 0;   // when the last observation was committed
unsigned long immersionEdgeMs = 0;   // when the immersion reading last flipped
bool      immersedRaw   = false;     // undebounced
bool      immersed      = false;     // debounced - what the state machine uses

// Live values, updated every FIELD_SAMPLE_MS rather than once per 15 s.
float     liveSalt    = 0.0f;   // compensated
float     liveSaltRaw = 0.0f;
float     liveVolts   = 0.0f;
bool      liveValid   = false;
// Whether the compensation was actually APPLIED to liveSalt, as opposed to
// merely being enabled. These differ whenever the DS18B20 is unreadable or the
// temperature falls outside what the current alpha can correct (see
// TEMPCOMP_MIN_DIVISOR), and conflating them is what let a raw number be
// recorded, served and uplinked wearing an "@25" label.
bool      liveComp    = false;

// Temperature in field mode is a free-running EMA fed by a permanently chained
// DS18B20 conversion, not a per-reading average: the probe's thermal mass means
// the water temperature takes tens of seconds to settle after a dip anyway, so
// a slow filter is closer to the truth than a 4-sample mean taken at an
// arbitrary moment.
const float TEMP_EMA_ALPHA = 0.25f;
float     liveTemp    = 0.0f;
bool      liveTempOk  = false;
unsigned long fieldTempMs = 0;
bool      fieldTempPending = false;

// DS18B20 ROM address, enumerated once at boot and reused (no per-read search).
byte tempAddr[8];
bool tempAddrValid = false;

// ================== LoRaWAN ==================
static osjob_t sendjob;

const uint64_t SLEEP_INTERVAL_US = 1ULL * 60ULL * 1000000ULL;

const unsigned long AWAKE_WINDOW_MS = 5UL * 60UL * 1000UL;

const unsigned long SEND_PERIOD_MS  = 15UL * 1000UL;   // one reading every 15 s

// When true the device may deep-sleep between awake windows (low-power field
// operation). When false it stays "always on" and keeps measuring every
// SEND_PERIOD_MS forever — the lab/bench default. Toggle at runtime with
// `sleep on` / `sleep off`. Lives in RAM, so a reset returns to this default.
bool sleepEnabled = false;

float temperature = 0.0f;
float salinity    = 0.0f;   // normalised to TEMP_REF_C
float salinityRaw = 0.0f;   // straight off the curve, before normalisation
float lastVoltage = 0.0f;   // last raw probe voltage, reported by the web console

// Uplink payload. The first four bytes are unchanged from the original format -
// temperature and salinity, each a big-endian signed int16 of the value x10 - so
// an existing TTN decoder keeps working untouched. Field mode appends three
// more: the standard deviation of the committed window (x100, unsigned) and the
// sample count, which is what turns a number into a number with a confidence
// attached. A decoder that only reads the first four bytes simply ignores them.
//
//   [0..1] temperature degC x10, int16 BE   (0x7FFF = no data)
//   [2..3] salinity g/L x10 at 25 degC, int16 BE (0x7FFF = no data)
//   [4..5] sd of the window, g/L x100, uint16 BE   (field mode only; 0 in logger)
//   [6]    salinity samples averaged               (field mode only; 0 in logger)
//
// "0 in logger" is part of the contract, not an accident: logger mode reads the
// ADC once, so it has neither a spread nor a sample count, and a decoder that
// sees n = 0 should read the pair as "not measured" rather than as a perfectly
// repeatable single sample.
static uint8_t mydata[7];

// Value packed in the payload when a reading is invalid (probe fault, sensor
// missing, CRC error). Decoder must treat 0x7FFF as "no data".
const int16_t ERR_SENTINEL = 0x7FFF;

// Downlink command byte (first payload byte) that forces recalibration.
const uint8_t DL_RECAL = 0xC1;

// V2 net -> GPIO, read off the fabricated PCB. The V1 values (rst 14, dio
// {26, 33, 32}) were wrong on every entry but NSS: 14 is DIO2 here, 26 is
// unconnected, and 33 is the battery divider — attaching a LoRa interrupt to
// that would have fought R20/R21/C16.
const lmic_pinmap lmic_pins = {
  .nss  = 5,                    // header pin 29, net NSS
  .rxtx = LMIC_UNUSED_PIN,
  .rst  = 27,                   // header pin 11, net RESET
  .dio  = {32, 25, 14}          // header pins 7/9/12, nets DIO0/DIO1/DIO2
};

unsigned long awakeStartMs = 0;
unsigned long lastSendMs   = 0;

// ---- display state: do we have a valid reading to show yet? ----
bool haveMeasurement = false;

// ---- non-blocking measurement state machine ----
// One reading averages `tempSamples` DS18B20 conversions: the machine chains
// them one after another (each ~750 ms), accumulating the results, then reads
// the salinity ADC once and finishes. Runtime-adjustable with `tsamples <N>`.
// The excitation stays driven across the whole chain, so the probe is well
// settled by the time the salinity ADC is sampled. Max keeps 6*750 ms < 15 s.
enum MeasState { MEAS_IDLE, MEAS_CONVERTING };
MeasState measState = MEAS_IDLE;
unsigned long measStartMs = 0;
// millis() when a reading last COMPLETED, whatever the sensors said. Feeds the
// F_STALLED watchdog, which is about the loop cycling at all - a different
// failure from either sensor going quiet, and one no per-sensor bit can see.
unsigned long lastMeasOkMs = 0;

int       tempSamples      = 4;    // DS18B20 conversions averaged per reading
const int TEMP_SAMPLES_MIN = 1;
const int TEMP_SAMPLES_MAX = 6;

double tempAccum    = 0.0;   // sum of valid conversions in the current reading
int    tempOkCount  = 0;     // how many conversions returned a valid value
int    tempConvDone = 0;     // how many conversions have been attempted

// ================== AUTO-CALIBRATION ==================
// Collected (voltage, known-salinity) reference points and the fit status.
const int MAX_CAL_POINTS = 8;
float calV[MAX_CAL_POINTS];
float calS[MAX_CAL_POINTS];   // what the standard reads AT calT - see calRecord()
float calSRef[MAX_CAL_POINTS];// the standard's nominal value at 25 degC, as typed
float calT[MAX_CAL_POINTS];   // bath temperature when the point was taken
bool  calTOk[MAX_CAL_POINTS]; // false if the DS18B20 was unreadable then
int   calCount      = 0;
bool  autoCalActive = false;
float lastFitR2     = -1.0f;              // <0 means "no fit yet"
// 40, not 26: the "pt%d V=%.3f S=%.1f" status has no bound on S beyond "> 0",
// so a fat-fingered `sample 100000` truncated the line on the OLED.
char  autoCalMsg[40] = "send sample <S>"; // short status shown on the OLED

// ================== BATTERY & CHARGER FUNCTIONS ==================
// The MCP73833 reports through two open-drain pins, both active LOW, pulled up
// to 3V3 on the board. The pair encodes four states:
//
//     STAT1  STAT2   meaning
//       L      H     charge in progress (preconditioning or fast charge)
//       H      L     charge complete
//       L      L     fault (timer, thermal, or bad cell)
//       H      H     standby / shutdown / no battery present
//
// PG is separate and also active LOW: asserted while valid input power is there.

void batteryInit() {
  // GPIO33-39 are input-only and have no internal pull-ups; R15/R16/R17 supply
  // them externally, so these calls are just making the intent explicit.
  pinMode(batSensePin, INPUT);
  pinMode(stat1Pin,    INPUT);
  pinMode(stat2Pin,    INPUT);
  pinMode(pgPin,       INPUT);
}

// Pack voltage in volts. Uses analogReadMilliVolts() rather than a raw count so
// the ESP32's per-chip eFuse ADC calibration is applied — the raw path is good
// enough for the ratiometric salinity reading, but a battery gauge wants the
// absolute value to be right.
float readBatteryVoltageRaw() {
  uint32_t sum = 0;
  for (int i = 0; i < batSamples; i++) sum += analogReadMilliVolts(batSensePin);
  return ((float)sum / batSamples) * BAT_DIVIDER / 1000.0f;
}

// What the rest of the firmware should use. BAT_DIVIDER is nominal - it assumes
// R20/R21 are exactly equal, and at 1 % each they are not - so the per-device
// gain/offset from battery_cal.h is applied on top. Uncalibrated boards get
// 1.0/0.0 and behave exactly as before. See `CALSINGLE` in COMMANDS.md.
float readBatteryVoltage() {
  return batCalApply(readBatteryVoltageRaw());
}

ChargeState readChargeState() {
  bool s1 = (digitalRead(stat1Pin) == LOW);   // LOW = asserted
  bool s2 = (digitalRead(stat2Pin) == LOW);
  if      ( s1 && !s2) return CHG_CHARGING;
  else if (!s1 &&  s2) return CHG_COMPLETE;
  else if ( s1 &&  s2) return CHG_FAULT;
  return CHG_STANDBY;
}

bool inputPowerGood() { return digitalRead(pgPin) == LOW; }

const char* chargeStateName(ChargeState s) {
  switch (s) {
    case CHG_CHARGING: return "charging";
    case CHG_COMPLETE: return "charge complete";
    case CHG_FAULT:    return "FAULT";
    // Both STAT pins high just means "not charging", and that covers two very
    // different situations. With no input power the MCP73833 is in shutdown and
    // has simply released its open-drain pins - reporting "no battery" there is
    // misleading when the pack is sitting at 3.9 V. Only when power IS present
    // does standby actually say something about the cell.
    default:           return inputPowerGood() ? "standby / no battery"
                                               : "idle (no input power)";
  }
}

// Short form for the 128 px OLED, where the long names do not fit.
const char* chargeStateShort(ChargeState s) {
  switch (s) {
    case CHG_CHARGING: return "charging";
    case CHG_COMPLETE: return "full";
    case CHG_FAULT:    return "FAULT";
    default:           return inputPowerGood() ? "no batt" : "idle";
  }
}

// What the gauge should SAY, as opposed to what it thinks the percentage is.
// With a cable in, the divider is across the charger output, not the resting
// cell: constant-voltage holds ~4.2 V for the whole back half of the charge, so
// any percentage shown there is 50-100 mV of charge current pretending to be
// state of charge. The MCP73833's own STAT pins know better than the curve does,
// so while power is present they win and the number is suppressed entirely.
// powerPathOk gates it: on a bare DevKitC all three STAT/PG pins float and would
// otherwise park the header on a permanent phantom "ERR".
void batteryLabel(char* buf, size_t n) {
  if (powerPathOk && chargerPresent) {
    switch (chargeState) {
      case CHG_FAULT:    snprintf(buf, n, "ERR");  return;
      case CHG_COMPLETE: snprintf(buf, n, "FULL"); return;
      case CHG_CHARGING: snprintf(buf, n, "CHG");  return;
      default: break;                              // standby: fall through to %
    }
  }
  if (batPercent < 0) snprintf(buf, n, "--");
  else                snprintf(buf, n, "%d%%", batPercent);
}

// Interpolate the discharge curve above. Clamped at both ends.
int batteryPercentFromVoltage(float v) {
  const int n = (int)(sizeof(BAT_CURVE) / sizeof(BAT_CURVE[0]));
  if (v >= BAT_CURVE[0].v)     return 100;
  if (v <= BAT_CURVE[n - 1].v) return 0;
  for (int i = 1; i < n; i++) {
    if (v >= BAT_CURVE[i].v) {
      float span = BAT_CURVE[i - 1].v - BAT_CURVE[i].v;
      float f    = (v - BAT_CURVE[i].v) / span;
      return (int)lroundf(BAT_CURVE[i].pct + f * (BAT_CURVE[i - 1].pct - BAT_CURVE[i].pct));
    }
  }
  return 0;
}

// Refresh the cached power state. Called every loop(); does the actual work on
// its own timers (charger pins at 5 Hz, pack voltage at 1 Hz).
void batteryTick() {
  unsigned long now = millis();

  static unsigned long lastPinMs = 0;
  if (now - lastPinMs >= 200) {
    lastPinMs   = now;
    chargeState = readChargeState();
    bool pg = inputPowerGood();
    if (pg == pgDebLast) { if (pgDebAgree < 3) pgDebAgree++; }
    else                 { pgDebLast = pg; pgDebAgree = 1; }
    if (pgDebAgree >= 3) chargerPresent = pgDebLast;
    checkPowerPath();
  }

  static unsigned long lastVMs  = 0;
  static bool          vPrimed  = false;
  if (!vPrimed || now - lastVMs >= 1000) {
    lastVMs = now;
    float v = readBatteryVoltage();
    // EMA once primed: 12 bits of ADC noise on a 1M/1M divider is worth a few
    // percent, and a number that jitters while you watch it looks broken.
    batVoltage = vPrimed ? (batVoltage * 0.8f + v * 0.2f) : v;
    vPrimed    = true;
    if (!powerPathOk) {
      batPercent = -1;
      batPctEma  = -1.0f;                      // re-prime cleanly when it returns
    } else {
      float pct  = (float)batteryPercentFromVoltage(batVoltage);
      batPctEma  = (batPctEma < 0.0f) ? pct : (batPctEma * 0.8f + pct * 0.2f);
      batPercent = (int)lroundf(batPctEma);
    }
  }

  // While charging, the divider sees the charger's output, not the resting cell
  // voltage: constant-voltage holds ~4.2 V long before the pack is actually
  // full, so the curve reads optimistically high. Nothing to do about that
  // without a coulomb counter — but "charge complete" is authoritative, so at
  // least pin that case to 100.
  if (chargerPresent && chargeState == CHG_COMPLETE) batPercent = 100;

  // Power-domain fault bits. powerPathOk gates the charger ones: with the module
  // out of the carrier the STAT/PG pins float and would otherwise raise a
  // permanent phantom charger fault. The battery-level ones are suppressed while
  // a cable is in for the same reason batPercent is unreliable there - the
  // divider is reading the charger output, not the resting cell.
  faultSet(F_POWER_PATH,   !powerPathOk);
  faultSet(F_CHARGER,      powerPathOk && chargeState == CHG_FAULT);
  faultSet(F_NO_BATTERY,   powerPathOk && chargerPresent && chargeState == CHG_STANDBY);
  faultSet(F_BAT_CRITICAL, powerPathOk && !chargerPresent &&
                           batVoltage > 0.5f && batVoltage < BAT_CRITICAL_V);
  // Same 0.5 V floor as F_BAT_CRITICAL: below that the divider is not connected
  // at all (a bare DevKitC off the carrier reads ~0.28 V), which is a wiring
  // fact, not a flat cell - an assembled board could not still be running there.
  faultSet(F_BAT_LOW,      powerPathOk && !chargerPresent && batVoltage > 0.5f &&
                           batPercent >= 0 && batPercent < BAT_LOW_PCT);
}

void logPowerPathWarning() {
  Log.println(F("WARNING: power monitoring looks disconnected. PG, STAT1 and STAT2 all"));
  Log.print  (F("         read LOW while BAT sits at "));
  Log.print(batVoltage, 3);
  Log.println(F(" V. Either this ESP32 is not seated"));
  Log.println(F("         in the V2 carrier board, or the pack is shorted."));
  Log.println(F("         Charge mode is disabled so measurements keep running."));
  Log.println(F("         'power' still reports the raw pins either way."));
}

// Decide whether the charger pins mean anything, and force chargerPresent off
// when they do not.
//
// GPIO35/36/39 are input-only and have NO internal pull-ups: the carrier board's
// 10k pull-ups to 3V3 are the only thing holding them high. Run the module on a
// bench outside the V2 carrier and all three float LOW, which decodes as "input
// power present, charger in FAULT" - and that is enough to drop the firmware
// into charge mode permanently, pausing every measurement, on a board that has
// no charger at all.
//
// The tell is that the trio is electrically impossible on a real board: a
// powered MCP73833 holds BAT well above 1 V even into a flat cell, so "power
// present + FAULT + a dead-flat divider" cannot happen when the divider is
// actually connected. A genuinely shorted pack sits in precondition (STAT1 LOW,
// STAT2 HIGH), not FAULT, so it does not trip this either.
void checkPowerPath() {
  bool ok = !(chargerPresent && chargeState == CHG_FAULT && batVoltage < 1.0f);
  if (ok != powerPathOk) {
    powerPathOk = ok;
    if (ok) Log.println(F("[power] charger readings look valid again."));
    else    logPowerPathWarning();
  }
  if (!powerPathOk) chargerPresent = false;
}

// Seed the cache in setup() so the first frame and the first loop() pass see
// real values instead of "--" and a false "no charger".
void batteryPrime() {
  chargeState    = readChargeState();
  chargerPresent = inputPowerGood();
  batVoltage     = readBatteryVoltage();
  checkPowerPath();                  // may clear chargerPresent
  pgDebLast      = chargerPresent;
  pgDebAgree     = 3;
  batPctEma      = powerPathOk ? (float)batteryPercentFromVoltage(batVoltage) : -1.0f;
  batPercent     = powerPathOk ? (int)lroundf(batPctEma) : -1;
}

// ================== FAULT BITMASK FUNCTIONS ==================
// One row per bit: what it means and, implicitly, where to go looking. The order
// here is only for printing - faultCode() derives the blink count from the bit
// position, so a row in the wrong place mislabels the text but never the LED.
struct FaultInfo { uint32_t bit; const char* name; };
static const FaultInfo FAULT_TABLE[] = {
  { F_POWER_PATH,   "power sense dead - ESP32 not seated in the V2 carrier, or pack shorted" },
  { F_SENSORS_DEAD, "no sensor answered - neither temperature nor salinity is readable" },
  { F_CHARGER,      "charger FAULT - MCP73833 timer, thermal, or bad cell" },
  { F_BAT_CRITICAL, "battery critical - pack below 3.40 V, shut down or charge now" },
  { F_STALLED,      "measurement loop stalled - no reading completed in 3 periods" },
  { F_TEMP,         "temperature failing - DS18B20 off the 1-Wire bus, or CRC errors" },
  { F_SALINITY,     "salinity probe reads ~0 V - probe disconnected or excitation dead" },
  { F_OLED,         "OLED did not initialise - check the DiplayConn1 I2C wiring" },
  { F_BAT_LOW,      "battery low - below 20 %" },
  { F_NO_BATTERY,   "input power good but charger in standby - no cell connected?" },
  { F_NO_TEMPCOMP,  "salinity NOT temperature-corrected - no valid temperature to correct with" },
};
const int FAULT_TABLE_N = (int)(sizeof(FAULT_TABLE) / sizeof(FAULT_TABLE[0]));

// Bit position, 0-15, or -1 for anything that is not a single bit.
static int faultSlotOf(uint32_t bit) {
  for (int i = 0; i < FAULT_SLOTS; i++) if (bit == (1UL << i)) return i;
  return -1;
}

// 'R' for the hard class, 'A' for the degraded class. This is the LED colour.
char faultClass(uint32_t bit) { return (bit & FAULT_RED_MASK) ? 'R' : 'A'; }

// How many times the LED flashes for this fault: position within its own byte,
// plus one. Counting past five across a room is optimistic, which is why each
// class is capped at eight bits and only five are used so far.
int faultCode(uint32_t bit) {
  int slot = faultSlotOf(bit);
  return (slot < 0) ? 0 : (slot % 8) + 1;
}

const char* faultName(uint32_t bit) {
  for (int i = 0; i < FAULT_TABLE_N; i++) {
    if (FAULT_TABLE[i].bit == bit) return FAULT_TABLE[i].name;
  }
  return "unknown fault bit";
}

// Raise or clear one condition. Only edges are logged, so a fault that stays up
// does not flood the 4 KB log ring - and a clear is announced too, because
// "[fault] cleared" in the log is how you confirm a repair actually worked.
void faultSet(uint32_t bit, bool on) {
  bool was = (faultRaw & bit) != 0;
  if (on == was) return;
  if (on) {
    faultRaw |= bit;
  } else {
    faultRaw &= ~bit;
    int slot = faultSlotOf(bit);
    if (slot >= 0) faultClearMs[slot] = millis();
  }
  Log.print(on ? F("[fault] RAISED  ") : F("[fault] cleared "));
  Log.print(faultClass(bit));
  Log.print(faultCode(bit));
  Log.print(F("  "));
  Log.println(faultName(bit));
}

// Fold the hold windows in to produce the set the LED actually shows. Called
// from loop() just before rgbUpdate().
void faultTick() {
  unsigned long now = millis();
  uint32_t held = 0;
  for (int i = 0; i < FAULT_SLOTS; i++) {
    uint32_t bit = 1UL << i;
    if (faultRaw & bit) continue;
    if (faultClearMs[i] && (now - faultClearMs[i] < FAULT_HOLD_MS)) held |= bit;
  }
  faultLatched = faultRaw | held;
}

// Next set bit after `cur`, wrapping; 0 when `active` is empty. This is what
// makes several simultaneous faults all visible: the renderer advances this
// cursor once per blink cycle instead of pinning the most severe one forever.
uint32_t faultNext(uint32_t active, uint32_t cur) {
  if (!active) return 0;
  int start = faultSlotOf(cur);
  for (int n = 1; n <= FAULT_SLOTS; n++) {
    uint32_t bit = 1UL << ((start + n) % FAULT_SLOTS);
    if (active & bit) return bit;
  }
  return 0;
}

// ================== RGB STATUS LED FUNCTIONS ==================
// Scale a 0-255 value by a 0-255 factor, rounding to nearest.
static uint8_t rgbScale8(uint8_t v, uint8_t s) {
  return (uint8_t)(((uint16_t)v * (uint16_t)s + 127) / 255);
}

// Cheap gamma ~2.0. LED brightness is linear in duty but the eye is not, so
// without this the breathing patterns spend most of their travel looking
// "fully on" and the low end is invisible.
static uint8_t rgbGamma8(uint8_t v) {
  return (uint8_t)(((uint16_t)v * (uint16_t)v + 127) / 255);
}

// Push one channel out, applying gamma and the common-anode inversion.
static void rgbWriteChannel(int pin, uint8_t level) {
  uint8_t duty = rgbGamma8(level);
#if RGB_COMMON_ANODE
  duty = 255 - duty;
#endif
  if (rgbHwPwm) {
    ledcWrite(pin, duty);
  } else {
    // No PWM available: treat anything past half as "on".
    bool on = (level >= 128);
#if RGB_COMMON_ANODE
    digitalWrite(pin, on ? LOW : HIGH);
#else
    digitalWrite(pin, on ? HIGH : LOW);
#endif
  }
}

// Render an RGB triplet: per-channel trim, then master brightness, then output.
void rgbWrite(uint8_t r, uint8_t g, uint8_t b) {
#if RGB_ENABLED
  if (!rgbEnabled) { r = g = b = 0; }
  // Auto-dim on battery. chargerPresent is the debounced PG line, so this
  // follows the cable in and out on its own. The boot stages are exempt: they
  // run before batteryPrime() has read PG, and a dim boot colour is exactly the
  // one you need to be able to see.
  // `rgb <R> <G> <B>` and `rgb test` are exempt too: a wiring check has to be at
  // the brightness you asked for, not at the field setting.
  bool fullBright = chargerPresent || rgbManual || (rgbStatus <= RGB_BOOT_DONE);
  uint8_t master = fullBright ? rgbBrightness
                              : rgbScale8(rgbBrightness, RGB_BATTERY_DIM);
  rgbWriteChannel(rgbPinR, rgbScale8(rgbScale8(r, RGB_GAIN_R), master));
  rgbWriteChannel(rgbPinG, rgbScale8(rgbScale8(g, RGB_GAIN_G), master));
  rgbWriteChannel(rgbPinB, rgbScale8(rgbScale8(b, RGB_GAIN_B), master));
#endif
}

void rgbAllOff() {
#if RGB_ENABLED
  rgbWriteChannel(rgbPinR, 0);
  rgbWriteChannel(rgbPinG, 0);
  rgbWriteChannel(rgbPinB, 0);
#endif
}

// Triangle wave 0..255 across `periodMs`, lifted off the floor so a breathing
// colour never fully disappears.
static uint8_t rgbBreathe(unsigned long now, unsigned long periodMs, uint8_t floorLvl) {
  unsigned long half = periodMs / 2;
  unsigned long ph   = now % periodMs;
  uint32_t tri = (ph < half) ? (ph * 255UL / half)
                             : ((periodMs - ph) * 255UL / half);
  return floorLvl + (uint8_t)((uint32_t)(255 - floorLvl) * tri / 255UL);
}

// Short flashes at the head of a window: `pulses` pulses of RGB_BLIP_ON_MS,
// spaced RGB_BLIP_SLOT_MS apart, then dark for the rest. `ph` is the offset into
// that window.
static bool rgbBlipTrain(unsigned long ph, int pulses) {
  if (ph >= (unsigned long)pulses * RGB_BLIP_SLOT_MS) return false;
  return (ph % RGB_BLIP_SLOT_MS) < RGB_BLIP_ON_MS;
}

// Flash out the current fault code, advancing the cursor once per cycle so that
// every latched fault gets its turn. Returns false when nothing is latched and
// the caller should fall through to the normal status pattern.
//
// Red = the device is not producing trustworthy data, amber = degraded but
// running; the flash count is the fault within that class. Colour and count are
// used instead of hue alone because R24 starves the blue die on this board (see
// the header note), so only red, amber, green and off render reliably.
static bool rgbRenderFault(unsigned long now, uint8_t& r, uint8_t& g, uint8_t& b) {
  uint32_t active = faultLatched;
  if (!active) { rgbCodeBit = 0; return false; }

  // Move on when this code's cycle has run out, when the fault it belongs to has
  // cleared, or on the first fault after a quiet period (rgbCodeBit == 0, which
  // never matches `active` and so falls into the same branch).
  unsigned long cycle = (unsigned long)faultCode(rgbCodeBit)
                      * (RGB_FLASH_ON_MS + RGB_FLASH_OFF_MS) + RGB_CODE_GAP_MS;
  if (!(active & rgbCodeBit) || now - rgbCodeStart >= cycle) {
    rgbCodeBit   = faultNext(active, rgbCodeBit);
    rgbCodeStart = now;
  }

  unsigned long slot  = RGB_FLASH_ON_MS + RGB_FLASH_OFF_MS;
  unsigned long train = (unsigned long)faultCode(rgbCodeBit) * slot;
  unsigned long ph    = now - rgbCodeStart;
  if (ph < train && (ph % slot) < RGB_FLASH_ON_MS) {
    r = 255;
    if (faultClass(rgbCodeBit) == 'A') g = 110;   // amber; the red class stays pure red
  }
  return true;
}

// Point the LED at a new state. Always recorded, even while a manual colour is
// held or the LED is blanked, so `rgb auto` / `rgb on` resume at the colour the
// firmware is actually in rather than a stale one.
void rgbSetStatus(RgbStatus s) {
  rgbStatus = s;
}

// The signalling vocabulary. Four MOTIONS carry the category and colour only
// narrows it, because the blue die is starved by R24 on this board and hue alone
// cannot be trusted - red, amber, green and off are the only reliable states:
//
//    MOTION             CATEGORY
//    short blip         healthy and running
//    breathe            waiting for a human
//    counted flashes    a fault code: colour is the class, count is the fault
//    solid              a boot stage (or hung in one)
//
//    green blip / 3 s        measuring, healthy; doubles on the tick a reading lands
//    amber blip / 2 s        charging
//    green blip / 2 s        charge complete
//    white breathe, slow     calibration mode, nobody attached
//    white breathe, fast     calibration mode, Bluetooth client attached
//    white double pulse      auto-calibration collecting points
//    solid red/amber/green   setup() stage 1/2/3 - if it stops, that is where it hung
//    N red flashes + gap     hard fault, code N    - `diag` names it
//    N amber flashes + gap   degraded, code N      - `diag` names it
//    off                     deep sleep, or `rgb off`
//
// Field mode adds five more, and they are the whole user interface: V2 has no
// button, no buzzer and the operator is looking at water, not at the OLED. The
// sequence green-breathe -> amber-quickening -> green-fast -> one long green ->
// green-double is meant to be followed without reading anything:
//
//    green breathe, slow     probe is dry - dip it
//    amber blips, quickening  settling; the rate IS the convergence
//    green blips, fast       stable, holding to be sure
//    one long green flash    committed - that dip is recorded
//    green double / 4 s      done here, lift the probe out
//
// Called from loop(); throttled to ~50 Hz so the patterns stay smooth without
// hammering LEDC on every iteration.
void rgbUpdate() {
#if RGB_ENABLED
  if (rgbManual) return;                  // a manual colour owns the LED

  static unsigned long lastRenderMs = 0;
  unsigned long now = millis();
  if (now - lastRenderMs < 20) return;
  lastRenderMs = now;

  uint8_t r = 0, g = 0, b = 0;

  // Faults outrank every running state. The two exceptions are the boot stages,
  // where the fault set has not been populated yet and the stage colour is the
  // only thing that can tell you where setup() hung, and sleep, where the LED is
  // off on purpose.
  bool booting = (rgbStatus <= RGB_BOOT_DONE);
  if (!booting && rgbStatus != RGB_SLEEP && rgbRenderFault(now, r, g, b)) {
    rgbWrite(r, g, b);
    return;
  }

  switch (rgbStatus) {
    case RGB_BOOT_EARLY:  r = 255; break;
    case RGB_BOOT_IO:     r = 255; g = 110; break;
    case RGB_BOOT_DONE:   g = 255; break;
    case RGB_CAL_IDLE:   { uint8_t l = rgbBreathe(now, 4000, 10);
                           r = l; g = l; b = l; } break;
    case RGB_CAL_LINKED: { uint8_t l = rgbBreathe(now, 1200, 30);
                           r = l; g = l; b = l; } break;
    case RGB_AUTOCAL:     if (rgbBlipTrain(now % 900, 2)) { r = g = b = 255; } break;
    // Healthy: one short green pulse every RGB_BEAT_MS says the MCU is alive,
    // two in quick succession say a clean reading just landed. That makes the
    // measurement cadence readable from across the room without the LED being
    // lit continuously - and nothing at all means the loop is wedged, which
    // F_STALLED will be saying in red by then anyway.
    case RGB_RUN:        { unsigned long since = now - rgbPipMs;
                           bool lit = (rgbPipMs && since < 2 * RGB_BLIP_SLOT_MS)
                                        ? rgbBlipTrain(since, 2)
                                        : rgbBlipTrain(now % RGB_BEAT_MS, 1);
                           if (lit) g = 255; } break;

    // ---- Field mode. Read as a sequence, not as five separate colours:
    // slow green breathe -> quickening amber -> fast green -> one long green ->
    // slow green double. The operator is looking at water, so the MOTION has to
    // carry it; only red/amber/green/off render reliably on this board.
    case RGB_FIELD_DRY:  { uint8_t l = rgbBreathe(now, 3600, 8);
                           g = l; } break;

    // The blip rate IS the convergence: ~1.2 s apart when the reading is all
    // over the place, tightening to ~0.26 s as it approaches the band. You can
    // watch it close in without reading anything.
    case RGB_FIELD_SETTLE: {
                           float p = stabProgress();
                           unsigned long per = (unsigned long)(1200.0f - 940.0f * p);
                           if (per < 260) per = 260;
                           if (rgbBlipTrain(now % per, 1)) { r = 255; g = 110; }
                         } break;

    case RGB_FIELD_STABLE: if (rgbBlipTrain(now % 300, 1)) { g = 255; } break;

    // One long flash, not a train: nothing else in the vocabulary looks like it,
    // so "it took the reading" cannot be confused with anything.
    case RGB_FIELD_REC:   if (now - fieldRecMs < 800) g = 255; break;

    case RGB_FIELD_LOCK:  if (rgbBlipTrain(now % 4000, 2)) { g = 255; } break;

    case RGB_CHARGING:    if (rgbBlipTrain(now % 2000, 1)) { r = 255; g = 110; } break;
    case RGB_CHARGED:     if (rgbBlipTrain(now % 2000, 1)) { g = 255; } break;
    case RGB_SLEEP:       break;          // stays off
  }
  rgbWrite(r, g, b);
#endif
}

// Show a boot stage immediately. setup() runs before loop() ever calls
// rgbUpdate(), so the stage colours have to be pushed out by hand - which is
// exactly the point: whichever colour the board is sitting on is the call that
// hung. Normally the whole walk (red, amber, green) is over in a couple of
// seconds and you only notice it as a flicker.
void rgbBootStage(RgbStatus s) {
  rgbSetStatus(s);
  uint8_t r = 0, g = 0, b = 0;
  switch (s) {
    case RGB_BOOT_EARLY: r = 255; break;
    case RGB_BOOT_IO:    r = 255; g = 110; break;
    default:             g = 255; break;
  }
  rgbWrite(r, g, b);
}

// Claim the three pins. Parks them at the OFF level first so nothing flashes
// while LEDC starts up, then attaches one PWM channel per colour.
void rgbInit() {
#if RGB_ENABLED
  pinMode(rgbPinR, OUTPUT); digitalWrite(rgbPinR, RGB_OFF_LEVEL);
  pinMode(rgbPinG, OUTPUT); digitalWrite(rgbPinG, RGB_OFF_LEVEL);
  pinMode(rgbPinB, OUTPUT); digitalWrite(rgbPinB, RGB_OFF_LEVEL);

  // Detach first so a warm restart can always re-attach (same reason as
  // excitationOn(): on ESP32 core 3.x attaching an attached pin just fails).
  ledcDetach(rgbPinR); ledcDetach(rgbPinG); ledcDetach(rgbPinB);
  bool ok = true;
  ok &= ledcAttach(rgbPinR, RGB_PWM_FREQ, RGB_PWM_BITS);
  ok &= ledcAttach(rgbPinG, RGB_PWM_FREQ, RGB_PWM_BITS);
  ok &= ledcAttach(rgbPinB, RGB_PWM_FREQ, RGB_PWM_BITS);
  rgbHwPwm = ok;
  if (!ok) {
    Log.println(F("WARN: RGB LEDC attach failed - falling back to on/off (no dimming)."));
    pinMode(rgbPinR, OUTPUT); pinMode(rgbPinG, OUTPUT); pinMode(rgbPinB, OUTPUT);
  }

  rgbManual = false;
  rgbSetStatus(RGB_BOOT_EARLY);
  rgbWrite(255, 0, 0);       // stage 1: solid red until the peripherals are up
#endif
}

// Blocking wiring check: red, green, blue, then white. If a colour comes out
// wrong here the net/colour mapping in the header comment is wrong for your
// build; if every colour is inverted, flip RGB_COMMON_ANODE.
void rgbSelfTest(Stream& out) {
#if RGB_ENABLED
  bool wasManual  = rgbManual;
  bool wasEnabled = rgbEnabled;
  rgbManual  = true;                      // keep rgbUpdate() out of the way
  rgbEnabled = true;                      // a wiring check must light up even after `rgb off`
  out.println(F("RGB test: red, green, blue, white (400 ms each)..."));
  rgbWrite(255, 0, 0);   delay(400);
  rgbWrite(0, 255, 0);   delay(400);
  rgbWrite(0, 0, 255);   delay(400);
  rgbWrite(255, 255, 255); delay(400);
  rgbAllOff();
  rgbManual  = wasManual;
  rgbEnabled = wasEnabled;
  out.println(F("RGB test done."));
#else
  out.println(F("RGB LED disabled at compile time (RGB_ENABLED 0)."));
#endif
}

// ================== DISPLAY FUNCTIONS ==================
void displayInit() {
  Wire.begin(DISPLAY_SDA, DISPLAY_SCL);
  Wire.setClock(400000);                       // 400 kHz -> smoother refresh
  if (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDR)) {
    Log.println(F("Error init OLED (buffer)"));
    displayOk = false;
    faultSet(F_OLED, true);
    return;
  }
  displayOk = true;
  faultSet(F_OLED, false);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Splash screen: device name + purpose
  display.setTextSize(2);
  display.setCursor(4, 6);
  display.print(F("SALINITY"));
  display.setTextSize(1);
  display.setCursor(4, 30);
  display.print(F("monitoring device"));
  display.setCursor(4, 42);
  display.print(F("TFM - Sebastian"));
  display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
}

// Battery outline with a proportional fill, drawn at any size: the header uses
// an 11x7 one, the charge screen a 62x26 one. `pct` < 0 draws the empty outline
// ("not sampled yet"). When `charging` is true the fill sweeps from the real
// level up to full and back over 2 s, which is the animation that says "a
// charger is plugged in" without needing any words.
void drawBatteryGlyph(int x, int y, int w, int h, int pct, bool charging) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);

  // Terminal nub on the right-hand end, scaled to the glyph.
  int nubH = (h >= 10) ? h / 2 : 3;
  int nubW = (w >= 20) ? 3 : 2;
  display.fillRect(x + w, y + (h - nubH) / 2, nubW, nubH, SSD1306_WHITE);

  if (pct < 0) return;
  int innerW = w - 4, innerH = h - 4;
  if (innerW <= 0 || innerH <= 0) return;

  int level = pct;
  if (charging) {
    unsigned long ph = millis() % 2000;                       // 0..100 triangle
    int sweep = (ph < 1000) ? (int)(ph / 10) : (int)((2000 - ph) / 10);
    level = pct + ((100 - pct) * sweep) / 100;
  }
  int fw = (innerW * level + 50) / 100;
  if (fw > 0) display.fillRect(x + 2, y + 2, fw, innerH, SSD1306_WHITE);
}

// The radio indicator: the usual three-arc fan over a dot, 9x7 so it sits on
// the header line next to the 11x7 battery. One bitmap is cheaper and far
// crisper at this size than Bresenham arcs of radius 2, which come out as
// lopsided blobs. Rows are byte-padded, MSB first, as drawBitmap() expects.
//
//   ..#####..
//   .#.....#.
//   #.......#
//   ...###...
//   ..#...#..
//   ...###...
//   ...###...
const int WIFI_GLYPH_W = 9;
const int WIFI_GLYPH_H = 7;
const unsigned char WIFI_GLYPH[] PROGMEM = {
  0x3E, 0x00,
  0x41, 0x00,
  0x80, 0x80,
  0x1C, 0x00,
  0x22, 0x00,
  0x1C, 0x00,
  0x1C, 0x00,
};

void drawWifiGlyph(int x, int y) {
  display.drawBitmap(x, y, WIFI_GLYPH, WIFI_GLYPH_W, WIFI_GLYPH_H, SSD1306_WHITE);
}

// Draws the shared top bar: title on the left, a small "alive" heartbeat dot
// that pulses ~1 Hz so you can tell the device is running, then the battery
// percentage and glyph hard right. The WiFi fan appears left of the heartbeat
// only while the radio is up, so the header doubles as the answer to "is the
// console reachable right now?".
//
// The battery block owns the right ~50 px of the line, and the WiFi glyph
// another 12 px when it is showing, so titles passed here must stay
// <= 11 characters or they will run into it.
//
// quietPct hides the percentage while the pack is healthy and leaves only the
// glyph: the field screen shows the number only once it is worth acting on.
// Charger words (CHG/FULL/ERR) and an unknown level always show.
void drawHeader(const char* title, bool quietPct) {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(title);

  drawBatteryGlyph(115, 0, 11, 7, batPercent,
                   chargerPresent && chargeState == CHG_CHARGING);

  char pctBuf[6];
  batteryLabel(pctBuf, sizeof(pctBuf));         // "CHG"/"FULL"/"ERR" beat a percentage
  if (quietPct && batPercent >= BAT_LOW_PCT && strchr(pctBuf, '%')) pctBuf[0] = '\0';
  int pctX = 113 - (int)strlen(pctBuf) * 6;     // right-aligned against the glyph
  display.setCursor(pctX, 0);
  display.print(pctBuf);

  int dotCx = pctX - 7;
  bool beat = (millis() / 600) % 2;
  if (beat) display.fillCircle(dotCx, 3, 2, SSD1306_WHITE);
  else      display.drawCircle(dotCx, 3, 2, SSD1306_WHITE);

  // Radio state, right-aligned against the heartbeat with a 3 px gap so it
  // travels with the rest of the block as the battery string changes width.
  // Absent = radio down (policy off, or auto on battery); blinking = a STA
  // join is still in flight; steady = console up and answering.
  if (wifiState == WIFI_READY ||
      (wifiState == WIFI_STA_CONNECTING && (millis() / 400) % 2)) {
    drawWifiGlyph(dotCx - 5 - WIFI_GLYPH_W, 0);
  }

  display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);
}

// An overload rather than a default argument: the Arduino prototype generator
// copies default arguments into its forward declarations, which then clash with
// the definition.
void drawHeader(const char* title) { drawHeader(title, false); }

// Format a value for the OLED into a bounded buffer.
//
// This replaces dtostrf(), which takes no size limit at all: it writes as many
// characters as the value needs and trusts the caller to have guessed right. The
// callers here passed char[12], which holds up to 1e9 at one decimal and only
// 1e8 at two - and the value is not bounded by anything. `a`, `b` and `c` are
// accepted from the manual calibration path with no validation whatsoever
// (`a = command.toFloat()`), and the power branch of voltageToSalinity() is
// a*pow(V,b)+c, so a mistyped coefficient or a degenerate auto-cal fit puts an
// arbitrarily large number on a stack buffer sitting directly below the
// getTextBounds() out parameters. The quadratic branch is bounded at the top by
// its own vertex, but a near-zero P2 with P1 at zero sends it the same way. Clamping is the right answer rather than a bigger buffer: a
// 128 px screen cannot show 1e9 g/L meaningfully anyway, and ">999" tells the
// operator the curve is wrong, which a wrapped or truncated number would not.
void fmtDisplayValue(char* buf, size_t n, float val, int decimals) {
  if (isnan(val))      { snprintf(buf, n, "nan");  return; }
  if (val >  99999.0f) { snprintf(buf, n, ">99999"); return; }
  if (val < -9999.0f)  { snprintf(buf, n, "<-9999"); return; }
  snprintf(buf, n, "%.*f", decimals, (double)val);
}

// Draws one measurement row: a small label on the left and a large,
// right-aligned value with its unit. Uses placeholders when no data yet.
void drawValueRow(int yTop, const char* label, bool haveData,
                  float val, const char* unit, bool degree) {
  // Small label, vertically centred against the large value.
  display.setTextSize(1);
  display.setCursor(0, yTop + 4);
  display.print(label);

  // Build the numeric string (or a placeholder).
  char numbuf[16];
  if (haveData) fmtDisplayValue(numbuf, sizeof(numbuf), val, 1);
  else          strcpy(numbuf, "--.-");

  // Measure the number at size 2 so we can right-align everything.
  display.setTextSize(2);
  int16_t bx, by; uint16_t nw, nh;
  display.getTextBounds(numbuf, 0, 0, &bx, &by, &nw, &nh);

  const int gap    = 2;
  const int degW   = degree ? 5 : 0;
  const int unitW  = (int)strlen(unit) * 6;     // size-1 char width
  int totalW = nw + gap + degW + unitW;
  int xStart = SCREEN_WIDTH - totalW;
  if (xStart < 30) xStart = 30;                 // never collide with the label

  // Large value.
  display.setTextSize(2);
  display.setCursor(xStart, yTop);
  display.print(numbuf);

  int cx = xStart + nw + gap;

  // Degree symbol drawn as a tiny ring (font has none by default).
  if (degree) {
    display.drawCircle(cx + 1, yTop + 2, 1, SSD1306_WHITE);
    cx += degW;
  }

  // Unit in size 1, vertically centred against the size-2 value.
  display.setTextSize(1);
  display.setCursor(cx, yTop + 4);
  display.print(unit);
}

// Main operating screen: header, temperature, salinity and a footer status
// line. This is shown continuously (there is no separate idle animation) so
// the latest reading and the device status are always visible.
// Throttled to ~5 fps; pass force=true to render immediately.
void drawMainScreen(bool haveData, float t, float s, const char* status, bool force) {
  if (!displayOk) return;
  static unsigned long last = 0;
  unsigned long now = millis();
  if (!force && now - last < 200) return;
  last = now;

  display.clearDisplay();
  drawHeader("THE GAMBIA");

  drawValueRow(13, "Temp", haveData, t, "C",   true);
  drawValueRow(31, "Salt", haveData, s, "g/L", false);

  display.drawFastHLine(0, 52, SCREEN_WIDTH, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print(status);

  display.display();
}

// Detail field screen (`screen detail`): everything the stability test is
// judging - spread, sample count, probe voltage - for bench work and for
// diagnosing a site that will not settle. Operators get displayFieldSimple().
void displayFieldDetail(unsigned long now) {
  display.clearDisplay();
  drawHeader("FIELD");

  // Live salinity, large. Dashes until the probe is in something conductive -
  // an air reading is not a measurement and should not look like one.
  char numbuf[16];
  if (liveValid) fmtDisplayValue(numbuf, sizeof(numbuf), liveSalt, 2);
  else           strcpy(numbuf, "--.--");

  display.setTextSize(2);
  int16_t bx, by; uint16_t nw, nh;
  display.getTextBounds(numbuf, 0, 0, &bx, &by, &nw, &nh);
  int xStart = SCREEN_WIDTH - nw - 26;
  if (xStart < 0) xStart = 0;
  display.setCursor(xStart, 13);
  display.print(numbuf);
  display.setTextSize(1);
  display.setCursor(SCREEN_WIDTH - 24, 20);
  display.print(F("g/L"));

  // Temperature and the compensation state. The "@25" is not decoration: it is
  // the difference between a number that can be compared with another site and
  // one that cannot, so when the correction is NOT being applied the screen has
  // to say "raw" rather than silently drop the marker.
  display.setCursor(0, 13);
  if (liveTempOk) { display.print(liveTemp, 1); display.print(F("C")); }
  else            display.print(F("--C"));
  display.setCursor(0, 23);
  // liveComp, not `tempCompEnabled && liveTempOk`: the latter says the
  // correction was ASKED for and a temperature exists, which is not the same as
  // it having been applied. salinityAt25() also declines when the temperature
  // falls outside what the current alpha can correct, and the screen has to
  // follow what actually happened to the number beside it.
  display.print(liveComp ? F("@25") : F("raw"));

  // Settling bar. Empty and moving = still converging; full = inside the band.
  // This is the only thing on the screen that answers "is it done".
  int barY = 34, barW = SCREEN_WIDTH - 2;
  display.drawRect(0, barY, barW, 7, SSD1306_WHITE);
  if (fieldState == FIELD_SETTLING || fieldState == FIELD_STABLE) {
    int fill = (int)(stabProgress() * (barW - 4));
    if (fill > 0) display.fillRect(2, barY + 2, fill, 3, SSD1306_WHITE);
  } else if (fieldState == FIELD_RECORDED || fieldState == FIELD_LOCKED) {
    display.fillRect(2, barY + 2, barW - 4, 3, SSD1306_WHITE);
  }

  // Spread of the window being judged, which is what the bar is measuring.
  display.setCursor(0, 44);
  if (stabCount >= 2) {
    display.print(F("+/-")); display.print(stabSd, 3);
    display.print(F(" n")); display.print(stabCount);
  } else {
    display.print(F("probe "));
    display.print(liveVolts, 3);
    display.print(F("V"));
  }

  // Last committed observation, right-aligned against the spread.
  if (recCount > 0) {
    const FieldRecord& lr = recs[recIndex(recCount - 1)];
    char buf[20];
    snprintf(buf, sizeof(buf), "#%lu %.2f", (unsigned long)lr.seq, lr.salt);
    display.setCursor(SCREEN_WIDTH - (int)strlen(buf) * 6, 44);
    display.print(buf);
  }

  display.drawFastHLine(0, 53, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(0, 56);
  switch (fieldState) {
    // A probe reading something but sitting under the immersion threshold is the
    // one failure of this design that looks like nothing happening: very fresh
    // water can read below a threshold set for brackish, and the operator would
    // stand there holding a probe in a river watching "Dip the probe". Say so.
    case FIELD_DRY:
      if (liveValid && liveVolts < immersionV)
        display.print(F("Below immers. thresh"));
      else
        display.print(F("Dip the probe"));
      break;
    case FIELD_SETTLING: display.print(F("Settling - hold still")); break;
    case FIELD_STABLE:
      if (autoRecord) {
        long left = (long)STABLE_HOLD_MS - (long)(now - fieldStableMs);
        if (left < 0) left = 0;
        display.print(F("Stable - recording "));
        display.print((left / 100) / 10.0f, 1);
      } else {
        display.print(F("Stable - send 'rec'"));
      }
      break;
    case FIELD_RECORDED: display.print(F("RECORDED")); break;
    case FIELD_LOCKED:   display.print(F("Lift probe to re-arm")); break;
  }

  display.display();
}

// Short, plain-language fault text for the OLED - at most 21 characters, one
// size-1 line. FAULT_TABLE's long names are for `diag`; these are for someone
// holding a probe who needs to know whether to trust the number and what to
// check. Faults with a better home elsewhere return nullptr: battery low is the
// header percentage, missing compensation is the RAW marker, and an OLED fault
// cannot be shown on the OLED.
const char* faultShortName(uint32_t bit) {
  switch (bit) {
    case F_POWER_PATH:   return "Power fault";
    case F_SENSORS_DEAD: return "Sensors not answering";
    case F_CHARGER:      return "Charger fault";
    case F_BAT_CRITICAL: return "Battery empty-charge";
    case F_STALLED:      return "Measuring stalled";
    case F_TEMP:         return "Temp sensor lost";
    case F_SALINITY:     return "No probe signal";
    case F_NO_BATTERY:   return "No battery";
    default:             return nullptr;
  }
}

// Of the faults in `set` that have OLED text, the one to show now. Several are
// taken in turn, 2 s each, like the LED's one-code-per-cycle rotation.
uint32_t fieldFaultToShow(uint32_t set, unsigned long now) {
  uint32_t shown[FAULT_SLOTS]; int n = 0;
  for (int i = 0; i < FAULT_SLOTS; i++) {
    uint32_t bit = 1UL << i;
    if ((set & bit) && faultShortName(bit)) shown[n++] = bit;
  }
  return n ? shown[(now / 2000) % n] : 0;
}

void printCentered(const char* txt, int y, uint8_t size) {
  display.setTextSize(size);
  int w = (int)strlen(txt) * 6 * size;
  display.setCursor(w < SCREEN_WIDTH ? (SCREEN_WIDTH - w) / 2 : 0, y);
  display.print(txt);
}

// Simple field screen (the default). One screen per moment of the dip, each
// showing only what that moment needs:
//   dry        DIP PROBE, and the last saved value to compare the next dip with
//   measuring  the live number, the settle bar, "Hold still"
//   saved      the COMMITTED value, frozen - the number to write down - never
//              the live one, which keeps drifting while the probe is lifted
// Faults are shown in words. A red fault replaces the number altogether,
// because a number the firmware knows is untrustworthy must not be on screen;
// an amber one takes turns with the instruction in the footer.
void displayFieldSimple(unsigned long now) {
  display.clearDisplay();
  drawHeader("", true);

  const bool hasLast = recCount > 0;
  const FieldRecord* lr = hasLast ? &recs[recIndex(recCount - 1)] : nullptr;
  char buf[24];

  // Current faults, not the 30 s latched set: the LED holds a fault so it can be
  // caught in passing, but a screen being read should not keep reporting a
  // sensor that has already come back.
  uint32_t red = faultRaw & FAULT_RED_MASK;
  if (fieldFaultToShow(red, now)) {
    printCentered("ERROR", 16, 2);
    printCentered(faultShortName(fieldFaultToShow(red, now)), 38, 1);
    display.drawFastHLine(0, 54, SCREEN_WIDTH, SSD1306_WHITE);
    printCentered("Readings not valid", 57, 1);
    display.display();
    return;
  }

  const bool saved = (fieldState == FIELD_RECORDED || fieldState == FIELD_LOCKED) && hasLast;

  if (fieldState == FIELD_DRY) {
    // A probe that reads something but sits under the immersion threshold looks
    // exactly like nothing happening - very fresh water can do this - so the
    // screen says so instead of asking for a dip that has already happened.
    const bool weak = liveValid && liveVolts < immersionV;
    printCentered(weak ? "LOW SIGNAL" : "DIP PROBE", 18, 2);
    if (hasLast) {
      snprintf(buf, sizeof(buf), "Last: %.2f g/L", (double)lr->salt);
      printCentered(buf, 40, 1);
    }
  } else {
    // Big number. The live value shows one decimal while it is still moving -
    // a flickering second decimal only pulls the eye - and gains the second
    // once it is inside the band, so the extra digit itself says "settled".
    float v   = saved ? lr->salt : liveSalt;
    int   dec = (saved || fieldState == FIELD_STABLE) ? 2 : 1;
    if (saved || liveValid) fmtDisplayValue(buf, sizeof(buf), v, dec);
    else                    strcpy(buf, "--.-");
    display.setTextSize(3);
    int w = (int)strlen(buf) * 18;
    int x = SCREEN_WIDTH - 20 - w;
    if (x < 0) x = 0;
    display.setCursor(x, 13);
    display.print(buf);
    display.setTextSize(1);
    display.setCursor(SCREEN_WIDTH - 18, 27);
    display.print(F("g/L"));

    // Settle bar: fills with convergence, then the commit countdown; full once saved.
    const int barY = 39, barW = SCREEN_WIDTH;
    display.drawRect(0, barY, barW, 6, SSD1306_WHITE);
    int fill = saved ? barW - 4 : (int)(stabProgress() * (barW - 4));
    if (fill > 0) display.fillRect(2, barY + 2, fill, 2, SSD1306_WHITE);

    // Temperature, and RAW only when the number is NOT corrected to 25 C - the
    // normal case carries no marker at all.
    bool tOk  = saved ? (lr->flags & 1) : liveTempOk;
    bool comp = saved ? (lr->flags & 4) : liveComp;
    float t   = saved ? lr->temp : liveTemp;
    display.setCursor(0, 47);
    if (tOk) { display.print(t, 1); display.print(F("C")); }
    else     display.print(F("--C"));
    if (!comp) display.print(F("  RAW"));
    if (saved) {
      snprintf(buf, sizeof(buf), "SAVED #%lu", (unsigned long)lr->seq);
      display.setCursor(SCREEN_WIDTH - (int)strlen(buf) * 6, 47);
      display.print(buf);
    }
  }

  // Footer: what to do next, taking turns with any amber fault.
  display.drawFastHLine(0, 55, SCREEN_WIDTH, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 57);
  uint32_t amber = fieldFaultToShow(faultRaw & FAULT_AMBER_MASK, now);
  if (amber && (now / 1500) % 2) {
    display.print(F("! ")); display.print(faultShortName(amber));
  } else {
    switch (fieldState) {
      case FIELD_DRY:
        display.print(liveValid && liveVolts < immersionV ? F("Check probe in water")
                                                          : F("Put probe in water"));
        break;
      case FIELD_SETTLING: display.print(F("Hold still")); break;
      case FIELD_STABLE:   display.print(autoRecord ? F("Stable - hold still")
                                                    : F("Stable - send 'rec'")); break;
      case FIELD_RECORDED: display.print(F("Saved")); break;
      case FIELD_LOCKED:   display.print(F("Lift probe for next")); break;
    }
  }

  display.display();
}

void displayField() {
  if (!displayOk) return;
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < 200) return;
  last = now;
  if (fieldDetailScreen) displayFieldDetail(now);
  else                   displayFieldSimple(now);
}

// Calibration-mode screen (manual a/b/c edit): shows Bluetooth connection
// state, the current a/b/c coefficients and the available commands.
void displayCalibration(bool btConnected) {
  if (!displayOk) return;
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < 250) return;
  last = now;

  display.clearDisplay();
  drawHeader("CALIBRATION");

  display.setTextSize(1);
  display.setCursor(0, 14);
  display.print(F("Bluetooth: "));
  display.print(btConnected ? F("linked") : F("waiting"));

  // 24 chars is 24 px-wide glyphs at text size 1 -> 21 fit across 128 px, and
  // the quadratic coefficients need every one of them: P1 is ~1e-4, so it is
  // shown at 5 decimals while P2/P3 get 4.
  char line[24];
  display.setCursor(0, 26);
  if (curveModel == CURVE_QUAD) {
    snprintf(line, sizeof(line), "P1=%.5f", p1);
    display.print(line);
    display.setCursor(0, 36);
    snprintf(line, sizeof(line), "P2=%.4f P3=%.4f", p2, p3);
    display.print(line);
  } else {
    snprintf(line, sizeof(line), "a=%.2f  b=%.2f", a, b);
    display.print(line);
    display.setCursor(0, 36);
    snprintf(line, sizeof(line), "c=%.2f", c);
    display.print(line);
  }

  display.drawFastHLine(0, 48, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(0, 52);
  display.print(F("'autocal' or 'help'"));

  display.display();
}

// Auto-calibration screen: how many reference points are collected, the
// current coefficients, the last fit quality (R2) and a short status line so
// the operator can follow the process without a serial terminal in view.
void displayAutoCal() {
  if (!displayOk) return;
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < 250) return;
  last = now;

  display.clearDisplay();
  drawHeader("AUTO-CAL");

  char line[24];
  display.setTextSize(1);

  display.setCursor(0, 14);
  snprintf(line, sizeof(line), "Points: %d/%d", calCount, MAX_CAL_POINTS);
  display.print(line);

  display.setCursor(0, 24);
  if (curveModel == CURVE_QUAD) snprintf(line, sizeof(line), "P1=%.5f P2=%.3f", p1, p2);
  else                          snprintf(line, sizeof(line), "a=%.2f b=%.2f", a, b);
  display.print(line);

  display.setCursor(0, 34);
  if (lastFitR2 >= 0.0f) snprintf(line, sizeof(line), "fit R2=%.3f", lastFitR2);
  else                   strcpy(line, "fit: pending");
  display.print(line);

  display.drawFastHLine(0, 46, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(0, 50);
  display.print(autoCalMsg);

  display.display();
}

// Charge screen: owns the display while the USB-C charger is plugged in and
// measurements are paused. Big battery with the charging sweep, the percentage
// in large type, the pack voltage and the MCP73833's own verdict, plus a line
// saying why no readings are happening.
void displayCharging() {
  if (!displayOk) return;
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < 200) return;
  last = now;

  display.clearDisplay();
  drawHeader("CHARGING");

  drawBatteryGlyph(2, 16, 62, 26, batPercent, chargeState == CHG_CHARGING);

  char buf[8];
  batteryLabel(buf, sizeof(buf));
  display.setTextSize(2);
  display.setCursor(SCREEN_WIDTH - (int)strlen(buf) * 12 - 2, 22);
  display.print(buf);

  display.setTextSize(1);
  display.setCursor(0, 46);
  display.print(batVoltage, 2);
  display.print(F(" V"));

  const char* st = chargeStateShort(chargeState);
  display.setCursor(SCREEN_WIDTH - (int)strlen(st) * 6, 46);
  display.print(st);

  display.drawFastHLine(0, 54, SCREEN_WIDTH, SSD1306_WHITE);
  display.setCursor(0, 56);
  // With the console up, alternate the hint with where to reach it.
  if (wifiState == WIFI_READY && (millis() / 3000) % 2) {
    display.print(F("http://")); display.print(wifiIpStr);
  } else {
    display.print(F("Unplug USB to measure"));
  }

  display.display();
}

// Full-screen SSID/IP card, shown for a few seconds when the console comes up
// so you know where to browse without asking over serial. Returns true if it
// drew, so the caller skips its own screen this pass.
bool displayWifiSplash() {
  if (!displayOk || wifiState == WIFI_DOWN || millis() > wifiSplashUntil) return false;

  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= 200) {
    last = now;
    display.clearDisplay();
    drawHeader("WIFI READY");

    display.setTextSize(1);
    display.setCursor(0, 15);
    display.print(wifiIsAp ? F("AP  ") : F("STA "));
    display.print(wifiSsid);

    if (wifiIsAp) {
      display.setCursor(0, 27);
      display.print(F("pass ")); display.print(WIFI_AP_PASS);
    }

    display.setCursor(0, 41);
    display.print(F("http://")); display.print(wifiIpStr);

    display.drawFastHLine(0, 54, SCREEN_WIDTH, SSD1306_WHITE);
    display.setCursor(0, 56);
    display.print(F("browse to control it"));
    display.display();
  }
  return true;
}

void displayStatus(const char* msg) {
  if (!displayOk) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 28);
  display.print(msg);
  display.display();
}

// ================== SENSOR FUNCTIONS ==================
const char* curveModelName(CurveModel m) {
  return (m == CURVE_QUAD) ? "quad" : "power";
}

// Below this |P1| the quadratic is not a quadratic any more: the vertex runs off
// to infinity and the two roots straddle it, so the formula loses all its
// precision to cancellation long before it divides by zero. 1e-9 puts the vertex
// past 1e7 g/L with P2 at its shipped value - nothing this instrument can reach.
const float QUAD_P1_MIN = 1e-9f;

// The FORWARD quadratic: what voltage a probe in `s` g/L should read. Used by
// the quadratic fit's R2 and by `coef` to print the curve's ceiling.
float salinityToVoltage(float s) {
  return p1 * s * s + p2 * s + p3;
}

// The salinity at the vertex of the forward curve, i.e. the largest salinity the
// model can express, and the voltage there - the ceiling voltageToSalinity()
// clamps to. Only meaningful when P1 < 0 (a curve that bends over); a P1 >= 0
// quadratic is monotonic over the useful range and has no ceiling.
//
// The sMax > 0 test is not a formality. It is the vertex of a parabola, so it is
// wherever the algebra puts it, including at a NEGATIVE salinity when P2 <= 0 -
// a curve whose voltage FALLS as the water gets saltier. That is backwards for
// this front end (a more conductive probe pulls the divider up), but `pcoef`
// will take whatever it is handed and a fit on bad points can land there too.
// Reporting a negative ceiling would put a negative salinity on the glass
// through the clamp below, so say there is no usable ceiling instead.
bool quadCeiling(float& sMax, float& vMax) {
  if (p1 >= -QUAD_P1_MIN) return false;
  float s = -p2 / (2.0f * p1);
  if (!(s > 0.0f)) return false;
  sMax = s;
  vMax = salinityToVoltage(s);
  return true;
}

// The fitted curve. Returns salinity AT THE TEMPERATURE THE PROBE IS IN - the
// fit is done in that domain on purpose (see calRecord), so this value still has
// the bath/river temperature in it and is not comparable across sites until
// salinityAt25() has been applied.
//
// CURVE_QUAD inverts V = P1*S^2 + P2*S + P3 by the quadratic formula:
//
//     S = ( -P2 + sqrt(P2^2 - 4*P1*(P3 - V)) ) / (2*P1)
//
// The + branch is the physical one. With P1 < 0 the two roots sit either side of
// the vertex at S = -P2/(2*P1), and dividing by a negative 2*P1 flips them: the
// + root is the SMALLER salinity, the one on the rising flank the probe actually
// operates on. The - root is the mirror image past the turnover, at 2*vertex
// minus the + root - not a reading this probe can produce. Picking it would put
// every reading up near twice the ceiling (fresh water at ~145 g/L as shipped)
// and, worse, would have it FALL as the water got saltier: wrong, but smoothly
// and plausibly wrong, which is the kind that survives a field trip.
//
// Two places the algebra stops describing the hardware, both clamped rather than
// allowed to produce a number that looks like a measurement:
//
//   * V < P3. P3 is the intercept - the voltage the model says pure water gives
//     (0.0990 V as shipped). Below it the + root goes negative, so return 0.
//     This is the normal case, not an error: the real probe reads ~0.090 V in
//     air, which is just under P3, so every lift-out lands here. Note that is
//     only 9 mV of margin - the dry offset and the curve's own zero nearly
//     coincide on this hardware, and a probe whose dry reading drifted up a few
//     mV would report tenths of a g/L in air rather than 0. The in-water/in-air
//     decision is NOT made here: it is a separate voltage test against
//     `immersionV` (0.110 V as shipped), set above the dry reading rather than
//     below P3, precisely so air cannot be mistaken for very fresh water.
//
//   * V above the vertex voltage. The discriminant goes negative and there is no
//     real root at all. Clamp to the vertex salinity - the most the curve can
//     say - and log it once, because a reading sitting exactly on the ceiling is
//     a curve that has run out of range, not water that happens to be there.
//
// The formula is used in its textbook form rather than the cancellation-free
// rearrangement, because the cancellation does not matter at this scale. On the
// shipped coefficients, float32 costs at most 1.2e-5 g/L over 0-40 g/L and
// 1.7e-4 g/L out to 70, rising to ~9e-3 g/L in the last tenth of a g/L before
// the vertex, where the curve is flat and the subtraction is worst. One ADC LSB
// (0.81 mV) is already worth 0.019 g/L at the fresh end, 0.036 g/L at seawater
// and 0.059 g/L at 50 - so the arithmetic stays some three orders of magnitude
// finer than the quantisation underneath it, everywhere it is used.
float voltageToSalinity(float voltage) {
  if (curveModel == CURVE_POWER) return a * pow(voltage, b) + c;

  // P1 ~ 0: the quadratic has degenerated to the straight line V = P2*S + P3.
  // Solve that directly - the general formula is numerically hopeless here.
  if (fabsf(p1) < QUAD_P1_MIN) {
    if (fabsf(p2) < 1e-12f) return 0.0f;        // no slope either: nothing to invert
    float s = (voltage - p3) / p2;
    return (s > 0.0f) ? s : 0.0f;
  }

  float disc = p2 * p2 - 4.0f * p1 * (p3 - voltage);
  if (disc < 0.0f) {
    // Past the turnover, or - with a descending curve - at a voltage no positive
    // salinity produces. quadCeiling() tells the two apart; when it declines
    // there is no reachable salinity here at all, so 0 is the honest answer.
    float sMax, vMax;
    if (!quadCeiling(sMax, vMax)) return 0.0f;
    if (!quadCeilingTripped) {
      quadCeilingTripped = true;
      Log.print(F("[curve] probe reads "));
      Log.print(voltage, 4);
      Log.print(F(" V, above the "));
      Log.print(vMax, 4);
      Log.print(F(" V ceiling of the quadratic curve. Reporting its maximum, "));
      Log.print(sMax, 2);
      Log.println(F(" g/L."));
      Log.println(F("        The reading is CLAMPED, not measured. Re-fit the curve, or"));
      Log.println(F("        send 'model power' if this water is outside what it covers."));
    }
    return sMax;
  }
  quadCeilingTripped = false;

  float s = (-p2 + sqrtf(disc)) / (2.0f * p1);
  return (s > 0.0f) ? s : 0.0f;   // V below P3 -> fresher than the model's zero
}

// Normalise a raw salinity to TEMP_REF_C. `tempOk` says whether `tC` can be
// believed; without it there is nothing to correct with, so the raw value is
// returned and `applied` comes back false - the caller is expected to flag that
// rather than pass the number off as corrected.
float salinityAt25(float rawS, float tC, bool tempOk, bool& applied) {
  applied = false;
  if (!tempCompEnabled || !tempOk) return rawS;
  float divisor = 1.0f + tempCoeff * (tC - TEMP_REF_C);
  if (divisor < TEMPCOMP_MIN_DIVISOR) {
    // The temperature is outside the range this alpha can correct (see
    // TEMPCOMP_MIN_DIVISOR). Say so once per excursion: without this the
    // operator gets amber 6 and a screen reading "raw" with a temperature that
    // looks perfectly reasonable on the same screen, and no way to tell that the
    // coefficient they set is what ruled it out.
    if (!tempCompGuardTripped) {
      tempCompGuardTripped = true;
      Log.print(F("[tempcomp] "));
      Log.print(tC, 2);
      Log.print(F(" C is outside the range alpha="));
      Log.print(tempCoeff, 4);
      Log.print(F(" can correct (limit "));
      Log.print(tempCompCutoffC(), 1);
      Log.println(F(" C). Reporting RAW salinity."));
      Log.println(F("           Lower alpha with 'tempco <a>' if the water really is this cold."));
    }
    return rawS;
  }
  tempCompGuardTripped = false;
  applied = true;
  return rawS / divisor;
}

// String::toFloat() returns 0.0 for anything it cannot parse, which is fine when
// the caller's valid range excludes zero and silently destructive when it does
// not. `tempco 0,023` (comma decimal separator - the default on a Spanish or
// French keyboard), `tempco x` and a bare `tempco ` all yielded alpha = 0, which
// passed the TEMPCO_MIN == 0.000 bound, was written to NVS and survived reboot:
// the divisor became a constant 1, so the correction was a no-op for every
// reading while `tempcomp` still reported ON and amber 6 never fired.
//
// Accepts leading/trailing space, an optional sign, digits with at most one '.',
// and nothing else. Rejects the empty string. No exponent: nobody types 2e-2 at
// a field terminal, and refusing it costs nothing.
bool parseFloatStrict(const String& in, float& out) {
  String t = in; t.trim();
  if (t.length() == 0) return false;
  int i = 0;
  if (t[i] == '+' || t[i] == '-') i++;
  int digits = 0, dots = 0;
  for (; i < (int)t.length(); i++) {
    char ch = t[i];
    if (ch >= '0' && ch <= '9')      digits++;
    else if (ch == '.' && dots == 0) dots++;
    else                             return false;
  }
  if (digits == 0) return false;
  out = t.toFloat();
  return true;
}

// The inverse, used when recording a calibration point: what a standard whose
// nominal value is `s25` would actually read in a bath at `tC`.
float salinityAtTemp(float s25, float tC) {
  return s25 * (1.0f + tempCoeff * (tC - TEMP_REF_C));
}

// ---- Salinity excitation + ADC helpers ----
void excitationOn() {
  // Detach first so a re-attach always succeeds: on ESP32 core 3.x, ledcAttach()
  // on an already-attached pin returns false and leaves the output stopped, which
  // would make `exc on` / a repeat measurement silently fail to restart the wave.
  ledcDetach(outputPin);
  ledcAttach(outputPin, frequency, resolution);
  ledcWrite(outputPin, 128);   // 50 % duty square wave
}

void excitationOff() {
  if (excAlwaysOn) return;   // latched on for continuous bench measurement
  ledcDetach(outputPin);
}

// Averaged ADC read -> voltage. Assumes excitation is already running.
float readAdcVoltageAveraged(int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) {
    sum += analogRead(adcPin);
    delayMicroseconds(400);
  }
  float adc = (float)sum / (float)n;
  return (adc / 4095.0f) * ADC_REF_VOLTAGE;
}

// Blocking one-shot voltage measurement (used during calibration sampling,
// where blocking is fine because LoRa is not running yet).
float measureVoltageBlocking(int n, int settleMs) {
  excitationOn();
  delay(settleMs);
  float v = readAdcVoltageAveraged(n);
  excitationOff();
  return v;
}

// ---- DS18B20 temperature: non-blocking (split start / read) ----
void tempInit() {
  ds.reset_search();
  if (ds.search(tempAddr) && OneWire::crc8(tempAddr, 7) == tempAddr[7]
      && (tempAddr[0] == 0x10 || tempAddr[0] == 0x28)) {
    tempAddrValid = true;
    Log.println(F("DS18B20 detected."));
  } else {
    tempAddrValid = false;
    Log.println(F("DS18B20 NOT found."));
  }
  faultSet(F_TEMP, !tempAddrValid);
  ds.reset_search();
}

// Kick off a temperature conversion and return immediately (no delay(750)).
void tempStartConversion() {
  if (!tempAddrValid) {
    tempInit();
    if (!tempAddrValid) return;
  }
  ds.reset();
  ds.select(tempAddr);
  ds.write(0x44, 1);   // start conversion; result read later by tempReadResult()
}

// Read the scratchpad after the conversion window has elapsed.
// Returns temperature in C, or -1000.0 on any error.
float tempReadResult() {
  if (!tempAddrValid) return -1000.0f;
  byte data[9];
  ds.reset();
  ds.select(tempAddr);
  ds.write(0xBE);      // read scratchpad
  for (int i = 0; i < 9; i++) data[i] = ds.read();

  if (OneWire::crc8(data, 8) != data[8]) {
    Log.println(F("DS18B20 scratchpad CRC invalid."));
    tempAddrValid = false;   // force a re-detect on the next cycle
    return -1000.0f;
  }

  int16_t raw = ((int16_t)data[1] << 8) | data[0];
  return raw / 16.0f;
}

// Blocking one-shot temperature read. Only used on the calibration path, where
// the 750 ms stall costs nothing because LoRa is not running and the operator is
// standing over the device anyway. The measuring modes use the split
// start/read pair above.
float tempReadBlocking() {
  if (!tempAddrValid) {
    tempInit();
    if (!tempAddrValid) return -1000.0f;
  }
  tempStartConversion();
  delay(DS18B20_CONV_MS + 30);
  return tempReadResult();
}

// ================== FIELD MODE: STABILITY WINDOW ==================
void stabReset() {
  stabCount = 0;
  stabHead  = 0;
  stabMean  = 0.0f;
  stabSd    = 0.0f;
}

// Push one live sample and recompute mean and sample standard deviation across
// the window. Two passes over 20 floats, four times a second - the naive form is
// used deliberately over a running-sums update, which loses precision badly once
// the sums are large relative to the variance.
void stabPush(float s, float v, float tC, uint8_t flags) {
  stabBuf[stabHead]  = s;
  stabVolt[stabHead] = v;
  stabTemp[stabHead] = tC;
  stabFlag[stabHead] = flags;
  stabHead = (stabHead + 1) % STAB_WINDOW;
  if (stabCount < STAB_WINDOW) stabCount++;

  float sum = 0.0f;
  for (int i = 0; i < stabCount; i++) sum += stabBuf[i];
  stabMean = sum / stabCount;

  if (stabCount < 2) { stabSd = 0.0f; return; }
  float ss = 0.0f;
  for (int i = 0; i < stabCount; i++) {
    float d = stabBuf[i] - stabMean;
    ss += d * d;
  }
  stabSd = sqrtf(ss / (stabCount - 1));
}

// Provenance of the window as a whole: a bit survives only if every sample in
// the window had it. Deliberately the AND and not the OR - "this observation is
// temperature-compensated" is a claim about all twenty samples that were
// averaged into it, and one raw sample among them makes the mean raw-ish, which
// is exactly the thing the flag exists to warn about.
uint8_t stabFlags() {
  if (stabCount == 0) return 0;
  uint8_t f = 0xFF;
  for (int i = 0; i < stabCount; i++) f &= stabFlag[i];
  return f;
}

// The band the reading has to sit inside to count as settled. Absolute floor so
// it stays achievable near zero, relative term so it does not demand 0.05 g/L
// repeatability out of a 35 g/L seawater reading.
float stabBand() {
  float rel = fabsf(stabMean) * STAB_REL;
  return (rel > stabAbs) ? rel : stabAbs;
}

// A full window inside the band. Requiring the window to be full is what stops a
// commit two samples after immersion, when the sd is trivially zero because
// nothing has happened yet.
bool stabIsStable() {
  return (stabCount >= STAB_WINDOW) && (stabSd <= stabBand());
}

// 0..1, how converged the reading is. Drives the LED's quickening rate and the
// bar on the OLED and the web console, so the operator can see progress rather
// than wait for a flag to flip.
float stabProgress() {
  if (stabCount < 2) return 0.0f;
  float band = stabBand();
  if (band <= 0.0f) return 0.0f;

  // Two things have to be true before a commit: the scatter is inside the band,
  // AND there are enough samples for that to mean anything. Reporting only the
  // first would show a full bar two samples after immersion, when the sd is
  // trivially small because nothing has had time to happen yet - the bar would
  // promise a commit that stabIsStable() then refuses. Take the worse of the two.
  float fill = (float)stabCount / (float)STAB_WINDOW;
  float p    = (stabSd <= band) ? 1.0f : (band / stabSd);
  if (p > 1.0f) p = 1.0f;
  if (fill < p) p = fill;
  return (p < 0.0f) ? 0.0f : p;
}

// ================== AUTO-CAL FIT ==================
// What point i contributes to the fit: the standard's nominal 25 degC value,
// converted into the temperature domain of the bath it was measured in, using
// the coefficient and the on/off state IN FORCE NOW.
//
// Recomputed here rather than read out of calS, which is the same number frozen
// at the instant the point was taken. Those differ whenever anything about the
// compensation moved between `sample` and `fit`, and the fit then lives in a
// different temperature domain from the salinityAt25() that undoes it at
// runtime:
//
//   - `sample` at alpha=0.020, then `tempco 0.023`, then `fit`: the curve is
//     built on points corrected by 0.020 while runtime divides by 0.023. In a
//     32 degC bath that is a standing ~2 % error, which the firmware only ever
//     warned about.
//   - `tempcomp off` while sampling, `tempcomp on` before fitting: the bath
//     temperature is baked into a and b AND divided out again at runtime -
//     double-corrected, ~14 % at a 32 degC bath.
//
// calSRef/calT/calTOk have always been stored; nothing read them. Now the point
// means "this standard, in this bath", and every interpretation of it is made at
// fit time where the settings that matter are the ones that will be used.
float calFitValue(int i) {
  if (calTOk[i] && tempCompEnabled) return salinityAtTemp(calSRef[i], calT[i]);
  return calSRef[i];
}

// Least-squares fit of S = a * V^b (c forced to 0) via a linear regression in
// log-log space: log(S) = log(a) + b*log(V). Needs >=2 points with S>0 and V>0.
// Returns the coefficients and the R2 of the log-log fit.
bool fitCalibration(float& outA, float& outB, float& outR2) {
  int m = 0;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int i = 0; i < calCount; i++) {
    float sy_i = calFitValue(i);
    if (sy_i > 0.0f && calV[i] > 0.0f) {
      double x = log(calV[i]);
      double y = log(sy_i);
      sx += x; sy += y; sxx += x * x; sxy += x * y;
      m++;
    }
  }
  if (m < 2) return false;

  double denom = (double)m * sxx - sx * sx;
  if (fabs(denom) < 1e-12) return false;      // all V equal -> undefined slope

  double slope     = ((double)m * sxy - sx * sy) / denom;   // = b
  double intercept = (sy - slope * sx) / m;                 // = log(a)

  // R2 in log-log space.
  double ybar = sy / m;
  double ssRes = 0, ssTot = 0;
  for (int i = 0; i < calCount; i++) {
    float sy_i = calFitValue(i);
    if (sy_i > 0.0f && calV[i] > 0.0f) {
      double x = log(calV[i]);
      double y = log(sy_i);
      double yhat = intercept + slope * x;
      ssRes += (y - yhat) * (y - yhat);
      ssTot += (y - ybar) * (y - ybar);
    }
  }

  outA  = (float)exp(intercept);
  outB  = (float)slope;
  outR2 = (ssTot > 0.0) ? (float)(1.0 - ssRes / ssTot) : 1.0f;
  return true;
}

// Least-squares fit of the FORWARD quadratic V = P1*S^2 + P2*S + P3 over the
// recorded points, by normal equations solved with Gaussian elimination and
// partial pivoting. Needs >=3 points at three distinct salinities: two points
// determine a line, and a quadratic through two points is a whole family.
//
// Fitting forward, not inverted, is deliberate and is what makes this different
// from the power fit above. The error that least squares should be minimising
// lives in V - that is the measured quantity, with the ADC noise on it - while S
// is the standard, known to whatever the bottle says. Fitting S against V would
// put the residuals on the axis that has no error in it, and it would also have
// to invert every point through a curve that does not exist yet.
//
// R2 is therefore in V space too, so it answers "how well does this curve
// predict the voltage these standards produce", which is the question the fit
// actually asked. It is not comparable with the log-log R2 the power fit prints.
bool fitCalibrationQuad(float& outP1, float& outP2, float& outP3, float& outR2) {
  // Sums for the normal equations. Doubles throughout: S^4 at seawater is ~1.5e6
  // and a float would be carrying only single-digit precision by the time these
  // are differenced.
  int    m = 0;
  double s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;   // sums of S^0..S^4
  double v0 = 0, v1 = 0, v2 = 0;                   // sums of V, S*V, S^2*V
  for (int i = 0; i < calCount; i++) {
    double s = calFitValue(i);
    double v = calV[i];
    if (!(s >= 0.0) || !(v > 0.0)) continue;       // also rejects NaN
    double ss = s * s;
    s0 += 1.0; s1 += s; s2 += ss; s3 += ss * s; s4 += ss * ss;
    v0 += v;   v1 += s * v;       v2 += ss * v;
    m++;
  }
  if (m < 3) return false;

  // Augmented 3x4 system, unknowns in the order P1, P2, P3.
  double M[3][4] = {
    { s4, s3, s2, v2 },
    { s3, s2, s1, v1 },
    { s2, s1, s0, v0 }
  };

  for (int col = 0; col < 3; col++) {
    int piv = col;
    for (int r = col + 1; r < 3; r++)
      if (fabs(M[r][col]) > fabs(M[piv][col])) piv = r;
    // A singular system means the points do not pin a quadratic down - in
    // practice three points at two distinct salinities, or all three at one.
    // The tolerance is scaled by the leading sum because these rows span many
    // orders of magnitude (S^4 against a plain count), so a fixed epsilon would
    // be meaningless at one end and trigger-happy at the other.
    double scale = (s4 > 1.0) ? s4 : 1.0;
    if (fabs(M[piv][col]) < 1e-12 * scale) return false;
    if (piv != col) for (int k = col; k < 4; k++) { double t = M[col][k]; M[col][k] = M[piv][k]; M[piv][k] = t; }
    for (int r = col + 1; r < 3; r++) {
      double f = M[r][col] / M[col][col];
      for (int k = col; k < 4; k++) M[r][k] -= f * M[col][k];
    }
  }

  double x[3];
  for (int r = 2; r >= 0; r--) {
    double acc = M[r][3];
    for (int k = r + 1; k < 3; k++) acc -= M[r][k] * x[k];
    x[r] = acc / M[r][r];
  }
  if (isnan(x[0]) || isnan(x[1]) || isnan(x[2])) return false;

  // R2 against the measured voltages, using the coefficients just solved for
  // rather than the live globals - `fit` does not commit until it has printed.
  double vbar = v0 / m;
  double ssRes = 0, ssTot = 0;
  for (int i = 0; i < calCount; i++) {
    double s = calFitValue(i);
    double v = calV[i];
    if (!(s >= 0.0) || !(v > 0.0)) continue;
    double vhat = x[0] * s * s + x[1] * s + x[2];
    ssRes += (v - vhat) * (v - vhat);
    ssTot += (v - vbar) * (v - vbar);
  }

  outP1 = (float)x[0];
  outP2 = (float)x[1];
  outP3 = (float)x[2];
  outR2 = (ssTot > 0.0) ? (float)(1.0 - ssRes / ssTot) : 1.0f;
  return true;
}

// ================== DEEP SLEEP ==================
void goToDeepSleep() {
  Log.println("Deep sleep 1 minute...");
  // Blank the LED before sleeping. Deep sleep floats these pads anyway, which
  // is "off" for a common-anode LED, but this avoids a lit pin during the
  // display shutdown below.
  rgbSetStatus(RGB_SLEEP);
  rgbAllOff();
  if (displayOk) {
    displayStatus(" Sleeping 1 min...");
    delay(600);
    display.ssd1306_command(SSD1306_DISPLAYOFF);  // turn off panel -> saves current
  }
  delay(100);
  esp_sleep_enable_timer_wakeup(SLEEP_INTERVAL_US);
  esp_deep_sleep_start();
}

// ================== LMIC EVENTS ==================
void onEvent(ev_t ev) {
  switch (ev) {
    case EV_JOINING:
      Log.println("EV_JOINING");
      break;
    case EV_JOINED:
      Log.println("EV_JOINED");
      LMIC_setLinkCheckMode(0);
      break;
    case EV_TXCOMPLETE:
      Log.println("EV_TXCOMPLETE");
      // A downlink may have been received in the RX window after this uplink.
      if (LMIC.dataLen > 0) {
        uint8_t cmd = LMIC.frame[LMIC.dataBeg];
        Log.print(F("Downlink received, "));
        Log.print(LMIC.dataLen);
        Log.print(F(" byte(s), first=0x"));
        Log.println(cmd, HEX);
        if (cmd == DL_RECAL) {
          Log.println(F("Downlink RECAL command."));
          triggerRecalibration(nullptr);   // reboots into calibration mode
        }
      }
      break;
    default:
      break;
  }
}

// ================== UPLINK ==================
// Encode and transmit one observation. Shared by both measuring modes, which is
// the point: logger mode calls it once per timed reading, field mode calls it
// once per committed dip. Values are SIGNED int16 (two's complement), big-endian;
// an unreadable sensor sends ERR_SENTINEL (0x7FFF) so the backend can tell a
// missing value from a real zero.
void sendUplink(float tC, float sGL, float sd, uint8_t n, bool tempOk, bool salOk) {
  int16_t temp_enc = tempOk ? (int16_t)lroundf(tC   * 10.0f) : ERR_SENTINEL;
  int16_t sal_enc  = salOk  ? (int16_t)lroundf(sGL  * 10.0f) : ERR_SENTINEL;

  // sd is a spread, never negative, and 655 g/L of scatter is not a thing that
  // happens - clamp rather than let it wrap into a plausible-looking number.
  long sdScaled = lroundf(sd * 100.0f);
  if (sdScaled < 0)     sdScaled = 0;
  if (sdScaled > 65535) sdScaled = 65535;
  uint16_t sd_enc = (uint16_t)sdScaled;

  mydata[0] = ((uint16_t)temp_enc >> 8) & 0xFF;
  mydata[1] =  (uint16_t)temp_enc       & 0xFF;
  mydata[2] = ((uint16_t)sal_enc  >> 8) & 0xFF;
  mydata[3] =  (uint16_t)sal_enc        & 0xFF;
  mydata[4] = (sd_enc >> 8) & 0xFF;
  mydata[5] =  sd_enc       & 0xFF;
  mydata[6] = n;

#if LORA_ENABLED
  if (LMIC.opmode & OP_TXRXPEND) {
    Log.println(F("Send pending, skipping this transmission."));
  } else {
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    Log.print(F("TTN message sent! Payload HEX: "));
    for (size_t i = 0; i < sizeof(mydata); ++i) { Log.print(mydata[i], HEX); Log.print(' '); }
    Log.println();
  }
#else
  Log.print(F("[LoRa disabled] Payload HEX: "));
  for (size_t i = 0; i < sizeof(mydata); ++i) { Log.print(mydata[i], HEX); Log.print(' '); }
  Log.println();
#endif
}

// ================== MEASUREMENT (non-blocking) ==================
// Start a measurement cycle: reset the temperature accumulator, begin the first
// async temperature conversion and turn on the salinity excitation so the probe
// settles while the DS18B20 conversions run.
void startMeasurement() {
  lastSendMs   = millis();
  measStartMs  = lastSendMs;
  measState    = MEAS_CONVERTING;
  tempAccum    = 0.0;
  tempOkCount  = 0;
  tempConvDone = 0;
  excitationOn();                // settles while the DS18B20 converts
  // No LED state for "measuring": the conversion chain is 3 s of every 15 and
  // showing it used to overwrite whatever fault the previous reading raised.
  tempStartConversion();
  drawMainScreen(haveMeasurement, temperature, salinity, "Measuring...", true);
}

// Called by the loop each time a conversion window (~750 ms) elapses. Reads that
// conversion into the accumulator; if more conversions are still needed it kicks
// off the next one and stays in MEAS_CONVERTING, otherwise it finishes. Returns
// true once the whole reading (all tempSamples conversions) is complete.
bool stepMeasurement() {
  float tC = tempReadResult();
  if (tC > -100.0f) { tempAccum += tC; tempOkCount++; }
  tempConvDone++;

  if (tempConvDone < tempSamples) {
    tempStartConversion();       // chain the next conversion
    measStartMs = millis();
    return false;
  }
  return true;
}

// Finish the cycle once all temperature conversions are in: average them, read
// the (now well-settled) salinity ADC, encode and transmit.
// Append a finished reading to the ring. Takes the already-filtered batVoltage
// rather than calling readBatteryVoltage() again - that is 64 ADC conversions
// and this runs inside the measurement path.
void histPush(float tC, float sGL, float sRaw, float v, float bv,
              bool tempOk, bool salOk, bool comp) {
  Reading& r = hist[histHead];
  r.t       = (uint32_t)(millis() / 1000);
  r.temp    = tC;
  r.salt    = sGL;
  r.saltRaw = sRaw;
  r.volts   = v;
  r.batV    = bv;
  r.flags   = (uint8_t)((tempOk ? 1 : 0) | (salOk ? 2 : 0) | (comp ? 4 : 0));
  histHead = (histHead + 1) % HIST_MAX;
  if (histCount < HIST_MAX) histCount++;
  histSeq++;
}

// Oldest-first index into the ring, 0 <= i < histCount.
int histIndex(int i) {
  int start = (histCount < HIST_MAX) ? 0 : histHead;
  return (start + i) % HIST_MAX;
}

void finishMeasurement() {
  // Salinity: excitation has been running across the whole conversion chain.
  float v = readAdcVoltageAveraged(salSamples);
  excitationOff();
  lastVoltage = v;
  bool salOk = (v > 0.005f);            // ~0 V => probe not driven / disconnected

  // Temperature: average of the valid conversions collected this reading.
  bool tempOk = (tempOkCount > 0);
  if (tempOk) temperature = (float)(tempAccum / tempOkCount);

  // Order matters: the curve is fitted in the probe's own temperature domain, so
  // it has to run before the normalisation, not after.
  bool comp = false;
  if (salOk) {
    salinityRaw = voltageToSalinity(v);
    salinity    = salinityAt25(salinityRaw, temperature, tempOk, comp);
  }

  if (!tempOk) Log.println(F("ERROR: temperature read failed, keeping last value."));
  if (!salOk)  Log.println(F("ERROR: salinity read failed (no probe signal)."));

  // A salinity reading with no temperature behind it is not wrong so much as
  // uncomparable: it carries whatever the water happened to be at. Amber 6 says
  // so rather than letting it pass as a corrected number.
  faultSet(F_NO_TEMPCOMP, salOk && tempCompEnabled && !comp);

  if (tempOk || salOk) haveMeasurement = true;

  histPush(temperature, salinity, salinityRaw, v, batVoltage, tempOk, salOk, comp);

  // Each sensor gets its own bit, so "the probe is unplugged" and "the DS18B20
  // fell off the bus" are different codes rather than one shared orange blink.
  // "Neither answered" is its own harder fault on top: the payload then carries
  // ERR_SENTINEL for both fields and nothing useful went out.
  faultSet(F_TEMP,         !tempOk);
  faultSet(F_SALINITY,     !salOk);
  faultSet(F_SENSORS_DEAD, !tempOk && !salOk);

  // The cycle completed, which is what the stall watchdog cares about; the
  // double-blip heartbeat is reserved for a reading that was actually clean.
  lastMeasOkMs = millis();
  if (tempOk && salOk) rgbPipMs = lastMeasOkMs;
  rgbSetStatus(RGB_RUN);

  Log.print(F("Measurement - Voltage: ")); Log.print(v, 3);
  Log.print(F(" V ("));                     Log.print(salSamples);
  Log.print(F(" smp), Salinity: "));        Log.print(salinity, 2);
  Log.print(F(" g/L, Temp: "));             Log.print(temperature, 2);
  Log.print(F(" C (avg of "));              Log.print(tempOkCount);
  Log.print(F("/"));                        Log.print(tempSamples);
  Log.print(F(" conv)), Batt: "));          Log.print(readBatteryVoltage(), 2);
  Log.print(F(" V ("));                     Log.print(chargeStateName(readChargeState()));
  Log.println(F(")"));

  drawMainScreen(true, temperature, salinity, "Reading updated", true);

  // sd = 0 and n = 0, not n = tempOkCount. Bytes 4-6 are documented at the
  // mydata[] declaration as field-mode only, byte 6 specifically as "samples
  // averaged" - meaning salinity samples in a stability window, of which logger
  // mode has none: it reads the ADC once. Sending the DS18B20 conversion count
  // there gave a decoder written from that contract a confident "averaged over 4
  // samples" for a single unaveraged reading. 0 is the honest value; a decoder
  // reading n = 0 alongside sd = 0 can tell "not measured" from a real spread.
  sendUplink(temperature, salinity, 0.0f, 0, tempOk, salOk);
}

// ================== FIELD MODE ==================
// One committed observation into the record ring.
void recPush(float sGL, float sd, float tC, float v, uint8_t n, bool tempOk, bool comp) {
  FieldRecord& r = recs[recHead];
  r.seq   = recSeq + 1;          // the number this observation keeps for good
  r.t     = (uint32_t)(millis() / 1000);
  r.salt  = sGL;
  r.sd    = sd;
  r.temp  = tC;
  r.volts = v;
  r.batV  = batVoltage;
  r.n     = n;
  r.flags = (uint8_t)((tempOk ? 1 : 0) | (comp ? 4 : 0));
  strncpy(r.site, siteLabel, sizeof(r.site) - 1);
  r.site[sizeof(r.site) - 1] = '\0';
  recHead = (recHead + 1) % REC_MAX;
  if (recCount < REC_MAX) recCount++;
  recSeq++;
}

// Oldest-first index into the record ring, 0 <= i < recCount. Derived from the
// head and the count rather than from a "has it wrapped yet" test, because
// `recs undo` walks the head backwards: after an undo on a full ring the wrapped
// form would report the wrong oldest entry and the table would come out
// scrambled.
int recIndex(int i) {
  return (recHead - recCount + i + REC_MAX) % REC_MAX;
}

// Commit the current stable window as one observation: the mean of everything in
// it, the spread that says how much to trust that mean, and an uplink. This is
// the only thing in field mode that writes a row or transmits - which is what
// keeps the uplink rate at a handful a day instead of one every 15 s, and leaves
// the duty cycle budget free enough to run a high spreading factor for range.
//
// Returns false when there is nothing worth committing.
bool fieldCommit(bool manual) {
  if (stabCount < 2) return false;

  float sSum = 0.0f, vSum = 0.0f, tSum = 0.0f, rSum = 0.0f;
  for (int i = 0; i < stabCount; i++) {
    sSum += stabBuf[i];
    vSum += stabVolt[i];
    tSum += stabTemp[i];
    rSum += voltageToSalinity(stabVolt[i]);   // the uncorrected mean, for the trace
  }
  float sMean = sSum / stabCount;
  float vMean = vSum / stabCount;
  float tMean = tSum / stabCount;
  float rMean = rSum / stabCount;
  uint8_t n   = (uint8_t)stabCount;

  // Provenance comes from the window, not from the live state at this instant.
  // `tempCompEnabled && liveTempOk` was wrong twice over: it ignored the
  // TEMPCOMP_MIN_DIVISOR guard, so a reading the guard had forced back to raw
  // still went out flagged "@25"; and it read the last sample only, so a
  // DS18B20 that dropped out a quarter-second before the commit condemned a
  // whole window of good compensated samples.
  uint8_t wf   = stabFlags();
  bool    tOk  = (wf & 1) != 0;
  bool    comp = (wf & 4) != 0;
  recPush(sMean, stabSd, tMean, vMean, n, tOk, comp);

  // Keep the headline values in step with what was just recorded, so the OLED,
  // the web console and `diag` all show the committed number rather than the
  // next live sample that lands a quarter second later.
  salinity        = sMean;
  salinityRaw     = rMean;
  temperature     = tMean;
  lastVoltage     = vMean;
  haveMeasurement = true;
  lastMeasOkMs    = millis();

  histPush(tMean, sMean, rMean, vMean, batVoltage, tOk, true, comp);
  sendUplink(tMean, sMean, stabSd, n, tOk, true);

  Log.print(F("[field] RECORDED #")); Log.print(recs[recIndex(recCount - 1)].seq);
  if (siteLabel[0]) { Log.print(F(" @")); Log.print(siteLabel); }
  Log.print(F("  ")); Log.print(sMean, 2);
  Log.print(F(" +/- "));  Log.print(stabSd, 3);
  Log.print(F(" g/L @25C, ")); Log.print(tMean, 2);
  Log.print(F(" C, n=")); Log.print(n);
  if (!comp) Log.print(F("  [UNCOMPENSATED - no temperature]"));
  if (manual) Log.print(F("  (manual)"));
  Log.println();

  fieldRecMs = millis();
  fieldState = FIELD_RECORDED;
  rgbSetStatus(RGB_FIELD_REC);
  return true;
}

// Take one live salinity sample. Fast by design: fieldSalSamples is a quarter of
// the logger-mode count, which is what makes a ~4 Hz update possible without the
// loop spending all its time inside analogRead().
void fieldSampleSalinity() {
  float v = readAdcVoltageAveraged(fieldSalSamples);
  liveVolts = v;
  lastVoltage = v;

  immersedRaw = (v >= immersionV);
  liveValid   = (v > 0.005f);

  if (liveValid) {
    liveSaltRaw = voltageToSalinity(v);
    liveSalt    = salinityAt25(liveSaltRaw, liveTemp, liveTempOk, liveComp);
    faultSet(F_NO_TEMPCOMP, tempCompEnabled && !liveComp);
  } else {
    liveComp = false;                 // nothing was compensated, because nothing was read
    faultSet(F_NO_TEMPCOMP, false);   // nothing to compensate yet
  }

  // A dry probe is the resting state in field mode - the operator walks between
  // sites with it out of the water - so neither of these may be raised on a low
  // reading the way logger mode raises them.
  //
  // Worth being explicit about the limit: a disconnected probe and a probe in
  // air are the same open circuit, so the firmware genuinely cannot tell them
  // apart from the voltage alone. Amber 2 is therefore only raised once the
  // probe is believed to be IN water and the signal still collapses, which is
  // the one case that is unambiguous. If the probe never seems to go wet, the
  // thing to check is the immersion threshold (`immersion`) and then `adc`.
  faultSet(F_SALINITY,     immersed && !liveValid);
  faultSet(F_SENSORS_DEAD, immersed && !liveValid && !liveTempOk);

  // The loop cycled, which is all the stall watchdog is about. Stamping this
  // only on a valid reading would make a probe resting in air look like a wedged
  // measurement loop after three periods.
  //
  // fieldTick() evaluates F_STALLED BEFORE calling this, never after - see the
  // note there. Stamping here and testing afterwards in the same pass is what
  // made the field-mode watchdog unreachable.
  lastMeasOkMs = millis();
}

// Free-running DS18B20 chain. One conversion is always in flight; each result
// feeds an EMA. Nothing here blocks - the conversion is started, and the result
// collected DS18B20_CONV_MS later on some subsequent pass through loop().
void fieldTempTick() {
  unsigned long now = millis();

  if (!fieldTempPending) {
    tempStartConversion();
    fieldTempPending = true;
    fieldTempMs = now;
    return;
  }
  if (now - fieldTempMs < DS18B20_CONV_MS) return;

  float tC = tempReadResult();
  fieldTempPending = false;
  fieldTempMs = now;

  if (tC > -100.0f) {
    if (!liveTempOk) { liveTemp = tC; liveTempOk = true; }   // seed, don't ramp
    else             liveTemp += TEMP_EMA_ALPHA * (tC - liveTemp);
    temperature = liveTemp;
  } else {
    liveTempOk = false;
  }
  faultSet(F_TEMP, !liveTempOk);
}

// Debounce immersion in both directions. Entry is debounced because a probe
// going into moving water splashes; exit because a momentary lift would
// otherwise re-arm mid-observation and throw the window away.
void fieldImmersionTick() {
  unsigned long now = millis();
  if (immersedRaw != immersed) {
    if (immersionEdgeMs == 0) immersionEdgeMs = now;
    else if (now - immersionEdgeMs >= IMMERSION_DEBOUNCE_MS) {
      immersed = immersedRaw;
      immersionEdgeMs = 0;
      Log.print(F("[field] probe "));
      Log.println(immersed ? F("IN the water") : F("out of the water"));
    }
  } else {
    immersionEdgeMs = 0;
  }
}

// The field state machine. Runs every loop() pass; the expensive parts throttle
// themselves.
//
//   DRY -> SETTLING   on debounced immersion
//   SETTLING -> STABLE on a full window inside the band
//   STABLE -> RECORDED after STABLE_HOLD_MS still inside it (auto-record on)
//   RECORDED -> LOCKED after the confirmation flash
//   LOCKED -> DRY     when the probe comes out
//
// Falling out of the band at any point drops straight back to SETTLING, so a
// disturbed reading can never be committed on the strength of an earlier calm.
void fieldTick() {
  unsigned long now = millis();

  // The stall watchdog means something different here: field mode samples
  // continuously, so "a reading completed" is every live sample (stamped in
  // fieldSampleSalinity), not a commit. A device that goes a whole logger period
  // without one has stopped sampling, which is the same wedged-loop condition
  // red 5 means everywhere else.
  //
  // It is evaluated FIRST, against the stamp left by a previous pass, and this
  // ordering is the whole detector. Evaluated after the sampling block it could
  // never fire: fieldSampleSalinity() restamps lastMeasOkMs on the same pass, so
  // the age tested was at most FIELD_SAMPLE_MS and never the 45 s threshold.
  // Read first, the age is real, and a fieldTempTick() that blocks on a 1-Wire
  // bus holding the line low is caught on the pass after it finally returns.
  //
  // The honest limit: a loop that never returns at all cannot be caught from
  // inside the loop. Nothing here substitutes for the hardware task watchdog;
  // this covers the wedge that eventually unwedges.
  faultSet(F_STALLED, (now - lastMeasOkMs) > 3 * SEND_PERIOD_MS);

  fieldTempTick();

  if (now - fieldSampleMs >= FIELD_SAMPLE_MS) {
    fieldSampleMs = now;
    fieldSampleSalinity();
    fieldImmersionTick();

    // Only feed the window while the probe is actually in water and the state
    // machine is working toward a commit. Air readings would otherwise fill it
    // with a rock-steady ~0 and satisfy the stability test perfectly.
    //
    // liveValid is part of that test and not an afterthought. On a dropout
    // sample fieldSampleSalinity() leaves liveSalt at its PREVIOUS value while
    // liveVolts collapses to ~0, so pushing it would insert a duplicate salinity
    // - shrinking the sd toward zero and pulling a premature commit - and at the
    // same time drag the window's mean probe voltage toward 0 V. The committed
    // row would then carry a falsely tight sd beside a probe_v that never
    // produced its salinity. An intermittent probe is exactly the case that
    // reaches this: immersedRaw chatters, so `immersed` never debounces false
    // and the state machine stays in SETTLING/STABLE throughout.
    if (liveValid && immersed &&
        (fieldState == FIELD_SETTLING || fieldState == FIELD_STABLE)) {
      stabPush(liveSalt, liveVolts, liveTemp,
               (uint8_t)((liveTempOk ? 1 : 0) | 2 | (liveComp ? 4 : 0)));
    }
  }

  switch (fieldState) {
    case FIELD_DRY:
      if (immersed) {
        stabReset();
        fieldState = FIELD_SETTLING;
        Log.println(F("[field] settling..."));
      }
      break;

    case FIELD_SETTLING:
      if (!immersed) { fieldState = FIELD_DRY; stabReset(); break; }
      if (stabIsStable()) {
        fieldStableMs = now;
        fieldState    = FIELD_STABLE;
      }
      break;

    case FIELD_STABLE:
      if (!immersed) { fieldState = FIELD_DRY; stabReset(); break; }
      if (!stabIsStable()) { fieldState = FIELD_SETTLING; break; }
      if (autoRecord && now - fieldStableMs >= STABLE_HOLD_MS) {
        fieldCommit(false);
      }
      break;

    case FIELD_RECORDED:
      // Long enough for the confirmation flash to be seen, then hold until the
      // probe is lifted. Re-dipping is what arms the next observation, which is
      // exactly the physical act that makes repeat readings at one site mean
      // anything - five samples without moving the probe measure the firmware's
      // averaging, not the site.
      if (now - fieldRecMs >= 900) fieldState = FIELD_LOCKED;
      break;

    case FIELD_LOCKED:
      if (!immersed) {
        fieldState = FIELD_DRY;
        stabReset();
        Log.println(F("[field] armed - dip again for the next observation."));
      }
      break;
  }

  // Point the LED at the state. Faults still outrank all of this inside
  // rgbUpdate(), so a probe fault during settling shows as amber 2, not as a
  // settling pattern that will never converge.
  switch (fieldState) {
    case FIELD_DRY:      rgbSetStatus(RGB_FIELD_DRY);    break;
    case FIELD_SETTLING: rgbSetStatus(RGB_FIELD_SETTLE); break;
    case FIELD_STABLE:   rgbSetStatus(RGB_FIELD_STABLE); break;
    case FIELD_RECORDED: rgbSetStatus(RGB_FIELD_REC);    break;
    case FIELD_LOCKED:   rgbSetStatus(RGB_FIELD_LOCK);   break;
  }
}

// Short label for the current state, shared by the OLED, `diag` and the console.
const char* fieldStateName(FieldState s) {
  switch (s) {
    case FIELD_DRY:      return "dry";
    case FIELD_SETTLING: return "settling";
    case FIELD_STABLE:   return "stable";
    case FIELD_RECORDED: return "recorded";
    default:             return "lift probe";
  }
}

// Enter field mode from wherever we were. Everything live is cleared: carrying a
// stability window across a mode change would let a commit inherit samples taken
// under different rules.
void fieldBegin() {
  currentMode = MODE_FIELD;
  fieldState  = FIELD_DRY;
  stabReset();
  immersed    = false;
  immersedRaw = false;
  immersionEdgeMs  = 0;
  fieldTempPending = false;
  // Clear ALL of it, not just the validity flag. Leaving liveTemp/liveTempOk set
  // means that for the first >=750 ms of field mode - longer if the DS18B20 has
  // to be re-enumerated - salinityAt25() normalises with the temperature of
  // wherever the device was last used, /api/status serves that temperature as
  // live with tcompNow true, and an operator who dips straight away carries it
  // into the committed window's mean. Same argument for liveSalt/liveVolts,
  // which /api/status and the OLED will otherwise show before the first sample.
  liveValid   = false;
  liveComp    = false;
  liveTempOk  = false;
  liveTemp    = 0.0f;
  liveSalt    = 0.0f;
  liveSaltRaw = 0.0f;
  liveVolts   = 0.0f;
  fieldSampleMs = millis();
  fieldTempMs   = millis();
  lastMeasOkMs  = millis();
  measState   = MEAS_IDLE;

  // Field mode is continuous by definition, and the wave is also what keeps the
  // probe drive AC (see handleChargeMode). Latching it on here means the
  // electrodes are never sitting under a DC bias while the operator works.
  excAlwaysOn = true;
  excitationOn();

  Log.println(F("FIELD mode: continuous read, auto-record on settle."));
  Log.print(F("  band +/-")); Log.print(stabAbs, 3);
  Log.print(F(" g/L or ")); Log.print(STAB_REL * 100.0f, 1);
  Log.print(F(" %, hold ")); Log.print(STABLE_HOLD_MS / 1000);
  Log.print(F(" s, immersion > ")); Log.print(immersionV, 3); Log.println(F(" V"));
  Log.print(F("  auto-record ")); Log.println(autoRecord ? F("ON") : F("OFF - send 'rec'"));
}

// Enter logger mode: timed readings, uplink per reading, deep sleep permitted.
void loggerBegin() {
  currentMode  = MODE_LOGGER;
  measState    = MEAS_IDLE;
  awakeStartMs = millis();
  lastSendMs   = awakeStartMs - SEND_PERIOD_MS;   // take the first reading now
  lastMeasOkMs = millis();
  rgbSetStatus(RGB_RUN);
  Log.print(F("LOGGER mode: a reading every "));
  Log.print(SEND_PERIOD_MS / 1000);
  Log.print(F(" s, sleep "));
  Log.println(sleepEnabled ? F("ENABLED") : F("disabled"));
}

// ================== WIFI CONSOLE: PAGE ==================
// Single self-contained page - the device has no internet, so nothing may be
// pulled from a CDN. Polls /api/status and /api/log once a second and posts
// commands to /api/cmd; the command box reaches the same processCommand() the
// serial terminal does, so anything you can type over USB works here.
static const char PAGE_HTML[] PROGMEM = R"HTML(<!DOCTYPE html><html lang=en data-t=dark><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Gambia Salinity Monitor</title><style>
:root{--bg:#0d0f13;--card:#151922;--card2:#1b202b;--line:#272d3a;--fg:#e8ecf3;--dim:#8d96a8;
--ok:#4ade80;--warn:#fbbf24;--err:#f87171;--acc:#5b9dff;--logbg:#080a0e;--logfg:#c3ccda}
html[data-t=light]{--bg:#f2f5f9;--card:#fff;--card2:#e9edf4;--line:#ccd4e0;--fg:#0f151f;--dim:#55606f;
--ok:#15803d;--warn:#a16207;--err:#b91c1c;--acc:#1d4ed8;--logbg:#fff;--logfg:#1b2431}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 ui-sans-serif,system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{display:flex;align-items:center;gap:10px;padding:11px 14px;border-bottom:1px solid var(--line);
position:sticky;top:0;background:var(--bg);z-index:5}
h1{font-size:15px;margin:0;font-weight:600;letter-spacing:-.01em}
#dot{width:9px;height:9px;border-radius:50%;background:var(--err);flex:none;transition:background .3s}
#dot.up{background:var(--ok);box-shadow:0 0 0 3px rgba(74,222,128,.22)}
.badge{font-size:11px;color:var(--dim);border:1px solid var(--line);border-radius:999px;padding:2px 9px;white-space:nowrap}
.sp{margin-left:auto}
.ico{background:var(--card);border:1px solid var(--line);color:var(--dim);border-radius:8px;
width:30px;height:28px;font-size:14px;cursor:pointer;line-height:1}
.ico:hover{color:var(--fg);border-color:var(--acc)}
main{padding:12px;max-width:880px;margin:0 auto}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px}
.hero{padding:14px 16px 10px;margin-bottom:10px}
.big{font-size:44px;font-weight:650;line-height:1;letter-spacing:-.02em;font-variant-numeric:tabular-nums}
.big em{font-size:16px;font-style:normal;color:var(--dim);font-weight:500;margin-left:6px}
.sub{color:var(--dim);font-size:13px;margin-top:6px}
.sub b{color:var(--fg);font-weight:600;font-variant-numeric:tabular-nums}
#chart{width:100%;height:auto;display:block;margin-top:10px;overflow:visible}
#chart .ln{fill:none;stroke:var(--acc);stroke-width:2;stroke-linejoin:round;stroke-linecap:round}
#chart .ar{fill:var(--acc);opacity:.12}
#chart .gr{stroke:var(--line);stroke-width:1}
#chart .pt{fill:var(--acc)}
#chart text{fill:var(--dim);font:10px ui-monospace,SFMono-Regular,Menlo,monospace}
.legend{display:flex;align-items:center;gap:6px;flex-wrap:wrap;margin-top:8px;font-size:12px}
.legend b{background:var(--card2);border:1px solid var(--line);color:var(--dim);border-radius:999px;
padding:3px 11px;font-weight:500;cursor:pointer;font-size:12px}
.legend b.on{background:var(--acc);border-color:var(--acc);color:#fff}
html[data-t=light] .legend b.on{color:#fff}
.legend .r{margin-left:auto;color:var(--dim);font:11px ui-monospace,Menlo,monospace}
a.dl{color:var(--acc);text-decoration:none;border:1px solid var(--line);border-radius:999px;padding:3px 11px;font-size:12px}
a.dl:hover{border-color:var(--acc)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:8px}
.tile{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:10px 12px}
.tile .k{font-size:10.5px;color:var(--dim);text-transform:uppercase;letter-spacing:.05em;font-weight:600}
.tile .v{font-size:21px;font-weight:600;margin-top:3px;font-variant-numeric:tabular-nums}
.tile .v.sm{font-size:14.5px;font-weight:600;padding:3px 0 4px}
.tile .v small{font-size:12px;color:var(--dim);font-weight:400;margin-left:4px}
.bar{height:5px;background:var(--card2);border-radius:3px;overflow:hidden;margin-top:7px}
.bar i{display:block;height:100%;background:var(--ok);border-radius:3px;transition:width .5s,background .3s}
.meta{margin:10px 2px;color:var(--dim);font-size:11.5px;display:flex;flex-wrap:wrap;gap:3px 14px;
font-family:ui-monospace,Menlo,monospace}
.note{border-radius:10px;padding:9px 12px;margin:10px 0;font-size:12.5px;line-height:1.5}
.note.w{background:#2a2110;border:1px solid #4a3a12;color:#fbbf24}
html[data-t=light] .note.w{background:#fdf4e0;border-color:#dfbc72;color:#8a5f08}
.logbar{display:flex;align-items:center;gap:6px;margin:12px 0 6px}
.logbar .k{font-size:10.5px;color:var(--dim);text-transform:uppercase;letter-spacing:.05em;font-weight:600}
.logbar input{flex:1;max-width:190px;margin-left:auto}
.logbar button{background:var(--card);border:1px solid var(--line);color:var(--dim);border-radius:8px;
padding:5px 11px;font-size:12px;cursor:pointer}
.logbar button:hover{color:var(--fg);border-color:var(--acc)}
.logbar button.on{background:var(--warn);border-color:var(--warn);color:#1b1405}
#log{background:var(--logbg);border:1px solid var(--line);border-radius:12px;padding:11px 12px;
height:290px;overflow:auto;white-space:pre-wrap;word-break:break-word;
font:12px/1.55 ui-monospace,SFMono-Regular,Menlo,monospace;color:var(--logfg)}
form{display:flex;gap:8px;margin-top:10px}
input{background:var(--card);border:1px solid var(--line);color:var(--fg);border-radius:9px;
padding:9px 12px;font:13px ui-monospace,Menlo,monospace}
#c{flex:1;min-width:0}
input:focus{outline:none;border-color:var(--acc)}
button.go{background:var(--acc);color:#fff;border:0;border-radius:9px;padding:9px 18px;font-weight:600;cursor:pointer;font-size:13px}
.q{display:flex;flex-wrap:wrap;gap:6px;margin-top:8px;padding-bottom:20px}
.q b{background:var(--card);border:1px solid var(--line);color:var(--dim);border-radius:999px;
padding:4px 11px;font:12px ui-monospace,Menlo,monospace;font-weight:400;cursor:pointer}
.q b:hover{color:var(--fg);border-color:var(--acc)}
.flt{display:flex;flex-wrap:wrap;gap:7px;margin-top:12px}
.flt b{border:1px solid;border-radius:10px;padding:6px 12px;font:12px/1.35 ui-monospace,Menlo,monospace;
font-weight:400;display:inline-block}
.flt b i{font-style:normal;font-weight:700;letter-spacing:.06em;margin-right:8px}
.flt b.r{color:var(--err);border-color:var(--err)}
.flt b.a{color:var(--warn);border-color:var(--warn)}
.flt b.h{opacity:.6}
/* field panel ---------------------------------------------------------- */
.fld{padding:12px 14px;margin-bottom:10px}
.fhead{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin-bottom:9px}
.fhead .st{font-size:13px;font-weight:650;letter-spacing:-.01em}
.fhead .st.dry{color:var(--dim)}
.fhead .st.settling{color:var(--warn)}
.fhead .st.stable,.fhead .st.recorded{color:var(--ok)}
.fhead .st.lift{color:var(--acc)}
.pill{font-size:11px;border:1px solid var(--line);border-radius:999px;padding:2px 9px;color:var(--dim);white-space:nowrap}
.pill.on{border-color:var(--ok);color:var(--ok)}
.pill.no{border-color:var(--warn);color:var(--warn)}
.sbar{height:9px;background:var(--card2);border-radius:5px;overflow:hidden;position:relative}
.sbar i{display:block;height:100%;width:0;background:var(--warn);border-radius:5px;transition:width .25s linear,background .3s}
.sbar i.ok{background:var(--ok)}
.snum{display:flex;gap:4px 16px;flex-wrap:wrap;margin-top:7px;color:var(--dim);
font:11.5px ui-monospace,Menlo,monospace}
.snum b{color:var(--fg);font-weight:600}
.frow{display:flex;gap:7px;flex-wrap:wrap;margin-top:11px;align-items:center}
.frow button{background:var(--card2);border:1px solid var(--line);color:var(--fg);border-radius:9px;
padding:7px 14px;font-size:12.5px;cursor:pointer;font-weight:500}
.frow button:hover{border-color:var(--acc)}
.frow button.pri{background:var(--acc);border-color:var(--acc);color:#fff}
.frow button.warn:hover{border-color:var(--err);color:var(--err)}
.frow button:disabled{opacity:.4;cursor:not-allowed}
.frow button:disabled:hover{border-color:var(--line)}
.frow input{padding:7px 10px;font-size:12.5px;width:130px}
table{width:100%;border-collapse:collapse;margin-top:9px;font-size:12px;
font-variant-numeric:tabular-nums}
th{text-align:left;color:var(--dim);font-size:10.5px;text-transform:uppercase;
letter-spacing:.05em;font-weight:600;padding:4px 6px;border-bottom:1px solid var(--line)}
td{padding:5px 6px;border-bottom:1px solid var(--line)}
tr:last-child td{border-bottom:0}
td.num{text-align:right}
td .raw{color:var(--warn);font-size:10px;margin-left:4px}
.empty{color:var(--dim);font-size:12.5px;padding:10px 2px}
.hide{display:none}
</style></head><body>
<header><span id=dot></span><h1>Gambia Salinity Monitor</h1>
<span class=badge id=mode>connecting</span>
<span class=sp></span><button class=ico id=theme title="Light / dark">&#9681;</button></header>
<main>
<section class="card hero">
 <div class=big><span id=salt>--</span><em id=saltu>g/L @25&deg;C</em></div>
 <div class=sub><b id=temp>--</b> &deg;C &middot; probe <b id=volts>--</b> V &middot; <span id=age>no reading yet</span></div>
 <svg id=chart viewBox="0 0 600 140" role=img aria-label="reading history"></svg>
 <div class=legend>
  <b data-s=salt class=on>salinity</b><b data-s=temp>temperature</b><b data-s=batV>battery</b>
  <span class=r id=range></span><a class=dl href=/api/csv download>&#8595; CSV</a>
 </div>
</section>

<!-- Field panel. Hidden entirely in logger and calibration mode: the settle bar
     and the observation table only mean anything when someone is holding the
     probe. -->
<section class="card fld hide" id=fpanel>
 <div class=fhead>
  <span class=st id=fst>&mdash;</span>
  <span class=pill id=fwet>probe dry</span>
  <span class=pill id=fauto>auto-record</span>
  <span class=sp></span>
  <span class=pill id=fsite hidden></span>
 </div>
 <div class=sbar><i id=fbar></i></div>
 <div class=snum id=fnum></div>
 <div class=frow>
  <button class=pri id=brec>Record now</button>
  <button id=bauto>Auto-record off</button>
  <input id=csite placeholder="site name" spellcheck=false maxlength=11>
  <button id=bsite>Set site</button>
 </div>
</section>

<section class="card fld hide" id=rpanel>
 <div class=fhead>
  <span class=st>Observations</span>
  <span class=pill id=rcount>0</span>
  <span class=sp></span>
  <a class=dl href=/api/records.csv download>&#8595; CSV</a>
 </div>
 <div id=rtab></div>
 <div class=frow>
  <button id=bundo>Undo last</button>
  <button class=warn id=bclear>Clear all</button>
 </div>
</section>

<div class=grid>
 <div class=tile><div class=k>Battery</div><div class=v id=bat>--</div><div class=bar><i id=batbar style=width:0></i></div></div>
 <div class=tile><div class=k>Charger</div><div class="v sm" id=chg>--</div><div class=k id=chgsub style=text-transform:none;letter-spacing:0;font-weight:400>&nbsp;</div></div>
 <div class=tile id=tnext><div class=k>Next reading</div><div class=v id=next>--</div></div>
 <div class=tile><div class=k>Readings kept</div><div class=v id=nhist>--</div></div>
</div>
<div class=flt id=faults></div>
<div class=meta id=meta></div>
<div class="note w" id=note hidden></div>
<div class=logbar><span class=k>Live log</span>
 <button id=pause title="Freeze the view">Pause</button>
 <button id=clear title="Clear what is on screen">Clear</button>
 <button id=copy title="Copy the visible log">Copy</button>
 <input id=filt placeholder=filter spellcheck=false></div>
<div id=log></div>
<form id=f autocomplete=off><input id=c placeholder="type a command &mdash; try 'help', or &uarr; for history" spellcheck=false><button class=go>Send</button></form>
<div class=q id=q></div>
</main><script>
const $=i=>document.getElementById(i);
let lseq=0,hseq=-1,stick=true,fails=0,paused=false,H=null,series='salt';
let lines=[''],filt='',cmds=[],ci=-1,R=null,rseq=-1,S=null;

// theme -------------------------------------------------------------------
try{const t=localStorage.getItem('gt');if(t)document.documentElement.dataset.t=t}catch(e){}
$('theme').onclick=()=>{const n=document.documentElement.dataset.t=='light'?'dark':'light';
 document.documentElement.dataset.t=n;try{localStorage.setItem('gt',n)}catch(e){}draw()};

// quick commands ----------------------------------------------------------
['diag','help','mode','mode field','mode logger','tempcomp','stability','immersion',
 'recs','autorec off','bat','power','hist','charge ignore','coef','points','adc',
 'exc on','rgb test','wifi']
 .forEach(c=>{const b=document.createElement('b');b.textContent=c;b.onclick=()=>send(c);$('q').appendChild(b)});

// field controls ----------------------------------------------------------
// Everything here goes through /api/cmd, the same path the serial terminal
// uses, so a button press and a typed command are the same event and both land
// in the one transcript below.
$('brec').onclick=()=>send('rec');
$('bauto').onclick=()=>send(S&&S.field&&S.field.auto?'autorec off':'autorec on');
$('bsite').onclick=()=>{const v=$('csite').value.trim();send(v?'site '+v:'site');$('csite').value=''};
$('csite').onkeydown=e=>{if(e.key=='Enter'){e.preventDefault();$('bsite').onclick()}};
$('bundo').onclick=()=>{if(confirm('Remove the most recent observation?'))send('recs undo')};
$('bclear').onclick=()=>{if(confirm('Clear every stored observation? This cannot be undone.'))send('recs clear')};

// log ---------------------------------------------------------------------
$('log').onscroll=()=>{const e=$('log');stick=e.scrollHeight-e.scrollTop-e.clientHeight<30};
$('filt').oninput=e=>{filt=e.target.value.toLowerCase();render()};
$('pause').onclick=()=>{paused=!paused;$('pause').classList.toggle('on',paused);
 $('pause').textContent=paused?'Resume':'Pause';if(!paused)render()};
$('clear').onclick=()=>{lines=[''];render()};
$('copy').onclick=()=>{const t=visible();
 if(navigator.clipboard&&window.isSecureContext){navigator.clipboard.writeText(t)}
 else{const a=document.createElement('textarea');a.value=t;document.body.appendChild(a);
  a.select();try{document.execCommand('copy')}catch(e){}a.remove()}
 $('copy').textContent='Copied';setTimeout(()=>$('copy').textContent='Copy',1200)};
function visible(){return(filt?lines.filter(l=>l.toLowerCase().includes(filt)):lines).join('\n')}
function render(){if(paused)return;const e=$('log');e.textContent=visible();if(stick)e.scrollTop=e.scrollHeight}
function append(t){const p=t.split('\n');lines[lines.length-1]+=p[0];
 for(let i=1;i<p.length;i++)lines.push(p[i]);
 if(lines.length>800)lines=lines.slice(-800);render()}

// command entry with history ----------------------------------------------
async function send(c){if(!c)return;$('c').value='';
 if(cmds[cmds.length-1]!=c)cmds.push(c);ci=cmds.length;
 try{await fetch('/api/cmd?c='+encodeURIComponent(c))}catch(e){append('\n[page] send failed\n')}
 poll()}
$('f').onsubmit=e=>{e.preventDefault();send($('c').value.trim())};
$('c').onkeydown=e=>{
 if(e.key=='ArrowUp'){if(ci>0){ci--;$('c').value=cmds[ci];e.preventDefault()}}
 else if(e.key=='ArrowDown'){if(ci<cmds.length-1){ci++;$('c').value=cmds[ci]}else{ci=cmds.length;$('c').value=''}e.preventDefault()}};

// chart -------------------------------------------------------------------
document.querySelectorAll('.legend b').forEach(b=>b.onclick=()=>{
 series=b.dataset.s;document.querySelectorAll('.legend b').forEach(x=>x.classList.toggle('on',x==b));draw()});
const CONF={salt:['g/L',2],temp:['°C',1],batV:['V',2]};
function draw(){
 const sv=$('chart'),u=CONF[series];
 if(!H||!H.n){sv.innerHTML='<text x=300 y=74 text-anchor=middle>waiting for the first reading</text>';
  $('range').textContent='';return}
 const pts=[];
 for(let i=0;i<H.n;i++){
  const ok=series=='salt'?(H.f[i]&2):series=='temp'?(H.f[i]&1):1;
  if(ok)pts.push([H.t[i],H[series][i]])}
 if(!pts.length){sv.innerHTML='<text x=300 y=74 text-anchor=middle>no valid points for this series</text>';
  $('range').textContent='';return}
 let lo=Math.min(...pts.map(p=>p[1])),hi=Math.max(...pts.map(p=>p[1]));
 if(hi-lo<1e-9){lo-=1;hi+=1}                       // a flat line still needs a box
 const pad=(hi-lo)*.14;lo-=pad;hi+=pad;
 const W=600,HH=140,L=46,R=6,T=10,B=18;
 const t0=H.t[0],t1=Math.max(H.now,H.t[H.n-1]);
 const X=t=>L+(t1==t0?1:(t-t0)/(t1-t0))*(W-L-R);
 const Y=v=>T+(1-(v-lo)/(hi-lo))*(HH-T-B);
 const d=pts.map(p=>X(p[0]).toFixed(1)+','+Y(p[1]).toFixed(1)).join(' ');
 let g='';
 for(let k=0;k<=2;k++){const y=(T+k*(HH-T-B)/2).toFixed(1);
  g+='<line class=gr x1='+L+' y1='+y+' x2='+(W-R)+' y2='+y+'/>'+
     '<text x='+(L-5)+' y='+(+y+3.5)+' text-anchor=end>'+(hi-k*(hi-lo)/2).toFixed(u[1])+'</text>'}
 const last=pts[pts.length-1];
 const area=pts.length>1?'<polygon class=ar points="'+X(pts[0][0]).toFixed(1)+','+(HH-B)+' '+d+' '+X(last[0]).toFixed(1)+','+(HH-B)+'"/>':'';
 const span=Math.round(t1-t0);
 sv.innerHTML=g+area+(pts.length>1?'<polyline class=ln points="'+d+'"/>':'')+
  '<circle class=pt cx='+X(last[0]).toFixed(1)+' cy='+Y(last[1]).toFixed(1)+' r=3/>'+
  '<text x='+L+' y='+(HH-5)+'>-'+fmtAge(span)+'</text>'+
  '<text x='+(W-R)+' y='+(HH-5)+' text-anchor=end>now</text>';
 $('range').textContent=pts.length+' pts · min '+Math.min(...pts.map(p=>p[1])).toFixed(u[1])+
  ' · max '+Math.max(...pts.map(p=>p[1])).toFixed(u[1])+' '+u[0];
}
function fmtAge(s){if(s<90)return s+'s';if(s<5400)return Math.round(s/60)+'m';return(s/3600).toFixed(1)+'h'}

// polling -----------------------------------------------------------------
const f=(x,d)=>x===null||x===undefined?'--':Number(x).toFixed(d);
// field panel -------------------------------------------------------------
const STCLS={dry:'dry',settling:'settling',stable:'stable',recorded:'recorded'};
const STTXT={dry:'Dip the probe',settling:'Settling',stable:'Stable',
 recorded:'Recorded',['lift probe']:'Lift the probe to re-arm'};
function drawField(s){
 const fl=s.mode=='field',F=s.field||{};
 $('fpanel').classList.toggle('hide',!fl);
 $('rpanel').classList.toggle('hide',!fl);
 $('tnext').classList.toggle('hide',fl);      // there is no "next reading" here
 if(!fl)return;

 const st=$('fst');
 st.textContent=STTXT[F.state]||F.state;
 st.className='st '+(STCLS[F.state]||'lift');

 const w=$('fwet');
 w.textContent=F.wet?'probe in water':(F.under?'below immersion threshold':'probe dry');
 w.className='pill'+(F.wet?' on':(F.under?' no':''));

 const a=$('fauto');a.textContent=F.auto?'auto-record on':'auto-record off';
 a.className='pill'+(F.auto?' on':' no');
 $('bauto').textContent=F.auto?'Auto-record off':'Auto-record on';
 $('brec').disabled=F.n<2;

 const si=$('fsite');si.hidden=!F.site;si.textContent=F.site||'';
 if(document.activeElement!=$('csite'))$('csite').placeholder=F.site||'site name';

 // The bar is the answer to "can I write it down yet". While it is stable and
 // counting down to the commit it switches to showing that countdown, so the
 // thing filling the bar is always the thing you are waiting for.
 const bar=$('fbar');
 let pct,ok=false;
 if(F.state=='stable'&&F.auto){pct=Math.min(100,100*F.heldMs/F.holdMs);ok=true}
 else if(F.state=='recorded'||F.state=='lift probe'){pct=100;ok=true}
 else{pct=100*(F.prog||0);ok=F.stable}
 bar.style.width=pct.toFixed(0)+'%';
 bar.classList.toggle('ok',ok);

 const parts=[];
 if(F.n>1){parts.push('sd <b>'+f(F.sd,4)+'</b> g/L');
  parts.push('band <b>'+f(F.band,4)+'</b>');
  parts.push('n <b>'+F.n+'</b>/'+F.nMax)}
 else parts.push('window <b>empty</b> — the probe is not in water');
 if(F.state=='stable'&&F.auto)
  parts.push('committing in <b>'+Math.max(0,(F.holdMs-F.heldMs)/1000).toFixed(1)+'</b> s');
 parts.push('immersion &gt; <b>'+f(F.immV,3)+'</b> V');
 $('fnum').innerHTML=parts.map(x=>'<span>'+x+'</span>').join('');

 $('rcount').textContent=F.recN+' kept · '+F.recSeq+' taken';
}

function drawRecords(){
 const e=$('rtab');
 if(!R||!R.n){e.innerHTML='<div class=empty>No observations yet. '+
  'Each committed dip lands here as one row — mean, spread and sample count.</div>';return}
 let h='<table><tr><th>#</th><th>age</th><th>site</th><th class=num>salinity @25&deg;C</th>'+
  '<th class=num>sd</th><th class=num>n</th><th class=num>temp</th></tr>';
 for(let i=R.rows.length-1;i>=0;i--){const r=R.rows[i];
  h+='<tr><td>'+r.seq+'</td><td>'+fmtAge(R.now-r.t)+'</td><td>'+(r.site?esc(r.site):'&mdash;')+'</td>'+
   '<td class=num><b>'+f(r.salt,2)+'</b>'+(r.comp?'':'<span class=raw title="no temperature — not corrected to 25 C">raw</span>')+'</td>'+
   '<td class=num>&plusmn;'+f(r.sd,3)+'</td><td class=num>'+r.n+'</td>'+
   // A dash, not 0.0 degrees. r.tempOk false means the DS18B20 never answered,
   // and the stored 0.00 is an initialiser rather than a measurement.
   '<td class=num>'+(r.tempOk?f(r.temp,1)+'&deg;':'&mdash;')+'</td></tr>'}
 e.innerHTML=h+'</table>'}
function esc(s){return s.replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}

async function poll(){try{
 const s=await(await fetch('/api/status',{cache:'no-store'})).json();
 S=s;fails=0;$('dot').className='up';
 $('mode').textContent=s.mode+(s.chargeMode?' · charging':'');
 $('salt').textContent=s.have?f(s.salt,s.mode=='field'?2:1):'--';
 $('temp').textContent=s.have?f(s.temp,1):'--';
 $('volts').textContent=f(s.volts,3);

 // Say plainly whether the headline number is comparable with anything else.
 // A salinity with no temperature behind it is not wrong, it just carries
 // whatever the water happened to be at — and that has to be visible, not
 // inferred from an amber LED somewhere else.
 $('saltu').innerHTML=!s.tcomp?'g/L raw':(s.tcompNow?'g/L @25&deg;C':'g/L uncorrected');

 const age=Math.max(0,s.period-s.next);
 $('age').textContent=s.mode=='field'?(s.have?'live':'probe dry')
  :(s.have?('updated '+fmtAge(age)+' ago'):'no reading yet');
 drawField(s);
 $('bat').innerHTML=(s.batPct<0?'--':s.batPct)+'<small>% · '+f(s.batV,2)+' V</small>';
 const bb=$('batbar');bb.style.width=Math.max(0,Math.min(100,s.batPct))+'%';
 bb.style.background=s.batPct<15?'var(--err)':s.batPct<35?'var(--warn)':'var(--ok)';
 $('chg').textContent=s.charge;
 $('chgsub').textContent=s.powerOk===false?'monitoring disconnected':s.power?'USB-C power present':'running on battery';
 $('next').innerHTML=s.chargeMode?'paused':(s.meas?'now':s.next+'<small>s</small>');
 $('nhist').innerHTML=s.histN+'<small>of '+s.seq+' taken</small>';
 const curve=s.model=='quad'
  ?['curve quad','P1='+f(s.p1,6),'P2='+f(s.p2,6),'P3='+f(s.p3,6)]
  :['curve power','a='+f(s.a,4),'b='+f(s.b,4),'c='+f(s.c,4)];
 $('meta').innerHTML=[...curve,'salt smp '+s.salSamples,
  'temp smp '+s.tempSamples,'exc '+(s.exc?'latched on':'per-reading'),
  s.tcomp?('tempco '+f(s.tempco,4)+'/°C → '+f(s.tref,0)+'°C'):'temp comp OFF',
  s.mode=='field'?('band '+f(s.field.stabAbs,3)+' g/L'):('sleep '+(s.sleep?'on':'off')),
  s.apMode?('AP '+s.ssid+' · '+s.clients+' client(s)'):('STA '+s.ssid+' · '+s.rssi+' dBm'),
  s.ip,'heap '+(s.heap/1024|0)+' KB','up '+fmtAge(s.uptime)].map(x=>'<span>'+x+'</span>').join('');
 $('faults').innerHTML=(s.faults||[]).map(x=>'<b class="'+(x.c=='R'?'r':'a')+(x.held?' h':'')+'">'
  +'<i>'+(x.c=='R'?'RED':'AMBER')+' &times;'+x.n+'</i>'+x.t+(x.held?' &middot; cleared, still held':'')+'</b>').join('');
 const n=$('note');
 if(s.powerOk===false){n.hidden=false;n.textContent='Charger monitoring is disabled: the STAT/PG pins read as disconnected, so this ESP32 is probably not seated in the V2 carrier board. Measurements keep running. Send "power" for detail.'}
 else if(s.chargeMode){n.hidden=false;n.textContent='Charging — measurements are paused. Send "charge ignore" to keep measuring with the cable in, or unplug USB-C.'}
 else if(s.mode=='field'&&s.field.under){n.hidden=false;n.textContent='The probe is reading '+f(s.volts,3)+' V — something is there, but it is under the '+f(s.field.immV,3)+' V immersion threshold, so nothing will be recorded. If the probe really is in the water, this site is fresher than the threshold expects: send "immersion '+(s.volts*0.6).toFixed(3)+'" to lower it.'}
 else if(s.tcomp&&!s.tcompNow){n.hidden=false;n.textContent='No temperature reading, so salinity is NOT being corrected to 25 °C. Conductivity moves about 2 % per °C, so these numbers are only comparable with others taken at the same water temperature. Check the DS18B20 — send "diag".'}
 else if(!s.tcomp){n.hidden=false;n.textContent='Temperature compensation is off: salinity is reported raw, at whatever temperature the water is. Readings from different times of day or different sites are not comparable. Send "tempcomp on" to enable it.'}
 else n.hidden=true;
 if(s.seq!=hseq){hseq=s.seq;
  try{H=await(await fetch('/api/history',{cache:'no-store'})).json();draw()}catch(e){}}
 // Key on the count as well as the running total: 'recs clear' empties the ring
 // without touching recSeq, so watching recSeq alone would leave a stale table.
 if(s.mode=='field'){const k=s.field.recSeq+'/'+s.field.recN;
  if(k!=rseq){rseq=k;
   try{R=await(await fetch('/api/records',{cache:'no-store'})).json();drawRecords()}catch(e){}}}
 const r=await fetch('/api/log?from='+lseq,{cache:'no-store'});
 const nx=r.headers.get('X-Log-Next'),t=await r.text();
 if(t)append(t);if(nx)lseq=+nx;
}catch(e){if(++fails>2){$('dot').className='';$('mode').textContent='reconnecting'}}}
poll();setInterval(poll,1000);
</script></body></html>)HTML";

// ================== WIFI CONSOLE: HANDLERS ==================
void processCommand(const String& command, Stream& out);   // defined below

// Minimal JSON string escaping. Only the SSID can realistically contain
// anything awkward, but an unescaped quote there would break the whole page.
static String jsonEscape(const String& in) {
  String o;
  o.reserve(in.length() + 4);
  for (size_t i = 0; i < in.length(); i++) {
    char ch = in[i];
    if (ch == '"' || ch == '\\') { o += '\\'; o += ch; }
    else if (ch < 0x20)          { o += ' '; }
    else                          o += ch;
  }
  return o;
}

void wifiHandleRoot() { server.send_P(200, "text/html", PAGE_HTML); }

// Everything the device knows about itself, once a second.
void wifiHandleStatus() {
  long remain = (long)SEND_PERIOD_MS - (long)(millis() - lastSendMs);
  if (remain < 0) remain = 0;

  // In field mode the headline numbers are the LIVE ones, updated four times a
  // second, not the last committed observation - the page is a remote copy of
  // the glass the operator is looking at.
  bool  fld      = (currentMode == MODE_FIELD);
  float shownS   = fld ? liveSalt  : salinity;
  float shownT   = fld ? liveTemp  : temperature;
  float shownV   = fld ? liveVolts : lastVoltage;
  bool  shownOk  = fld ? liveValid : haveMeasurement;
  // What the LAST reading actually did, not what the hardware could do in
  // principle. tempAddrValid only says the DS18B20 answered the enumeration at
  // boot; a probe pulled since, or one returning CRC-valid garbage outside the
  // range alpha can correct, leaves finishMeasurement() reporting comp=false and
  // raising amber 6 while this line still claimed "@25". The truth is already in
  // the reading's flags bit 2, so read it from there.
  bool  lastComp = (histCount > 0) && (hist[histIndex(histCount - 1)].flags & 4);
  bool  compNow  = fld ? liveComp : (haveMeasurement && lastComp);

  String j;
  j.reserve(1280);
  j  = "{\"mode\":\"";      j += (currentMode == MODE_CALIBRATION ? "calibration"
                                  : (fld ? "field" : "logger"));
  j += "\",\"uptime\":";    j += (uint32_t)(millis() / 1000);
  j += ",\"have\":";        j += (shownOk ? "true" : "false");
  j += ",\"meas\":";        j += (measState == MEAS_CONVERTING ? "true" : "false");
  j += ",\"temp\":";        j += String(shownT, 2);
  j += ",\"salt\":";        j += String(shownS, 2);
  j += ",\"saltRaw\":";     j += String(fld ? liveSaltRaw : salinityRaw, 2);
  j += ",\"volts\":";       j += String(shownV, 4);

  // ---- temperature compensation ----
  j += ",\"tcomp\":";       j += (tempCompEnabled ? "true" : "false");
  j += ",\"tcompNow\":";    j += (compNow ? "true" : "false");
  j += ",\"tempco\":";      j += String(tempCoeff, 4);
  j += ",\"tref\":";        j += String(TEMP_REF_C, 0);

  // ---- field mode ----
  j += ",\"field\":{";
  j += "\"state\":\"";      j += fieldStateName(fieldState);
  j += "\",\"wet\":";       j += (immersed ? "true" : "false");
  // Reading something, but under the threshold: almost always a threshold set
  // for brackish water being used on a fresh site, and it presents as the device
  // simply refusing to do anything at all.
  j += ",\"under\":";       j += ((!immersed && liveValid && liveVolts < immersionV)
                                   ? "true" : "false");
  j += ",\"immV\":";        j += String(immersionV, 3);
  j += ",\"n\":";           j += stabCount;
  j += ",\"nMax\":";        j += STAB_WINDOW;
  j += ",\"sd\":";          j += String(stabSd, 4);
  j += ",\"mean\":";        j += String(stabMean, 3);
  j += ",\"band\":";        j += String(stabBand(), 4);
  j += ",\"prog\":";        j += String(stabProgress(), 3);
  j += ",\"stable\":";      j += (stabIsStable() ? "true" : "false");
  j += ",\"auto\":";        j += (autoRecord ? "true" : "false");
  j += ",\"stabAbs\":";     j += String(stabAbs, 3);
  j += ",\"holdMs\":";      j += (uint32_t)STABLE_HOLD_MS;
  j += ",\"heldMs\":";      j += (uint32_t)((fieldState == FIELD_STABLE)
                                            ? (millis() - fieldStableMs) : 0);
  j += ",\"recN\":";        j += recCount;
  j += ",\"recSeq\":";      j += recSeq;
  j += ",\"site\":\"";      j += jsonEscape(String(siteLabel));
  j += "\"}";
  j += ",\"next\":";        j += (remain / 1000);
  j += ",\"batV\":";        j += String(batVoltage, 3);
  j += ",\"batPct\":";      j += batPercent;
  j += ",\"charge\":\"";    j += chargeStateName(chargeState);
  j += "\",\"power\":";     j += (chargerPresent ? "true" : "false");
  j += ",\"powerOk\":";    j += (powerPathOk ? "true" : "false");
  j += ",\"chargeMode\":";  j += (chargeModeActive ? "true" : "false");
  j += ",\"exc\":";         j += (excAlwaysOn ? "true" : "false");
  j += ",\"sleep\":";       j += (sleepEnabled ? "true" : "false");
  j += ",\"model\":\"";     j += curveModelName(curveModel);
  j += "\",\"a\":";          j += String(a, 4);
  j += ",\"b\":";           j += String(b, 4);
  j += ",\"c\":";           j += String(c, 4);
  j += ",\"p1\":";          j += String(p1, 6);
  j += ",\"p2\":";          j += String(p2, 6);
  j += ",\"p3\":";          j += String(p3, 6);
  j += ",\"salSamples\":";  j += salSamples;
  j += ",\"tempSamples\":"; j += tempSamples;
  j += ",\"apMode\":";      j += (wifiIsAp ? "true" : "false");
  j += ",\"ssid\":\"";      j += jsonEscape(wifiSsid);
  j += "\",\"ip\":\"";      j += wifiIpStr;
  j += "\",\"clients\":";   j += (wifiIsAp ? (int)WiFi.softAPgetStationNum() : 1);
  j += ",\"rssi\":";        j += (wifiIsAp ? 0 : (int)WiFi.RSSI());
  j += ",\"seq\":";        j += histSeq;
  j += ",\"histN\":";      j += histCount;
  j += ",\"period\":";     j += (uint32_t)(SEND_PERIOD_MS / 1000);
  j += ",\"heap\":";        j += (uint32_t)ESP.getFreeHeap();

  // The fault set, in LED order, each with the blink code the board is showing
  // for it. Same word the LED and `diag` read, so the page can never disagree
  // with what you are looking at.
  j += ",\"faults\":[";
  bool firstFault = true;
  for (int i = 0; i < FAULT_TABLE_N; i++) {
    uint32_t bit = FAULT_TABLE[i].bit;
    if (!(faultLatched & bit)) continue;
    if (!firstFault) j += ',';
    firstFault = false;
    j += "{\"c\":\"";   j += faultClass(bit);
    j += "\",\"n\":";   j += faultCode(bit);
    j += ",\"held\":";  j += (faultRaw & bit) ? "false" : "true";
    j += ",\"t\":\"";   j += jsonEscape(FAULT_TABLE[i].name);
    j += "\"}";
  }
  j += "]";
  j += "}";
  server.send(200, "application/json", j);
}

// Run a command exactly as the serial terminal would. The reply goes back as
// the HTTP body AND through Log, so it also shows up in the live log pane and
// on the USB monitor - one transcript, whoever typed it.
void wifiHandleCmd() {
  if (!server.hasArg("c")) { server.send(400, "text/plain", "missing ?c="); return; }
  String cmd = server.arg("c");
  cmd.trim();
  if (cmd.length() == 0) { server.send(400, "text/plain", "empty command"); return; }

  Log.print(F("[wifi] > ")); Log.println(cmd);
  StringStream cap;
  processCommand(cmd, cap);          // note: 'recal' reboots and never returns
  if (cap.out.length()) Log.print(cap.out);
  server.send(200, "text/plain", cap.out);
}

// Incremental log. The client passes the byte offset it has already seen and
// gets everything since; the new offset comes back in X-Log-Next. Plain text
// rather than JSON so the log never has to be escaped.
void wifiHandleLog() {
  uint32_t end    = logSeq;
  uint32_t oldest = (end > LOG_BUF_SIZE) ? (end - LOG_BUF_SIZE) : 0;
  uint32_t from   = 0;
  if (server.hasArg("from")) from = (uint32_t)strtoul(server.arg("from").c_str(), nullptr, 10);
  if (from < oldest) from = oldest;   // client fell behind the ring
  if (from > end)    from = end;      // client ahead of us: we rebooted

  String chunk;
  chunk.reserve(end - from + 1);
  for (uint32_t i = from; i < end; i++) chunk += logBuf[i % LOG_BUF_SIZE];

  server.sendHeader("X-Log-Next", String(end));
  server.send(200, "text/plain", chunk);
}

// Reading history, oldest first. Compact parallel arrays rather than an array
// of objects: 120 readings of five fields would repeat the key names 120 times,
// and this whole response has to be built in a String on a chip with ~130 KB of
// heap once the radio is up.
void wifiHandleHistory() {
  String j;
  j.reserve((size_t)histCount * 46 + 128);
  j  = "{\"n\":";    j += histCount;
  j += ",\"seq\":";  j += histSeq;
  j += ",\"now\":";  j += (uint32_t)(millis() / 1000);
  const char* keys[5] = { "t", "temp", "salt", "volts", "batV" };
  for (int k = 0; k < 5; k++) {
    j += ",\""; j += keys[k]; j += "\":[";
    for (int i = 0; i < histCount; i++) {
      if (i) j += ',';
      const Reading& r = hist[histIndex(i)];
      switch (k) {
        case 0: j += r.t;                  break;
        case 1: j += String(r.temp,  2);   break;
        case 2: j += String(r.salt,  2);   break;
        case 3: j += String(r.volts, 4);   break;
        default: j += String(r.batV, 3);   break;
      }
    }
    j += ']';
  }
  j += ",\"f\":[";
  for (int i = 0; i < histCount; i++) { if (i) j += ','; j += hist[histIndex(i)].flags; }
  j += "]}";
  server.send(200, "application/json", j);
}

// The same history as a spreadsheet. Served as an attachment so the browser
// saves it instead of rendering it - this is the device's only data export now
// that LoRa is off, so it is worth making it one click.
void wifiHandleCsv() {
  String c;
  c.reserve((size_t)histCount * 54 + 96);
  c = "seconds_since_boot,temp_c,salinity_g_l_at_25c,salinity_g_l_raw,probe_v,"
      "battery_v,temp_ok,salinity_ok,temp_compensated\n";
  for (int i = 0; i < histCount; i++) {
    const Reading& r = hist[histIndex(i)];
    c += r.t;                    c += ',';
    c += String(r.temp,    2);   c += ',';
    c += String(r.salt,    2);   c += ',';
    c += String(r.saltRaw, 2);   c += ',';
    c += String(r.volts,   4);   c += ',';
    c += String(r.batV,    3);   c += ',';
    c += (r.flags & 1) ? '1' : '0';  c += ',';
    c += (r.flags & 2) ? '1' : '0';  c += ',';
    c += (r.flags & 4) ? '1' : '0';  c += '\n';
  }
  server.sendHeader("Content-Disposition", "attachment; filename=\"gambia-readings.csv\"");
  server.send(200, "text/csv", c);
}

// Committed observations, oldest first. An array of objects rather than the
// parallel arrays /api/history uses: there are at most REC_MAX of them, each
// carries a string, and the table that renders them wants whole rows.
void wifiHandleRecords() {
  String j;
  j.reserve((size_t)recCount * 126 + 96);
  j  = "{\"n\":";     j += recCount;
  j += ",\"seq\":";   j += recSeq;
  j += ",\"now\":";   j += (uint32_t)(millis() / 1000);
  j += ",\"rows\":[";
  for (int i = 0; i < recCount; i++) {
    const FieldRecord& r = recs[recIndex(i)];
    if (i) j += ',';
    j += "{\"seq\":";     j += r.seq;
    j += ",\"t\":";       j += r.t;
    j += ",\"salt\":";    j += String(r.salt, 3);
    j += ",\"sd\":";      j += String(r.sd, 4);
    j += ",\"temp\":";    j += String(r.temp, 2);
    j += ",\"volts\":";   j += String(r.volts, 4);
    j += ",\"batV\":";    j += String(r.batV, 3);
    j += ",\"n\":";       j += r.n;
    // tempOk is NOT implied by comp: comp is also false whenever compensation is
    // simply switched off, so on its own it cannot tell a real reading from an
    // absent sensor's 0.00 degC placeholder.
    j += ",\"tempOk\":";  j += (r.flags & 1) ? "true" : "false";
    j += ",\"comp\":";    j += (r.flags & 4) ? "true" : "false";
    j += ",\"site\":\"";  j += jsonEscape(String(r.site));
    j += "\"}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

// The observations as a spreadsheet. This is the file a hydrologist opens, so
// the header names units and the reference temperature explicitly rather than
// leaving "salinity" to be guessed at, and the uncompensated rows are marked
// instead of being silently mixed in with the corrected ones.
void wifiHandleRecordsCsv() {
  String c;
  c.reserve((size_t)recCount * 92 + 176);
  // temp_valid earns its column. When the DS18B20 is absent liveTemp is never
  // invalidated, so the record stores its initialiser and the sheet showed a
  // flat 0.00 in temp_c - indistinguishable from real 0 degC water, which is a
  // number a hydrologist would have no reason to disbelieve. temp_compensated
  // could not stand in for it, since that is also 0 whenever compensation is
  // simply turned off. The flag was in FieldRecord::flags all along; nothing
  // published it.
  c = "observation,seconds_since_boot,site,salinity_g_l_at_25c,sd_g_l,n_samples,"
      "temp_c,temp_valid,probe_v,battery_v,temp_compensated\n";
  for (int i = 0; i < recCount; i++) {
    const FieldRecord& r = recs[recIndex(i)];
    bool tOk = (r.flags & 1) != 0;
    c += r.seq;               c += ',';
    c += r.t;                 c += ',';
    c += r.site;              c += ',';
    c += String(r.salt,  3);  c += ',';
    c += String(r.sd,    4);  c += ',';
    c += r.n;                 c += ',';
    // Empty rather than 0.00 when there was no sensor: a blank cell reads as
    // "missing" in every spreadsheet and plotting tool, a zero does not.
    if (tOk) c += String(r.temp, 2);
    c += ',';
    c += tOk ? '1' : '0';     c += ',';
    c += String(r.volts, 4);  c += ',';
    c += String(r.batV,  3);  c += ',';
    c += (r.flags & 4) ? '1' : '0';
    c += '\n';
  }
  server.sendHeader("Content-Disposition", "attachment; filename=\"gambia-observations.csv\"");
  server.send(200, "text/csv", c);
}

// ================== WIFI CONSOLE: RADIO LIFECYCLE ==================
// Routes are registered once at boot: WebServer::on() appends to a list and
// stop() does not clear it, so re-registering on every bring-up would leak.
void wifiInitRoutes() {
  server.on("/",           wifiHandleRoot);
  server.on("/api/status", wifiHandleStatus);
  server.on("/api/cmd",    wifiHandleCmd);
  server.on("/api/log",    wifiHandleLog);
  server.on("/api/history", wifiHandleHistory);
  server.on("/api/csv",     wifiHandleCsv);
  server.on("/api/records",     wifiHandleRecords);
  server.on("/api/records.csv", wifiHandleRecordsCsv);
  server.onNotFound(wifiHandleRoot);
}

void wifiServeBegin() {
  server.begin();
  wifiState       = WIFI_READY;
  wifiSplashUntil = millis() + 6000;   // OLED shows the SSID/IP card for 6 s
  Log.print(F("[wifi] console up on "));
  Log.print(wifiIsAp ? F("AP ") : F("STA "));
  Log.print(wifiSsid);
  Log.print(F("  ->  http://"));
  Log.println(wifiIpStr);
}

void wifiStartAp() {
  char ssid[20];
  // eFuse MAC, so two boards on the same bench never collide. Read directly:
  // it does not need the radio to be initialised.
  snprintf(ssid, sizeof(ssid), "Gambia-%04X", (uint16_t)(ESP.getEfuseMac() >> 32));
  WiFi.persistent(false);              // don't wear the flash storing state
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(ssid, WIFI_AP_PASS)) {
    Log.println(F("[wifi] softAP failed to start."));
    wifiState = WIFI_DOWN;
    return;
  }
  wifiIsAp  = true;
  wifiSsid  = ssid;
  wifiIpStr = WiFi.softAPIP().toString();
  wifiServeBegin();
}

void wifiBringUp() {
  if (ESP.getFreeHeap() < WIFI_MIN_HEAP) {
    // Most likely cause: Bluetooth Classic is also up in calibration mode.
    Log.print(F("[wifi] refusing to start, only "));
    Log.print(ESP.getFreeHeap());
    Log.println(F(" bytes of heap free. Policy set to off."));
    wifiPolicy = WIFI_POL_OFF;         // don't retry on every loop pass
    return;
  }
  if (staSsid.length() > 0) {
    Log.print(F("[wifi] joining ")); Log.println(staSsid);
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.begin(staSsid.c_str(), staPass.c_str());
    wifiStaStartMs = millis();
    wifiState      = WIFI_STA_CONNECTING;
    return;
  }
  wifiStartAp();
}

void wifiTakeDown() {
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  wifiState = WIFI_DOWN;
  wifiSsid  = "";
  wifiIpStr = "";
  Log.println(F("[wifi] console down."));
}

// Called every loop(), before anything that might return early - the console
// has to keep answering while charge mode or calibration mode owns the loop.
void wifiTick() {
#if WIFI_ENABLED
  bool want = (wifiPolicy == WIFI_POL_ON) ||
              (wifiPolicy == WIFI_POL_AUTO && chargerPresent);

  if (want  && wifiState == WIFI_DOWN)  wifiBringUp();
  if (!want && wifiState != WIFI_DOWN)  wifiTakeDown();

  if (wifiState == WIFI_STA_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiIsAp  = false;
      wifiSsid  = staSsid;
      wifiIpStr = WiFi.localIP().toString();
      wifiServeBegin();
    } else if (millis() - wifiStaStartMs > WIFI_STA_TIMEOUT_MS) {
      Log.println(F("[wifi] join timed out, falling back to AP."));
      WiFi.disconnect(true, true);
      wifiStartAp();
    }
    return;                            // nothing to serve yet
  }

  if (wifiState == WIFI_READY) server.handleClient();
#endif
}

const char* wifiPolicyName() {
  switch (wifiPolicy) {
    case WIFI_POL_ON:  return "on   - forced up (deep sleep suppressed)";
    case WIFI_POL_OFF: return "off  - radio stays down";
    default:           return "auto - up only while USB-C power is present";
  }
}

// Persist the radio policy. It used to live only in RAM, like `sleep` and
// `exc`, which meant every reset - and every reflash - quietly put it back to
// `auto`; and under `auto` on battery there is no AP broadcasting at all, which
// from the outside looks exactly like a console that does not work.
void wifiSavePolicy() {
  prefs.begin("wifi", false);
  prefs.putUChar("pol", (uint8_t)wifiPolicy);
  prefs.end();
}

// Persist STA credentials. Empty ssid means "AP mode".
void wifiSaveSta(const String& ssid, const String& pass) {
  staSsid = ssid;
  staPass = pass;
  prefs.begin("wifi", false);
  prefs.putString("ssid", staSsid);
  prefs.putString("pass", staPass);
  prefs.end();
}

void wifiLoadSta() {
  prefs.begin("wifi", true);
  staSsid = prefs.getString("ssid", "");
  staPass = prefs.getString("pass", "");
  uint8_t pol = prefs.getUChar("pol", (uint8_t)WIFI_POL_AUTO);
  prefs.end();
  // Guard the stored value: a corrupt or future-version NVS entry must not cast
  // into a policy that does not exist.
  wifiPolicy = (pol <= (uint8_t)WIFI_POL_ON) ? (WifiPolicy)pol : WIFI_POL_AUTO;
}

// ================== COMMAND PROCESSING ==================
// Prints the full command list to whichever stream asked for it.
// `diag`: the whole diagnostic picture in one reply, ending with the exact blink
// code the LED is showing right now. That last part is the point - a code you
// counted from across the room can be checked against the firmware itself,
// rather than against a card that may have gone stale.
void faultReport(Stream& out) {
  out.println(F("=== DIAGNOSTICS ==="));
  out.print(F("Mode      : "));
  out.print(currentMode == MODE_CALIBRATION ? F("calibration")
            : (currentMode == MODE_FIELD ? F("field") : F("logger")));
  if (chargeModeActive) out.print(F("   (paused: charge mode)"));
  out.println();
  out.print(F("Uptime    : ")); out.print(millis() / 1000); out.println(F(" s"));
  out.print(F("Readings  : ")); out.print(histSeq);
  if (currentMode == MODE_LOGGER && !chargeModeActive) {
    out.print(F(" taken, last completed "));
    out.print((millis() - lastMeasOkMs) / 1000);
    out.print(F(" s ago"));
  }
  out.println();

  if (currentMode == MODE_FIELD) {
    out.print(F("Field     : ")); out.print(fieldStateName(fieldState));
    out.print(F(", probe ")); out.print(immersed ? F("IN water") : F("dry"));
    out.print(F(" ("));       out.print(liveVolts, 3);
    out.print(F(" V, threshold ")); out.print(immersionV, 3); out.println(F(" V)"));
    out.print(F("  window  : n=")); out.print(stabCount);
    out.print(F("/"));              out.print(STAB_WINDOW);
    out.print(F("  mean ")); out.print(stabMean, 3);
    out.print(F("  sd ")); out.print(stabSd, 4);
    out.print(F("  band ")); out.print(stabBand(), 4);
    out.println(F(" g/L"));
    out.print(F("  records : ")); out.print(recCount);
    out.print(F(" kept of ")); out.print(recSeq);
    out.print(F(" committed, auto-record "));
    out.println(autoRecord ? F("ON") : F("OFF"));
    if (siteLabel[0]) { out.print(F("  site    : ")); out.println(siteLabel); }
  }

  out.print(F("Salinity  : "));
  if (!tempCompEnabled) {
    out.println(F("RAW - temperature compensation is OFF ('tempcomp on')"));
  } else {
    out.print(F("normalised to ")); out.print(TEMP_REF_C, 0);
    out.print(F(" C, alpha=")); out.print(tempCoeff, 4); out.print(F("/C"));
    out.println(tempAddrValid ? F("") : F("  [NO TEMPERATURE - reporting raw]"));
  }
  out.print(F("Battery   : ")); out.print(batVoltage, 2); out.print(F(" V, "));
  out.print(batPercent);        out.print(F(" %,  charger: "));
  out.println(chargeStateName(chargeState));

  out.println(F("Peripherals:"));
  out.print(F("  OLED      : ")); out.println(displayOk     ? F("ok") : F("NOT RESPONDING"));
  out.print(F("  DS18B20   : ")); out.println(tempAddrValid ? F("on the bus") : F("NOT FOUND"));
  out.print(F("  Power sense: ")); out.println(powerPathOk  ? F("ok") : F("DISCONNECTED"));
  out.print(F("  RGB driver: ")); out.println(rgbHwPwm      ? F("LEDC PWM")
                                                            : F("on/off only (LEDC attach failed)"));
  out.print(F("  WiFi      : "));
  if      (wifiState == WIFI_READY)          { out.print(F("up, http://")); out.println(wifiIpStr); }
  else if (wifiState == WIFI_STA_CONNECTING)   out.println(F("joining..."));
  else                                         out.println(F("down"));

  out.println(F("Faults (what the LED is flashing):"));
  if (!faultLatched) {
    out.println(F("  none - the LED is on its healthy heartbeat"));
  } else {
    for (int i = 0; i < FAULT_TABLE_N; i++) {
      uint32_t bit = FAULT_TABLE[i].bit;
      if (!(faultLatched & bit)) continue;
      out.print(F("  "));
      out.print(faultClass(bit) == 'R' ? F("RED   x") : F("AMBER x"));
      out.print(faultCode(bit));
      out.print(F("  "));
      out.print(FAULT_TABLE[i].name);
      if (!(faultRaw & bit)) out.print(F("   [cleared, still held]"));
      out.println();
    }
    out.println(F("  (with more than one up, the codes are flashed in turn - one per cycle)"));
  }

  out.print(F("LED       : "));
  if      (!rgbEnabled) out.println(F("blanked - 'rgb on' to restore"));
  else if (rgbManual)   out.println(F("held at a manual colour - 'rgb auto' to restore"));
  else                  out.println(F("automatic"));
}

void printHelp(Stream& out) {
  out.println(F("Available commands:"));
  out.println(F("  help        - show this list"));
  out.println(F("  coef        - print the coefficients of the curve in use"));
  out.println(F("  model       - salinity curve: quad (V=P1*S^2+P2*S+P3) or power (a*V^b+c)"));
  out.println(F("  pcoef P1 P2 P3 - type the quadratic coefficients in directly"));
  out.println(F("  cal         - manual edit of a, b, c (power curve)"));
  out.println(F("  autocal     - start guided auto-calibration"));
  out.println(F("  sample <S>  - record a point at known salinity S (g/L)"));
  out.println(F("  points      - list recorded calibration points"));
  out.println(F("  undo        - remove the last recorded point"));
  out.println(F("  fit         - fit the live curve to the points (>=3 points for quad)"));
  out.println(F("  cancel      - leave auto-calibration"));
  out.println(F("  start       - save the coefficients and run Measuring mode"));
  out.println(F("  recal       - reboot into calibration mode (any time)"));
  out.println(F("  adc [N]     - raw ADC read, OFF vs ON excitation (N bursts)"));
  out.println(F("  exc on/off  - latch excitation square wave always ON / restore"));
  out.println(F("  samples [N] - show/set salinity ADC samples averaged per reading"));
  out.println(F("  tsamples [N]- show/set DS18B20 temp conversions averaged per reading"));
  out.println(F("  sleep on/off- allow deep sleep between readings / stay always on"));
  out.println(F(" -- field work --"));
  out.println(F("  mode        - field (operator present) or logger (unattended)"));
  out.println(F("  site <name> - label stamped on every observation from now on"));
  out.println(F("  rec         - commit the current window as one observation"));
  out.println(F("  recs        - list committed observations; 'recs clear', 'recs undo'"));
  out.println(F("  autorec     - on/off: commit automatically once the reading settles"));
  out.println(F("  screen      - simple/detail: field-mode OLED layout"));
  out.println(F("  stability   - show/set the settle band in g/L"));
  out.println(F("  immersion   - show/set the probe voltage that counts as 'in water'"));
  out.println(F(" -- measurement quality --"));
  out.println(F("  tempcomp    - on/off: normalise salinity to 25 C (~2 %/degC)"));
  out.println(F("  tempco <a>  - the temperature coefficient itself, per degC"));
  out.println(F("  diag        - full self-check: peripherals, faults, and the LED code"));
  out.println(F("  rgb         - status LED: on/off/auto/test, bright <N>, <R> <G> <B>"));
  out.println(F("  bat         - battery voltage, percentage, charger state, input power"));
  out.println(F("  CALSINGLE V - calibrate the battery divider against a multimeter"));
  out.println(F("                CAL1 V + CAL2 V = two-point; CALSHOW, CALRESET"));
  out.println(F("  power       - raw charger pins + a verdict (USB-C troubleshooting)"));
  out.println(F("  hist        - the last readings as a table; /api/csv downloads them"));
  out.println(F("  charge      - auto/ignore: pause measurements while USB-C is plugged in"));
  out.println(F("  wifi        - web console: on/off/auto, ap, sta <ssid> <pass>"));
}

// Reboots the device into calibration mode. Works from USB, Bluetooth or a
// LoRa downlink: it clears the NVS "done" flag and restarts, so on the next
// boot setup() opens Bluetooth + USB calibration again. Rebooting is the clean
// way to release the LoRa SPI bus / LMIC state and reclaim the pins.
void triggerRecalibration(Stream* out) {
  Log.println(F("Recalibration requested. Rebooting into calibration mode..."));
  if (out) {
    out->println(F("Recalibration requested. Rebooting into calibration mode..."));
    out->flush();
  }
  if (displayOk) displayStatus(" Recalibrating...");
  prefs.begin("calib", false);
  prefs.putBool("done", false);
  prefs.end();
  delay(400);
  ESP.restart();
}

// Shared parser: handles one trimmed command coming from EITHER Bluetooth or
// USB serial. `out` is the stream we reply on, so answers go back to whoever
// sent the command (SerialBT for BT clients, Serial for the USB terminal).
void processCommand(const String& command, Stream& out) {
  Log.print(F("Command: '"));
  Log.print(command);
  Log.println(F("'"));

  // ---- Global commands (any mode, any state) ----
  if (command.equalsIgnoreCase("help")) {
    printHelp(out);
    return;
  }
  if (command.equalsIgnoreCase("recal")) {
    triggerRecalibration(&out);
    return;
  }

  // ---- diag: every fault bit, spelled out, with its blink code (any mode) ----
  if (command.equalsIgnoreCase("diag") || command.equalsIgnoreCase("diagnostics")) {
    faultReport(out);
    return;
  }

  // ---- coef: print the active calibration coefficients (any mode) ----
  if (command.equalsIgnoreCase("coef") || command.equalsIgnoreCase("coeff") ||
      command.equalsIgnoreCase("coeffs") || command.equalsIgnoreCase("coefficients")) {
    if (curveModel == CURVE_QUAD) {
      out.println(F("Calibration coefficients (model: V = P1*S^2 + P2*S + P3, inverted):"));
      out.print(F("  P1 = ")); out.println(p1, 6);
      out.print(F("  P2 = ")); out.println(p2, 6);
      out.print(F("  P3 = ")); out.println(p3, 6);
      out.print(F("  zero-salinity intercept: ")); out.print(p3, 4);
      out.println(F(" V  (anything below reads 0 g/L)"));
      float sMax, vMax;
      if (quadCeiling(sMax, vMax)) {
        out.print(F("  ceiling: ")); out.print(sMax, 2);
        out.print(F(" g/L at ")); out.print(vMax, 4);
        out.println(F(" V, above which readings are CLAMPED"));
      }
      out.println(F("  the power coefficients a/b/c are stored but idle ('model power')"));
    } else {
      out.println(F("Calibration coefficients (model: S = a * V^b + c):"));
      out.print(F("  a = ")); out.println(a, 4);
      out.print(F("  b = ")); out.println(b, 4);
      out.print(F("  c = ")); out.println(c, 4);
      out.println(F("  the quadratic P1/P2/P3 are stored but idle ('model quad')"));
    }
    if (lastFitR2 >= 0.0f) { out.print(F("  last fit R2 = ")); out.println(lastFitR2, 4); }
    return;
  }

  // ---- model: which curve turns probe volts into salinity (any mode) ----
  if (command.equalsIgnoreCase("model") ||
      (command.length() >= 6 && command.substring(0, 6).equalsIgnoreCase("model "))) {
    String arg = (command.length() > 6) ? command.substring(6) : String("");
    arg.trim();

    if (arg.length()) {
      CurveModel want;
      if      (arg.equalsIgnoreCase("quad")  || arg.equalsIgnoreCase("quadratic")) want = CURVE_QUAD;
      else if (arg.equalsIgnoreCase("power") || arg.equalsIgnoreCase("pow"))       want = CURVE_POWER;
      else { out.println(F("Usage: model quad | power   (or bare 'model' to show it)")); return; }

      curveModel = want;
      quadCeilingTripped = false;
      // The R2 on screen belongs to whichever curve was last fitted; keeping it
      // across a model switch would attach one curve's fit quality to another's
      // coefficients. `fit` sets it again.
      lastFitR2 = -1.0f;
      prefs.begin("calib", false);
      prefs.putUChar("model", (uint8_t)curveModel);
      prefs.end();
      out.println(F("Saved. This takes effect on the very next reading."));
    }

    out.print(F("Salinity curve: "));
    out.println(curveModel == CURVE_QUAD
                  ? F("quad   V = P1*S^2 + P2*S + P3, solved for S")
                  : F("power  S = a * V^b + c"));
    out.print(F("  probe now: ")); out.print(liveVolts, 4);
    out.print(F(" V -> ")); out.print(voltageToSalinity(liveVolts), 3);
    out.println(F(" g/L (uncompensated)"));
    out.println(F("'coef' prints the live coefficients; 'pcoef' types the quadratic ones."));
    return;
  }

  // ---- pcoef: type the quadratic coefficients directly (any mode) ----
  // The manual `cal` walk-through covers a/b/c and is calibration-mode only,
  // because changing the curve mid-deployment used to mean a re-flash. This is
  // the same idea for P1/P2/P3 in one line, and it is allowed in any mode: the
  // whole point of a bench-characterised curve is being able to load it into a
  // device that is already running.
  if (command.equalsIgnoreCase("pcoef") ||
      (command.length() >= 6 && command.substring(0, 6).equalsIgnoreCase("pcoef "))) {
    String arg = (command.length() > 6) ? command.substring(6) : String("");
    arg.trim();

    if (arg.length()) {
      // Three numbers separated by whitespace. parseFloatStrict() per field, so
      // `pcoef -0,0006 ...` is refused outright rather than silently read as 0 -
      // a zero P1 is a legal-looking quadratic that is actually a straight line.
      float v[3];
      int   got = 0;
      int   i   = 0;
      while (got < 3 && i < (int)arg.length()) {
        while (i < (int)arg.length() && arg[i] == ' ') i++;
        int start = i;
        while (i < (int)arg.length() && arg[i] != ' ') i++;
        if (i == start) break;
        if (!parseFloatStrict(arg.substring(start, i), v[got])) {
          out.print(F("Not a number: '")); out.print(arg.substring(start, i));
          out.println(F("'  (a dot, not a comma; no exponent)"));
          return;
        }
        got++;
      }
      if (got < 3) {
        out.println(F("Usage: pcoef <P1> <P2> <P3>     e.g.  pcoef -0.0003 0.0437 0.0990"));
        return;
      }
      if (fabsf(v[1]) < 1e-12f && fabsf(v[0]) < QUAD_P1_MIN) {
        out.println(F("Refused: P1 and P2 both ~0 leaves a constant, which cannot be"));
        out.println(F("inverted - every voltage would read the same salinity."));
        return;
      }
      // P2 is the slope of the forward curve at S=0. Negative means voltage
      // FALLS as the water gets saltier, which this front end does not do, and
      // the + branch of the quadratic formula has no positive root on such a
      // curve - every reading would come back 0.000 g/L with nothing to say why.
      // Warned, not refused: the coefficients are the operator's to set, and a
      // sign typo is far likelier than a genuinely inverted probe.
      if (v[1] <= 0.0f) {
        out.println(F("WARNING: P2 <= 0 means the curve FALLS as salinity rises. This probe"));
        out.println(F("         reads the other way, and the + root has no positive solution"));
        out.println(F("         on such a curve - every reading will be 0.000 g/L."));
        out.println(F("         Check the sign of P2."));
      }

      p1 = v[0]; p2 = v[1]; p3 = v[2];
      quadCeilingTripped = false;
      lastFitR2 = -1.0f;          // typed in, not fitted: there is no R2 for these
      prefs.begin("calib", false);
      prefs.putFloat("p1", p1);
      prefs.putFloat("p2", p2);
      prefs.putFloat("p3", p3);
      prefs.end();
      snprintf(autoCalMsg, sizeof(autoCalMsg), "P1/P2/P3 set by hand");
      out.println(F("Saved to NVS."));
      if (curveModel != CURVE_QUAD)
        out.println(F("NOTE: the power curve is live, so these are not in use. 'model quad'."));
    }

    out.println(F("Quadratic curve  V = P1*S^2 + P2*S + P3   (salinity by the quadratic formula)"));
    out.print(F("  P1 = ")); out.println(p1, 6);
    out.print(F("  P2 = ")); out.println(p2, 6);
    out.print(F("  P3 = ")); out.println(p3, 6);
    float sMax, vMax;
    if (quadCeiling(sMax, vMax)) {
      out.print(F("  valid ")); out.print(p3, 4);
      out.print(F(" V .. ")); out.print(vMax, 4);
      out.print(F(" V  ->  0 .. ")); out.print(sMax, 2); out.println(F(" g/L"));
    } else {
      out.print(F("  P1 >= 0: the curve does not turn over, so there is no upper limit"));
      out.println(F(" beyond the ADC's own 3.3 V."));
    }
    return;
  }

  // ---- bat: battery voltage + charger state (any mode). ----
  if (command.equalsIgnoreCase("bat") || command.equalsIgnoreCase("battery")) {
    float vraw = readBatteryVoltageRaw();
    float v    = batCalApply(vraw);
    out.print(F("Battery: ")); out.print(v, 3);
    out.print(F(" V  (")); out.print(vraw / BAT_DIVIDER, 3);
    out.print(F(" V at GPIO")); out.print(batSensePin);
    out.print(F(", x")); out.print(BAT_DIVIDER, 1); out.println(F(" divider)"));
    out.print(F("Raw:     ")); out.print(vraw, 3);
    out.print(F(" V uncorrected   GAIN ")); out.print(batGain, 5);
    out.print(F("  OFFSET ")); out.print(batOffset, 4); out.println(F(" V"));
    if (batCalIsDefault())
      out.println(F("         divider uncalibrated - run CALSINGLE <multimeter volts>"));
    out.print(F("Level:   ")); out.print(batteryPercentFromVoltage(v));
    out.print(F(" %  (filtered ")); out.print(batVoltage, 3);
    out.print(F(" V -> ")); out.print(batPercent); out.println(F(" % on screen)"));
    out.print(F("Charger: ")); out.print(chargeStateName(readChargeState()));
    out.print(F("   |   Input power: "));
    out.println(inputPowerGood() ? F("present") : F("absent"));
    if (chargerPresent && chargeState == CHG_CHARGING) {
      out.println(F("NOTE: while charging the divider reads the charger output, not the"));
      out.println(F("      resting cell voltage - the percentage reads high."));
    }
    if (!powerPathOk) {
      out.println(F("NOTE: charger monitoring is disabled - the STAT/PG pins look"));
      out.println(F("      disconnected. Run 'power' for the detail."));
    }
    if (chargeModeActive) out.println(F("Charge mode ACTIVE: measurements paused."));
    return;
  }

  // ---- CAL1/CAL2/CALSINGLE/CALSHOW/CALRESET: per-device battery divider
  //      calibration (any mode). Nothing to do with `cal`, which is the
  //      salinity probe's own routine and only runs in calibration mode - the
  //      names are close because both are calibration, but they touch different
  //      NVS namespaces and neither can clobber the other.
  if (batCalIsCalCommand(command)) {
    // The raw reading is taken here, once, and handed in: every point must be
    // recorded on the UNCORRECTED scale or each calibration would compound the
    // last one.
    if (batCalCommand(command, out, readBatteryVoltageRaw())) return;
  }

  // ---- power: raw charger-pin dump. There is NO charge indicator LED
  //      anywhere on the V2 board, and Q1 isolates the battery the instant
  //      VBUS appears, so these three open-drain pins are the only evidence
  //      the board can give that a cable is actually delivering power. This
  //      is what separates "the charger is broken" from "the charger is
  //      working silently, exactly as designed". ----
  if (command.equalsIgnoreCase("power") || command.equalsIgnoreCase("usb")) {
    int rpg = digitalRead(pgPin);
    int rs1 = digitalRead(stat1Pin);
    int rs2 = digitalRead(stat2Pin);

    out.println(F("---- MCP73833 raw pins (open drain, active LOW, 10k to 3V3) ----"));
    out.print(F("  PG    GPIO")); out.print(pgPin);
    out.print(rpg == LOW ? F(" = LOW   -> input power PRESENT")
                         : F(" = HIGH  -> input power ABSENT"));
    out.println();
    out.print(F("  STAT1 GPIO")); out.print(stat1Pin);
    out.println(rs1 == LOW ? F(" = LOW") : F(" = HIGH"));
    out.print(F("  STAT2 GPIO")); out.print(stat2Pin);
    out.println(rs2 == LOW ? F(" = LOW") : F(" = HIGH"));
    out.print(F("  decoded:      ")); out.println(chargeStateName(readChargeState()));
    out.print(F("  battery:      ")); out.print(readBatteryVoltage(), 3);
    out.println(F(" V at BAT (via the 1M/1M divider on GPIO33)"));
    out.println();

    out.println(F("---- verdict ----"));
    if (!powerPathOk) {
      out.println(F("  These readings are NOT trustworthy: all three pins are low while"));
      out.println(F("  BAT reads flat, which cannot happen on a connected V2 board. The"));
      out.println(F("  most likely cause is that this ESP32 is not seated in the carrier,"));
      out.println(F("  so the input-only pins are floating with nothing to pull them up."));
      out.println(F("  Charge mode is disabled while this holds, so measurements continue."));
      return;
    }
    if (rpg == LOW) {
      out.println(F("  USB-C power IS reaching the charger."));
      if      (rs1 == LOW  && rs2 == HIGH) out.println(F("  The cell is charging right now. This is the normal working state."));
      else if (rs1 == HIGH && rs2 == LOW ) out.println(F("  Charge complete - the cell is full, so no current is flowing."));
      else if (rs1 == LOW  && rs2 == LOW ) out.println(F("  FAULT: safety timer, thermal, or a dead cell. Check the battery."));
      else                                 out.println(F("  Standby: power is in, but no battery is detected on BAT."));
    } else {
      out.println(F("  No input power seen - nothing is reaching the MCP73833."));
      out.println(F("  The dev-kit's own micro-USB does NOT feed the charger: 5VIN is"));
      out.println(F("  unconnected on V2, so only the board's USB-C jack can charge."));
      out.println(F("  Check, in this order:"));
      out.println(F("   1. Cable in the BOARD's USB-C jack, not the dev-kit's micro-USB?"));
      out.println(F("   2. Try a USB-A -> USB-C cable from a plain phone charger. A"));
      out.println(F("      C-to-C source only switches VBUS on if it sees the 5.1k CC"));
      out.println(F("      pulldowns (R29/R30) - an unfitted or dry one kills C-to-C"));
      out.println(F("      while A-to-C keeps working. That asymmetry is the tell."));
      out.println(F("   3. Meter it: 5 V between the D1 anode and GND with the cable in."));
      out.println(F("      5 V present but PG still HIGH -> MCP73833 or its VDD joint."));
      out.println(F("      No 5 V there            -> cable, source, or the jack's pads."));
    }
    out.println();
    out.println(F("NOTE: with the device switched OFF a healthy charger gives no sign at"));
    out.println(F("      all - no LED exists, and the STAT pull-ups live on the 3V3 rail"));
    out.println(F("      that only appears when Q2 is on. Silence is not a fault."));
    return;
  }

  // ---- hist: the reading ring as a table (any mode). ----
  if (command.equalsIgnoreCase("hist") || command.equalsIgnoreCase("history")) {
    if (histCount == 0) {
      out.println(F("No readings recorded yet."));
      return;
    }
    out.print(F("Last ")); out.print(histCount);
    out.print(F(" reading(s) of ")); out.print(histSeq);
    out.print(F(" taken (ring holds ")); out.print(HIST_MAX); out.println(F("):"));
    out.println(F("   age     temp     salinity   probe    batt"));
    uint32_t now = (uint32_t)(millis() / 1000);
    for (int i = 0; i < histCount; i++) {
      const Reading& r = hist[histIndex(i)];
      // 112, not 72: the numbers alone run ~48 characters and the two failure
      // markers add another 34. At 72 a reading that failed BOTH sensors - the
      // one row anybody scanning this listing is looking for - had
      // "[salinity failed]" silently cut off the end.
      char line[112];
      snprintf(line, sizeof(line), "  %4lus  %6.2f C  %7.2f g/L  %6.4f V  %5.3f V%s%s",
               (unsigned long)(now - r.t), r.temp, r.salt, r.volts, r.batV,
               (r.flags & 1) ? "" : "  [temp failed]",
               (r.flags & 2) ? "" : "  [salinity failed]");
      out.println(line);
    }
    out.println(F("Download the same data as a spreadsheet from http://<ip>/api/csv"));
    return;
  }

  // ---- rgb: status LED control and diagnostics (any mode). ----
  //      "rgb"               -> report state
  //      "rgb on|off"        -> blank the LED / bring it back
  //      "rgb auto"          -> drop a manual colour, resume status reporting
  //      "rgb test"          -> red/green/blue/white wiring check
  //      "rgb bright <0-255>"-> master brightness
  //      "rgb <R> <G> <B>"   -> hold a fixed colour until 'rgb auto'
  if (command.equalsIgnoreCase("rgb") ||
      (command.length() >= 4 && command.substring(0, 4).equalsIgnoreCase("rgb "))) {
    String arg = (command.length() > 4) ? command.substring(4) : String("");
    arg.trim();

    if (arg.length() == 0) {
      out.print(F("RGB LED: "));          out.print(rgbEnabled ? F("on") : F("off"));
      out.print(F(", mode "));            out.print(rgbManual ? F("manual") : F("auto"));
      out.print(F(", brightness "));      out.print(rgbBrightness);
      out.print(F("/255, driver "));      out.println(rgbHwPwm ? F("PWM") : F("on/off"));
      out.print(F("  showing: "));
      if (faultLatched) {
        out.print(faultClass(rgbCodeBit) == 'R' ? F("RED x") : F("AMBER x"));
        out.print(faultCode(rgbCodeBit));
        out.print(F(" - ")); out.println(faultName(rgbCodeBit));
        out.println(F("  ('diag' lists every fault that is up)"));
      } else {
        out.println(F("no faults - healthy heartbeat"));
      }
      out.println(F("  codes: N red flashes = hard fault, N amber = degraded, count = which"));
      out.println(F("  wiring: R=GPIO4 G=GPIO15 B=GPIO17, common anode (LOW = lit)"));
      out.println(F("  usage: rgb on|off|auto|test | rgb bright <0-255> | rgb <R> <G> <B>"));
      return;
    }
    if (arg.equalsIgnoreCase("off")) {
      rgbEnabled = false;
      rgbAllOff();
      out.println(F("RGB LED blanked (status tracking continues; 'rgb on' to restore)."));
      return;
    }
    if (arg.equalsIgnoreCase("on")) {
      rgbEnabled = true;
      out.println(F("RGB LED on."));
      return;
    }
    if (arg.equalsIgnoreCase("auto")) {
      rgbEnabled = true;
      rgbManual  = false;
      out.println(F("RGB LED back to automatic status colours."));
      return;
    }
    if (arg.equalsIgnoreCase("test")) {
      rgbSelfTest(out);
      return;
    }
    if (arg.length() >= 7 && arg.substring(0, 7).equalsIgnoreCase("bright ")) {
      int n = constrain(arg.substring(7).toInt(), 0, 255);
      rgbBrightness = (uint8_t)n;
      out.print(F("RGB brightness set to ")); out.print(rgbBrightness); out.println(F("/255"));
      return;
    }

    // Anything else: expect three space-separated 0-255 components.
    int sp1 = arg.indexOf(' ');
    int sp2 = (sp1 > 0) ? arg.indexOf(' ', sp1 + 1) : -1;
    if (sp1 > 0 && sp2 > sp1) {
      int r = constrain(arg.substring(0, sp1).toInt(), 0, 255);
      int g = constrain(arg.substring(sp1 + 1, sp2).toInt(), 0, 255);
      int b = constrain(arg.substring(sp2 + 1).toInt(), 0, 255);
      rgbEnabled = true;
      rgbManual  = true;                 // hold it until 'rgb auto'
      rgbWrite((uint8_t)r, (uint8_t)g, (uint8_t)b);
      out.print(F("RGB held at ")); out.print(r); out.print(' ');
      out.print(g); out.print(' '); out.print(b);
      out.println(F("  ('rgb auto' to resume status colours)"));
      return;
    }

    out.println(F("Unknown rgb option. Try: rgb on|off|auto|test, rgb bright <N>, rgb <R> <G> <B>"));
    return;
  }

  // ---- samples [N]: show or set the ADC samples averaged per reading (any mode) ----
  if (command.equalsIgnoreCase("samples") ||
      (command.length() >= 8 && command.substring(0, 8).equalsIgnoreCase("samples "))) {
    if (command.length() > 8) {
      int n = command.substring(8).toInt();
      n = constrain(n, SAL_SAMPLES_MIN, SAL_SAMPLES_MAX);
      salSamples = n;
      out.print(F("ADC samples per reading set to ")); out.println(salSamples);
    } else {
      out.print(F("ADC samples per reading: ")); out.print(salSamples);
      out.print(F("  (range ")); out.print(SAL_SAMPLES_MIN);
      out.print(F("-")); out.print(SAL_SAMPLES_MAX);
      out.println(F(", ~400 us/sample)"));
    }
    return;
  }

  // ---- tsamples [N]: show or set the DS18B20 conversions averaged per reading
  //      (any mode). Each conversion is ~750 ms, chained non-blocking. ----
  if (command.equalsIgnoreCase("tsamples") ||
      (command.length() >= 9 && command.substring(0, 9).equalsIgnoreCase("tsamples "))) {
    if (command.length() > 9) {
      int n = command.substring(9).toInt();
      n = constrain(n, TEMP_SAMPLES_MIN, TEMP_SAMPLES_MAX);
      tempSamples = n;
      out.print(F("Temp conversions per reading set to ")); out.println(tempSamples);
    } else {
      out.print(F("Temp conversions per reading: ")); out.print(tempSamples);
      out.print(F("  (range ")); out.print(TEMP_SAMPLES_MIN);
      out.print(F("-")); out.print(TEMP_SAMPLES_MAX);
      out.println(F(", ~750 ms/conversion)"));
    }
    return;
  }

  // ---- sleep on/off: deep sleep between readings. Logger mode only - field
  //      mode has an operator standing over it and a hardware power latch for
  //      an off switch, so sleeping between dips would only lose the reading. ----
  if (command.equalsIgnoreCase("sleep on")) {
    sleepEnabled = true;
    out.println(F("Sleep ENABLED: device may deep-sleep between awake windows."));
    if (currentMode == MODE_FIELD)
      out.println(F("  (no effect in field mode - 'mode logger' to use it)"));
    return;
  }
  if (command.equalsIgnoreCase("sleep off")) {
    sleepEnabled = false;
    awakeStartMs = millis();   // reopen the window so measuring resumes immediately
    out.println(F("Sleep DISABLED: always on, measuring every 15 s (bench)."));
    return;
  }

  // ---- mode field|logger: pick the instrument (any mode). Persisted, because
  //      the power latch means the device cannot be asked on the way up. ----
  if (command.equalsIgnoreCase("mode") || command.equalsIgnoreCase("mode field") ||
      command.equalsIgnoreCase("mode logger")) {
    bool change = !command.equalsIgnoreCase("mode");
    if (change) {
      measureMode = command.equalsIgnoreCase("mode field") ? MODE_FIELD : MODE_LOGGER;
      prefs.begin("calib", false);
      prefs.putUChar("mmode", (uint8_t)measureMode);
      prefs.end();
      // Only switch live if we are already measuring - from calibration mode the
      // setting is stored and takes effect on 'start'.
      if (currentMode != MODE_CALIBRATION) {
        if (measureMode == MODE_FIELD) fieldBegin();
        else                           loggerBegin();
      }
    }
    out.print(F("Measuring mode: "));
    out.println(measureMode == MODE_FIELD ? F("FIELD") : F("LOGGER"));
    out.println(F("  field  - operator holds the probe: continuous read, the firmware"));
    out.println(F("           watches it settle, one averaged observation per dip,"));
    out.println(F("           uplink on the commit. No sleep (the power switch is off)."));
    out.println(F("  logger - unattended: a reading every 15 s inside a 5 min window,"));
    out.println(F("           uplink each one, then deep-sleep. 'sleep on' to enable."));
    if (currentMode == MODE_CALIBRATION)
      out.println(F("Stored - takes effect on 'start'."));
    return;
  }

  // ---- tempcomp / tempco: salinity temperature normalisation (any mode). ----
  if (command.equalsIgnoreCase("tempcomp") || command.equalsIgnoreCase("tempcomp on") ||
      command.equalsIgnoreCase("tempcomp off")) {
    if (!command.equalsIgnoreCase("tempcomp")) {
      tempCompEnabled = command.equalsIgnoreCase("tempcomp on");
      prefs.begin("calib", false);
      prefs.putBool("tcomp", tempCompEnabled);
      prefs.end();
    }
    out.print(F("Temperature compensation: "));
    out.println(tempCompEnabled ? F("ON") : F("OFF"));
    out.print(F("  model  : S25 = S / (1 + alpha * (T - "));
    out.print(TEMP_REF_C, 0); out.println(F("))"));
    out.print(F("  alpha  : ")); out.print(tempCoeff, 4);
    out.println(F(" per degC   ('tempco <alpha>' to trim)"));
    out.print(F("  status : "));
    if (!tempCompEnabled)        out.println(F("reporting RAW salinity at probe temperature"));
    else if (!tempAddrValid)     out.println(F("NO TEMPERATURE - falling back to raw, amber 6"));
    else if (currentMode == MODE_FIELD && !liveComp)
                                 out.println(F("enabled but NOT applied to the live reading - see the log"));
    else                         out.println(F("applied"));
    // Where the guard sits depends on alpha, so it is printed rather than left
    // to be discovered as an unexplained amber 6 on cold water.
    out.print(F("  limit  : water colder than ")); out.print(tempCompCutoffC(), 1);
    out.println(F(" C cannot be corrected at this alpha"));
    out.println(F("Conductivity moves ~2 %/degC: without this, the same water reads"));
    out.println(F("~25 % apart between a 22 C morning and a 35 C afternoon."));
    return;
  }
  if (command.length() >= 7 && command.substring(0, 7).equalsIgnoreCase("tempco ")) {
    float v = 0.0f;
    if (!parseFloatStrict(command.substring(7), v)) {
      out.println(F("Not a number. Usage: tempco 0.020   (a dot, not a comma)"));
      return;
    }
    if (v < TEMPCO_MIN || v > TEMPCO_MAX) {
      out.print(F("alpha must be between ")); out.print(TEMPCO_MIN, 3);
      out.print(F(" and ")); out.print(TEMPCO_MAX, 3); out.println(F(" per degC."));
      return;
    }
    tempCoeff = v;
    prefs.begin("calib", false);
    prefs.putFloat("tempco", tempCoeff);
    prefs.end();
    out.print(F("Temperature coefficient set to ")); out.print(tempCoeff, 4);
    out.println(F(" per degC."));
    out.println(F("Typical: 0.019 NaCl-dominated, 0.020 general, 0.023 carbonate-rich."));
    out.print(F("Coldest water this alpha can still correct: "));
    out.print(tempCompCutoffC(), 1); out.println(F(" C."));
    if (tempCoeff <= 1e-6f) {
      out.println(F("WARNING: alpha 0 makes the correction a no-op - every reading is raw,"));
      out.println(F("         even though 'tempcomp' will still report ON. Use 'tempcomp off'"));
      out.println(F("         if that is what you meant."));
    }
    out.println(F("Stored calibration points are re-derived at 'fit' time from the"));
    out.println(F("standard and bath temperature recorded with each, so they follow this"));
    out.println(F("change automatically - 'points' shows what they now fit to."));
    return;
  }

  // ---- site <name>: label applied to observations as they are committed.
  //      There is no keyboard on the device, so this is how a dip gets a name. ----
  if (command.equalsIgnoreCase("site") ||
      (command.length() >= 5 && command.substring(0, 5).equalsIgnoreCase("site "))) {
    if (command.length() > 5) {
      String s = command.substring(5);
      s.trim();
      // The label goes into a CSV column verbatim, so anything that would split
      // a field or open a quote has to go. Replacing rather than rejecting keeps
      // the operator moving - they are naming a site in the field, not filling
      // in a form, and a silently mangled name beats a refused command.
      for (size_t i = 0; i < s.length(); i++) {
        char ch = s[i];
        if (ch == ',' || ch == '"' || ch == '\n' || ch == '\r') s[i] = '_';
      }
      strncpy(siteLabel, s.c_str(), sizeof(siteLabel) - 1);
      siteLabel[sizeof(siteLabel) - 1] = '\0';
      out.print(F("Site label set to '")); out.print(siteLabel);
      out.println(F("'. It is stamped on every observation from now on."));
    } else if (siteLabel[0]) {
      siteLabel[0] = '\0';
      out.println(F("Site label cleared."));
    } else {
      out.println(F("No site label set. Usage: site <name>  (max 11 chars)"));
    }
    return;
  }

  // ---- rec / recs / recs clear: the observation ring (field mode). ----
  if (command.equalsIgnoreCase("rec")) {
    if (currentMode != MODE_FIELD) {
      out.println(F("'rec' is a field-mode command. Send 'mode field' first."));
      return;
    }
    if (stabCount < 2) {
      out.println(F("Nothing to record yet - put the probe in the water and let it settle."));
      return;
    }
    if (!stabIsStable()) {
      out.print(F("WARNING: not settled (sd "));
      out.print(stabSd, 4); out.print(F(" > band "));
      out.print(stabBand(), 4); out.println(F(" g/L). Recording anyway."));
    }
    if (!fieldCommit(true)) { out.println(F("Nothing to record.")); return; }
    const FieldRecord& r = recs[recIndex(recCount - 1)];
    out.print(F("Recorded #")); out.print(r.seq);
    if (r.site[0]) { out.print(F(" @")); out.print(r.site); }
    out.print(F(": ")); out.print(r.salt, 2);
    out.print(F(" +/- ")); out.print(r.sd, 3);
    out.print(F(" g/L @25C over n=")); out.print(r.n);
    if (!(r.flags & 4)) out.print(F("  [raw - no temperature]"));
    out.println();
    return;
  }

  if (command.equalsIgnoreCase("recs") || command.equalsIgnoreCase("records")) {
    if (recCount == 0) {
      out.println(F("No observations committed yet."));
      return;
    }
    out.print(F("Last ")); out.print(recCount);
    out.print(F(" observation(s) of ")); out.print(recSeq);
    out.print(F(" (ring holds ")); out.print(REC_MAX); out.println(F("):"));
    out.println(F("    #    age    site        salinity@25   sd      temp     n"));
    uint32_t now = (uint32_t)(millis() / 1000);
    for (int i = 0; i < recCount; i++) {
      const FieldRecord& r = recs[recIndex(i)];
      char line[88];
      // "  --  " and not "0.00 C" when the sensor never answered, for the same
      // reason the CSV leaves the cell blank.
      char tbuf[10];
      if (r.flags & 1) snprintf(tbuf, sizeof(tbuf), "%5.2f C", r.temp);
      else             snprintf(tbuf, sizeof(tbuf), "   -- C");
      snprintf(line, sizeof(line), " %4lu  %5lus  %-11s %7.2f g/L  %6.3f  %s  %3u%s",
               (unsigned long)r.seq, (unsigned long)(now - r.t),
               r.site[0] ? r.site : "-",
               r.salt, r.sd, tbuf, (unsigned)r.n,
               (r.flags & 4) ? "" : "  [raw, uncompensated]");
      out.println(line);
    }
    out.println(F("Download from http://<ip>/api/records.csv"));
    return;
  }

  if (command.equalsIgnoreCase("recs clear")) {
    recCount = 0;
    recHead  = 0;
    out.println(F("Observation ring cleared. The running count is kept."));
    return;
  }

  if (command.equalsIgnoreCase("recs undo")) {
    if (recCount == 0) { out.println(F("No observations to remove.")); return; }
    recHead = (recHead + REC_MAX - 1) % REC_MAX;
    recCount--;
    // recSeq deliberately NOT decremented. It is "observations ever committed",
    // and the next commit must not reuse a number that has already appeared in
    // the serial transcript, on the OLED corner and in any LoRa uplink that went
    // out for the discarded reading - two different measurements sharing one
    // identifier is precisely what a monotonic counter exists to prevent.
    // recIndex() derives everything from recHead/recCount and never needed it;
    // `recs clear` already keeps the running count, so this also makes the two
    // consistent with each other.
    out.print(F("Removed the last observation. Remaining: ")); out.println(recCount);
    out.print(F("The running count stays at ")); out.print(recSeq);
    out.println(F(" - the next dip gets a fresh number, not the discarded one."));
    return;
  }

  // ---- autorec on/off: commit automatically on settle, or wait for 'rec'. ----
  if (command.equalsIgnoreCase("autorec") || command.equalsIgnoreCase("autorec on") ||
      command.equalsIgnoreCase("autorec off")) {
    if (!command.equalsIgnoreCase("autorec")) {
      autoRecord = command.equalsIgnoreCase("autorec on");
      prefs.begin("calib", false);
      prefs.putBool("autorec", autoRecord);
      prefs.end();
    }
    out.print(F("Auto-record: "));
    out.println(autoRecord ? F("ON - commits after the reading holds steady")
                           : F("OFF - send 'rec' to commit"));
    out.print(F("  hold: ")); out.print(STABLE_HOLD_MS / 1000);
    out.println(F(" s inside the band before it commits."));
    return;
  }

  // ---- screen simple/detail: which field-mode OLED layout to draw. ----
  if (command.equalsIgnoreCase("screen") || command.equalsIgnoreCase("screen simple") ||
      command.equalsIgnoreCase("screen detail")) {
    if (!command.equalsIgnoreCase("screen")) {
      fieldDetailScreen = command.equalsIgnoreCase("screen detail");
      prefs.begin("calib", false);
      prefs.putBool("fscreen", fieldDetailScreen);
      prefs.end();
    }
    out.print(F("Field screen: "));
    out.println(fieldDetailScreen ? F("DETAIL - spread, sample count and probe voltage shown")
                                  : F("SIMPLE - value, settle bar and next step only"));
    return;
  }

  // ---- stability <g/L>: the absolute floor of the settle band. ----
  if (command.equalsIgnoreCase("stability") ||
      (command.length() >= 10 && command.substring(0, 10).equalsIgnoreCase("stability "))) {
    if (command.length() > 10) {
      float v = 0.0f;
      if (!parseFloatStrict(command.substring(10), v)) {
        out.println(F("Not a number. Usage: stability 0.05   (a dot, not a comma)"));
        return;
      }
      if (v < STAB_ABS_MIN || v > STAB_ABS_MAX) {
        out.print(F("Band must be between ")); out.print(STAB_ABS_MIN, 3);
        out.print(F(" and ")); out.print(STAB_ABS_MAX, 1); out.println(F(" g/L."));
        return;
      }
      stabAbs = v;
      prefs.begin("calib", false);
      prefs.putFloat("stab", stabAbs);
      prefs.end();
    }
    out.print(F("Settle band: +/-")); out.print(stabAbs, 3);
    out.print(F(" g/L, or ")); out.print(STAB_REL * 100.0f, 1);
    out.println(F(" % of the reading, whichever is larger."));
    out.print(F("  window ")); out.print(STAB_WINDOW);
    out.print(F(" samples at ")); out.print(FIELD_SAMPLE_MS);
    out.print(F(" ms = ")); out.print(STAB_WINDOW * FIELD_SAMPLE_MS / 1000.0f, 1);
    out.println(F(" s of history."));
    out.print(F("  now: sd ")); out.print(stabSd, 4);
    out.print(F(" over n=")); out.print(stabCount);
    out.print(F("  -> ")); out.println(stabIsStable() ? F("STABLE") : F("settling"));
    return;
  }

  // ---- immersion <V>: probe voltage above which the probe counts as wet. ----
  if (command.equalsIgnoreCase("immersion") ||
      (command.length() >= 10 && command.substring(0, 10).equalsIgnoreCase("immersion "))) {
    if (command.length() > 10) {
      float v = 0.0f;
      if (!parseFloatStrict(command.substring(10), v)) {
        out.println(F("Not a number. Usage: immersion 0.030   (a dot, not a comma)"));
        return;
      }
      if (v < IMMERSION_MIN || v > IMMERSION_MAX) {
        out.print(F("Threshold must be between ")); out.print(IMMERSION_MIN, 3);
        out.print(F(" and ")); out.print(IMMERSION_MAX, 3); out.println(F(" V."));
        return;
      }
      immersionV = v;
      prefs.begin("calib", false);
      prefs.putFloat("immv", immersionV);
      prefs.end();
    }
    out.print(F("Immersion threshold: ")); out.print(immersionV, 3);
    out.print(F(" V  (~")); out.print(voltageToSalinity(immersionV), 3);
    out.println(F(" g/L on the current curve)"));
    out.print(F("  probe now: ")); out.print(liveVolts, 4);
    out.print(F(" V -> ")); out.println(immersed ? F("IN water") : F("dry"));
    out.println(F("Set it between the dry reading and the freshest water you expect."));
    out.println(F("'adc' shows the dry reading with the excitation on."));
    return;
  }

  // ---- wifi: the web console (any mode). ----
  if (command.equalsIgnoreCase("wifi") ||
      (command.length() >= 5 && command.substring(0, 5).equalsIgnoreCase("wifi "))) {
    String arg = (command.length() > 5) ? command.substring(5) : String("");
    arg.trim();

    if (arg.equalsIgnoreCase("on"))        wifiPolicy = WIFI_POL_ON;
    else if (arg.equalsIgnoreCase("off"))  wifiPolicy = WIFI_POL_OFF;
    else if (arg.equalsIgnoreCase("auto")) wifiPolicy = WIFI_POL_AUTO;
    else if (arg.equalsIgnoreCase("ap")) {
      wifiSaveSta("", "");
      out.println(F("Stored network cleared: the console will serve its own AP."));
      if (wifiState != WIFI_DOWN) { wifiTakeDown(); }   // restart on the next tick
    }
    else if (arg.length() >= 4 && arg.substring(0, 4).equalsIgnoreCase("sta ")) {
      String rest = arg.substring(4); rest.trim();
      int sp = rest.indexOf(' ');
      if (sp < 1 || sp >= (int)rest.length() - 1) {
        out.println(F("Usage: wifi sta <ssid> <password>   (ssid may not contain spaces)"));
        return;
      }
      String ss = rest.substring(0, sp);
      String pw = rest.substring(sp + 1); pw.trim();
      wifiSaveSta(ss, pw);
      out.print(F("Stored network: ")); out.println(ss);
      out.println(F("It will be joined on the next bring-up (AP fallback if it times out)."));
      if (wifiState != WIFI_DOWN) { wifiTakeDown(); }
    }
    else if (arg.length() > 0) {
      out.println(F("Usage: wifi [on|off|auto|ap|sta <ssid> <pass>]"));
      return;
    }

    wifiSavePolicy();   // survives the next reset, unlike sleep/exc

    // Act on the new policy now, so the report below describes reality rather
    // than what the next loop() pass will do. Only the bring-up/tear-down half:
    // calling wifiTick() here would re-enter server.handleClient() when this
    // command arrived over the web console itself.
    bool want = (wifiPolicy == WIFI_POL_ON) ||
                (wifiPolicy == WIFI_POL_AUTO && chargerPresent);
    if (want  && wifiState == WIFI_DOWN) wifiBringUp();
    if (!want && wifiState != WIFI_DOWN) wifiTakeDown();

    out.print(F("Console: "));
    if (wifiState == WIFI_READY) {
      out.print(wifiIsAp ? F("AP ") : F("STA "));
      out.print(wifiSsid);
      out.print(F("  ->  http://")); out.println(wifiIpStr);
      if (wifiIsAp) { out.print(F("Password: ")); out.println(WIFI_AP_PASS); }
      out.print(wifiIsAp ? F("Clients: ") : F("RSSI: "));
      out.println(wifiIsAp ? (int)WiFi.softAPgetStationNum() : (int)WiFi.RSSI());
    } else if (wifiState == WIFI_STA_CONNECTING) {
      out.print(F("joining ")); out.println(staSsid);
    } else {
      out.println(F("down"));
    }
    out.print(F("Policy:  ")); out.println(wifiPolicyName());
    out.print(F("Network: "));
    if (staSsid.length()) { out.print(F("station, ssid ")); out.println(staSsid); }
    else                    out.println(F("own AP (no stored network)"));
    out.print(F("Free heap: ")); out.println(ESP.getFreeHeap());
    if (wifiPolicy == WIFI_POL_ON) {
      out.println(F("NOTE: WiFi costs ~100 mA. Deep sleep is suppressed while it is up."));
    }
    return;
  }

  // ---- charge auto/ignore: what to do when USB-C power shows up. ----
  if (command.equalsIgnoreCase("charge") || command.equalsIgnoreCase("charge auto") ||
      command.equalsIgnoreCase("charge ignore")) {
    if (command.equalsIgnoreCase("charge auto"))   pauseOnCharge = true;
    if (command.equalsIgnoreCase("charge ignore")) pauseOnCharge = false;
    out.print(F("Charger: "));
    out.print(chargerPresent ? F("PLUGGED IN") : F("not connected"));
    out.print(F("  ("));  out.print(chargeStateName(chargeState));
    out.print(F(", ")); out.print(batVoltage, 2); out.print(F(" V, "));
    out.print(batPercent); out.println(F(" %)"));
    out.print(F("Policy:  "));
    if (pauseOnCharge) out.println(F("auto   - measurements PAUSE while USB-C is plugged in"));
    else               out.println(F("ignore - keep measuring with USB-C plugged in (bench)"));
    out.print(F("Probe:   excitation "));
    out.println(excAlwaysOn ? F("latched ON (AC drive, no DC on the electrodes)")
                            : F("off between readings"));
    out.println(F("Change with 'charge auto' or 'charge ignore'."));
    return;
  }

  // ---- exc on/off: latch the excitation square wave for continuous
  //      bench measurement (any mode). ----
  if (command.equalsIgnoreCase("exc on")) {
    excAlwaysOn = true;
    if (chargeModeActive) {
      out.println(F("Charging: latch set, wave stays off until USB-C is unplugged."));
      return;
    }
    excitationOn();
    out.print(F("Excitation LATCHED ON: ")); out.print(frequency);
    out.print(F(" Hz square wave on GPIO")); out.print(outputPin);
    out.println(F(" (net G0, stays on until 'exc off')."));
    return;
  }
  if (command.equalsIgnoreCase("exc off")) {
    // Refused in field mode, because there is no "per-reading pulsing" to go
    // back to there: fieldSampleSalinity() never drives the excitation itself,
    // it relies entirely on the latch fieldBegin() sets. Clearing it would
    // detach LEDC from the excitation pin for good, and every live sample would
    // then read an undriven probe - v ~= 0, liveValid false, immersedRaw false -
    // so the state machine could never leave FIELD_DRY and no observation could
    // ever be committed. The OLED would sit on "Dip the probe" with no fault
    // raised and nothing pointing at the cause. (Logger mode is unaffected:
    // startMeasurement() calls excitationOn() every cycle.)
    if (currentMode == MODE_FIELD) {
      out.println(F("REFUSED: field mode reads continuously and needs the excitation"));
      out.println(F("         latched on. Clearing it would leave the probe undriven and"));
      out.println(F("         the reading stuck at 'Dip the probe' for good."));
      out.println(F("         Send 'mode logger' first if you want it off."));
      return;
    }
    excAlwaysOn = false;
    excitationOff();
    out.println(F("Excitation latch cleared: back to per-reading pulsing."));
    return;
  }

  // ---- adc: raw ADC diagnostic (any mode). "adc" = one burst,
  //      "adc N" = N bursts ~1 s apart so you can watch the value move. ----
  if (command.equalsIgnoreCase("adc") ||
      (command.length() >= 4 && command.substring(0, 4).equalsIgnoreCase("adc "))) {
    int reps = 1;
    if (command.length() > 4) reps = max(1, (int)command.substring(4).toInt());

    if (chargeModeActive) {
      out.println(F("Charging: measurements paused. Unplug USB-C, or send 'charge ignore'."));
      return;
    }
    out.print(F("ADC diagnostic on GPIO")); out.print(adcPin);
    out.println(F(" (12-bit, 11 dB, 0-4095 => 0-3.3 V)"));

    for (int r = 0; r < reps; r++) {
      // Excitation OFF (probe not driven). Detach directly so we get a true
      // baseline even when the excitation is latched on via `exc on`.
      ledcDetach(outputPin);
      delay(30);
      long rawOff = 0;
      for (int i = 0; i < salSamples; i++) { rawOff += analogRead(adcPin); delayMicroseconds(400); }
      float cntOff = (float)rawOff / salSamples;

      // Excitation ON (probe driven by the 25 kHz square wave)
      excitationOn();
      delay(SAMPLE_SETTLE_MS);
      long rawOn = 0;
      for (int i = 0; i < salSamples; i++) { rawOn += analogRead(adcPin); delayMicroseconds(400); }
      float cntOn = (float)rawOn / salSamples;
      // Leave excitation in whatever state the latch dictates.
      if (!excAlwaysOn) ledcDetach(outputPin);

      out.print(F("  OFF: ")); out.print(cntOff, 1);
      out.print(F(" cts / ")); out.print((cntOff / 4095.0f) * ADC_REF_VOLTAGE, 3);
      out.print(F(" V   |   ON: ")); out.print(cntOn, 1);
      out.print(F(" cts / ")); out.print((cntOn / 4095.0f) * ADC_REF_VOLTAGE, 3);
      out.println(F(" V"));

      if (r + 1 < reps) delay(1000);
    }
    return;
  }

  // In Measuring mode only the global commands above are meaningful.
  if (currentMode != MODE_CALIBRATION) {
    out.println(F("In Measuring mode. Send 'recal' to re-enter calibration, or 'help'."));
    return;
  }

  // ---- start: save and switch to Measuring mode (calibration mode only) ----
  if (command.equalsIgnoreCase("start")) {
    out.print(F("Exiting calibration mode. Stopping Bluetooth and starting "));
    out.print(measureMode == MODE_FIELD ? F("FIELD") : F("LOGGER"));
    out.println(F(" mode."));
    out.flush();
    delay(200);

    // Save a,b,c, the quadratic P1/P2/P3, which curve is live, and the
    // calibration flag to NVS. Both curves are written whichever one was
    // fitted: they are cheap, and `model` has to be able to switch back to a
    // curve that still has its coefficients rather than the compile-time ones.
    prefs.begin("calib", false);
    prefs.putFloat("a", a);
    prefs.putFloat("b", b);
    prefs.putFloat("c", c);
    prefs.putFloat("p1", p1);
    prefs.putFloat("p2", p2);
    prefs.putFloat("p3", p3);
    prefs.putUChar("model", (uint8_t)curveModel);
    prefs.putBool("done", true);
    // Persist the mode alongside the curve. Without this a freshly calibrated
    // device leaves NVS with no "mmode" key at all, so which instrument it is
    // depends on a compile-time default rather than on anything the operator
    // chose - and changing that default later would silently re-purpose every
    // such device on its next firmware update.
    prefs.putUChar("mmode", (uint8_t)measureMode);
    prefs.end();

    autoCalActive = false;

    SerialBT.end();
    btStop();

#if LORA_ENABLED
    // Safe on V2: the OLED lives on its own I2C pads (GPIO21/22), so claiming
    // GPIO18/19/23/5 for the radio no longer costs us the display.
    SPI.begin(18, 19, 23, 5);   // SCK, MISO, MOSI, NSS
    os_init();
    LMIC_reset();
#endif

    if (measureMode == MODE_FIELD) fieldBegin();
    else                           loggerBegin();
    return;
  }

  // ---- Auto-calibration commands ----
  if (command.equalsIgnoreCase("autocal")) {
    autoCalActive = true;
    calCount      = 0;
    lastFitR2     = -1.0f;
    currentState  = IDLE;
    strncpy(autoCalMsg, "send sample <S>", sizeof(autoCalMsg));
    out.println(F("Auto-calibration started. Any previous points cleared."));
    out.println(F("Dip the probe in a known solution, then send: sample <salinity g/L>"));
    out.println(F("Use solutions with S>0 (e.g. 5, 15, 35). Then 'fit', then 'start' to save."));
    out.print(F("Fitting the "));
    out.println(curveModel == CURVE_QUAD
                  ? F("QUADRATIC curve: needs 3+ points at 3 different salinities.")
                  : F("POWER curve: 2 points will do, 3+ gives a meaningful R2."));
    return;
  }

  if (command.length() >= 6 && command.substring(0, 6).equalsIgnoreCase("sample")) {
    if (chargeModeActive) {
      out.println(F("Charging: measurements paused. Unplug USB-C, or send 'charge ignore'."));
      return;
    }
    if (!autoCalActive) {
      out.println(F("Send 'autocal' first."));
      return;
    }
    String arg = command.substring(6);
    arg.trim();
    if (arg.length() == 0) {
      out.println(F("Usage: sample <salinity g/L>   e.g.  sample 15"));
      return;
    }
    float knownS = 0.0f;
    if (!parseFloatStrict(arg, knownS)) {
      out.println(F("Not a number. Usage: sample 15   (a dot, not a comma)"));
      return;
    }
    if (knownS <= 0.0f) {
      out.println(F("Salinity must be > 0 g/L for the fit."));
      return;
    }
    if (calCount >= MAX_CAL_POINTS) {
      out.println(F("Point buffer full. Send 'fit' or 'undo'."));
      return;
    }

    out.println(F("Measuring... keep the probe still."));
    float v  = measureVoltageBlocking(salSamples * 2, SAMPLE_SETTLE_MS);
    float tC = tempReadBlocking();
    bool  tOk = (tC > -100.0f);

    // The value typed is the standard's nominal figure, which by convention is
    // quoted at 25 degC. The probe is not at 25 degC, so fitting (V, knownS)
    // directly would bake the bath temperature into a and b - and then the
    // runtime correction would be applied on top of a curve that already
    // contains it, double-counting. Convert the standard to what it actually
    // reads in THIS bath and fit that instead; the curve then lives purely in
    // the probe's own temperature domain and salinityAt25() is the only place
    // temperature is ever removed.
    float sAtBath = (tOk && tempCompEnabled) ? salinityAtTemp(knownS, tC) : knownS;

    calV[calCount]    = v;
    calS[calCount]    = sAtBath;
    calSRef[calCount] = knownS;
    calT[calCount]    = tOk ? tC : TEMP_REF_C;
    calTOk[calCount]  = tOk;
    calCount++;

    snprintf(autoCalMsg, sizeof(autoCalMsg), "pt%d V=%.3f S=%.1f", calCount, v, knownS);
    out.print(F("Recorded point "));  out.print(calCount);
    out.print(F(": V="));             out.print(v, 4);
    out.print(F(" V, standard "));    out.print(knownS, 2);
    out.print(F(" g/L @25C"));
    if (tOk) {
      out.print(F(", bath "));   out.print(tC, 2);
      out.print(F(" C -> fitting ")); out.print(sAtBath, 2); out.println(F(" g/L"));
      if (fabsf(tC - TEMP_REF_C) > 8.0f) {
        out.println(F("  NOTE: the bath is a long way from 25 C. The correction handles it,"));
        out.println(F("        but a bath nearer 25 C leans on the model less."));
      }
    } else {
      out.println(F(", NO TEMPERATURE"));
      out.println(F("  WARNING: the DS18B20 did not answer, so this point is fitted as if"));
      out.println(F("           the bath were at 25 C. If it was not, the curve is wrong."));
    }
    return;
  }

  if (command.equalsIgnoreCase("points")) {
    out.print(F("Points: "));
    out.println(calCount);
    out.println(F("   #   probe V   standard@25   bath C   fitted@bath"));
    bool moved = false;
    for (int i = 0; i < calCount; i++) {
      // 'fitted@bath' is recomputed from the CURRENT compensation settings, so
      // this listing always shows what 'fit' would actually use. calS[i] is the
      // same figure as it stood when the point was taken; a difference means
      // tempco or tempcomp moved since, which is worth saying out loud.
      float now = calFitValue(i);
      bool  chg = fabsf(now - calS[i]) > 0.005f;
      if (chg) moved = true;
      // 96: 48 characters of columns plus "[no temp]" (11) and
      // "[settings changed since]" (26), which can both be present at once.
      char line[96];
      snprintf(line, sizeof(line), "  %2d   %7.4f   %9.2f   %6.2f   %9.2f%s%s",
               i + 1, calV[i], calSRef[i], calT[i], now,
               calTOk[i] ? "" : "  [no temp]",
               chg ? "  [settings changed since]" : "");
      out.println(line);
    }
    if (calCount) {
      out.println(F("The fit uses 'fitted@bath': the standard corrected to the bath it was"));
      out.println(F("measured in, so the curve carries no temperature of its own."));
      if (moved) {
        out.println(F("NOTE: alpha or 'tempcomp' has changed since some of these were taken."));
        out.println(F("      They are shown, and will be fitted, under the settings in force"));
        out.println(F("      NOW - which is what runtime will undo. No need to re-sample."));
      }
    }
    return;
  }

  if (command.equalsIgnoreCase("undo")) {
    if (calCount > 0) {
      calCount--;
      out.print(F("Removed last point. Remaining: "));
      out.println(calCount);
      snprintf(autoCalMsg, sizeof(autoCalMsg), "removed, %d left", calCount);
    } else {
      out.println(F("No points to remove."));
    }
    return;
  }

  if (command.equalsIgnoreCase("fit")) {
    // Whichever curve is live is the one that gets fitted. Fitting the other one
    // would print coefficients that nothing then uses, which is the worst of
    // both: it reads like a successful calibration and changes no reading.
    if (curveModel == CURVE_QUAD) {
      float n1, n2, n3, r2;
      if (fitCalibrationQuad(n1, n2, n3, r2)) {
        p1 = n1; p2 = n2; p3 = n3;
        lastFitR2 = r2;
        quadCeilingTripped = false;
        snprintf(autoCalMsg, sizeof(autoCalMsg), "fit R2=%.3f", r2);
        out.println(F("Fit OK (model: V = P1*S^2 + P2*S + P3):"));
        out.print(F("  P1=")); out.println(p1, 6);
        out.print(F("  P2=")); out.println(p2, 6);
        out.print(F("  P3=")); out.println(p3, 6);
        out.print(F("  R2=")); out.println(r2, 4);
        out.println(F("  (R2 is in VOLTS - not comparable with the power model's log-log R2)"));
        float sMax, vMax;
        if (quadCeiling(sMax, vMax)) {
          out.print(F("  covers up to ")); out.print(sMax, 2);
          out.print(F(" g/L / ")); out.print(vMax, 4);
          out.println(F(" V, then the curve turns over"));
          // A ceiling under the points it was just fitted to means the fit is
          // upside down or the points are on the far flank; either way every
          // reading at the top of the range will come back clamped.
          for (int i = 0; i < calCount; i++) {
            if (calFitValue(i) > sMax) {
              out.println(F("  WARNING: a recorded point is ABOVE that ceiling. This curve cannot"));
              out.println(F("           reproduce its own calibration - add points and re-fit."));
              break;
            }
          }
        }
        out.println(F("Send 'start' to save & run, or add more samples and 'fit' again."));
      } else {
        out.println(F("Fit failed: the quadratic needs >=3 points at three DIFFERENT"));
        out.println(F("salinities (with V>0). Two points only ever pin down a line."));
        out.println(F("'model power' fits from two points if that is all you have."));
      }
      return;
    }

    float na, nb, r2;
    if (fitCalibration(na, nb, r2)) {
      a = na; b = nb; c = 0.0f;
      lastFitR2 = r2;
      snprintf(autoCalMsg, sizeof(autoCalMsg), "fit R2=%.3f", r2);
      out.println(F("Fit OK (model: S = a * V^b, c = 0):"));
      out.print(F("  a="));  out.println(a, 4);
      out.print(F("  b="));  out.println(b, 4);
      out.print(F("  R2=")); out.println(r2, 4);
      out.println(F("Send 'start' to save & run, or add more samples and 'fit' again."));
    } else {
      out.println(F("Fit failed: need >=2 points with S>0 and V>0 (and varying V)."));
    }
    return;
  }

  if (command.equalsIgnoreCase("cancel")) {
    autoCalActive = false;
    out.println(F("Auto-calibration cancelled."));
    return;
  }

  // ---- Manual calibration state machine ----
  switch (currentState) {
    case IDLE:
      if (command.equalsIgnoreCase("cal")) {
        // `cal` only ever edited a/b/c, and it still does. Under the quadratic
        // curve that is an edit to coefficients nothing reads, which looks
        // exactly like a successful calibration from the terminal.
        if (curveModel == CURVE_QUAD) {
          out.println(F("NOTE: the QUADRATIC curve is live, so a/b/c are not in use."));
          out.println(F("      'pcoef <P1> <P2> <P3>' edits the live one, 'model power' switches."));
        }
        out.print(F("Current values: a="));
        out.print(a);
        out.print(F(", b="));
        out.print(b);
        out.print(F(", c="));
        out.println(c);
        out.println(F("Do you want to change them? (yes/no)"));
        currentState = AWAIT_CHANGE_CONFIRM;
      } else {
        out.println(F("Unknown command. Send 'help' for the list."));
      }
      break;

    case AWAIT_CHANGE_CONFIRM:
      if (command.equalsIgnoreCase("yes") || command.equalsIgnoreCase("y")) {
        out.println(F("Enter the new value for 'a':"));
        currentState = AWAIT_VALUE_A;
      } else {
        out.println(F("Calibration cancelled."));
        currentState = IDLE;
      }
      break;

    case AWAIT_VALUE_A:
      a = command.toFloat();
      out.print(F("Value 'a' updated to: "));
      out.println(a);
      out.println(F("Enter the new value for 'b':"));
      currentState = AWAIT_VALUE_B;
      break;

    case AWAIT_VALUE_B:
      b = command.toFloat();
      out.print(F("Value 'b' updated to: "));
      out.println(b);
      out.println(F("Enter the new value for 'c':"));
      currentState = AWAIT_VALUE_C;
      break;

    case AWAIT_VALUE_C:
      c = command.toFloat();
      out.print(F("Value 'c' updated to: "));
      out.println(c);
      out.println(F("Calibration finished! Send 'start' to save & run."));
      Log.print(F("New values: a= "));
      Log.print(a);
      Log.print(F(", b= "));
      Log.println(b);
      Log.print(F(", c= "));
      Log.print(c);
      currentState = IDLE;
      break;
  }
}

// ================== BLUETOOTH (CALIBRATION) ==================
void handleBluetoothCommands() {
  if (SerialBT.hasClient()) {
    if (clientJustConnected) {
      Log.println(F("New BT client connected."));
      SerialBT.println(F("Connected. Send 'help' for commands, 'autocal' to calibrate, 'start' to run."));
      clientJustConnected = false;
    }
  } else {
    clientJustConnected = true;
    currentState = IDLE;
  }

  if (!SerialBT.available()) return;

  String command = SerialBT.readStringUntil('\n');
  command.trim();
  if (command.length() == 0) return;
  processCommand(command, SerialBT);
}

// ================== USB SERIAL (COMMANDS) ==================
// Same commands as Bluetooth, but typed into the USB serial monitor. Lets you
// drive the board from the VSCode terminal (macOS can't open a BT-classic SPP
// port). Works in BOTH modes: in Measuring mode only the global commands act
// (help, coef, adc, exc, samples, sleep, recal). Replies go back on the USB
// Serial so you see them in the monitor.
void handleSerialCommands() {
  if (!Log.available()) return;

  String command = Log.readStringUntil('\n');
  command.trim();
  if (command.length() == 0) return;   // ignore stray newlines / empty lines
  processCommand(command, Log);
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);

  // RGB status LED — claim GPIO15/4/17 before anything else so the board
  // shows white the moment it is powered, through the boot delay below.
  rgbInit();

  // Battery / charger sense lines. All three are input-only pins with external
  // 10k pull-ups on the board, so no pinMode configuration is needed for them.
  batteryInit();
  delay(2000);

  // The radio stays off at boot; wifiTick() brings it up per policy (default:
  // only while USB-C power is present, so the field power budget is untouched).
  WiFi.mode(WIFI_OFF);
  wifiLoadSta();
  wifiInitRoutes();
  setCpuFrequencyMhz(80);
  analogSetWidth(12);
  analogSetAttenuation(ADC_11db);
  // Restate it for the battery pin specifically: the global call above is the
  // only thing setting it today, but the salinity front-end is one edit away
  // from wanting a different attenuation, and the gauge must stay on 11 dB to
  // reach a 4.2 V pack through the /2 divider.
  analogSetPinAttenuation(batSensePin, ADC_11db);

  // Per-device divider calibration, before batteryPrime() takes the first
  // reading, so the very first frame is already corrected.
  batCalLoad();

  // Seed the power cache now the ADC is configured, so the first frame shows
  // real numbers and the first loop() pass already knows about a plugged cable.
  batteryPrime();

  // Start the excitation square wave immediately when latched on (default), so
  // the probe is driven from power-up regardless of measurement timing/mode.
  if (excAlwaysOn) excitationOn();

  // Stage 2 (solid amber): everything from here to tempInit() talks to a bus
  // that can hang - I2C to the OLED, 1-Wire to the probe. A board parked on
  // amber never got past one of them.
  rgbBootStage(RGB_BOOT_IO);

  // Initialize the OLED display (I2C) as early as possible
  displayInit();
  delay(1200);   // let the startup screen be seen

  // Enumerate the DS18B20 once so per-reading searches are not needed.
  tempInit();

  // Stage 3 (solid green): peripherals are up; NVS and the mode decision left.
  rgbBootStage(RGB_BOOT_DONE);

  // Load stored a/b/c (fall back to the compile-time defaults) and the
  // "already calibrated" flag from NVS.
  prefs.begin("calib", true);
  bool done = prefs.getBool("done", false);
  a = prefs.getFloat("a", a);
  b = prefs.getFloat("b", b);
  c = prefs.getFloat("c", c);
  p1 = prefs.getFloat("p1", p1);
  p2 = prefs.getFloat("p2", p2);
  p3 = prefs.getFloat("p3", p3);
  // Which curve is live. Guarded the same way as "mmode" below, so a corrupt or
  // future-version byte cannot cast into a model that does not exist.
  bool    haveModel = prefs.isKey("model");
  uint8_t cm        = prefs.getUChar("model", (uint8_t)curveModel);
  curveModel        = (cm == (uint8_t)CURVE_POWER) ? CURVE_POWER : CURVE_QUAD;
  // Measuring mode and the compensation settings persist alongside a/b/c: all of
  // them are properties of how this device is deployed, and the power latch means
  // it cannot be told any of it at boot. Guard the mode value so a corrupt or
  // future-version entry cannot cast into something that is not a measuring mode.
  bool    haveMode = prefs.isKey("mmode");
  uint8_t mm    = prefs.getUChar("mmode", (uint8_t)MODE_FIELD);
  measureMode   = (mm == (uint8_t)MODE_LOGGER) ? MODE_LOGGER : MODE_FIELD;
  // Read into locals FIRST. constrain() is a macro that expands its first
  // argument up to three times - ((x)<(a)?(a):((x)>(b)?(b):(x))) - so passing a
  // prefs.getFloat() call directly performed two or three NVS reads per line,
  // nine across the three. Harmless only because getFloat() happens to be side
  // effect free; the same shape around anything that advances state is a
  // correctness bug, and it is the classic Arduino macro-argument trap.
  float fTempco = prefs.getFloat("tempco", tempCoeff);
  float fStab   = prefs.getFloat("stab",   stabAbs);
  float fImmv   = prefs.getFloat("immv",   immersionV);
  tempCoeff     = constrain(fTempco, TEMPCO_MIN, TEMPCO_MAX);
  tempCompEnabled = prefs.getBool("tcomp", tempCompEnabled);
  stabAbs       = constrain(fStab, STAB_ABS_MIN, STAB_ABS_MAX);
  immersionV    = constrain(fImmv, IMMERSION_MIN, IMMERSION_MAX);
  autoRecord    = prefs.getBool("autorec", autoRecord);
  fieldDetailScreen = prefs.getBool("fscreen", fieldDetailScreen);
  prefs.end();

  // A calibrated device with no stored mode was flashed by a build that predates
  // the FIELD/LOGGER split, where the only behaviour was timed logging. Defaulting
  // it to FIELD would silently convert a deployed, unattended node into a
  // field instrument on nothing more than a firmware update: no 15 s readings, no
  // periodic uplink (field mode transmits only on a committed dip, and there is
  // nobody there to dip it), no deep sleep, and the excitation latched on
  // permanently. The site would simply stop reporting while the pack drained,
  // with nothing in the log saying why. Keep what such a device was doing, say so,
  // and write the key once so the question never comes up again.
  if (done && !haveMode) {
    measureMode = MODE_LOGGER;
    prefs.begin("calib", false);
    prefs.putUChar("mmode", (uint8_t)measureMode);
    prefs.end();
    Log.println(F("No stored measuring mode: this device was calibrated by a build that"));
    Log.println(F("predates the field/logger split, so it is kept in LOGGER mode."));
    Log.println(F("Send 'mode field' if it is meant to be a handheld instrument."));
  }

  // Same shape, same reason, for the curve. A calibrated device with no stored
  // model was calibrated by a build where the power curve was the only one there
  // was, so its stored a/b ARE its calibration - the standards it was dipped in,
  // the bath they were at, the whole bench session. Letting the new default take
  // over would throw that away on a firmware update alone and leave the device
  // reading through a curve that was never fitted to this probe, with nothing on
  // screen saying the number had changed. Keep what it was calibrated with, write
  // the key so this is decided once, and say so.
  if (done && !haveModel) {
    curveModel = CURVE_POWER;
    prefs.begin("calib", false);
    prefs.putUChar("model", (uint8_t)curveModel);
    prefs.end();
    Log.println(F("No stored salinity curve: this device was calibrated by a build that"));
    Log.println(F("predates the quadratic model, so it keeps its fitted POWER curve"));
    Log.print(F("(a=")); Log.print(a, 4); Log.print(F(", b=")); Log.print(b, 4);
    Log.println(F(")."));
    Log.println(F("Send 'model quad' to move it to V = P1*S^2 + P2*S + P3 - and re-run"));
    Log.println(F("'autocal' afterwards, or 'pcoef' the bench coefficients in."));
  }

  if (!done) {
    // FIRST TIME (or after 'recal'): calibration mode
    Log.println("Boot: Bluetooth + USB CALIBRATION mode.");
    SerialBT.begin("ESP32_Gambia_Calibrator");
    Log.println("Bluetooth ACTIVE for calibration.");
    Log.println("USB terminal ACTIVE too: type 'help', 'autocal' or 'start'.");
    currentMode = MODE_CALIBRATION;
    rgbSetStatus(RGB_CAL_IDLE);
  } else {
    // Already calibrated: straight into whichever measuring mode was last set.
    // This has to come from NVS rather than a compile-time default: the V2 power
    // button is a hardware latch with no GPIO behind it, so there is no way to
    // ask the operator which instrument they want on the way up.
    Log.print("Calibration already done. Entering ");
    Log.print(measureMode == MODE_FIELD ? "FIELD" : "LOGGER");
    Log.println(" mode directly.");
    Log.println("Type 'recal' here (USB) to re-enter calibration, or 'mode' to switch.");
    btStop();

#if LORA_ENABLED
    SPI.begin(18, 19, 23, 5);   // SCK, MISO, MOSI, NSS
    os_init();
    LMIC_reset();
#endif

    if (measureMode == MODE_FIELD) fieldBegin();
    else                           loggerBegin();
  }
}

// ================== CHARGE MODE ==================
// While the USB-C charger is plugged in the device does nothing but charge: any
// conversion chain in flight is abandoned, no new measurement starts, no LoRa
// uplink goes out, and the OLED switches to the charge screen. Returns true when
// it has taken over the loop.
//
// Serial (and, in calibration mode, Bluetooth) command handling deliberately
// stays live: a device on the bench with a cable in it still has to answer.
//
// On the excitation: this calls excitationOff(), which honours the `exc on`
// latch and so leaves the 25 kHz square wave running by default. That is on
// purpose. The wave is what keeps the probe drive AC; parking GPIO13 static
// would leave the DG636E switched one way and put a DC bias across the
// electrodes for the whole charge, which is how you plate and corrode them.
// Send `exc off` first if you want the drive genuinely dead while charging.
bool handleChargeMode() {
  bool pause = chargerPresent && pauseOnCharge;

  if (pause && !chargeModeActive) {
    chargeModeActive = true;
    Log.println(F("USB-C power detected: measurements PAUSED, charging only."));
    measState = MEAS_IDLE;        // abandon any conversion chain in flight
    excitationOff();
    if (excAlwaysOn) {
      Log.println(F("  excitation stays latched ON: the square wave is what keeps the"));
      Log.println(F("  probe drive AC. Send 'exc off' to stop it while charging."));
    }
  } else if (!pause && chargeModeActive) {
    chargeModeActive = false;
    Log.println(F("USB-C power removed: resuming measurements."));
    if (excAlwaysOn) excitationOn();
    awakeStartMs = millis();                       // reopen the awake window
    lastSendMs   = awakeStartMs - SEND_PERIOD_MS;  // and read straight away
    lastMeasOkMs = awakeStartMs;                   // re-arm the stall watchdog

    // Field mode starts over rather than resuming: nothing was sampled during
    // the charge, so the stability window is stale and the probe could have been
    // anywhere. Committing a window half of which predates the cable would be
    // committing a reading nobody took.
    if (currentMode == MODE_FIELD) {
      fieldState  = FIELD_DRY;
      stabReset();
      immersed    = false;
      immersedRaw = false;
      immersionEdgeMs  = 0;
      fieldTempPending = false;
      fieldSampleMs    = awakeStartMs;
      fieldTempMs      = awakeStartMs;
    }
  }

  if (!chargeModeActive) return false;

  // Measurements are paused on purpose here, so the stall watchdog must not fire.
  faultSet(F_STALLED, false);

#if LORA_ENABLED
  os_runloop_once();   // let an uplink caught mid-flight finish; nothing new is queued
#endif
  handleSerialCommands();
  if (currentMode == MODE_CALIBRATION) handleBluetoothCommands();

  // The LED only says "charging" or "full" here. A charger FAULT and a standby
  // with no cell are conditions, not modes: batteryTick() raises F_CHARGER and
  // F_NO_BATTERY for them and they flash out as red 3 and amber 5 - which is the
  // collision this rework exists to fix. They used to share RGB_ERROR and
  // RGB_WARN with "both sensors dead" and "one sensor dead".
  rgbSetStatus(chargeState == CHG_COMPLETE ? RGB_CHARGED : RGB_CHARGING);
  if (!displayWifiSplash()) displayCharging();
  return true;
}

// ================== LOOP ==================
void loop() {
  batteryTick();       // refresh the cached pack voltage / charger state
  wifiTick();          // bring the console up/down per policy, then serve it
  faultTick();         // expire the fault hold windows -> faultLatched
  rgbUpdate();         // render the RGB status LED (non-blocking, self-throttled)

  // Charger plugged in: charge only, nothing else runs (see handleChargeMode).
  if (handleChargeMode()) return;

  if (currentMode == MODE_CALIBRATION) {
    handleBluetoothCommands();
    handleSerialCommands();                     // same commands over the USB terminal
    // Calibration mode breathes white throughout - it is a "waiting for a human"
    // state, and a device left in it in the field will never measure. The rhythm
    // says which sub-state: slow alone, fast with a client, pulsing on auto-cal.
    faultSet(F_STALLED, false);                 // nothing is meant to be measuring
    if (autoCalActive)              rgbSetStatus(RGB_AUTOCAL);
    else if (SerialBT.hasClient())  rgbSetStatus(RGB_CAL_LINKED);
    else                            rgbSetStatus(RGB_CAL_IDLE);
    if (displayWifiSplash())    { /* console card owns the screen */ }
    else if (autoCalActive)     displayAutoCal();   // live auto-cal status
    else                        displayCalibration(SerialBT.hasClient());
    return;
  }

  // ---- MODE_FIELD: operator present, continuous read, commit per dip ----
  if (currentMode == MODE_FIELD) {
#if LORA_ENABLED
    os_runloop_once();
#endif
    handleSerialCommands();
    fieldTick();
    if (!displayWifiSplash()) displayField();
    return;
  }

  // ---- MODE_LOGGER: unattended, timed readings, deep sleep ----
#if LORA_ENABLED
  os_runloop_once();
#endif

  // 'recal' (and 'help') accepted over USB even during normal operation.
  handleSerialCommands();

  unsigned long now = millis();

  // Advance any in-flight async measurement: each elapsed conversion window
  // feeds the temperature accumulator and chains the next conversion; once all
  // tempSamples conversions are in, finish (salinity read + encode + transmit).
  if (measState == MEAS_CONVERTING && now - measStartMs >= DS18B20_CONV_MS) {
    if (stepMeasurement()) {
      finishMeasurement();
      measState = MEAS_IDLE;
      // finishMeasurement() takes ~200 ms (salSamples ADC reads at 400 us each)
      // and stamps lastMeasOkMs on its way out, so `now` is older than that stamp
      // by the time the stall watchdog below runs. Left stale, the unsigned
      // subtraction wraps and F_STALLED fires on every single good reading.
      now = millis();
    }
  }

  // When sleep is disabled (`sleep off`, the bench default) we never leave the
  // measuring branch, so the device stays always on and reads every 15 s. When
  // sleep is enabled we keep measuring only while inside the awake window.
  if (!sleepEnabled || now - awakeStartMs < AWAKE_WINDOW_MS) {
    if (measState == MEAS_IDLE && now - lastSendMs >= SEND_PERIOD_MS) {
      startMeasurement();
    }

    // Stall watchdog. A reading has to complete every SEND_PERIOD_MS; three
    // periods with nothing landing means the cycle is wedged - a 1-Wire bus
    // holding the line low, or a conversion chain that never came back. No
    // per-sensor bit catches this, because in that state neither sensor is ever
    // asked. Only evaluated inside the awake window, where measuring is expected.
    faultSet(F_STALLED, (now - lastMeasOkMs) > 3 * SEND_PERIOD_MS);
    rgbSetStatus(RGB_RUN);

    // Always keep the latest reading on screen with a countdown to the next one.
    long remainMs = (long)SEND_PERIOD_MS - (long)(now - lastSendMs);
    if (remainMs < 0) remainMs = 0;
    char footer[24];
    if (measState == MEAS_CONVERTING) {
      strncpy(footer, "Measuring...", sizeof(footer));
    } else if (haveMeasurement) {
      snprintf(footer, sizeof(footer), "Next reading: %lds", remainMs / 1000);
    } else {
      snprintf(footer, sizeof(footer), "First read: %lds", remainMs / 1000);
    }
    if (!displayWifiSplash()) {
      drawMainScreen(haveMeasurement, temperature, salinity, footer, false);
    }
  } else {
    // Window ended and sleep is enabled: only sleep once a measurement in
    // flight has finished. Not measuring is the intent here, so stand the
    // watchdog down.
    faultSet(F_STALLED, false);
    if (wifiState != WIFI_DOWN) {
      // Sleeping would drop the console mid-session. Hold the window open and
      // keep measuring for as long as someone has the radio up.
      awakeStartMs = now;
    } else if (measState == MEAS_IDLE) {
#if LORA_ENABLED
      goToDeepSleep();
#else
      // Without LoRa hardware we can't deep-sleep: reset the window to keep testing.
      awakeStartMs = now;
#endif
    }
  }
}
