# CoMotion Firmware — Functional Specification for Zephyr Port

This document describes every algorithm, data structure, timing, and protocol in the current firmware. Use this as the exact specification when implementing the Zephyr version — the goal is **byte-identical BLE packets** and **identical behavior**.

---

## 1. BLE Broadcast Packet (20-byte Manufacturer Data)

**Manufacturer ID:** `0xFFFF` (little-endian: `0xFF, 0xFF`)

The packet is broadcast as BLE manufacturer-specific data. The app reads it via passive BLE scanning — no connection required.

**Update rate:** Every 500ms (normal) or 100ms (focus mode).

### Byte Map

| Byte(s) | Name | Type | Encoding |
|---------|------|------|----------|
| 0 | Status flags | uint8 | Bitfield (see below) |
| 1 | Battery % | uint8 | 0–100, linear from voltage |
| 2 | Intensity (1s) | uint8 | `min(intensityCurrent × 100, 255)` |
| 3 | Intensity (1min) | uint8 | `min(intensity1Min × 100, 255)` |
| 4–5 | Intensity (10min) | uint16 LE | `min(intensity10MinAvg × 1000, 65535)` |
| 6 | Speed now | uint8 | `min(gpsSpeed × 2.0, 255)` — 0.5 km/h resolution |
| 7 | Max speed (session) | uint8 | `min(maxSpeed × 2.0, 255)` — 0.5 km/h resolution |
| 8 | Impact count | uint8 | Session total, 0–255, never resets mid-session |
| 9 | GPS status | uint8 | High nibble: age in seconds (0–15); Low nibble: satellite count (0–15) |
| 10–11 | Move count | uint16 LE | Rising-edge movement counter |
| 12–13 | Session seconds | uint16 LE | Seconds since logging started; 0 if not logging |
| 14 | Audio peak | uint8 | `min(rawPeak / 128, 255)` — scaled from PDM int16 range |
| 15–16 | Lat offset | int16 LE | `(latitude - FIELD_CENTER_LAT) × 100000` — units of 0.00001°, ~1.1m |
| 17–18 | Lng offset | int16 LE | `(longitude - FIELD_CENTER_LNG) × 100000` — same units |
| 19 | Reserved | uint8 | Always 0 |

### Status Flags (Byte 0)

| Bit | Mask | Meaning |
|-----|------|---------|
| 0 | `0x01` | Logging active |
| 1 | `0x02` | GPS has fix (valid + ≥4 sats) |
| 2 | `0x04` | Battery low (voltage < 3.3V) |
| 3 | `0x08` | Recent impact (auto-clears after 2 seconds) |
| 4 | `0x10` | BLE connected (a central is connected via NUS) |
| 5 | `0x20` | Focus mode active |
| 6–7 | | Reserved (0) |

### GPS Position Encoding

- **Reference point:** `FIELD_CENTER_LAT = -25.7479`, `FIELD_CENTER_LNG = 28.2293`
- **Formula:** `offset = (coordinate - center) × 100000`, cast to int16
- **Range:** ±32767 × 0.00001° = ±0.33° ≈ ±36 km
- **Precision:** ~1.1m per unit
- **No-fix sentinel:** `0x7FFF` (32767) for both lat and lng = "no GPS position"

### Battery Percentage

```
battPct = clamp((voltage - 3.3) / 0.9 × 100, 0, 100)
```
- 3.3V = 0%, 4.2V = 100% (LiPo range)

---

## 2. Intensity Calculation

### 2.1 Raw Intensity (computed at 104 Hz, every IMU sample)

```
accelMag = sqrt(ax² + ay² + az²)           // in g
gyroMag  = sqrt(gx² + gy² + gz²)           // in dps

accelActivity = max(0, accelMag - 1.0)      // subtract 1g gravity
gyroActivity  = gyroMag / 100.0             // normalize (0-500 dps → 0-5)

intensityRaw = 0.7 × accelActivity + 0.3 × gyroActivity
```

### 2.2 Exponential Moving Averages (updated at 104 Hz)

**1-second EMA:**
```
alpha_fast = 0.019                          // = 2 / (1 × 104 + 1)
intensityCurrent = intensityCurrent × (1 - alpha_fast) + intensityRaw × alpha_fast
```

**1-minute EMA:**
```
alpha_slow = 0.00032                        // = 2 / (60 × 104 + 1)
intensity1Min = intensity1Min × (1 - alpha_slow) + intensityRaw × alpha_slow
```

### 2.3 10-Minute Rolling Average (sampled at 2 Hz)

Circular buffer of 1200 floats (600 seconds × 2 Hz).

```
// Called every 500ms:
intensity10MinSum -= intensityHistory[index]
intensityHistory[index] = intensityCurrent
intensity10MinSum += intensityCurrent
if (samplesInHistory < 1200) samplesInHistory++
index = (index + 1) % 1200

// To read:
intensity10MinAvg = intensity10MinSum / samplesInHistory
```

During the first 10 minutes, `samplesInHistory` tracks actual fill level for an accurate average.

### 2.4 Movement Counter

Counts rising edges of `intensityCurrent` crossing above 0.3:

```
isAbove = intensityCurrent > 0.3
if (isAbove && !wasAbove) moveCount++
wasAbove = isAbove
```

Updated at 104 Hz. Stored as uint16 in BLE packet.

---

## 3. Impact Detection (Dual-Trigger)

Requires **both** an audio spike **and** an IMU high-g event within 200ms of each other. This eliminates false positives from shouts (audio only) or hard steps (IMU only).

### 3.1 IMU Trigger (checked at 104 Hz)

```
accelMag = sqrt(ax² + ay² + az²)
if (accelMag > 3.0g):
    lastImuSpike = now
```

### 3.2 Audio Trigger (checked at 10 Hz)

```
// Relative threshold: peak must be 8× above baseline
aboveRelative = (audioPeak > audioBaseline × 8.0)
// Absolute minimum: peak must exceed 3000 (PDM range 0–32767)
aboveAbsolute = (audioPeak > 3000)
audioImpact = aboveRelative AND aboveAbsolute

if (audioImpact):
    lastAudioSpike = now
```

### 3.3 Baseline Adaptation

The audio baseline (RMS) adapts slowly to ambient noise. It does NOT update during impact events:

```
if (NOT audioImpact):
    audioBaseline = 0.005 × audioRMS + 0.995 × audioBaseline
```

Initial baseline: 500.0

### 3.4 Confirmation Logic (checked at 10 Hz in processAudioRMS)

```
imuConfirmed   = (lastImuSpike > 0)   AND (now - lastImuSpike   < 200ms)
audioConfirmed = (lastAudioSpike > 0)  AND (now - lastAudioSpike < 200ms)

if (imuConfirmed AND audioConfirmed AND (now - lastImpact > 3000ms)):
    // CONFIRMED IMPACT
    impactCount++
    lastImpact = now
    lastImuSpike = 0      // Reset so same spike can't fire twice
    lastAudioSpike = 0
    set impact flag in BLE (auto-clears after 2 seconds)
    if logging: mark "IMPACT" event in CSV
```

**Debounce:** 3 seconds minimum between impacts.

### 3.5 Impact Flag in BLE

When an impact is confirmed:
- Set `impactFlagSetTime = now`
- Byte 0 bit 3 (`0x08`) is set in all BLE packets for 2 seconds
- After 2 seconds: auto-clears (checked each advertising update)
- `impactCount` (byte 8) increments permanently for the session

---

## 4. Audio Processing (PDM Microphone)

**Sample rate:** 16 kHz mono
**Processing rate:** 10 Hz (every 100ms)
**PDM gain:** 30 (Arduino PDM library setting)
**Buffer:** 512 int16 samples

### Per-window metrics:

```
for each sample in buffer:
    sumSquares += sample × sample
    if |sample| > peak: peak = |sample|
    if sign change from previous: zeroCrossings++

audioRMS = sqrt(sumSquares / sampleCount)
audioPeak = peak
audioZCR = zeroCrossings
```

### BLE encoding:

Audio peak is scaled for the BLE byte: `audioPeakScaled = min(rawPeak / 128, 255)`

---

## 5. GPS (NMEA Parser)

**UART:** 9600 baud, interrupt-driven
**Sentences parsed:** `$GPRMC` / `$GNRMC` (position + speed) and `$GPGGA` / `$GNGGA` (satellite count)
**Checksum:** XOR of bytes between `$` and `*` (exclusive), validated before parsing

### RMC fields used:
- Field 2: Status (`A` = active, anything else = invalid)
- Field 3–4: Latitude (DDMM.MMMM, N/S)
- Field 5–6: Longitude (DDDMM.MMMM, E/W)
- Field 7: Speed in knots → multiply by 1.852 for km/h
- Field 8: Course in degrees

### GGA fields used:
- Field 7: Number of satellites

### Coordinate conversion:
```
rawLat = 2547.8740    // DDMM.MMMM format
degrees = int(rawLat / 100)          // = 25
minutes = rawLat - degrees × 100     // = 47.8740
latitude = degrees + minutes / 60.0  // = 25.79790
if hemisphere == 'S': latitude = -latitude
```

### GPS staleness:
GPS data is considered stale (not included in CSV) if `now - lastUpdate > 2000ms`.

### GPS status byte (BLE byte 9):
```
age  = clamp((now - lastUpdate) / 1000, 0, 15)  // seconds, capped at 15
sats = clamp(satellites, 0, 15)
gpsStatus = (age << 4) | sats
```

---

## 6. BLE Commands (Nordic UART Service)

**Service UUID:** `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
**RX (write to device):** `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
**TX (notify from device):** `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

Commands are plain text, newline-terminated. Responses sent back over TX characteristic in ≤20-byte chunks with 5ms delay between chunks, terminated by `\n`.

| Command | Response | Side Effects |
|---------|----------|-------------|
| `start` | `OK:Logging started` or `ERR:Failed to start logging` | Opens next LOGnnn.CSV, resets session stats |
| `stop` | `OK:Logging stopped` | Flushes buffer, closes file |
| `status` | Multi-line: `STATUS:LOGGING\|IDLE`, `GPS:OK\|NO_FIX SATS:n`, `BAT:x.xxV`, (if logging: `FILE:`, `SAMPLES:`, `ELAPSED:`) | None |
| `event:TAG` | `OK:Event marked` | Next CSV row includes TAG in event column |
| `bat` | `BAT:x.xxV` | None |
| `info` | `CoMotion v2.0.0` | None |
| `ping` | `pong` | None |
| `focus` | `OK:Focus mode ON (60s)` | Advertising interval → 100ms for 60s |
| `normal` | `OK:Normal mode` | Advertising interval → 500ms |
| (unknown) | `ERR:Unknown command` | None |

### Focus Mode
- Activated by `focus` command
- Sets advertising interval to 100ms (vs normal 500ms)
- Auto-expires after 60 seconds → reverts to 500ms
- Status flag bit 5 (`0x20`) is set while active

---

## 7. Session Lifecycle

### On `start`:
1. Find next available filename: `LOG001.CSV` → `LOG999.CSV`
2. Create file, write CSV header
3. Reset: `bufferHead=0, bufferTail=0, bufferCount=0, sampleCount=0`
4. Reset session stats: `maxSpeed=0, impactCount=0, moveCount=0, intensityCurrent=0, intensity1Min=0, intensity10MinSum=0, history zeroed`
5. `logStartTime = now`
6. `isLogging = true`

### On `stop`:
1. Flush ring buffer to SD
2. Close file
3. `isLogging = false`

### Session stats that reset on `start`:
- maxSpeedSession
- impactCountSession
- moveCountSession
- intensityCurrent, intensity1Min
- intensity10MinSum + entire 1200-sample history array
- sampleCount

---

## 8. SD Card Logging

### CSV Format:
```
timestamp,ax,ay,az,gx,gy,gz,lat,lng,speed,course,sats,audio_rms,audio_peak,audio_zcr,event
```

- **timestamp:** milliseconds since boot (uint32, wraps at ~49 days)
- **ax,ay,az:** Accelerometer in g (4 decimal places)
- **gx,gy,gz:** Gyroscope in dps, calibration offset applied (2 decimal places)
- **lat,lng:** Degrees (6 decimal places) — empty if no valid GPS
- **speed:** km/h (1 decimal) — empty if no GPS
- **course:** degrees (1 decimal) — empty if no GPS
- **sats:** integer — empty if no GPS
- **audio_rms:** float, no decimals
- **audio_peak:** integer (raw PDM amplitude)
- **audio_zcr:** integer (zero crossings per window)
- **event:** string, one-shot (cleared after first logged row). Wrapped in quotes if contains comma.

### Ring Buffer:
- 4096-byte circular char buffer
- Written at 104 Hz (one CSV line per IMU sample, ~160–200 bytes each)
- Flushed to SD when ≥50% full OR every 5 seconds
- Flush uses `irq_lock()` to atomically copy from ring buffer to write buffer
- Then writes + syncs to SD outside the lock

### File naming:
Auto-increment: `LOG001.CSV`, `LOG002.CSV`, ..., `LOG999.CSV`

---

## 9. Gyro Calibration (Startup)

Runs once at boot. Determines gyroscope zero-offset.

### Phase 1: Wait for stillness (up to 10 seconds)
```
loop:
    read gyro XYZ
    gyroMag = sqrt(gx² + gy² + gz²)
    if gyroMag < 5.0 dps:
        if wasn't still: mark still start time
        else if still for ≥ 1000ms: proceed to phase 2
    else:
        reset still flag
    sleep 10ms
```

If not still within 10 seconds: skip calibration (use zero offsets).

### Phase 2: Collect 100 samples
```
for i in 0..99:
    sum += readGyroXYZ
    sleep 5ms
offset = sum / 100
```

All subsequent gyro readings have this offset subtracted.

---

## 10. Battery ADC

**Hardware:** nRF52840 SAADC, channel 0, AIN7 (P0.31)
**Divider:** 1/3 voltage divider on XIAO board
**Enable pin:** P0.14 (output LOW to enable VBAT reading)

### Configuration:
- Resolution: 12-bit (0–4095)
- Reference: Internal (0.6V)
- Gain: 1/6 → full-scale input = 3.6V
- Acquisition time: 10µs
- Mode: Single-ended, no oversampling

### Voltage formula:
```
voltage = adcValue / 4095.0 × 3.6 × 3.0
```
(3.0 multiplier = compensate for 1/3 divider)

**Read interval:** Every 30 seconds.

---

## 11. LED Status

| State | LED | Pattern |
|-------|-----|---------|
| Booting | Blue | Solid |
| Idle (not logging) | Blue | Slow blink (500ms toggle) |
| Logging + GPS fix | Green | Solid |
| Logging, no GPS | Green | Fast blink (250ms toggle) |
| Low battery (<3.3V) | Red | Solid (overrides others) |
| Event marked | Blue | Brief 50ms flash |

LEDs are active-low on XIAO nRF52840 (LOW = on, HIGH = off).

---

## 12. Timing Summary

| Task | Rate | Triggered By |
|------|------|-------------|
| IMU read + intensity update | 104 Hz | Timer/micros |
| SD card log (if logging) | 104 Hz | Same as IMU |
| Audio RMS processing | 10 Hz | Timer/millis |
| Intensity history sample | 2 Hz | Timer/millis |
| BLE advertising update | 2 Hz (500ms) or 10 Hz (100ms focus) | Timer/millis |
| SD buffer flush | When ≥50% full OR every 5s | Checked in main loop |
| LED update | 2 Hz (500ms) | Timer/millis |
| Battery read | Every 30s | Timer/millis |
| GPS NMEA parse | Continuous | UART IRQ/available |

---

## 13. Constants Reference

```c
// Field center
FIELD_CENTER_LAT          = -25.7479
FIELD_CENTER_LNG          =  28.2293

// Sampling
IMU_SAMPLE_RATE_HZ        = 104
AUDIO_RMS_RATE_HZ         = 10
GPS_TIMEOUT_MS            = 2000

// Impact detection
AUDIO_IMPACT_THRESHOLD    = 8.0      // Peak / baseline ratio
AUDIO_IMPACT_MIN_PEAK     = 3000     // Absolute minimum peak
AUDIO_BASELINE_ALPHA      = 0.005    // Baseline adaptation rate
AUDIO_IMPACT_DEBOUNCE     = 3000     // 3s between impacts
IMPACT_ACCEL_THRESHOLD    = 3.0      // g for IMU confirmation
IMPACT_CONFIRM_WINDOW     = 200      // ms for dual-trigger window

// Intensity
INTENSITY_ACCEL_WEIGHT    = 0.7
INTENSITY_GYRO_WEIGHT     = 0.3
INTENSITY_GYRO_SCALE      = 100.0
INTENSITY_EMA_FAST        = 0.019    // 1s at 104Hz
INTENSITY_EMA_SLOW        = 0.00032  // 60s at 104Hz
INTENSITY_HISTORY_SIZE    = 1200     // 10 min at 2Hz
INTENSITY_SAMPLE_HZ       = 2
MOVEMENT_THRESHOLD        = 0.3      // intensity level for move count

// BLE
BLE_MANUFACTURER_ID       = 0xFFFF
BLE_ADV_INTERVAL_NORMAL   = 500      // ms
BLE_ADV_INTERVAL_FOCUS    = 100      // ms
BLE_ADV_UPDATE_MS         = 500      // ms
IMPACT_FLAG_DURATION_MS   = 2000     // ms
FOCUS_MODE_TIMEOUT        = 60000    // ms

// SD
SD_BUFFER_SIZE            = 4096
SD_FLUSH_INTERVAL_MS      = 5000

// Gyro calibration
GYRO_CAL_TIMEOUT_MS       = 10000
GYRO_CAL_STILL_TIME_MS    = 1000
GYRO_CAL_SAMPLES          = 100
GYRO_CAL_THRESHOLD_DPS    = 5.0

// Battery
BATTERY_READ_INTERVAL_S   = 30
BATTERY_EMPTY_V           = 3.3
BATTERY_FULL_V            = 4.2
```
