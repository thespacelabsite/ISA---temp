# Flight Software Hardware Integration & Reversion Guide

**Document Purpose**: This guide provides the hardware team with the necessary instructions to revert the test-mode bypasses and integrate real sensor data/conversions into the flight software codebase.

---

## 1. Exit Test Mode
The flight software is currently controlled by a global `testMode` flag. When true, it bypasses hardware sensor processing and relies on CSV injection.

**Action Required**:
- In `fswtest.c` (or wherever `systemState` is initialized), set `systemState.testMode = false;`.
- Ensure `isOBCReset` and `isT0Set` are triggered by actual flight events rather than hardcoded in `main()`.

---

## 2. Sensor Data Injection & LSB Conversions
In `src/major.c` and `src/minor.c`, the raw hardware interfaces are currently bypassed or commented out.

### GNSS Data (`src/major.c`)
- **Location**: `set_gnss_position_ecef` and `set_gnss_velocity_ecef` (Lines 188-231).
- **Status**: LSB-to-Double conversion functions are currently commented out. 
- **Action Required**: 
  - Uncomment the calls to `convert_ecef_position_lsb_to_double` and `convert_ecef_velocity_lsb_to_double`.
  - Implement these conversion functions in `src/type_convert.c` (ensure big-endian signed 32-bit to double conversion matches the ICD).
- **Critical Note on Coordinate Frames**: 
  - **WARNING**: The variables are named `ECEF` (Earth-Centered Earth-Fixed), but the Guidance Algorithm follows a legacy design that **expects ECI (Earth-Centered Inertial)** coordinates for accurate target vectoring.
  - If your hardware provides true ECEF, you must either:
    1. Update the guidance algorithm to handle rotation.
    2. Convert ECEF -> ECI before calling `set_gnss_...`.
    3. (Recommended) If the hardware provides ECI, feed it directly into the variables named "ECEF".

### IMU/Magnetometer Data (`src/minor.c`)
- **Location**: `minor_cycle` (Lines 1850-1885).
- **Status**: When `testMode` is false, it calls `process_accelerometer_data()`, `process_gyroscope_data()`, etc.
- **Action Required**: 
  - Implement the "Read Hardware Register" logic in each of these functions.
  - Ensure the raw LSB values are passed to `src/type_convert.c` for proper unit conversion to physical units (m/s², rad/s, mG).

---

## 3. Health Checks & Fail-Safes
Health checks were bypassed to allow simulation to run without real status bits.

### GNSS Lock & Fail-Safe (`src/major.c`)
- **Location**: `process_gnss_data` (Lines 60-122).
- **Status**: The `if (systemState.testMode)` branch bypasses the check for `gnss_lock`.
- **Action Required**:
  - The hardware team must ensure the `gnss_lock` volatile boolean is mapped to the actual GNSS hardware LOCK signal.
  - Implement the "TBD" parsing logic (Line 82) to extract coordinates from the DMA/UART `gnssDataBuffer`.

### Sensor Health (`src/minor.c`)
- **Location**: `check_sensor_health` (Line 48).
- **Action Required**: Implement parity/status bit checks for all SPI/I2C sensors. If health fails, error flags in `systemState` must be set to trigger fall-back modes.

---

## 4. Local Coordinate Frame Pre-computation
To match the precision of the reference implementation, the Local Frame axes are now pre-computed once using the **Geodetic Normal** method (+1m altitude perturbation) instead of a radial approximation.

- **Location**: `guidance_init` in `src/major.c` (Line 286).
- **Logic**: This uses the `PEFCS_ORIGIN_...` constants from `pefcs_init.h`.
- **Integration Note**: If the launch point is dynamic, the hardware team must ensure `guidance_init()` is called **after** the first valid GNSS lock is obtained at the launch pad, or update the origin variables dynamically.

---

## 5. Timing Constants
- **Major Cycle**: 10Hz (100ms).
- **Minor Cycle**: 100Hz (10ms).
- **Integration Step**: `INTEGRATION_STEP_SIZE` is defined as `0.01` (100Hz).
- **Warning**: Ensure the hardware timer/scheduler accurately triggers `minor_cycle` every 10ms. If the timing drifts, all navigation integrations (quaternions, position) and the guidance `timeToGo` will accumulate errors.

---

## Summary of Reversion Checklist
1. [ ] Set `systemState.testMode = false`.
2. [ ] Uncomment conversion calls in `src/major.c`.
3. [ ] Implement LSB-to-SI-unit logic in `src/type_convert.c`.
4. [ ] Map `gnss_lock` and IMU registers to real hardware peripherals.
5. [ ] Implement register reading in `src/minor.c`'s `process_..._data` functions.
6. [ ] Verify that `gnssDataBuffer` parsing is operational.
