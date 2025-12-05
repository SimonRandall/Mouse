# Copilot Instructions for Gloopin

**Project Type**: ESP32-based robotics platform with motor control and IMU sensor integration  
**Target**: ESP32-S3 (via `idf_build_set_property(MINIMAL_BUILD ON)` optimization)

## Architecture Overview

Gloopin is a mobile robot with:
- **Motor Control**: Dual DC motors (Motor A/B) via LEDC PWM on pins 4-7 (duty 0-1023)
- **Inertial Measurement**: MPU9250 I2C sensor (address 0x68) on I2C_NUM_0 (SDA=8, SCL=9, 400kHz)
- **Control Loop**: FreeRTOS-based with 10ms task timing (gyro-driven angle tracking for turns)

**Key Data Flow**:
1. `setup()` → I2C init, MPU9250 init, LEDC timer/channel config, gyro bias calibration
2. Movement functions (`goForward`, `turnRight/Left`) → motor duty control + gyro feedback
3. `app_main()` → movement sequencing (currently: forward 2s loops)

## Critical Build & Deployment

- **Build System**: CMake with ESP-IDF integration
  - Root: `idf_build_set_property(MINIMAL_BUILD ON)` enables minimal component build
  - Component: `main/CMakeLists.txt` registers source + dependencies (`spi_flash`, `driver/ledc`, `driver/i2c`)
- **Build Command**: `idf.py build` (configured via ESP-IDF environment)
- **Flash Command**: `idf.py -p <PORT> flash` (auto-detects port if unambiguous)
- **Monitor**: `idf.py -p <PORT> monitor` (view logs + reboot signals)
- **Full Rebuild**: `idf.py fullclean; idf.py build`

## Sensor & Hardware Conventions

### MPU9250 Driver (`mpu9250_simple.{c,h}`)
- **I2C Register Interface**: WHO_AM_I (0x75) verifies 0x71, wake-up via REG_PWR_MGMT_1
- **Raw → Physical**: Accelerometer divisor 16384.0, Gyro divisor 131.0 (default ±2g, ±250 dps)
- **Error Handling**: All read/gyro functions return `esp_err_t`; check return vs ESP_OK
- **Pattern**: Register read helpers (`read_bytes()`, `write_reg()`) handle I2C command queuing

### Motor Control via LEDC
- **Timer**: `LEDC_LOW_SPEED_MODE`, `LEDC_TIMER_0`, 10-bit resolution (1023 max), 5kHz frequency
- **Channel Mapping**: 
  - Motor A: LEDC_CHANNEL_0 (pin 4), LEDC_CHANNEL_1 (pin 5)
  - Motor B: LEDC_CHANNEL_2 (pin 6), LEDC_CHANNEL_3 (pin 7)
- **Duty Pattern**: `ledc_set_duty()` → `ledc_update_duty()` (always call both for changes)
- **Control Logic**: Motor speed via duty cycle; direction via IN1/IN2 differential (1023 vs duty value)

## Motion Control Patterns

### Gyro Bias Calibration
```c
// ~200 samples at 5ms intervals = ~1 second
// Stores global `gyro_bias_z` for Z-axis drift compensation
calibrate_gyro_bias();
```

### Angle Tracking in Turns
```c
// turnRight/turnLeft pattern:
// 1. Set opposite motor duty (left forward=500, right=1023 for right turn)
// 2. Loop: read gyro → subtract bias → integrate angle → 10ms delay
// 3. Exit when angle reaches ±90° target
// 4. Stop all motors (duty=0)
```

**Timing Critical**: `vTaskDelayUntil()` ensures 10ms fixed tick; `dt=0.01` (100 Hz gyro effective rate)

## Common Workflows

### Adding New Movement
1. Define function taking motor parameters (duty cycles, angles)
2. Initialize LEDC channels via loop (see `setup()`)
3. Use gyro feedback loop for angle-based movements (see `turnRight/Left`)
4. Always reset motors to 0 duty after completion

### Testing New Sensor Code
- `pytest_hello_world.py` available for automated testing
- Monitor serial output: `idf.py monitor` (watch for `ESP_LOG*` messages)
- MPU9250 outputs float values; validate against gyro bias after calibration

### Debugging Hardware Issues
- **No MPU9250 response**: Check I2C pins (8=SDA, 9=SCL), verify 0x68 address
- **Motors unresponsive**: Confirm duty update (must call both set + update), check pin assignments
- **Gyro drift**: Extend calibration sample count (200 is baseline, increase for stability)

## Code Style & Patterns

- **Naming**: GPIO defines (e.g., `motorA_in1`), snake_case functions, CamelCase variables
- **I2C Ops**: Always use command queue pattern (`i2c_cmd_link_create/delete`)
- **Error Propagation**: Return `esp_err_t`, check vs `ESP_OK` before using output
- **Task Safety**: `vTaskDelay/vTaskDelayUntil` for timing; use `pdMS_TO_TICKS()` for milliseconds
- **Includes**: Standard ESP-IDF headers grouped (freertos, driver, esp_*); custom last
