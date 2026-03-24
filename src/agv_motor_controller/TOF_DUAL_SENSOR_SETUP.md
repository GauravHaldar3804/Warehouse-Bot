# Dual TOF Sensor Setup Guide

## Overview
This guide explains how to set up and use 2 VL53L0X TOF (Time of Flight) sensors simultaneously with your AGV motor controller.

## Hardware Setup

### I2C Address Problem
Both VL53L0X sensors have the same default I2C address (0x29). To use two simultaneously, you need to:
1. Change the I2C address of one sensor to a different address (e.g., 0x30)
2. Use GPIO pins to control the XSHUT (shutdown) pins

### Wiring Diagram

```
Raspberry Pi (or your controller board)
│
├─ I2C SDA (GPIO 2)  ──┬─ Sensor 1 SDA
│                      └─ Sensor 2 SDA
│
├─ I2C SCL (GPIO 3)  ──┬─ Sensor 1 SCL
│                      └─ Sensor 2 SCL
│
├─ GPIO 4 (board.D4)  ──── Sensor 1 XSHUT
│
├─ GPIO 5 (board.D5)  ──── Sensor 2 XSHUT
│
├─ 3.3V ────┬─ Sensor 1 VDD
│           └─ Sensor 2 VDD
│
└─ GND  ────┬─ Sensor 1 GND
            └─ Sensor 2 GND
```

### PIN Configuration (Customizable in Code)

Edit the `__init__` method in `tof_test_node.py`:

```python
# GPIO pins for XSHUT control (change to your GPIO pins if needed)
self.XSHUT_PIN_1 = board.D4   # XSHUT pin for sensor 1
self.XSHUT_PIN_2 = board.D5   # XSHUT pin for sensor 2

# I2C addresses
self.SENSOR1_ADDRESS = 0x29   # Default VL53L0X address
self.SENSOR2_ADDRESS = 0x30   # Changed address for second sensor
```

**Change `board.D4` and `board.D5` to your actual GPIO pins.**

Common GPIO pin mappings:
- `board.D4` = GPIO 4
- `board.D5` = GPIO 5
- `board.D17` = GPIO 17
- `board.D27` = GPIO 27

## Changing I2C Address

The code automatically handles I2C address configuration:
1. Powers down both sensors (XSHUT = LOW)
2. Powers up Sensor 1, initializes it at default address 0x29
3. Powers up Sensor 2, changes its address to 0x30

## ROS Topics Published

### Float32 Topics (Distance in cm)
- `tof_distance_1` - Distance from Sensor 1 in centimeters
- `tof_distance_2` - Distance from Sensor 2 in centimeters

### Range Message Topics (Distance in meters)
- `tof_range_1` - Standard ROS Range message from Sensor 1
- `tof_range_2` - Standard ROS Range message from Sensor 2

## Usage

### Run the dual sensor node:
```bash
ros2 run agv_motor_controller tof_test_node
```

### Monitor sensor data:
```bash
# View Sensor 1 data
ros2 topic echo /tof_distance_1

# View Sensor 2 data
ros2 topic echo /tof_distance_2

# View Sensor 1 range message
ros2 topic echo /tof_range_1

# View Sensor 2 range message
ros2 topic echo /tof_range_2
```

## Changing I2C Address (Manual Method)

If you need to change the address to something other than 0x30, modify in `tof_test_node.py`:

```python
self.SENSOR2_ADDRESS = 0x31  # Change 0x30 to your desired address
```

Valid addresses for VL53L0X: 0x29-0x7F (any address not already in use)

## Troubleshooting

### Problem: "Sensor not found" error
- Check XSHUT pin connections
- Verify GPIO pins match your board configuration
- Make sure both sensors are powered
- Test each sensor individually first

### Problem: Address conflict error
- Verify XSHUT pins are properly connected and working
- Check that SENSOR2_ADDRESS is not 0x29 (default)
- Ensure only one sensor has the default address

### Problem: Readings are 0 or very high
- Check if sensors are within their operating range (up to 2 meters)
- Ensure clear line of sight to the target
- Verify the sensor is not damaged

### Problem: One sensor works but the other doesn't
- Swap physical sensor connections to identify the faulty sensor
- Test each sensor individually:
  - Connect only one sensor at a time
  - Use the original single-sensor code

## Required Dependencies

Make sure these packages are installed:
```bash
pip install adafruit-circuitpython-vl53l0x
pip install adafruit-blinka  # For GPIO support
```

## Switching Back to Single Sensor

If you need to revert to single sensor, uncomment the old code or use:
```bash
git checkout setup.py  # If you version control it
```

Or manually change the code to remove Sensor 2 initialization and related publishers.
