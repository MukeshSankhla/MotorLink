# Arduino Uno Self-Balancing Robot Controller

This directory contains the firmware and PC control script to run the self-balancing robot with an **Arduino Uno** and an **MPU-6050 IMU**.

---

## 1. Hardware Connections (Arduino Uno)

| MPU-6050 Pin | Arduino Uno Pin | Description |
| :--- | :--- | :--- |
| **VCC** | **5V** (or **3.3V**) | Sensor power |
| **GND** | **GND** | Ground reference |
| **SDA** | **Pin A4** | Hardware I2C Data line |
| **SCL** | **Pin A5** | Hardware I2C Clock line |

Connect the Arduino Uno to your PC via a USB cable. It will register as a COM port (e.g. `COM10`). 
Connect the RS485 motor bus adapter to your PC (e.g. `COM13`).

---

## 2. Arduino Firmware Upload

1. Open the Arduino IDE.
2. Select **File > Open** and load [arduino_mpu_stream.ino](file:///c:/Users/mukes/Downloads/MotorLink/uno_balance_robot/arduino_mpu_stream/arduino_mpu_stream.ino).
3. Select board: **Arduino Uno** under **Tools > Board**.
4. Select the correct COM Port under **Tools > Port**.
5. Upload the code.
6. Keep the robot **completely stationary** during startup to allow gyroscope offset calibration to complete.

---

## 3. Running the Python balancing script

### Prerequisites
Make sure you have `pyserial` installed:
```bash
pip install pyserial
```

### Running Modes
The Python script `balance_robot.py` dynamically detects the running environment:

#### Mode A: Running directly on the Arduino Uno Q Board (MCU + Linux MPU)
When running the script directly on the Qualcomm Linux MPU of the Uno Q board:
- **No USB Cable/COM Port for IMU**: The script dynamically imports `arduino.app_utils` and fetches telemetry via the internal **Bridge** RPC (`Bridge.call("get_mpu_data")`).
- **Motor Port**: On Linux/Uno Q, the default motor port is `/dev/ttyACM0`. Simply run the script:
  ```bash
  python balance_robot.py
  ```

#### Mode B: Running on an External PC (connected to standard Arduino Uno)
When running the script on a PC connected to a separate Arduino Uno over USB:
- **Serial Connection**: The script reads telemetry over standard USB Serial (default: `COM10`).
- **Running command**:
  ```bash
  python balance_robot.py --arduino COM10 --motor COM13
  ```

---

## 4. Web Dashboard Control (Hosted on Local Network)

When the script starts, it spins up a local web server and prints the connection links:
```text
[✓] Web Dashboard running at:
    - Local Host:    http://localhost:8000
    - Local Network: http://192.168.1.100:8000
```

1. **Local Network Access**: You can open the dashboard on **any device** (like a smartphone, tablet, or secondary laptop) connected to the **same Wi-Fi network** by navigating to the `Local Network` address shown in your terminal.
2. **Dashboard Controls**:
   - **Real-time Telemetry**: Views pitch angles and active RPMs at 10Hz.
   - **Tactile Drive controls**: Drag or touch the D-pad buttons, or press **Arrow Keys** / keyboard **W A S D** to command the robot to drive.
   - **PID Tuning**: Modify Kp, Ki, Kd, speed limits, and slew rates live from the browser without restarting.
   - **Safety Reset**: Re-enable balancing with one click after a tilt shutdown.
