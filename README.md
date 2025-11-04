# ESP32 Flow Meter with Web Interface

A high-precision flow meter implementation using ESP32 with a modern web interface. This project uses the MJ-HW83C flow sensor and provides real-time monitoring through a responsive web interface.

## Features

- 🚰 **Accurate Flow Measurement**
  - Pulse-counting with hardware PCNT
  - Configurable pulses per liter (1319 PPL by default)
  - ±5% calibration trim

- 🌐 **Web Interface**
  - Real-time flow rate visualization
  - Total volume tracking
  - Responsive design for desktop and mobile
  - WebSocket for live updates

- ⚡ **Power Management**
  - Light sleep mode during inactivity
  - Automatic wake-up on flow detection
  - Power-optimized WiFi handling

- 🛡️ **Security**
  - Basic HTTP authentication
  - Configurable credentials

- 📊 **Advanced Features**
  - Adaptive EMA smoothing
  - Overflow detection and recovery
  - Watchdog timer
  - mDNS support (esp32flow.local)

## Hardware Requirements

- ESP32 development board
- MJ-HW83C flow sensor (or compatible)
- 3.3V to 5V power supply
- Jumper wires

## Wiring

| ESP32 Pin | Flow Sensor |
|-----------|-------------|
| 3.3V      | Red (VCC)   |
| GND       | Black (GND) |
| GPIO34    | Yellow (Pulse) |

## Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/dvirchakim/ESP32-FLOW-METER-MJ-HW83C.git
   cd ESP32-FLOW-METER-MJ-HW83C
   ```

2. **Install required libraries**
   - WebServer (included with ESP32 Arduino core)
   - WebSockets (install via Arduino Library Manager)
   - ArduinoJson (install via Arduino Library Manager)

3. **Configure settings**
   Open `flow_meter.ino` and update:
   - WiFi credentials
   - Authentication credentials (optional)
   - Calibration values if needed

4. **Upload to ESP32**
   - Select your ESP32 board in Arduino IDE
   - Select the correct port
   - Click Upload

## Usage

1. **Connect to the device**
   - Connect to the ESP32's WiFi network (default: ESP32-Flow/12345678)
   - Or connect to your local network (if configured)

2. **Access the web interface**
   - Open a web browser and go to:
     - `http://esp32flow.local` (if mDNS works on your network)
     - `http://<ESP32_IP>` (check Serial Monitor for IP)

3. **Calibration**
   - The default calibration is 1319 pulses per liter
   - Adjust `PULSES_PER_LITER` in the code if needed
   - Use `CAL_TRIM` for fine-tuning (±5%)

## Configuration

### Flow Sensor
- `PULSES_PER_LITER`: 1319.0f (default)
- `CAL_TRIM`: 1.00f (1.00 = no trim, 0.95 = -5%, 1.05 = +5%)

### Power Management
- `DEEP_SLEEP_ENABLED`: true/false
- `FLOW_TIMEOUT_MS`: 300000 (5 minutes of inactivity before sleep)

### WiFi
- `WIFI_SSID`: Your WiFi network name
- `WIFI_PASSWORD`: Your WiFi password
- `AP_SSID`: "ESP32-Flow" (Access Point name)
- `AP_PASS`: "12345678" (Access Point password)

### Security
- `www_username`: "admin" (web interface username)
- `www_password`: "flowmeter" (web interface password)

## Troubleshooting

### No Flow Reading
- Check wiring connections
- Verify sensor is receiving power (LED should blink with flow)
- Check Serial Monitor for error messages

### Can't Connect to Web Interface
- Verify WiFi connection
- Check IP address in Serial Monitor
- Ensure no firewall is blocking the connection

### Sensor Reading Fluctuates
- Ensure no air bubbles in the flow
- Check for kinks in the tubing
- Adjust `PCNT_GLITCH_FILTER` if needed

## License

This project is open source and available under the [MIT License](LICENSE).

## Author

Dvir Chakim - [GitHub](https://github.com/dvirchakim)

## Acknowledgments

- ESP32 Arduino Core Team
- WebSockets Library
- ArduinoJson Library
- SWE-1 + WINDSURF  