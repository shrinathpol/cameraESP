# ESP32-S3 Camera Web Streamer (ESP-IDF)

A complete, high-performance ESP-IDF project for ESP32-S3 camera modules (OV2640, OV3660, OV5640). Features live MJPEG streaming, high-resolution snapshot capture, PSRAM optimization, and a responsive web dashboard.

---

## 📁 Project Structure

```
cameraESP/
├── CMakeLists.txt              # Root CMake configuration
├── sdkconfig.defaults          # Target (esp32s3), PSRAM, and system settings
├── main/
│   ├── CMakeLists.txt          # Component registration
│   ├── idf_component.yml       # Official espressif/esp32-camera dependency
│   ├── camera_pins.h           # Predefined pin maps for common ESP32-S3 boards
│   └── main.c                  # Wi-Fi STA, HTTP web server & camera streamer
└── .vscode/                    # VS Code ESP-IDF extension settings
```

---

## ⚙️ Configuration

### 1. Set Your Wi-Fi Credentials
Open `main/main.c` and enter your Wi-Fi SSID and password:
```c
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
```

### 2. Select Your Board Model
Open `main/camera_pins.h` and uncomment the definition matching your hardware:
- `CAMERA_MODEL_FREENOVE_ESP32S3_CAM` *(Default: covers Freenove ESP32-S3 WROOM CAM and generic AI-Thinker S3 clones)*
- `CAMERA_MODEL_XIAO_ESP32S3_SENSE` *(Seeed Studio XIAO ESP32S3 Sense)*
- `CAMERA_MODEL_WAVESHARE_ESP32S3` *(Waveshare ESP32-S3-CAM)*
- `CAMERA_MODEL_ESP32S3_EYE` *(Espressif ESP32-S3-EYE official dev board)*

---

## 🚀 Building & Flashing

### Using VS Code ESP-IDF Extension
1. Open the project folder in VS Code.
2. At the bottom status bar, select:
   - Target: `esp32s3`
   - Port: `COM6` (or your device's COM port)
   - Flash method: `UART`
3. Click the **Build** icon (or press `Ctrl+E`, then `B`).
4. Click the **Flash & Monitor** icon (or press `Ctrl+E`, then `D`).

### Using ESP-IDF Terminal
```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COM6 flash monitor
```

---

## 🌐 Web Endpoints

Once connected, the device prints its assigned IP address to the monitor:

| Endpoint | Description |
| :--- | :--- |
| `http://<ESP32-IP>/` | Web UI with live preview, refresh controls, and quick links |
| `http://<ESP32-IP>/stream` | Raw MJPEG stream (can be embedded in `<img>` or VLC / Home Assistant) |
| `http://<ESP32-IP>/capture` | Single still JPEG snapshot |
| `http://<ESP32-IP>/status` | JSON payload with sensor PID, free heap, and free PSRAM |
