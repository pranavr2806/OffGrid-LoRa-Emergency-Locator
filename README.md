# OffGrid LoRa Emergency Locator

OffGrid LoRa Emergency Locator is a dual-node, off-grid telemetry and tracking system designed to locate lost individuals in remote areas (such as dense forests or canyons) without cellular network coverage. 

The system consists of a compact, low-power **Transmitter Beacon** carried by the lost person, and a hand-held **Receiver Locator Console** carried by the rescuer. The receiver hosts a local Wi-Fi Hotspot and Web Server, allowing the rescuer to track distance and signal strength trends in real-time through an interactive, browser-based dashboard on their smartphone.

---

## Key Features

*   **Register-Level STM32 Firmware:** The transmitter uses direct SPI register-level programming of the Semtech SX1278 LoRa transceiver on an STM32F407 platform, bypassing standard heavy libraries.
*   **Low-Power Standby Mode:** Optimizes battery consumption by entering low-power Standby between periodic transmissions.
*   **Offline Web Dashboard:** The ESP32 receiver spins up a Soft Access Point (AP) and Web Server, hosting a graphical console at `http://192.168.4.1` with zero internet dependencies.
*   **Hot-Cold Distance Estimation:** Uses the Log-Distance Path Loss model to estimate distance, mapping signal strength to a dynamic color-coded sonar gauge (Blue = Cold, Yellow = Warm, Red = Hot/Immediate Vicinity).
*   **Real-Time RSSI Filtering:** Employs an Exponential Moving Average (EMA) filter ($\alpha = 0.10$) with step-change bypass detection to filter out multipath indoor/foliage reflections while preventing lag during rapid movements.
*   **Auditory Geiger-Counter Ticks:** Leverages the HTML5 Web Audio API to generate synthesized pitch-shifted beeps in the rescuer's smartphone browser, ticking faster as the rescuer approaches the beacon.
*   **Link Quality Diagnostics:** Monitors sliding-window packet loss over the last 20 expected sequence IDs, displaying real-time link reliability.

---

## Hardware Architecture

### Transmitter (Lost Person's Node)
*   STM32F407VG Discovery Board
*   SX1278 LoRa Module ($433\text{ MHz}$)
*   $433\text{ MHz}$ antenna ($17.3\text{ cm}$ quarter-wave whip)

### Receiver (Rescuer's Node)
*   ESP32 Dev Module
*   SX1278 LoRa Module ($433\text{ MHz}$)
*   $433\text{ MHz}$ antenna ($17.3\text{ cm}$ quarter-wave whip)
*   Smartphone (for visual/audio console interface)

---

## Connection Wiring Diagrams

### 1. STM32F407 $\rightarrow$ SX1278 Connections (SPI1)

| SX1278 Pin | STM32F407 Pin | Description |
| :--- | :---: | :--- |
| **VCC** | 3.3V | Power Input ($3.3\text{ V}$) |
| **GND** | GND | Ground Reference |
| **NSS (CS)** | PA4 | SPI Chip Select (Software Controlled) |
| **SCK** | PA5 | SPI Serial Clock |
| **MISO** | PA6 | SPI Master In Slave Out |
| **MOSI** | PA7 | SPI Master Out Slave In |
| **RESET** | PA3 | Transceiver Reset |
| **DIO0** | PA2 | Interrupt Output (TxDone/RxDone) |
| **PD12** | *Onboard* | Green LED (Blinks during TX time-on-air) |

---

### 2. ESP32 Dev Module $\rightarrow$ SX1278 Connections

| SX1278 Pin | ESP32 Pin | Description |
| :--- | :---: | :--- |
| **VCC** | 3.3V | Power Input ($3.3\text{ V}$) |
| **GND** | GND | Ground Reference |
| **NSS (CS)** | GPIO5 | SPI Chip Select |
| **SCK** | GPIO18 | SPI Serial Clock |
| **MISO** | GPIO19 | SPI Master In Slave Out |
| **MOSI** | GPIO23 | SPI Master Out Slave In |
| **RESET** | GPIO14 | Transceiver Reset |
| **DIO0** | GPIO26 | Interrupt Output |
| **GPIO2** | *Onboard* | Blue LED (Blinks at a rate proportional to distance) |

---

## System Configuration Details

*   **Carrier Frequency:** $433.0\text{ MHz}$
*   **Signal Bandwidth:** $125\text{ kHz}$
*   **Spreading Factor:** SF7
*   **Error Coding Rate:** 4/5
*   **Sync Word:** `0x12` (Private network default)
*   **Preamble Length:** 8 symbols
*   **CRC Checking:** Enabled
*   **Transmit Power:** $+17\text{ dBm}$ (using `PA_BOOST` output pin)

---

## Installation and Setup

### 1. Programming the Transmitter (STM32)
1. Open the project in **STM32CubeIDE**.
2. Connect your STM32F407 Discovery board to your PC via the USB Mini-B debugger port.
3. Build the project. If utilizing an external battery pack, configure the hardware as detailed in the user guide.
4. Flash the code onto the MCU. The green LED on the board (`PD12`) will flash every 2 seconds, indicating active radio transmissions.

### 2. Programming the Receiver (ESP32)
1. Install the Arduino IDE.
2. Go to **Preferences** and paste the ESP32 board manager URL:
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Go to **Tools -> Board -> Boards Manager**, search for `esp32` by Espressif, and click **Install**.
4. Go to **Sketch -> Include Library -> Manage Libraries**, search for `LoRa` by Sandeep Mistry, and click **Install**.
5. Open the [esp32_receiver_locator.ino](esp32_receiver/esp32_receiver_locator.ino) sketch.
6. Connect the ESP32 to your PC, select the `ESP32 Dev Module` board and port, and click **Upload**.

---

## How to Access the Operations Dashboard

1. Power on the ESP32 Receiver Node (using a USB power bank or battery).
2. Open the Wi-Fi settings panel on your smartphone.
3. Connect to the hotspot:
   * **SSID:** `Rescue_Locator_AP`
   * **Password:** `12345678`
4. Open your web browser and navigate to:
   ```text
   http://192.168.4.1
   ```
5. Tap **"Enable Audio Search Beeps"** on the screen. The dashboard is now active and will guide your search direction.

---

## Calibration Guide

To adapt the distance locator to your specific search environment:

1. **Set the 1-Meter Reference (`A`):**
   * Place both nodes exactly 1 meter apart at chest height.
   * Read the **Raw RSSI** value on the Diagnostics tab (e.g., $-45\text{ dBm}$).
   * Set `const double A = -45.0;` in the ESP32 code and re-flash.
2. **Tune the Path Loss Exponent (`n`):**
   * Move to a known distance (e.g., 10 meters).
   * If the dashboard reads *larger* than 10 meters, increase `n` in the code (e.g., to `3.8`).
   * If the dashboard reads *smaller* than 10 meters, decrease `n` in the code (e.g., to `2.2`).
   * In open fields, `n` will stabilize near `2.0`. In moderate woods, it will hover near `3.0` to `3.5`.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
