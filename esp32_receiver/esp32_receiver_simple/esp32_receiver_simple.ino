/**
 * OffGrid Search & Rescue Locator - Receiver Node (Simplified Hot-Cold Version)
 * 
 * This is a fail-safe backup firmware that strips out all directional sweeps, 
 * compass sensors, and rotating needles. It acts as a pure signal strength 
 * Proximity Finder, using a large Color-Changing Hot-Cold Bar, audio beeps, 
 * and simplified OLED text diagnostics.
 * 
 * Hardware:
 * - ESP32 DevKit V1
 * - SX1278 LoRa Module (433 MHz SPI)
 * - SH1106 OLED Display (128x64 I2C, pins 21/22)
 * - Onboard Blue LED (GPIO 2)
 */

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFi.h>
#include <WebServer.h>

#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    64
#define OLED_RESET       -1
#define OLED_I2C_ADDRESS 0x3C
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Screen refresh variables
unsigned long lastDisplayUpdateTime = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 250;

// ESP32 -> SX1278 SPI Connections
#define SCK_PIN    18
#define MISO_PIN   19
#define MOSI_PIN   23
#define SS_PIN     5
#define RST_PIN    14
#define DIO0_PIN   26
#define LED_PIN    2

// Wi-Fi AP Config
const char* ssid = "Rescue_Locator_AP";
const char* password = "12345678";
WebServer server(80);

// Log-Distance Path Loss Model Constants (Dynamic In-Situ Calibration)
double A = -75.0; // Calibrated RSSI at 1 meter (Indoor classroom default)
double n = 2.0;   // Path loss exponent (Indoor line of sight)

// Exponential Moving Average (EMA) Filter Variables
double filtered_rssi = -120.0;

// Tracking statistics
int last_raw_rssi = -120;
double last_snr = 0.0;
uint32_t packet_count = 0;
uint32_t last_packet_time = 0;
bool signal_lost = true;
double current_estimated_distance = 999.0;
double packet_loss_rate = 0.0;
String last_payload = "---";

// Sliding window packet loss variables
const int WINDOW_SIZE = 20;
bool window_packets[WINDOW_SIZE];
int window_idx = 0;
uint32_t last_seq_num = 0;
uint32_t total_expected_packets = 0;

// Onboard LED blink state
unsigned long last_blink_time = 0;
bool led_state = false;

// HTML Dashboard content (Proximity Hot-Cold Bar + Tabs + Diagnostics Chart)
const char HTML_CONTENT[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>LoRa Proximity Finder</title>
  <style>
    :root {
      --bg-main: #FAFCF9;
      --card-bg: #FFFFFF;
      --text-main: #2B2621;
      --text-muted: #857E78;
      --accent-blue: #3e6b89;
      --accent-yellow: #d4a373;
      --accent-green: #2a6f47;
      --accent-red: #d95d5d;
      --shadow-subtle: 0 4px 20px rgba(43, 38, 33, 0.04);
      --border-light: 1px solid rgba(43, 38, 33, 0.06);
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body { background-color: var(--bg-main); color: var(--text-main); padding: 16px; padding-bottom: 90px; display: flex; flex-direction: column; align-items: center; }
    .container { width: 100%; max-width: 440px; display: flex; flex-direction: column; gap: 16px; }
    
    .card {
      background: var(--card-bg);
      border-radius: 20px;
      padding: 24px;
      box-shadow: var(--shadow-subtle);
      border: var(--border-light);
      display: flex;
      flex-direction: column;
      align-items: center;
      text-align: center;
    }
    .card-title { font-size: 14px; font-weight: 700; text-transform: uppercase; letter-spacing: 0.08em; color: var(--text-muted); margin-bottom: 16px; width: 100%; text-align: left; }
    
    /* Proximity Gauge */
    .gauge-container { width: 160px; height: 160px; border-radius: 50%; display: flex; align-items: center; justify-content: center; position: relative; margin: 12px 0; border: 8px solid rgba(0,0,0,0.03); transition: border-color 0.5s ease; }
    .gauge-inner { text-align: center; }
    .proximity-percentage { font-size: 40px; font-weight: 800; line-height: 1; }
    .proximity-lbl { font-size: 11px; font-weight: 700; color: var(--text-muted); text-transform: uppercase; margin-top: 4px; }
    
    .distance-readout { font-size: 32px; font-weight: 800; margin-top: 12px; }
    .distance-unit { font-size: 16px; font-weight: 600; color: var(--text-muted); }
    
    /* Hot-Cold Progress Bar */
    .bar-container { width: 100%; margin-top: 20px; text-align: left; }
    .bar-labels { display: flex; justify-content: space-between; font-size: 11px; font-weight: 600; color: var(--text-muted); margin-bottom: 4px; }
    .bar-bg { background: rgba(0,0,0,0.06); height: 16px; border-radius: 8px; overflow: hidden; width: 100%; border: 1px solid rgba(0,0,0,0.02); }
    .bar-fill { width: 0%; height: 100%; background: var(--accent-blue); transition: width 0.3s ease, background-color 0.5s ease; border-radius: 8px; }
    
    .status-badge { display: inline-block; padding: 6px 14px; border-radius: 20px; font-size: 12px; font-weight: 700; text-transform: uppercase; margin-top: 16px; }
    .status-lost { background: #FFEBEB; color: var(--accent-red); }
    .status-cold { background: #EBF3FF; color: var(--accent-blue); }
    .status-warm { background: #FFF4EB; color: var(--accent-yellow); }
    .status-hot { background: #EBFEEF; color: var(--accent-green); }

    /* Audio Switch */
    .btn { display: block; width: 100%; padding: 14px; border: var(--border-light); background: var(--card-bg); color: var(--text-main); font-size: 14px; font-weight: 700; border-radius: 14px; cursor: pointer; transition: background 0.2s, color 0.2s; text-align: center; }
    .btn-active { background: var(--text-main); color: var(--card-bg); border-color: var(--text-main); }

    /* Tabs Navigation */
    .navbar { position: fixed; bottom: 0; left: 0; right: 0; background: var(--card-bg); border-top: var(--border-light); height: 72px; display: flex; justify-content: space-around; align-items: center; box-shadow: 0 -4px 20px rgba(0,0,0,0.03); z-index: 1000; }
    .tab-btn { background: none; border: none; font-size: 11px; font-weight: 700; color: var(--text-muted); cursor: pointer; display: flex; flex-direction: column; align-items: center; gap: 4px; transition: color 0.2s; text-transform: uppercase; }
    .tab-btn svg { width: 22px; height: 22px; stroke: currentColor; fill: none; stroke-width: 2.5; stroke-linecap: round; stroke-linejoin: round; }
    .tab-btn.active { color: var(--text-main); }
    
    .tab-content { display: none; width: 100%; }
    .tab-content.active { display: flex; flex-direction: column; gap: 16px; }

    /* Diagnostics Table & Chart */
    .stats-table { width: 100%; border-collapse: collapse; margin-top: 8px; }
    .stats-row { display: flex; justify-content: space-between; padding: 12px 0; border-bottom: 1px solid rgba(43, 38, 33, 0.05); font-size: 13px; }
    .stats-row:last-child { border-bottom: none; }
    .stats-label { font-weight: 600; color: var(--text-muted); }
    .stats-value { font-weight: 700; font-variant-numeric: tabular-nums; }
    .chart-container { width: 100%; height: 130px; margin-top: 12px; }
    canvas { width: 100%; height: 100%; border-radius: 12px; background: rgba(0,0,0,0.02); }

    /* Instructions Guide */
    .instruction-step { display: flex; gap: 16px; text-align: left; margin-bottom: 16px; width: 100%; }
    .step-number { background: rgba(43, 38, 33, 0.06); width: 28px; height: 28px; border-radius: 50%; display: flex; align-items: center; justify-content: center; font-size: 14px; font-weight: 700; flex-shrink: 0; }
    .step-text { font-size: 14px; line-height: 1.5; color: #4A4540; }
  </style>
</head>
<body>
  <div class="container">
    
    <!-- TAB 1: FINDER CONTENT -->
    <div id="tab-finder" class="tab-content active">
      <div class="card" style="margin-top: 20px;">
        <div class="card-title">Proximity Locator</div>
        
        <div id="gauge" class="gauge-container">
          <div class="gauge-inner">
            <div id="prox-val" class="proximity-percentage">0%</div>
            <div id="prox-status" class="proximity-lbl">Searching...</div>
          </div>
        </div>
        
        <div class="distance-readout">
          <span id="dist-val">---</span>
          <span id="dist-unit" class="distance-unit">meters</span>
        </div>
        
        <div class="bar-container">
          <div class="bar-labels">
            <span>COLD (Far)</span>
            <span>HOT (Near)</span>
          </div>
          <div class="bar-bg">
            <div id="bar-fill" class="bar-fill"></div>
          </div>
        </div>
        
        <div id="status-badge" class="status-badge status-lost">No Connection</div>

        <!-- In-Situ 1m Calibration & Environment Tuning -->
        <div style="margin-top: 16px; padding: 12px; background: rgba(0,0,0,0.03); border-radius: 14px; width: 100%; border: 1px solid rgba(0,0,0,0.06); text-align: left;">
          <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:8px;">
            <span style="font-size:11px; font-weight:700; color:var(--text-muted); text-transform:uppercase;">1m Calib (A) & Path Loss (n)</span>
            <span id="cal-info" style="font-size:11px; font-weight:700; color:var(--text-main);">-75.0 dBm | n=2.0</span>
          </div>
          <button onclick="calibrate1m()" class="btn" style="padding: 10px; font-size: 13px; background: var(--accent-blue); color: #fff; margin-bottom: 8px;">🎯 Set 1m Baseline (Hold at 1 Meter)</button>
          <div style="display: flex; gap: 6px;">
            <button onclick="setEnv(2.0)" class="btn" style="padding: 6px; font-size: 11px;">Classroom (n=2.0)</button>
            <button onclick="setEnv(2.4)" class="btn" style="padding: 6px; font-size: 11px;">Corridor (n=2.4)</button>
            <button onclick="setEnv(3.0)" class="btn" style="padding: 6px; font-size: 11px;">Outdoor (n=3.0)</button>
          </div>
        </div>
      </div>
      
      <button id="sound-btn" class="btn">Enable Audio Search Beeps</button>
    </div>

    <!-- TAB 2: DIAGNOSTICS CONTENT -->
    <div id="tab-diagnostics" class="tab-content">
      <div class="card" style="margin-top: 20px;">
        <div class="card-title">Signal Analytics</div>
        <div class="chart-container">
          <canvas id="rssiChart"></canvas>
        </div>
      </div>

      <div class="card">
        <div class="card-title">Hardware Telemetry</div>
        <div class="stats-table">
          <div class="stats-row"><span class="stats-label">Raw Signal (RSSI)</span><span id="diag-raw" class="stats-value">---</span></div>
          <div class="stats-row"><span class="stats-label">Smoothed Signal (EMA)</span><span id="diag-filt" class="stats-value">---</span></div>
          <div class="stats-row"><span class="stats-label">Signal-to-Noise (SNR)</span><span id="diag-snr" class="stats-value">---</span></div>
          <div class="stats-row"><span class="stats-label">Packet Link Reliability</span><span id="diag-loss" class="stats-value">---</span></div>
          <div class="stats-row"><span class="stats-label">Beacon Packet ID</span><span id="diag-id" class="stats-value">---</span></div>
        </div>
      </div>
    </div>

    <!-- TAB 3: GUIDE CONTENT -->
    <div id="tab-guide" class="tab-content">
      <div class="card" style="margin-top: 20px;">
        <div class="card-title">Search Guidelines</div>
        
        <div class="instruction-step">
          <div class="step-number">1</div>
          <div class="step-text">Verify that the transmitter beacon is turned on and flashing its green status LED.</div>
        </div>
        <div class="instruction-step">
          <div class="step-number">2</div>
          <div class="step-text">Enable the <strong>Audio Search Beeps</strong>. Sound is highly responsive and lets you search without staring at the screen.</div>
        </div>
        <div class="instruction-step">
          <div class="step-number">3</div>
          <div class="step-text">Walk slowly in any direction. Observe the **Proximity Percentage** and **Distance reading** on the screen.</div>
        </div>
        <div class="instruction-step">
          <div class="step-number">4</div>
          <div class="step-text">If the proximity value drops (gets colder) and the beep interval slows down, you are walking away. Turn around and search in other directions until the signal peaks.</div>
        </div>
      </div>
    </div>

  </div>

  <!-- Tabs Navigation Bar -->
  <div class="navbar">
    <button id="btn-finder" class="tab-btn active" onclick="switchTab('finder')">
      <svg><circle cx="12" cy="12" r="10"></circle><path d="M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10zM2 12h20"></path></svg>
      Finder
    </button>
    <button id="btn-diagnostics" class="tab-btn" onclick="switchTab('diagnostics')">
      <svg><path d="M3 3v18h18"></path><path d="M18.7 8l-5.1 5.2-2.8-2.7L7 14.3"></path></svg>
      Diagnostics
    </button>
    <button id="btn-guide" class="tab-btn" onclick="switchTab('guide')">
      <svg><circle cx="12" cy="12" r="10"></circle><path d="M9.09 9a3 3 0 0 1 5.83 1c0 2-3 3-3 3"></path><line x1="12" y1="17" x2="12.01" y2="17"></line></svg>
      Guide
    </button>
  </div>

  <script>
    let rssiHistory = [];
    const maxHistory = 30;
    let audioCtx = null;
    let audioEnabled = false;
    let nextBeepTime = 0;
    let isLost = true;

    // Line Chart Canvas Configuration
    const canvas = document.getElementById('rssiChart');
    const ctx = canvas.getContext('2d');
    
    function resizeCanvas() {
      const dpr = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
      ctx.scale(dpr, dpr);
    }
    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();

    function switchTab(tabId) {
      document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
      document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));
      
      document.getElementById('tab-' + tabId).classList.add('active');
      document.getElementById('btn-' + tabId).classList.add('active');
      
      if (tabId === 'diagnostics') {
        setTimeout(resizeCanvas, 50);
      }
    }

    function playBeep(freq, duration) {
      if (!audioEnabled || !audioCtx) return;
      try {
        let osc = audioCtx.createOscillator();
        let gain = audioCtx.createGain();
        osc.connect(gain);
        gain.connect(audioCtx.destination);
        osc.frequency.value = freq;
        gain.gain.setValueAtTime(0.12, audioCtx.currentTime);
        gain.gain.exponentialRampToValueAtTime(0.01, audioCtx.currentTime + duration);
        osc.start(audioCtx.currentTime);
        osc.stop(audioCtx.currentTime + duration);
      } catch(e) {}
    }

    document.getElementById('sound-btn').addEventListener('click', function() {
      if (!audioCtx) {
        audioCtx = new (window.AudioContext || window.webkitAudioContext)();
      }
      audioEnabled = !audioEnabled;
      this.textContent = audioEnabled ? "Disable Audio Search Beeps" : "Enable Audio Search Beeps";
      this.classList.toggle('btn-active', audioEnabled);
      if (audioEnabled) {
        if (audioCtx.state === 'suspended') {
          audioCtx.resume();
        }
        // Play a short tone immediately on click to unlock audio for iOS Safari
        playBeep(440, 0.05);
      }
    });

    function drawChart() {
      const w = canvas.width / (window.devicePixelRatio || 1);
      const h = canvas.height / (window.devicePixelRatio || 1);
      ctx.clearRect(0, 0, w, h);

      if (rssiHistory.length < 2) return;

      ctx.strokeStyle = '#3e6b89';
      ctx.lineWidth = 2.5;
      ctx.beginPath();

      for (let i = 0; i < rssiHistory.length; i++) {
        let x = (i / (maxHistory - 1)) * w;
        let val = rssiHistory[i];
        if (val < -120) val = -120;
        if (val > -40) val = -40;
        let y = h - ((val - (-120)) / 80) * h;
        
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    function updateTelemetry() {
      fetch('/data')
        .then(res => res.json())
        .then(data => {
          const proxVal = document.getElementById('prox-val');
          const proxStatus = document.getElementById('prox-status');
          const distVal = document.getElementById('dist-val');
          const distUnit = document.getElementById('dist-unit');
          const barFill = document.getElementById('bar-fill');
          const gauge = document.getElementById('gauge');
          const statusBadge = document.getElementById('status-badge');
          
          const diagRaw = document.getElementById('diag-raw');
          const diagFilt = document.getElementById('diag-filt');
          const diagSnr = document.getElementById('diag-snr');
          const diagLoss = document.getElementById('diag-loss');
          const diagId = document.getElementById('diag-id');
          
          isLost = data.signal_lost;
          
          if (data.signal_lost) {
            proxVal.textContent = "0%";
            proxStatus.textContent = "Offline";
            distVal.textContent = "LOST";
            distVal.style.color = "var(--accent-red)";
            distUnit.textContent = "";
            barFill.style.width = "0%";
            barFill.style.background = "var(--accent-red)";
            gauge.style.borderColor = "var(--accent-red)";
            statusBadge.className = "status-badge status-lost";
            statusBadge.textContent = "Signal Lost";

            diagRaw.textContent = "---";
            diagFilt.textContent = "---";
            diagSnr.textContent = "---";
            diagLoss.textContent = "---";
            diagId.textContent = "---";
          } else {
            distVal.style.color = "var(--text-main)";
            
            // Format distance readout
            if (data.distance < 1.0) {
              distVal.textContent = Math.round(data.distance * 100);
              distUnit.textContent = "cm";
            } else {
              distVal.textContent = data.distance.toFixed(1);
              distUnit.textContent = "meters";
            }
            
            // Calculate proximity percentage (RSSI -105 to -50)
            let proximity = 0;
            if (data.filtered_rssi > -105) {
              proximity = Math.round(((data.filtered_rssi - (-105)) / (-50 - (-105))) * 100);
              if (proximity > 100) proximity = 100;
              if (proximity < 0) proximity = 0;
            }
            proxVal.textContent = proximity + "%";
            barFill.style.width = proximity + "%";
            
            // Determine Hot/Cold state colors
            if (proximity >= 70) { // HOT (Green)
              proxStatus.textContent = "HOT (Very Close)";
              barFill.style.background = "var(--accent-green)";
              gauge.style.borderColor = "var(--accent-green)";
              statusBadge.className = "status-badge status-hot";
              statusBadge.textContent = "Immediate Vicinity";
            } else if (proximity >= 35) { // WARM (Yellow/Gold)
              proxStatus.textContent = "WARM (Closing In)";
              barFill.style.background = "var(--accent-yellow)";
              gauge.style.borderColor = "var(--accent-yellow)";
              statusBadge.className = "status-badge status-warm";
              statusBadge.textContent = "Signal Strong";
            } else { // COLD (Blue)
              proxStatus.textContent = "COLD (Far)";
              barFill.style.background = "var(--accent-blue)";
              gauge.style.borderColor = "var(--accent-blue)";
              statusBadge.className = "status-badge status-cold";
              statusBadge.textContent = "Signal Detected";
            }

            // Diagnostics update
            diagRaw.textContent = data.raw_rssi + " dBm";
            diagFilt.textContent = data.filtered_rssi.toFixed(1) + " dBm";
            diagSnr.textContent = data.snr.toFixed(1) + " dB";
            
            let reliability = (100.0 - data.packet_loss_rate);
            diagLoss.textContent = reliability.toFixed(1) + "%";
            diagLoss.style.color = reliability > 90 ? "var(--accent-green)" : (reliability > 70 ? "var(--accent-yellow)" : "var(--accent-red)");
            diagId.textContent = data.last_payload;

            rssiHistory.push(data.raw_rssi);
            if (rssiHistory.length > maxHistory) {
              rssiHistory.shift();
            }
            
            if (document.getElementById('tab-diagnostics').classList.contains('active')) {
              drawChart();
            }

            if (data.cal_a !== undefined && data.cal_n !== undefined) {
              const calEl = document.getElementById('cal-info');
              if (calEl) calEl.textContent = data.cal_a.toFixed(1) + " dBm | n=" + data.cal_n.toFixed(1);
            }

            // Audio Geiger tone scheduling (Synchronized perfectly with distance changes)
            let now = Date.now();
            if (now >= nextBeepTime) {
              let interval = 2000;
              let pitch = 440;
              if (data.distance <= 1.5) {
                interval = 120;
                pitch = 880;
              } else if (data.distance < 40.0) {
                interval = 120 + (data.distance - 1.5) * 48.8;
                pitch = 880 - (data.distance - 1.5) * 11.4;
              }
              playBeep(pitch, 0.08);
              nextBeepTime = now + interval;
            }
          }
        });
    }

    function calibrate1m() {
      fetch('/calibrate')
        .then(res => res.json())
        .then(d => {
          if (d.status === "ok") {
            alert('Calibrated! 1m Reference A = ' + d.A + ' dBm. Distance set to 1.0m.');
          }
        })
        .catch(e => alert('Calibration failed: ' + e));
    }

    function setEnv(val) {
      fetch('/calibrate?n=' + val)
        .then(res => res.json())
        .then(d => {
          const calEl = document.getElementById('cal-info');
          if (calEl) calEl.textContent = d.A.toFixed(1) + " dBm | n=" + d.n.toFixed(1);
        });
    }

    setInterval(updateTelemetry, 200); // 5 updates/second for zero lag
  </script>
</body>
</html>
)rawliteral";

// Web server routing handlers
void handleRoot() {
  server.send(200, "text/html", HTML_CONTENT);
}

void handleData() {
  String json = "{";
  json += "\"raw_rssi\":" + String(last_raw_rssi) + ",";
  json += "\"filtered_rssi\":" + String(filtered_rssi) + ",";
  json += "\"snr\":" + String(last_snr) + ",";
  json += "\"distance\":" + String(current_estimated_distance) + ",";
  json += "\"cal_a\":" + String(A, 1) + ",";
  json += "\"cal_n\":" + String(n, 2) + ",";
  json += "\"packet_count\":" + String(packet_count) + ",";
  json += "\"packet_loss_rate\":" + String(packet_loss_rate) + ",";
  json += "\"signal_lost\":" + String(signal_lost ? "true" : "false") + ",";
  json += "\"last_payload\":\"" + last_payload + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleCalibrate() {
  if (server.hasArg("a")) {
    A = server.arg("a").toDouble();
  } else if (!signal_lost && filtered_rssi > -115.0) {
    A = filtered_rssi; // 1-meter instant calibration from current RSSI
  }
  if (server.hasArg("n")) {
    n = server.arg("n").toDouble();
  }
  
  if (!signal_lost && filtered_rssi > -115.0) {
    double exponent = (A - filtered_rssi) / (10.0 * n);
    current_estimated_distance = pow(10.0, exponent);
  }
  drawOledDisplay(true);

  String res = "{\"status\":\"ok\",\"A\":" + String(A, 1) + ",\"n\":" + String(n, 2) + "}";
  server.send(200, "application/json", res);
}

void drawOledDisplay(bool force = false) {
  unsigned long now = millis();
  if (!force && (now - lastDisplayUpdateTime < DISPLAY_UPDATE_INTERVAL_MS)) {
    return;
  }
  lastDisplayUpdateTime = now;

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  // Draw Header Bar
  display.setCursor(0, 0);
  display.println("--- LORA LOCATOR ---");
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SH110X_WHITE);

  // Draw Proximity & Status
  display.setCursor(0, 16);
  if (signal_lost) {
    display.setTextSize(2);
    display.println("SIGNAL LOST");
    display.setTextSize(1);
    display.print("Waiting for Beacon...");
  } else {
    display.print("DIST: ");
    display.setTextSize(2);
    if (current_estimated_distance < 1.0) {
      display.print((int)(current_estimated_distance * 100));
      display.setTextSize(1);
      display.println(" cm");
    } else {
      display.print(current_estimated_distance, 1);
      display.setTextSize(1);
      display.println(" m");
    }
    
    // Convert filtered RSSI to proximity percent
    int proximity = 0;
    if (filtered_rssi > -105) {
      proximity = (int)(((filtered_rssi - (-105)) / (-50 - (-105))) * 100);
      if (proximity > 100) proximity = 100;
      if (proximity < 0) proximity = 0;
    }

    display.setTextSize(1);
    display.setCursor(0, 36);
    display.print("PROX: ");
    display.print(proximity);
    display.println(" %");

    display.setCursor(0, 46);
    display.print("RSSI: ");
    display.print(last_raw_rssi);
    display.println(" dBm");

    display.setCursor(0, 56);
    display.print("STATUS: ");
    if (proximity >= 70) {
      display.println("HOT");
    } else if (proximity >= 35) {
      display.println("WARM");
    } else {
      display.println("COLD");
    }
  }

  display.display();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("--- Starting Simplified Proximity Receiver ---");

  // Initialize I2C OLED SSD1306/SH1106
  Wire.begin(21, 22);
  if(!display.begin(OLED_I2C_ADDRESS, true)) {
    Serial.println("[ERROR] SH1106 OLED initialization failed!");
  } else {
    display.clearDisplay();
    display.display();
  }

  // Setup SX1278 SPI Connections
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  if (!LoRa.begin(433E6)) {
    Serial.println("[ERROR] SX1278 LoRa initialization failed!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }

  // Set LoRa parameters
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();

  Serial.println("[INFO] LoRa Transceiver initialized successfully!");

  // Start Wi-Fi Access Point
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("[INFO] Hotspot IP Address: ");
  Serial.println(IP);

  // Bind Web Server handlers
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/calibrate", handleCalibrate);
  server.begin();
  Serial.println("[INFO] Web server started.");

  // Clear packet loss window
  for (int i = 0; i < WINDOW_SIZE; i++) {
    window_packets[i] = true;
  }

  drawOledDisplay(true);
}

void loop() {
  server.handleClient();

  // Handle on-board LED blinking
  unsigned long current_time = millis();
  unsigned long blink_interval = 2000; // Standby blink when signal is lost

  if (!signal_lost) {
    if (current_estimated_distance > 0.0 && current_estimated_distance <= 1.5) {
      blink_interval = 50;
    } else if (current_estimated_distance > 1.5 && current_estimated_distance <= 40.0) {
      blink_interval = 150 + (unsigned long)((current_estimated_distance - 1.5) * (1350.0 / 38.5));
    }
  }

  if (current_time - last_blink_time >= blink_interval) {
    last_blink_time = current_time;
    led_state = !led_state;
    digitalWrite(LED_PIN, led_state ? HIGH : LOW);
  }

  // Handle incoming LoRa packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String payload = "";
    while (LoRa.available()) {
      payload += (char)LoRa.read();
    }
    last_payload = payload;
    packet_count++;
    last_packet_time = millis();
    signal_lost = false;

    // Parse sequence number
    int underscore_idx = payload.indexOf('_');
    if (underscore_idx != -1) {
      String seq_str = payload.substring(underscore_idx + 1);
      uint32_t current_seq = seq_str.toInt();

      if (total_expected_packets == 0 || current_seq < last_seq_num) {
        // First packet or transmitter rebooted
        last_seq_num = current_seq;
        total_expected_packets = 1;
        window_idx = 0;
        for (int i = 0; i < WINDOW_SIZE; i++) window_packets[i] = true;
      } else {
        uint32_t gap = current_seq - last_seq_num - 1;
        if (gap > 0) {
          // Log missed packets in sliding window
          for (uint32_t i = 0; i < gap; i++) {
            window_packets[window_idx] = false;
            window_idx = (window_idx + 1) % WINDOW_SIZE;
          }
          total_expected_packets += gap;
        }
        window_packets[window_idx] = true;
        window_idx = (window_idx + 1) % WINDOW_SIZE;
        total_expected_packets++;
        last_seq_num = current_seq;
      }

      int lost_in_window = 0;
      for (int i = 0; i < WINDOW_SIZE; i++) {
        if (!window_packets[i]) lost_in_window++;
      }
      packet_loss_rate = ((double)lost_in_window / (double)WINDOW_SIZE) * 100.0;
    }

    last_raw_rssi = LoRa.packetRssi();
    last_snr = LoRa.packetSnr();

    // Fast Responsive Adaptive EMA Filter (Eliminates Lag)
    if (filtered_rssi == -120.0) {
      filtered_rssi = last_raw_rssi;
    } else {
      double diff = abs(last_raw_rssi - filtered_rssi);
      double dyn_alpha;
      if (diff > 7.0) {
        dyn_alpha = 0.80; // Sudden step change / rapid movement (instant snap)
      } else if (diff > 2.5) {
        dyn_alpha = 0.45; // Walking speed (updates within 0.5s)
      } else {
        dyn_alpha = 0.20; // Stationary noise rejection (rock-solid stability)
      }
      filtered_rssi = (dyn_alpha * last_raw_rssi) + ((1.0 - dyn_alpha) * filtered_rssi);
    }

    // Distance Calculation (Standard model)
    double exponent = (A - filtered_rssi) / (10.0 * n);
    current_estimated_distance = pow(10.0, exponent);

    drawOledDisplay(true); // Force redraw
  }

  // Handle signal loss timeout (5 seconds)
  if (!signal_lost && (millis() - last_packet_time > 5000)) {
    signal_lost = true;
    current_estimated_distance = 999.0;
    filtered_rssi = -120.0;
    drawOledDisplay(true); // Force redraw
  }
}
