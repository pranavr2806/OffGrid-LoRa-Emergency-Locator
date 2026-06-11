/*
 * ESP32 LoRa Search and Rescue Web Server & Locator Dashboard
 * Uses the sandeepmistry/arduino-LoRa library.
 *
 * Configured to:
 * 1. Initialize LoRa at 433.0 MHz matching the STM32 configuration.
 * 2. Spin up a Wi-Fi Access Point named "Rescue_Locator_AP" (password: 12345678).
 * 3. Host a local Web Server at http://192.168.4.1 serving a responsive search dashboard.
 * 4. Apply EMA filtering and Log-Distance Path Loss model for distance calculations.
 * 5. Run a non-blocking indicator LED on GPIO 2.
 * 6. Track packet loss rate and signal direction trend.
 */

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WebServer.h>

// ESP32 -> SX1278 SPI Connections
#define SCK_PIN    18
#define MISO_PIN   19
#define MOSI_PIN   23
#define SS_PIN     5
#define RST_PIN    14
#define DIO0_PIN   26

// Onboard LED Pin
#define LED_PIN    2

// Wi-Fi AP Config
const char* ssid = "Rescue_Locator_AP";
const char* password = "12345678";

WebServer server(80);

// Log-Distance Path Loss Model Constants
const double A = -45.0; // Calibrated RSSI at 1 meter (held at chest height)
const double n = 3.5;   // Path loss exponent for woods/forest

// Exponential Moving Average (EMA) Filter Variables
double filtered_rssi = -120.0;
const double alpha = 0.10;

// Tracking statistics
int last_raw_rssi = -120;
double last_snr = 0.0;
uint32_t packet_count = 0;
String last_payload = "None";
double current_estimated_distance = 999.0;
unsigned long last_packet_time = 0;
bool signal_lost = true;

// Sliding window size for real-time packet loss calculation
#define WINDOW_SIZE 20
bool window_packets[WINDOW_SIZE]; // true = received, false = lost
uint32_t last_seq_num = 0;
uint32_t total_expected_packets = 0;
double packet_loss_rate = 0.0;

void record_packet(bool success) {
  for (int i = 0; i < WINDOW_SIZE - 1; i++) {
    window_packets[i] = window_packets[i + 1];
  }
  window_packets[WINDOW_SIZE - 1] = success;
}

// LED blink timing
unsigned long last_blink_time = 0;
bool led_state = false;

// HTML/JS/CSS Dashboard Page served from Flash memory
const char HTML_CONTENT[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>LoRa Rescue Radar Dashboard</title>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;800&display=swap');
    
    :root {
      --bg-dark: #080c10;
      --bg-card: rgba(16, 22, 30, 0.85);
      --border-color: #212c3d;
      --accent-blue: #0088ff;
      --accent-green: #00e676;
      --accent-red: #ff3d00;
      --accent-yellow: #ffeb3b;
      --text-main: #f0f6fc;
      --text-muted: #8b949e;
    }
    
    body {
      background-color: var(--bg-dark);
      color: var(--text-main);
      font-family: 'Outfit', sans-serif;
      margin: 0;
      padding: 0;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
      overflow-x: hidden;
    }
    
    .container {
      width: 92%;
      max-width: 480px;
      margin: 15px auto;
      text-align: center;
    }
    
    h1 {
      font-size: 26px;
      font-weight: 800;
      margin: 5px 0 2px 0;
      color: var(--text-main);
      letter-spacing: 0.5px;
      background: linear-gradient(90deg, #58a6ff, #0088ff);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
    }
    
    .subtitle {
      font-size: 12px;
      color: var(--text-muted);
      margin-bottom: 15px;
      text-transform: uppercase;
      letter-spacing: 2px;
    }
    
    /* Tab Control styling */
    .tabs-nav {
      display: flex;
      background: #121820;
      border: 1px solid var(--border-color);
      border-radius: 12px;
      margin-bottom: 16px;
      overflow: hidden;
      padding: 3px;
    }
    
    .tab-btn {
      flex: 1;
      background: none;
      border: none;
      color: var(--text-muted);
      padding: 10px;
      font-size: 13px;
      font-weight: 600;
      cursor: pointer;
      border-radius: 9px;
      transition: all 0.2s ease;
    }
    
    .tab-btn.active {
      background: var(--accent-blue);
      color: white;
      box-shadow: 0 4px 12px rgba(0, 136, 255, 0.25);
    }
    
    .tab-content {
      display: none;
      width: 100%;
    }
    
    .tab-content.active {
      display: block;
    }
    
    /* Card Styles */
    .card {
      background: var(--bg-card);
      border: 1px solid var(--border-color);
      border-radius: 20px;
      padding: 20px;
      margin-bottom: 16px;
      box-shadow: 0 12px 36px rgba(0, 0, 0, 0.6);
      backdrop-filter: blur(12px);
      text-align: left;
    }
    
    .card-title {
      font-size: 14px;
      font-weight: 600;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-bottom: 15px;
      border-bottom: 1px dashed var(--border-color);
      padding-bottom: 6px;
    }
    
    /* Sonar/Radar Animation screen */
    .sonar-panel {
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 10px 0;
    }
    
    .radar-wrapper {
      position: relative;
      width: 210px;
      height: 210px;
      margin-bottom: 15px;
    }
    
    .radar-grid {
      position: absolute;
      width: 100%;
      height: 100%;
      border-radius: 50%;
      border: 2px solid rgba(0, 136, 255, 0.15);
      background: radial-gradient(circle, rgba(16, 25, 41, 0.7) 0%, rgba(8, 12, 16, 0.95) 100%);
      box-sizing: border-box;
    }
    
    .radar-ring-2 {
      position: absolute;
      top: 25px; left: 25px; right: 25px; bottom: 25px;
      border-radius: 50%;
      border: 1.5px dashed rgba(0, 136, 255, 0.15);
    }
    
    .radar-ring-3 {
      position: absolute;
      top: 55px; left: 55px; right: 55px; bottom: 55px;
      border-radius: 50%;
      border: 1px solid rgba(0, 136, 255, 0.12);
    }
    
    .radar-cross-h {
      position: absolute;
      top: 50%; left: 0; right: 0;
      height: 1px;
      background: rgba(0, 136, 255, 0.12);
    }
    
    .radar-cross-v {
      position: absolute;
      left: 50%; top: 0; bottom: 0;
      width: 1px;
      background: rgba(0, 136, 255, 0.12);
    }
    
    /* Sonar sweeping line */
    .radar-sweep {
      position: absolute;
      width: 50%;
      height: 50%;
      top: 0; left: 50%;
      background: linear-gradient(90deg, rgba(0, 136, 255, 0.4) 0%, rgba(0, 136, 255, 0) 100%);
      transform-origin: bottom left;
      border-radius: 0 100% 0 0;
      animation: sweep 4s linear infinite;
    }
    
    @keyframes sweep {
      from { transform: rotate(0deg); }
      to { transform: rotate(360deg); }
    }
    
    /* Pulse ring emanating from center */
    .radar-pulse {
      position: absolute;
      top: 50%; left: 50%;
      transform: translate(-50%, -50%);
      border-radius: 50%;
      border: 2px solid var(--accent-blue);
      animation: pulse 2.5s cubic-bezier(0.1, 0.8, 0.3, 1) infinite;
      box-sizing: border-box;
      pointer-events: none;
    }
    
    @keyframes pulse {
      0% { width: 0px; height: 0px; opacity: 1; border-color: var(--accent-blue); }
      75% { border-color: var(--accent-green); }
      100% { width: 210px; height: 210px; opacity: 0; }
    }
    
    /* Distance Reading display overlay */
    .radar-display {
      position: absolute;
      top: 0; left: 0; width: 100%; height: 100%;
      display: flex;
      flex-direction: column;
      justify-content: center;
      align-items: center;
      z-index: 5;
    }
    
    .radar-val {
      font-size: 46px;
      font-weight: 800;
      line-height: 1;
      margin-bottom: 2px;
    }
    
    .radar-unit {
      font-size: 14px;
      color: var(--text-muted);
      font-weight: 600;
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }
    
    /* Trend Panel styling */
    .trend-container {
      width: 100%;
      background: #111823;
      border: 1px solid var(--border-color);
      border-radius: 14px;
      padding: 12px;
      box-sizing: border-box;
      text-align: center;
    }
    
    .trend-label {
      font-size: 11px;
      color: var(--text-muted);
      text-transform: uppercase;
      margin-bottom: 4px;
      font-weight: 600;
      letter-spacing: 0.5px;
    }
    
    .trend-indicator {
      font-size: 20px;
      font-weight: 800;
    }
    
    .trend-closer { color: var(--accent-green); }
    .trend-away { color: var(--accent-red); }
    .trend-neutral { color: var(--accent-yellow); }
    
    /* Stat boxes */
    .stats-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 12px;
    }
    
    .stat-box {
      background: #101622;
      border: 1px solid var(--border-color);
      border-radius: 14px;
      padding: 14px;
    }
    
    .stat-lbl {
      font-size: 10px;
      color: var(--text-muted);
      text-transform: uppercase;
      font-weight: bold;
      margin-bottom: 4px;
      letter-spacing: 0.5px;
    }
    
    .stat-val {
      font-size: 20px;
      font-weight: 800;
    }
    
    .chart-container {
      width: 100%;
      height: 120px;
    }
    
    .btn {
      background-color: var(--accent-blue);
      color: white;
      border: none;
      padding: 15px 20px;
      border-radius: 12px;
      font-size: 14px;
      cursor: pointer;
      font-weight: 800;
      transition: all 0.2s ease;
      width: 100%;
      text-transform: uppercase;
      letter-spacing: 1px;
      box-shadow: 0 4px 15px rgba(0, 136, 255, 0.35);
    }
    
    .btn:active {
      transform: scale(0.98);
    }
    
    .btn-active {
      background-color: var(--accent-red);
      box-shadow: 0 4px 15px rgba(255, 61, 0, 0.35);
    }
    
    /* Instruction list style */
    .guide-step {
      display: flex;
      align-items: flex-start;
      margin-bottom: 15px;
    }
    
    .step-num {
      width: 24px;
      height: 24px;
      border-radius: 50%;
      background: var(--accent-blue);
      color: white;
      display: flex;
      justify-content: center;
      align-items: center;
      font-size: 12px;
      font-weight: bold;
      margin-right: 12px;
      flex-shrink: 0;
    }
    
    .step-text {
      font-size: 14px;
      line-height: 1.4;
    }
    
    .status-badge {
      display: inline-block;
      padding: 4px 10px;
      border-radius: 20px;
      font-size: 11px;
      font-weight: bold;
      text-transform: uppercase;
      letter-spacing: 0.5px;
      margin-top: 10px;
    }
    
    .status-tracking {
      background: rgba(0, 230, 118, 0.15);
      color: var(--accent-green);
      border: 1px solid rgba(0, 230, 118, 0.3);
    }
    
    .status-lost {
      background: rgba(255, 61, 0, 0.15);
      color: var(--accent-red);
      border: 1px solid rgba(255, 61, 0, 0.3);
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>LoRa Rescue Locator</h1>
    <div class="subtitle">Search Operations Console</div>
    
    <!-- Tab Controls -->
    <div class="tabs-nav">
      <button onclick="switchTab('tracker')" id="btn-tracker" class="tab-btn active">TRACKER</button>
      <button onclick="switchTab('diagnostics')" id="btn-diagnostics" class="tab-btn">DIAGNOSTICS</button>
      <button onclick="switchTab('guide')" id="btn-guide" class="tab-btn">RESCUE GUIDE</button>
    </div>
    
    <!-- TAB 1: TRACKER -->
    <div id="tab-tracker" class="tab-content active">
      <div class="card" style="text-align: center;">
        <div class="sonar-panel">
          <div class="radar-wrapper">
            <div class="radar-grid"></div>
            <div class="radar-sweep"></div>
            <div class="radar-ring-2"></div>
            <div class="radar-ring-3"></div>
            <div class="radar-cross-h"></div>
            <div class="radar-cross-v"></div>
            <div id="radar-pulse" class="radar-pulse"></div>
            <div class="radar-display">
              <div id="dist-val" class="radar-val">---</div>
              <div id="dist-unit" class="radar-unit"></div>
            </div>
          </div>
          
          <div id="status-badge" class="status-badge status-lost">No Connection</div>
        </div>
      </div>
      
      <div class="card">
        <div class="card-title">Tracking Assistant</div>
        <div class="trend-container">
          <div class="trend-label">Relative Target Direction</div>
          <div id="trend-indicator" class="trend-indicator trend-neutral">Awaiting Signal</div>
        </div>
      </div>
      
      <div class="card">
        <button id="sound-btn" class="btn">Enable Audio Search Beeps</button>
      </div>
    </div>
    
    <!-- TAB 2: DIAGNOSTICS -->
    <div id="tab-diagnostics" class="tab-content">
      <div class="card">
        <div class="card-title">Live Signal Analysis</div>
        <div class="stats-grid">
          <div class="stat-box">
            <div class="stat-lbl">Raw RSSI</div>
            <div id="diag-raw" class="stat-val">---</div>
          </div>
          <div class="stat-box">
            <div class="stat-lbl">Filtered RSSI</div>
            <div id="diag-filt" class="stat-val">---</div>
          </div>
          <div class="stat-box">
            <div class="stat-lbl">Signal-to-Noise Ratio</div>
            <div id="diag-snr" class="stat-val">---</div>
          </div>
          <div class="stat-box">
            <div class="stat-lbl">Link Reliability</div>
            <div id="diag-loss" class="stat-val">---</div>
          </div>
        </div>
      </div>
      
      <div class="card">
        <div class="card-title">Signal History Graph</div>
        <canvas id="chart" class="chart-container"></canvas>
      </div>
      
      <div class="card">
        <div class="card-title">Beacon Telemetry Packet</div>
        <div class="stats-grid" style="grid-template-columns: 1fr;">
          <div class="stat-box" style="display: flex; justify-content: space-between; align-items: center; padding: 10px 14px;">
            <div class="stat-lbl" style="margin:0;">Received Beacon ID</div>
            <div id="diag-id" style="font-weight: 800; font-size:16px;">---</div>
          </div>
          <div class="stat-box" style="display: flex; justify-content: space-between; align-items: center; padding: 10px 14px;">
            <div class="stat-lbl" style="margin:0;">Time Since Last Update</div>
            <div id="diag-time" style="font-weight: 800; font-size:16px;">---</div>
          </div>
        </div>
      </div>
    </div>
    
    <!-- TAB 3: RESCUE GUIDE -->
    <div id="tab-guide" class="tab-content">
      <div class="card">
        <div class="card-title">Direction Hunting Protocol</div>
        
        <div class="guide-step">
          <div class="step-num">1</div>
          <div class="step-text">
            <strong>Calibrate Position:</strong> Lift the locator to chest height, keeping the antenna straight up. Connect your smartphone and ensure sound is enabled.
          </div>
        </div>
        
        <div class="guide-step">
          <div class="step-num">2</div>
          <div class="step-text">
            <strong>Directional Sweep:</strong> Walk slowly in a 5-meter circle. Watch the "Relative Target Direction" panel. Note which direction causes the RSSI trend to report <strong>"APPROACHING"</strong>.
          </div>
        </div>
        
        <div class="guide-step">
          <div class="step-num">3</div>
          <div class="step-text">
            <strong>Follow the Peak:</strong> Proceed in the direction of the highest signal strength. The audio beeps will speed up and get higher in pitch as you get closer.
          </div>
        </div>
        
        <div class="guide-step">
          <div class="step-num">4</div>
          <div class="step-text">
            <strong>Pinpoint the Target:</strong> When you are within 1.5 meters, the radar will turn red and report <strong>"IMMEDIATE VICINITY"</strong>. The indicator LED will transition to a constant rapid flash.
          </div>
        </div>
      </div>
    </div>
    
  </div>

  <script>
    let rssiHistory = [];
    const maxHistory = 30;
    let audioCtx = null;
    let audioEnabled = false;
    let nextBeepTime = 0;
    let currentDistance = 999.0;
    let isLost = true;
    let lastReceivedTime = 0;
    
    // Arrays for rolling slope trend analysis
    let rssiHistoryTrend = [];
    
    function switchTab(tabId) {
      document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
      document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));
      
      document.getElementById('tab-' + tabId).classList.add('active');
      document.getElementById('btn-' + tabId).classList.add('active');
      
      if (tabId === 'diagnostics') {
        setTimeout(drawChart, 100);
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
        gain.gain.setValueAtTime(0.15, audioCtx.currentTime);
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
      if (audioEnabled && audioCtx.state === 'suspended') {
        audioCtx.resume();
      }
    });

    // Sound tick generator based on distance
    setInterval(function() {
      if (!audioEnabled || isLost) return;
      let now = Date.now();
      if (now >= nextBeepTime) {
        let interval = 2000;
        if (currentDistance <= 1.5) {
          interval = 120;
        } else if (currentDistance >= 40.0) {
          interval = 2000;
        } else {
          interval = 120 + (currentDistance - 1.5) * (1880.0 / 38.5);
        }
        
        let pitch = 440;
        if (currentDistance <= 1.5) {
          pitch = 880;
        } else if (currentDistance >= 40.0) {
          pitch = 440;
        } else {
          pitch = 880 - (currentDistance - 1.5) * (440.0 / 38.5);
        }

        playBeep(pitch, 0.06);
        nextBeepTime = now + interval;
      }
    }, 40);

    function drawChart() {
      const canvas = document.getElementById('chart');
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      canvas.width = w;
      canvas.height = h;

      ctx.clearRect(0, 0, w, h);
      
      ctx.strokeStyle = '#212c3d';
      ctx.lineWidth = 1;
      for (let y = 0; y <= h; y += h/4) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
      }

      if (rssiHistory.length < 2) return;

      // Draw background gradient area
      let grad = ctx.createLinearGradient(0, 0, 0, h);
      grad.addColorStop(0, 'rgba(0, 136, 255, 0.25)');
      grad.addColorStop(1, 'rgba(0, 136, 255, 0.0)');
      
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.moveTo(0, h);
      for (let i = 0; i < rssiHistory.length; i++) {
        let x = (i / (maxHistory - 1)) * w;
        let val = rssiHistory[i];
        if (val < -120) val = -120;
        if (val > -40) val = -40;
        let y = h - ((val - (-120)) / 80) * h;
        ctx.lineTo(x, y);
      }
      ctx.lineTo(w, h);
      ctx.fill();

      // Draw line
      ctx.strokeStyle = '#0088ff';
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

    // Analyzes the slope of the last 4 RSSI packets to determine tracking direction
    function analyzeSignalTrend(newRssi) {
      rssiHistoryTrend.push(newRssi);
      if (rssiHistoryTrend.length > 5) {
        rssiHistoryTrend.shift();
      }
      
      if (rssiHistoryTrend.length < 3) {
        return "Awaiting Trend...";
      }
      
      // Calculate simple average slope
      let sumSlope = 0;
      for (let i = 1; i < rssiHistoryTrend.length; i++) {
        sumSlope += (rssiHistoryTrend[i] - rssiHistoryTrend[i-1]);
      }
      let avgSlope = sumSlope / (rssiHistoryTrend.length - 1);
      
      const indicator = document.getElementById('trend-indicator');
      indicator.className = "trend-indicator";
      
      if (avgSlope > 1.2) {
        indicator.classList.add('trend-closer');
        return "APPROACHING (HOTTER)";
      } else if (avgSlope < -1.2) {
        indicator.classList.add('trend-away');
        return "RECEDING (COLDER)";
      } else {
        indicator.classList.add('trend-neutral');
        return "STATIONARY (STABLE)";
      }
    }

    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          isLost = data.signal_lost;
          currentDistance = data.distance;

          const distVal = document.getElementById('dist-val');
          const distUnit = document.getElementById('dist-unit');
          const signalState = document.getElementById('signal-state');
          const statusBadge = document.getElementById('status-badge');
          
          const diagRaw = document.getElementById('diag-raw');
          const diagFilt = document.getElementById('diag-filt');
          const diagSnr = document.getElementById('diag-snr');
          const diagLoss = document.getElementById('diag-loss');
          const diagId = document.getElementById('diag-id');
          const diagTime = document.getElementById('diag-time');
          
          const radarPulse = document.getElementById('radar-pulse');
          const trendIndicator = document.getElementById('trend-indicator');

          if (isLost) {
            distVal.textContent = "LOST";
            distVal.style.color = "var(--accent-red)";
            distUnit.textContent = "";
            signalState.textContent = "OUT OF RANGE";
            signalState.style.color = "var(--text-muted)";
            statusBadge.className = "status-badge status-lost";
            statusBadge.textContent = "Beacon Lost";
            radarPulse.style.animation = "none";
            
            trendIndicator.className = "trend-indicator trend-neutral";
            trendIndicator.textContent = "Signal Lost";
            
            diagRaw.textContent = "---";
            diagFilt.textContent = "---";
            diagSnr.textContent = "---";
            diagLoss.textContent = "---";
            diagId.textContent = "---";
            diagTime.textContent = "---";
          } else {
            lastReceivedTime = Date.now();
            statusBadge.className = "status-badge status-tracking";
            statusBadge.textContent = "Tracking Active";
            distVal.style.color = "var(--text-main)";
            
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

            // Trend analysis
            trendIndicator.textContent = analyzeSignalTrend(data.filtered_rssi);

            // Display distance with unit scaling
            if (data.distance < 1.0) {
              distVal.textContent = Math.round(data.distance * 100);
              distUnit.textContent = "cm";
            } else {
              distVal.textContent = data.distance.toFixed(1);
              distUnit.textContent = "meters";
            }

            // Radar speed pulse adjusting based on distance
            let pulseDuration = 3.0; // Slow pulse for long range
            if (data.distance <= 1.5) {
              pulseDuration = 0.5; // Fast pulse close range
            } else if (data.distance < 40.0) {
              pulseDuration = 0.5 + (data.distance - 1.5) * (2.5 / 38.5);
            }
            radarPulse.style.animation = `pulse ${pulseDuration}s cubic-bezier(0.1, 0.8, 0.3, 1) infinite`;

            // Signal classification labels
            if (data.distance <= 1.5) {
              signalState.textContent = "VICINITY";
              signalState.style.color = "var(--accent-red)";
            } else if (data.distance <= 8.0) {
              signalState.textContent = "HOT";
              signalState.style.color = "var(--accent-red)";
            } else if (data.distance <= 20.0) {
              signalState.textContent = "WARM";
              signalState.style.color = "var(--accent-yellow)";
            } else {
              signalState.textContent = "COLD";
              signalState.style.color = "var(--accent-blue)";
            }
          }
          
          // Draw diagnostics chart if tab is visible
          if (document.getElementById('tab-diagnostics').classList.contains('active')) {
            drawChart();
          }
        })
        .catch(err => {
          document.getElementById('signal-state').textContent = "Server Error";
          document.getElementById('signal-state').style.color = "var(--accent-red)";
        });
    }

    // Timer to calculate elapsed time since last packet receipt
    setInterval(function() {
      if (isLost || lastReceivedTime === 0) {
        document.getElementById('diag-time').textContent = "---";
        return;
      }
      let elapsed = ((Date.now() - lastReceivedTime) / 1000).toFixed(1);
      document.getElementById('diag-time').textContent = elapsed + "s ago";
    }, 100);

    setInterval(updateData, 450);
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
  json += "\"packet_count\":" + String(packet_count) + ",";
  json += "\"packet_loss_rate\":" + String(packet_loss_rate) + ",";
  json += "\"signal_lost\":" + String(signal_lost ? "true" : "false") + ",";
  json += "\"last_payload\":\"" + last_payload + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("--- Starting Search & Rescue Receiver & WebServer ---");

  // Spin up Soft Access Point
  WiFi.softAP(ssid, password);
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("WiFi AP SSID: "); Serial.println(ssid);
  Serial.print("AP IP Address: "); Serial.println(apIP);

  // Configure Web Server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Local HTTP Server started.");

  // SPI override for ESP32 Dev Module
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  // Initialize LoRa at 433 MHz
  if (!LoRa.begin(433E6)) {
    Serial.println("[ERROR] SX1278 initialization failed.");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }

  // Parameter alignment matching the STM32
  LoRa.setSignalBandwidth(125E3);    // 125 kHz
  LoRa.setSpreadingFactor(7);        // SF7
  LoRa.setCodingRate4(5);            // Coding Rate 4/5
  LoRa.setSyncWord(0x12);            // Sync Word 0x12
  LoRa.setPreambleLength(8);         // Preamble length of 8 symbols
  LoRa.enableCrc();                  // CRC Check

  Serial.println("LoRa receiver initialized at 433.0 MHz.");
}

void loop() {
  // Handle client requests
  server.handleClient();

  // Parse incoming packet
  int packetSize = LoRa.parsePacket();
  
  if (packetSize) {
    last_packet_time = millis();
    signal_lost = false;
    packet_count++;
    
    // Read payload
    String temp_payload = "";
    while (LoRa.available()) {
      temp_payload += (char)LoRa.read();
    }
    last_payload = temp_payload;

    // Extract sequence number for packet loss tracking
    int underscore_idx = last_payload.indexOf('_');
    if (underscore_idx != -1) {
      uint32_t current_seq = last_payload.substring(underscore_idx + 1).toInt();
      
      if (total_expected_packets > 0) {
        if (current_seq < last_seq_num) {
          // Reset history on transmitter reboot to prevent underflow
          for (int i = 0; i < WINDOW_SIZE; i++) window_packets[i] = true;
          total_expected_packets = 1;
        } else {
          uint32_t gap = current_seq - last_seq_num - 1;
          // Record lost packets in the gap
          for (uint32_t g = 0; g < gap; g++) {
            record_packet(false);
          }
          // Record the currently received packet
          record_packet(true);
          total_expected_packets += (gap + 1);
        }
      } else {
        // Initialize window history as all successful for the first packet
        for (int i = 0; i < WINDOW_SIZE; i++) window_packets[i] = true;
        total_expected_packets = 1;
      }
      
      last_seq_num = current_seq;
      
      // Calculate sliding window loss rate over the last 20 expected packets
      int lost_in_window = 0;
      for (int i = 0; i < WINDOW_SIZE; i++) {
        if (!window_packets[i]) lost_in_window++;
      }
      packet_loss_rate = ((double)lost_in_window / (double)WINDOW_SIZE) * 100.0;
    }

    // Capture RSSI and SNR
    last_raw_rssi = LoRa.packetRssi();
    last_snr = LoRa.packetSnr();

    // Filter RSSI with step-change detection
    if (filtered_rssi == -120.0) {
      filtered_rssi = last_raw_rssi;
    } else if (abs(last_raw_rssi - (int)filtered_rssi) > 5) {
      filtered_rssi = last_raw_rssi;
    } else {
      filtered_rssi = (alpha * last_raw_rssi) + ((1.0 - alpha) * filtered_rssi);
    }

    // Distance Calculation (Log-Distance Path Loss Model)
    double exponent = (A - filtered_rssi) / (10.0 * n);
    current_estimated_distance = pow(10.0, exponent);

    // Serial console logger
    Serial.print("ID: "); Serial.print(last_payload);
    Serial.print(" | Raw: "); Serial.print(last_raw_rssi);
    Serial.print(" | Loss Rate: "); Serial.print(packet_loss_rate, 1);
    Serial.print("% | Distance: "); Serial.println(current_estimated_distance, 2);
  }

  // Handle signal loss timeout
  if (!signal_lost && (millis() - last_packet_time > 5000)) {
    signal_lost = true;
    current_estimated_distance = 999.0;
    filtered_rssi = -120.0;
    Serial.println("[WARNING] SIGNAL LOST");
  }

  // Indicator LED control
  handleLedIndicator(current_estimated_distance);
}

void handleLedIndicator(double distance) {
  unsigned long blink_interval = 2000;

  if (distance > 0.0 && distance <= 1.5) {
    blink_interval = 50;
  } 
  else if (distance > 1.5 && distance <= 40.0) {
    blink_interval = 150 + (unsigned long)((distance - 1.5) * (1350.0 / 38.5));
  } 
  else {
    blink_interval = 2000;
  }

  if (millis() - last_blink_time >= blink_interval) {
    last_blink_time = millis();
    led_state = !led_state;
    digitalWrite(LED_PIN, led_state ? HIGH : LOW);
  }
}
