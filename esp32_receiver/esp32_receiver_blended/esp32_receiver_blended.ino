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
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// OLED Display Settings
#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT    64
#define OLED_RESET       -1
#define OLED_I2C_ADDRESS 0x3C
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Screen refresh variables to avoid blocking high-speed execution
unsigned long lastDisplayUpdateTime = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 250; // Refresh display 4 times per second

// Tracking relative target heading uploaded via GET parameters
volatile double relative_target_heading = -999.0;

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

// Log-Distance Path Loss Model Constants (Dynamic Blend Model)
const double A_close = -68.0; // Calibrated RSSI at 1m (close range)
const double n_close = 2.4;   // Path loss exponent (close range)

const double A_far   = -57.0; // Calibrated RSSI at 1m (far range)
const double n_far   = 3.6;   // Path loss exponent (far range)

const double RSSI_close = -65.0; // Blend start threshold (stronger signal)
const double RSSI_far   = -90.0; // Blend end threshold (weaker signal)

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
  <title>LoRa Precision Locator Dashboard</title>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@300;400;600;700;800&family=Outfit:wght@300;400;600;800&display=swap');
    
    :root {
      --bg-cream: #faf8f5;
      --bg-card: #ffffff;
      --border-color: rgba(43, 38, 33, 0.08);
      --accent-blue: #3e6b89;
      --accent-green: #2a6f47;
      --accent-red: #b23a48;
      --accent-yellow: #d4a373;
      --text-main: #2b2520;
      --text-muted: #82786f;
      --shadow-light: 0 10px 30px rgba(43, 38, 33, 0.03);
      --shadow-hover: 0 15px 40px rgba(43, 38, 33, 0.06);
    }
    
    body {
      background-color: var(--bg-cream);
      color: var(--text-main);
      font-family: 'Plus Jakarta Sans', sans-serif;
      margin: 0;
      padding: 0;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
      overflow-x: hidden;
      -webkit-tap-highlight-color: transparent;
    }
    
    .container {
      width: 92%;
      max-width: 480px;
      margin: 15px auto;
      text-align: center;
    }
    
    h1 {
      font-family: 'Outfit', sans-serif;
      font-size: 28px;
      font-weight: 800;
      margin: 10px 0 2px 0;
      color: var(--text-main);
      letter-spacing: -0.5px;
      background: linear-gradient(90deg, var(--accent-blue), #2c4d63);
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
    }
    
    .subtitle {
      font-size: 11px;
      color: var(--text-muted);
      margin-bottom: 22px;
      text-transform: uppercase;
      letter-spacing: 2px;
      font-weight: 700;
    }
    
    /* Navigation Tabs */
    .tabs-nav {
      display: flex;
      background: rgba(43, 38, 33, 0.04);
      border: 1px solid var(--border-color);
      border-radius: 16px;
      margin-bottom: 20px;
      overflow: hidden;
      padding: 4px;
    }
    
    .tab-btn {
      flex: 1;
      background: none;
      border: none;
      color: var(--text-muted);
      padding: 12px;
      font-size: 13px;
      font-weight: 700;
      cursor: pointer;
      border-radius: 12px;
      transition: all 0.25s cubic-bezier(0.4, 0, 0.2, 1);
    }
    
    .tab-btn.active {
      background: var(--bg-card);
      color: var(--text-main);
      box-shadow: 0 4px 12px rgba(43, 38, 33, 0.05);
    }
    
    .tab-content {
      display: none;
      width: 100%;
    }
    
    .tab-content.active {
      display: block;
    }
    
    /* Card Design */
    .card {
      background: var(--bg-card);
      border: 1px solid var(--border-color);
      border-radius: 24px;
      padding: 22px;
      margin-bottom: 18px;
      box-shadow: var(--shadow-light);
      text-align: left;
      transition: background-color 0.4s cubic-bezier(0.4, 0, 0.2, 1), 
                  border-color 0.4s cubic-bezier(0.4, 0, 0.2, 1),
                  transform 0.2s ease;
    }
    
    .card:hover {
      box-shadow: var(--shadow-hover);
    }
    
    .card-title {
      font-size: 13px;
      font-weight: 800;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 1.5px;
      margin-bottom: 18px;
      border-bottom: 1px dashed var(--border-color);
      padding-bottom: 8px;
    }
    
    .card-info-text {
      font-size: 13px;
      line-height: 1.5;
      color: var(--text-muted);
      margin-top: 0;
      margin-bottom: 15px;
    }
    
    /* Buttons */
    .btn {
      background-color: var(--accent-blue);
      color: white;
      border: none;
      padding: 15px 20px;
      border-radius: 14px;
      font-size: 14px;
      cursor: pointer;
      font-weight: 700;
      transition: all 0.2s cubic-bezier(0.4, 0, 0.2, 1);
      width: 100%;
      text-transform: uppercase;
      letter-spacing: 1px;
      box-shadow: 0 4px 12px rgba(62, 107, 137, 0.12);
    }
    
    .btn:active {
      transform: scale(0.98);
    }
    
    .btn-secondary {
      background-color: rgba(43, 38, 33, 0.04);
      color: var(--text-main);
      box-shadow: none;
      border: 1px solid var(--border-color);
    }
    
    .btn-active {
      background-color: var(--accent-red);
      box-shadow: 0 4px 12px rgba(178, 58, 72, 0.15);
    }
    
    .btn-text {
      background: none;
      border: none;
      color: var(--accent-blue);
      font-size: 12px;
      font-weight: 700;
      cursor: pointer;
      padding: 5px 0;
      text-decoration: underline;
    }
    
    /* Precision Finder Compass Widget */
    .precision-card {
      display: flex;
      flex-direction: column;
      align-items: center;
      text-align: center;
    }
    
    .finder-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 10px 0;
      width: 100%;
    }
    
    .compass-wrapper {
      position: relative;
      width: 230px;
      height: 230px;
      display: flex;
      justify-content: center;
      align-items: center;
      margin-bottom: 12px;
    }
    
    .compass-ring {
      position: absolute;
      top: 0; left: 0; width: 100%; height: 100%;
      border-radius: 50%;
      border: 2px dashed rgba(43, 38, 33, 0.08);
      pointer-events: none;
      box-sizing: border-box;
      transition: transform 0.12s cubic-bezier(0.1, 0.8, 0.3, 1);
    }
    
    .compass-label {
      position: absolute;
      font-size: 13px;
      font-weight: 800;
      color: var(--text-muted);
      transform: translate(-50%, -50%);
    }
    
    .label-n { top: 15px; left: 50%; color: var(--accent-red); }
    .label-e { top: 50%; right: 15px; transform: translate(50%, -50%); }
    .label-s { bottom: 15px; left: 50%; }
    .label-w { top: 50%; left: 15px; transform: translate(-50%, -50%); }
    
    .radar-canvas {
      position: absolute;
      top: 5px; left: 5px;
      width: 220px;
      height: 220px;
      pointer-events: none;
      z-index: 1;
    }
    
    .arrow-container {
      position: absolute;
      width: 95px;
      height: 95px;
      display: flex;
      justify-content: center;
      align-items: center;
      z-index: 3;
      pointer-events: none;
      transition: transform 0.12s cubic-bezier(0.1, 0.8, 0.3, 1);
    }
    
    .guidance-arrow {
      width: 100%;
      height: 100%;
      color: var(--accent-blue);
      filter: drop-shadow(0 4px 10px rgba(62, 107, 137, 0.2));
      transition: color 0.3s ease;
    }
    
    /* When user is facing target (locked state) */
    .precision-card.locked {
      background-color: #ebf5ee;
      border-color: rgba(42, 111, 71, 0.15);
    }
    
    .precision-card.locked .guidance-arrow {
      color: var(--accent-green);
      filter: drop-shadow(0 4px 12px rgba(42, 111, 71, 0.3));
      animation: pulse-arrow 1.2s ease-in-out infinite;
    }
    
    @keyframes pulse-arrow {
      0% { transform: scale(1); }
      50% { transform: scale(1.08); }
      100% { transform: scale(1); }
    }
    
    /* Distance display styling */
    .distance-display {
      text-align: center;
      margin-top: 5px;
    }
    
    .dist-val {
      font-family: 'Outfit', sans-serif;
      font-size: 58px;
      font-weight: 800;
      line-height: 1;
      color: var(--text-main);
      letter-spacing: -1.5px;
    }
    
    .dist-unit {
      font-size: 13px;
      color: var(--text-muted);
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-top: 4px;
    }
    
    .finder-footer {
      margin-top: 15px;
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 10px;
      border-top: 1px solid var(--border-color);
      padding-top: 16px;
      width: 100%;
    }
    
    .direction-text {
      font-size: 18px;
      font-weight: 800;
      letter-spacing: -0.2px;
      color: var(--text-main);
    }
    
    .precision-card.locked .direction-text {
      color: var(--accent-green);
    }
    
    /* Compass Alignment Sweep Overlay */
    .sweep-progress-container {
      margin-top: 15px;
      background: rgba(43, 38, 33, 0.02);
      border: 1px solid var(--border-color);
      border-radius: 16px;
      padding: 14px;
    }
    
    .progress-bar-bg {
      width: 100%;
      height: 6px;
      background: rgba(43, 38, 33, 0.06);
      border-radius: 3px;
      overflow: hidden;
      margin-bottom: 8px;
    }
    
    .progress-bar-fill {
      height: 100%;
      background: var(--accent-blue);
      width: 0%;
      transition: width 0.1s linear;
    }
    
    .sweep-status {
      font-size: 12px;
      font-weight: 700;
      color: var(--text-main);
      text-align: center;
    }
    
    /* Diagnostics grids */
    .stats-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 12px;
    }
    
    .stat-box {
      background: rgba(43, 38, 33, 0.02);
      border: 1px solid var(--border-color);
      border-radius: 18px;
      padding: 14px;
    }
    
    .stat-lbl {
      font-size: 10px;
      color: var(--text-muted);
      text-transform: uppercase;
      font-weight: 700;
      margin-bottom: 4px;
      letter-spacing: 0.5px;
    }
    
    .stat-val {
      font-size: 18px;
      font-weight: 800;
      color: var(--text-main);
    }
    
    .chart-container {
      width: 100%;
      height: 120px;
    }
    
    /* Guide step style */
    .guide-step {
      display: flex;
      align-items: flex-start;
      margin-bottom: 16px;
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
      font-size: 11px;
      font-weight: 800;
      margin-right: 12px;
      flex-shrink: 0;
    }
    
    .step-text {
      font-size: 13.5px;
      line-height: 1.45;
      color: var(--text-main);
    }
    
    .status-badge {
      display: inline-block;
      padding: 4px 12px;
      border-radius: 20px;
      font-size: 11px;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }
    
    .status-tracking {
      background: rgba(42, 111, 71, 0.06);
      color: var(--accent-green);
      border: 1px solid rgba(42, 111, 71, 0.12);
    }
    
    .status-lost {
      background: rgba(178, 58, 72, 0.06);
      color: var(--accent-red);
      border: 1px solid rgba(178, 58, 72, 0.12);
    }
    
    .compass-status-lbl {
      font-size: 12px;
      color: var(--text-muted);
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>LoRa Emergency Locator</h1>
    <div class="subtitle">Precision Finding Dashboard</div>
    
    <!-- Tab Navigation -->
    <div class="tabs-nav">
      <button onclick="switchTab('tracker')" id="btn-tracker" class="tab-btn active">TRACKER</button>
      <button onclick="switchTab('diagnostics')" id="btn-diagnostics" class="tab-btn">DIAGNOSTICS</button>
      <button onclick="switchTab('guide')" id="btn-guide" class="tab-btn">GUIDE</button>
    </div>
    
    <!-- TAB 1: TRACKER -->
    <div id="tab-tracker" class="tab-content active">
      <div class="card precision-card" id="precision-card">
        <div class="card-title">Precision Finder</div>
        
        <div class="finder-container">
          <div class="compass-wrapper">
            <!-- Rotating Compass Face -->
            <div id="compass-ring" class="compass-ring">
              <span class="compass-label label-n">N</span>
              <span class="compass-label label-e">E</span>
              <span class="compass-label label-s">S</span>
              <span class="compass-label label-w">W</span>
            </div>
            
            <!-- Rotating Heatmap / Radar Grid Canvas -->
            <canvas id="radar-canvas" class="radar-canvas"></canvas>
            
            <!-- Real-time Arrow (points to target relative to phone) -->
            <div id="arrow-container" class="arrow-container" style="opacity: 0.15; transform: rotate(0deg);">
              <svg class="guidance-arrow" viewBox="0 0 100 100">
                <polygon points="50,12 82,78 50,60 18,78" fill="currentColor"/>
              </svg>
            </div>
          </div>
          
          <!-- Large Distance Readout -->
          <div class="distance-display">
            <div id="dist-val" class="dist-val">---</div>
            <div id="dist-unit" class="dist-unit">meters</div>
          </div>
          
          <!-- Proximity Progress Bar (Hot-Cold) -->
          <div class="proximity-container" style="margin: 16px 16px 0 16px; background: rgba(0,0,0,0.05); border-radius: 6px; padding: 4px; border: 1px solid rgba(0,0,0,0.05);">
            <div style="display: flex; justify-content: space-between; font-size: 11px; font-weight: 600; color: var(--text-muted); margin-bottom: 2px;">
              <span>COLD (Far)</span>
              <span id="proximity-val">0%</span>
              <span>HOT (Near)</span>
            </div>
            <div style="background: rgba(0,0,0,0.06); height: 12px; border-radius: 4px; overflow: hidden; width: 100%;">
              <div id="proximity-fill" style="width: 0%; height: 100%; background: linear-gradient(90deg, #3e6b89, #d4a373, #2a6f47); transition: width 0.3s ease;"></div>
            </div>
          </div>

          <div id="drift-warning" style="display: none; background: #FFF0F0; border: 1px solid #FFD2D2; color: #D32F2F; border-radius: 8px; padding: 10px; margin: 12px 16px 0 16px; font-size: 13px; text-align: center; font-weight: 600;">⚠️ Drift Detected! Distance increasing. Recalibrate sweep.</div>
        </div>
        
        <div class="finder-footer">
          <div id="status-badge" class="status-badge status-lost">No Connection</div>
          <div id="direction-text" class="direction-text">Awaiting Alignment Sweep</div>
        </div>
      </div>
      
      <!-- Calibration Sweep Tool -->
      <div class="card">
        <div class="card-title">Direction Mapping</div>
        <p class="card-info-text">
          LoRa is omnidirectional. Sweep your phone in a circle to map the signal strength in each direction and find the target.
        </p>
        <div class="stats-grid" style="grid-template-columns: 1fr; gap:10px;">
          <button id="sweep-btn" class="btn" onclick="startSweep()">Start Sweep (10s Turn)</button>
        </div>
        
        <div id="sweep-progress-container" class="sweep-progress-container" style="display: none;">
          <div class="progress-bar-bg">
            <div id="sweep-progress-bar" class="progress-bar-fill"></div>
          </div>
          <div id="sweep-status" class="sweep-status">Rotate slowly in a full circle...</div>
        </div>
        
        <div style="margin-top: 12px; display: flex; justify-content: right; align-items: center;">
          <button onclick="resetSignalMap()" class="btn-text">Reset Signal Map</button>
        </div>
        <div id="compass-status" style="font-size: 11px; color: var(--text-muted); margin-top: 6px; text-align: center;">Compass offline</div>
      </div>
      
      <div class="card">
        <button id="sound-btn" class="btn btn-secondary">Enable Audio Search Beeps</button>
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
    
    <!-- TAB 3: GUIDE -->
    <div id="tab-guide" class="tab-content">
      <div class="card">
        <div class="card-title">Direction Hunting Protocol</div>
        
        <div class="guide-step">
          <div class="step-num">1</div>
          <div class="step-text">
            <strong>Calibrate Position:</strong> Turn on your receiver, connect your phone, and stand in an open area. Click <strong>"Connect Compass"</strong>.
          </div>
        </div>
        
        <div class="guide-step">
          <div class="step-num">2</div>
          <div class="step-text">
            <strong>Start Calibrate Sweep:</strong> Click <strong>"Calibrate Sweep"</strong> and slowly turn in a full circle. Your body blocks the signal, helping the system detect the direction of peak reception.
          </div>
        </div>
        
        <div class="guide-step">
          <div class="step-num">3</div>
          <div class="step-text">
            <strong>Follow the Precision Arrow:</strong> Once calibrated, the arrow points towards the beacon. Turn until the arrow points straight up, turning the screen green and triggering haptic vibration.
          </div>
        </div>
        
        <div class="guide-step">
          <div class="step-num">4</div>
          <div class="step-text">
            <strong>Pinpoint at Close Range:</strong> When the distance drops below 1.5 meters, you are in the immediate vicinity.
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
    
    // Compass and sweep variables
    let userHeading = 0;
    let compassX = 1.0;
    let compassY = 0.0;
    const compassBeta = 0.15; // Jitter filter factor (lower = smoother, higher = faster)
    let targetAbsoluteHeading = null;
    let distanceTrendHistory = [];

    function filterCompassHeading(rawHeading) {
      let rad = rawHeading * Math.PI / 180;
      let x = Math.cos(rad);
      let y = Math.sin(rad);
      
      // Smooth Cartesian vector components to handle 360/0 degree wrap-around correctly
      compassX = (compassBeta * x) + ((1 - compassBeta) * compassX);
      compassY = (compassBeta * y) + ((1 - compassBeta) * compassY);
      
      let smoothHeading = Math.atan2(compassY, compassX) * 180 / Math.PI;
      if (smoothHeading < 0) {
    let isSweeping = false;
    let sweepDuration = 10000; // 10 seconds for a full circle turn
    let sweepStartTime = 0;
    let lastRawRssi = -120;
    let packetCount = 0;
    let lastPacketCount = 0;
    
    // Divide 360° into 12 sectors of 30° each
    let sectors = new Array(12).fill(null);
    let sectorTimes = new Array(12).fill(0);
    
    // Load cached calibration values if available
    try {
      let cachedTarget = localStorage.getItem('targetAbsoluteHeading');
      if (cachedTarget !== null) {
        targetAbsoluteHeading = parseFloat(cachedTarget);
      }
      let cachedSectors = localStorage.getItem('sectors');
      if (cachedSectors !== null) {
        sectors = JSON.parse(cachedSectors);
      }
      let cachedTimes = localStorage.getItem('sectorTimes');
      if (cachedTimes !== null) {
        sectorTimes = JSON.parse(cachedTimes);
      }
    } catch(e) {}

    // Canvas configuration
    const rCanvas = document.getElementById('radar-canvas');
    const rCtx = rCanvas.getContext('2d');
    let sweepLineAngle = 0;

    // Resize canvas
    rCanvas.width = 220;
    rCanvas.height = 220;

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

    // Device compass alignment initialization (Bypassed in compass-free mode)
    function initCompass() {}

    function updateCompassUI() {
      document.getElementById('user-heading-val').textContent = Math.round(userHeading) + "°";
      
      // Rotate the physical cardinal direction labels ring
      const ring = document.getElementById('compass-ring');
      if (ring) {
        ring.style.transform = `rotate(${-userHeading}deg)`;
      }
    }

    // Direction sweep calibration tool
    function startSweep() {
      if (isSweeping) return;
      isSweeping = true;
      sweepStartTime = Date.now();
      
      // Clear data to start fresh
      sectors.fill(null);
      sectorTimes.fill(0);
      distanceTrendHistory = [];
      
      document.getElementById('sweep-progress-container').style.display = "block";
      const sweepBtn = document.getElementById('sweep-btn');
      sweepBtn.disabled = true;
      sweepBtn.textContent = "Sweeping...";
      
      // Request audio engine wake up
      if (audioEnabled && audioCtx && audioCtx.state === 'suspended') {
        audioCtx.resume();
      }
    }

    function resetSignalMap() {
      sectors.fill(null);
      sectorTimes.fill(0);
      distanceTrendHistory = [];
      targetAbsoluteHeading = null;
      localStorage.removeItem('targetAbsoluteHeading');
      localStorage.removeItem('sectors');
      localStorage.removeItem('sectorTimes');
      document.getElementById('direction-text').textContent = "Awaiting Alignment Sweep";
      document.getElementById('arrow-container').style.opacity = "0.15";
      document.getElementById('precision-card').classList.remove('locked');
      document.getElementById('drift-warning').style.display = "none";
      document.getElementById('proximity-val').textContent = "0%";
      document.getElementById('proximity-fill').style.width = "0%";
      
      // Clear canvas sector drawing
      drawRadarCanvas();
    }

    // Render local radar sector signal strength sweep map
    function drawRadarCanvas() {
      rCtx.clearRect(0, 0, 220, 220);
      
      // 1. Draw sector sweeps (Heatmap)
      for (let i = 0; i < 12; i++) {
        if (sectors[i] !== null) {
          // Decaying check (older than 90s gets transparent)
          let age = Date.now() - sectorTimes[i];
          let opacity = 0.45;
          if (age > 90000) {
            opacity = Math.max(0.05, 0.45 - (age - 90000) / 30000);
          }
          
          let rssiVal = sectors[i];
          let fillColor = `rgba(62, 107, 137, ${opacity})`; // Default Weak Blue
          if (rssiVal >= -65) {
            fillColor = `rgba(42, 111, 71, ${opacity})`; // Strong Green
          } else if (rssiVal >= -82) {
            fillColor = `rgba(212, 163, 115, ${opacity})`; // Medium Gold
          }
          
          // Draw sector wedge relative to starting direction (no userHeading offset)
          let startAngle = i * 30 - 90;
          let endAngle = (i + 1) * 30 - 90;
          
          rCtx.beginPath();
          rCtx.moveTo(110, 110);
          rCtx.arc(110, 110, 100, (startAngle * Math.PI) / 180, (endAngle * Math.PI) / 180);
          rCtx.closePath();
          rCtx.fillStyle = fillColor;
          rCtx.fill();
        }
      }
      
      // 2. Draw radar visual grids
      rCtx.strokeStyle = 'rgba(43, 38, 33, 0.05)';
      rCtx.lineWidth = 1.5;
      
      // Outer border circle
      rCtx.beginPath();
      rCtx.arc(110, 110, 100, 0, 2 * Math.PI);
      rCtx.stroke();
      
      // Inner rings
      rCtx.beginPath();
      rCtx.arc(110, 110, 66, 0, 2 * Math.PI);
      rCtx.stroke();
      
      rCtx.beginPath();
      rCtx.arc(110, 110, 33, 0, 2 * Math.PI);
      rCtx.stroke();
      
      // Crosshairs
      rCtx.strokeStyle = 'rgba(43, 38, 33, 0.04)';
      rCtx.beginPath();
      rCtx.moveTo(110, 10);
      rCtx.lineTo(110, 210);
      rCtx.moveTo(10, 110);
      rCtx.lineTo(210, 110);
      rCtx.stroke();
      
      // 3. Draw scanning radar sweep line
      if (isSweeping) {
        let elapsed = Date.now() - sweepStartTime;
        sweepLineAngle = Math.min(360, (elapsed / sweepDuration) * 360);
      } else {
        sweepLineAngle = (sweepLineAngle + 0.5) % 360;
      }
      let sweepRad = (sweepLineAngle - 90) * Math.PI / 180;
      
      let grad = rCtx.createRadialGradient(110, 110, 0, 110, 110, 100);
      grad.addColorStop(0, 'rgba(62, 107, 137, 0.15)');
      grad.addColorStop(1, 'rgba(62, 107, 137, 0.0)');
      
      rCtx.fillStyle = grad;
      rCtx.beginPath();
      rCtx.moveTo(110, 110);
      rCtx.arc(110, 110, 100, sweepRad - 0.25, sweepRad + 0.05);
      rCtx.closePath();
      rCtx.fill();
      
      rCtx.strokeStyle = 'rgba(62, 107, 137, 0.2)';
      rCtx.lineWidth = 1;
      rCtx.beginPath();
      rCtx.moveTo(110, 110);
      rCtx.lineTo(110 + 100 * Math.cos(sweepRad), 110 + 100 * Math.sin(sweepRad));
      rCtx.stroke();
    }

    // Animation frame for canvas rendering
    function animateCanvas() {
      drawRadarCanvas();
      requestAnimationFrame(animateCanvas);
    }
    requestAnimationFrame(animateCanvas);

    // Audio beeper ticks engine
    let lastVibrateTime = 0;
    setInterval(function() {
      if (!audioEnabled || isLost) return;
      let now = Date.now();
      if (now >= nextBeepTime) {
        let interval = 2000;
        let pitch = 440;
        
        // Dynamic adjust based on distance
        if (currentDistance <= 1.5) {
          interval = 120;
          pitch = 880;
        } else if (currentDistance >= 40.0) {
          interval = 2000;
          pitch = 440;
        } else {
          interval = 120 + (currentDistance - 1.5) * (1880.0 / 38.5);
          pitch = 880 - (currentDistance - 1.5) * (440.0 / 38.5);
        }
        
        // Speed up audio ticks even more when pointing directly at beacon
        let isPointingAtTarget = false;
        if (targetAbsoluteHeading !== null) {
          let theta = targetAbsoluteHeading - userHeading;
          let relAngle = ((theta + 180) % 360 + 360) % 360 - 180;
          if (Math.abs(relAngle) <= 15) {
            isPointingAtTarget = true;
            interval = interval * 0.6; // 40% faster tick frequency
            pitch = pitch * 1.15; // Higher-pitched confirm sound
          }
        }
        
        playBeep(pitch, 0.05);
        nextBeepTime = now + interval;

        // Vibrate if pointing directly at target (Android Chrome Vibration API)
        if (isPointingAtTarget && navigator.vibrate && (now - lastVibrateTime > interval)) {
          navigator.vibrate(currentDistance <= 1.5 ? [60] : [40]);
          lastVibrateTime = now;
        }
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
      
      ctx.strokeStyle = 'rgba(43, 38, 33, 0.05)';
      ctx.lineWidth = 1;
      for (let y = 0; y <= h; y += h/4) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(w, y);
        ctx.stroke();
      }

      if (rssiHistory.length < 2) return;

      let grad = ctx.createLinearGradient(0, 0, 0, h);
      grad.addColorStop(0, 'rgba(62, 107, 137, 0.15)');
      grad.addColorStop(1, 'rgba(62, 107, 137, 0.0)');
      
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

    function updateData() {
      let queryUrl = '/data';
      if (targetAbsoluteHeading !== null) {
        let relAngle = targetAbsoluteHeading;
        while (relAngle > 180) relAngle -= 360;
        while (relAngle < -180) relAngle += 360;
        queryUrl += '?target_rel_heading=' + Math.round(relAngle);
      }
      fetch(queryUrl)
        .then(response => response.json())
        .then(data => {
          isLost = data.signal_lost;
          currentDistance = data.distance;
          lastRawRssi = data.raw_rssi;
          packetCount = data.packet_count;

          const distVal = document.getElementById('dist-val');
          const distUnit = document.getElementById('dist-unit');
          const statusBadge = document.getElementById('status-badge');
          
          const diagRaw = document.getElementById('diag-raw');
          const diagFilt = document.getElementById('diag-filt');
          const diagSnr = document.getElementById('diag-snr');
          const diagLoss = document.getElementById('diag-loss');
          const diagId = document.getElementById('diag-id');
          const diagTime = document.getElementById('diag-time');
          
          const arrowContainer = document.getElementById('arrow-container');
          const precisionCard = document.getElementById('precision-card');
          const directionText = document.getElementById('direction-text');

          if (isLost) {
            distVal.textContent = "LOST";
            distVal.style.color = "var(--accent-red)";
            distUnit.textContent = "";
            statusBadge.className = "status-badge status-lost";
            statusBadge.textContent = "Signal Lost";
            arrowContainer.style.opacity = "0.1";
            precisionCard.classList.remove('locked');
            directionText.textContent = "Signal Lost";
            
            // Reset proximity indicator
            document.getElementById('proximity-val').textContent = "0%";
            document.getElementById('proximity-fill').style.width = "0%";

            diagRaw.textContent = "---";
            diagFilt.textContent = "---";
            diagSnr.textContent = "---";
            diagLoss.textContent = "---";
            diagId.textContent = "---";
            diagTime.textContent = "---";
          } else {
            lastReceivedTime = Date.now();
            statusBadge.className = "status-badge status-tracking";
            statusBadge.textContent = "Connected";
            distVal.style.color = "var(--text-main)";
            
            diagRaw.textContent = data.raw_rssi + " dBm";
            diagFilt.textContent = data.filtered_rssi.toFixed(1) + " dBm";
            diagSnr.textContent = data.snr.toFixed(1) + " dB";
            
            let reliability = (100.0 - data.packet_loss_rate);
            diagLoss.textContent = reliability.toFixed(1) + "%";
            diagLoss.style.color = reliability > 90 ? "var(--accent-green)" : (reliability > 70 ? "var(--accent-yellow)" : "var(--accent-red)");
            
            diagId.textContent = data.last_payload;

            // Calculate Proximity (0% to 100% based on filtered RSSI -105 to -50)
            let proximity = 0;
            if (data.filtered_rssi > -105) {
              proximity = Math.round(((data.filtered_rssi - (-105)) / (-50 - (-105))) * 100);
              if (proximity > 100) proximity = 100;
              if (proximity < 0) proximity = 0;
            }
            document.getElementById('proximity-val').textContent = proximity + "%";
            document.getElementById('proximity-fill').style.width = proximity + "%";

            rssiHistory.push(data.raw_rssi);
            if (rssiHistory.length > maxHistory) {
              rssiHistory.shift();
            }

            // Display distance value
            if (data.distance < 1.0) {
              distVal.textContent = Math.round(data.distance * 100);
              distUnit.textContent = "cm";
            } else {
              distVal.textContent = data.distance.toFixed(1);
              distUnit.textContent = "meters";
            }

            // Drift detection logic
            distanceTrendHistory.push(data.distance);
            if (distanceTrendHistory.length > 5) {
              distanceTrendHistory.shift();
            }
            
            let isIncreasing = false;
            if (distanceTrendHistory.length >= 4) {
              isIncreasing = (distanceTrendHistory[distanceTrendHistory.length - 1] > distanceTrendHistory[distanceTrendHistory.length - 2]) && 
                             (distanceTrendHistory[distanceTrendHistory.length - 2] > distanceTrendHistory[distanceTrendHistory.length - 3]) &&
                             (distanceTrendHistory[distanceTrendHistory.length - 3] > distanceTrendHistory[distanceTrendHistory.length - 4]);
            }
            
            let isPointingAhead = false;
            if (targetAbsoluteHeading !== null) {
              let theta = targetAbsoluteHeading - userHeading;
              let relAngle = ((theta + 180) % 360 + 360) % 360 - 180;
              isPointingAhead = Math.abs(relAngle) <= 25;
            }
            
            const driftWarning = document.getElementById('drift-warning');
            if (isIncreasing && isPointingAhead && data.distance > 2.0) {
              driftWarning.style.display = "block";
            } else {
              driftWarning.style.display = "none";
            }

            // Continuous calibration capture (ONLY during active sweep)
            if (packetCount !== lastPacketCount) {
              lastPacketCount = packetCount;
              
              if (isSweeping) {
                let elapsed = Date.now() - sweepStartTime;
                let sweepAngle = Math.min(360, (elapsed / sweepDuration) * 360);
                let currentSector = Math.floor(sweepAngle / 30) % 12;
                sectors[currentSector] = data.filtered_rssi;
                sectorTimes[currentSector] = Date.now();
                
                // Save to cache
                try {
                  localStorage.setItem('sectors', JSON.stringify(sectors));
                  localStorage.setItem('sectorTimes', JSON.stringify(sectorTimes));
                } catch(e) {}
              }
            }

            // Active Sweep Progress tracking
            if (isSweeping) {
              let elapsed = Date.now() - sweepStartTime;
              let pct = Math.min(100, (elapsed / sweepDuration) * 100);
              
              document.getElementById('sweep-progress-bar').style.width = pct + "%";
              document.getElementById('sweep-status').textContent = `Sweeping... rotate phone slowly (${Math.round((sweepDuration - elapsed)/1000)}s)`;
              
              if (elapsed >= sweepDuration) {
                isSweeping = false;
                document.getElementById('sweep-progress-container').style.display = "none";
                const sweepBtn = document.getElementById('sweep-btn');
                sweepBtn.disabled = false;
                sweepBtn.textContent = "Start Sweep (10s Turn)";
                
                // Identify target direction
                let maxR = -120;
                let bestSec = -1;
                for (let i = 0; i < 12; i++) {
                  if (sectors[i] !== null && sectors[i] > maxR) {
                    maxR = sectors[i];
                    bestSec = i;
                  }
                }
                
                if (bestSec !== -1) {
                  targetAbsoluteHeading = bestSec * 30 + 15; // Midpoint angle
                  try {
                    localStorage.setItem('targetAbsoluteHeading', targetAbsoluteHeading);
                  } catch(e) {}
                } else {
                  alert("Sweep failed: No signals detected. Please try again.");
                }
              }
            }

            // Direction Arrow and relative angle indicators (Compass-Free Clock face)
            if (targetAbsoluteHeading !== null) {
              arrowContainer.style.opacity = "1";
              
              let relAngle = targetAbsoluteHeading;
              while (relAngle > 180) relAngle -= 360;
              while (relAngle < -180) relAngle += 360;
              
              arrowContainer.style.transform = `rotate(${relAngle}deg)`;
              
              if (Math.abs(relAngle) <= 20) {
                precisionCard.classList.add('locked');
                directionText.textContent = "AHEAD";
              } else {
                precisionCard.classList.remove('locked');
                if (relAngle > 20 && relAngle <= 160) {
                  directionText.textContent = `Target is to your Right (${Math.round(relAngle)}°)`;
                } else if (relAngle < -20 && relAngle >= -160) {
                  directionText.textContent = `Target is to your Left (${Math.round(Math.abs(relAngle))}°)`;
                } else {
                  directionText.textContent = "Target is Behind You";
                }
              }
            } else {
              arrowContainer.style.opacity = "0.15";
              precisionCard.classList.remove('locked');
              directionText.textContent = "Awaiting Sweep";
            }
          }
          
          if (document.getElementById('tab-diagnostics').classList.contains('active')) {
            drawChart();
          }
        })
        .catch(err => {
          document.getElementById('direction-text').textContent = "Dashboard Connection Error";
        });
    }

    // Diagnostics clock elapsed time indicator
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
  if (server.hasArg("target_rel_heading")) {
    relative_target_heading = server.arg("target_rel_heading").toDouble();
  }

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

  // Draw Distance Readout
  display.setCursor(0, 16);
  if (signal_lost) {
    display.setTextSize(2);
    display.println("SIGNAL LOST");
    display.setTextSize(1);
    display.print("Time: ---");
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
    
    // Draw Diagnostics
    display.setTextSize(1);
    display.setCursor(0, 36);
    display.print("RSSI: ");
    display.print(last_raw_rssi);
    display.println(" dBm");

    display.setCursor(0, 46);
    display.print("LOSS: ");
    display.print(packet_loss_rate, 1);
    display.println(" %");

    display.setCursor(0, 56);
    display.print("SEQ : ");
    display.println(last_seq_num);
  }

  // Draw Rotating Heading Arrow on the right
  if (!signal_lost && relative_target_heading != -999.0) {
    double rel_angle = relative_target_heading;
    while (rel_angle > 180) rel_angle -= 360;
    while (rel_angle < -180) rel_angle += 360;

    int x_c = 104;
    int y_c = 36;
    int r = 16;

    // Draw circular dial housing
    display.drawCircle(x_c, y_c, r, SH110X_WHITE);
    // Draw cardinal reference points
    display.drawPixel(x_c, y_c - r, SH110X_WHITE);
    display.drawPixel(x_c + r, y_c, SH110X_WHITE);
    display.drawPixel(x_c, y_c + r, SH110X_WHITE);
    display.drawPixel(x_c - r, y_c, SH110X_WHITE);

    // Convert relative angle to screen-space coordinates
    double rad = (rel_angle - 90.0) * DEG_TO_RAD;
    int x_tip = x_c + (int)(13.0 * cos(rad));
    int y_tip = y_c + (int)(13.0 * sin(rad));

    // Draw pointer line
    display.drawLine(x_c, y_c, x_tip, y_tip, SH110X_WHITE);
    display.drawCircle(x_tip, y_tip, 2, SH110X_WHITE);

    // Ahead text notifier
    if (abs(rel_angle) <= 20) {
      display.setCursor(88, 56);
      display.print("AHEAD");
    }
  } else {
    // Draw scanning sweep placeholder
    int x_c = 104;
    int y_c = 36;
    int r = 16;
    display.drawCircle(x_c, y_c, r, SH110X_WHITE);
    
    static double placeholder_angle = 0;
    placeholder_angle = fmod(placeholder_angle + 10.0, 360.0);
    double rad = (placeholder_angle - 90.0) * DEG_TO_RAD;
    display.drawLine(x_c, y_c, x_c + (int)(14.0 * cos(rad)), y_c + (int)(14.0 * sin(rad)), SH110X_WHITE);
  }

  display.display();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("--- Starting Search & Rescue Receiver & WebServer ---");

  // Initialize I2C OLED SSD1306/SH1106
  Wire.begin(21, 22); // Explicitly bind standard I2C pins
  if(!display.begin(OLED_I2C_ADDRESS, true)) {
    Serial.println("[ERROR] SH1106 OLED initialization failed! Check wiring.");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 20);
    display.println("SEARCHING BEACON...");
    display.display();
    Serial.println("OLED Display initialized successfully.");
  }

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

  // Draw OLED screen periodically (non-blocking)
  drawOledDisplay(false);

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

    // Dynamic Blending Factor Calculation based on filtered RSSI
    double t = (filtered_rssi - RSSI_far) / (RSSI_close - RSSI_far);
    if (t > 1.0) t = 1.0;
    if (t < 0.0) t = 0.0;

    // Linearly interpolate the path loss constants A and n
    double A_blend = (t * A_close) + ((1.0 - t) * A_far);
    double n_blend = (t * n_close) + ((1.0 - t) * n_far);

    // Blended Distance Calculation
    double exponent = (A_blend - filtered_rssi) / (10.0 * n_blend);
    current_estimated_distance = pow(10.0, exponent);

    // Serial console logger
    Serial.print("ID: "); Serial.print(last_payload);
    Serial.print(" | Raw: "); Serial.print(last_raw_rssi);
    Serial.print(" | Loss Rate: "); Serial.print(packet_loss_rate, 1);
    Serial.print("% | Distance: "); Serial.println(current_estimated_distance, 2);

    drawOledDisplay(true); // Force redraw to show updated distance on OLED
  }

  // Handle signal loss timeout
  if (!signal_lost && (millis() - last_packet_time > 5000)) {
    signal_lost = true;
    current_estimated_distance = 999.0;
    filtered_rssi = -120.0;
    drawOledDisplay(true); // Force redraw to show SIGNAL LOST on OLED
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
