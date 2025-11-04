/*
  ESP32 Flow Monitor (MJ-HW83C on GPIO 34)
  - PCNT pulse counting
  - Integer metrics (flow L/min, Hz, total liters)
  - Modern web gauge (live JSON polling)
  - Serial Plotter prints integers for easy reading

  Calibration:
    PULSES_PER_LITER = 1319  (your measured value)
    CAL_TRIM lets you nudge ±5% without changing PPL.

  Access:
    http://<esp32-ip>/    or  http://esp32flow.local/
    SoftAP fallback: SSID ESP32-Flow  PASS 12345678
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <driver/pcnt.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>

// ------------ Dual Core Setup ------------
#if CONFIG_FREERTOS_UNICORE
  #define ARDUINO_RUNNING_CORE 0
#else
  #define ARDUINO_RUNNING_CORE 1  // Core 0 is the default for Arduino, we'll use Core 1 for our main task
#endif

// Mutex for thread-safe access to shared variables
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Task handles
TaskHandle_t SensorTask;

// ------------ Constants ------------
// Memory optimization settings
#define MAX_OVERFLOW_COUNT 10   // Maximum number of overflows before resetting counter
#define JSON_DOC_SIZE 128       // Reduced from 256 to save RAM
#define WS_TIMEOUT 5000         // WebSocket timeout in ms
#define MAX_WS_CLIENTS 2        // Limit number of WebSocket clients

// Global variables for ISR
volatile bool overflow_detected = false;
volatile uint32_t overflow_count = 0;

// PCNT ISR Handler
void IRAM_ATTR pcnt_isr_handler(void* arg) {
  overflow_detected = true;
  overflow_count++;
  if (overflow_count >= MAX_OVERFLOW_COUNT) {
    pcnt_counter_clear(PCNT_UNIT_0);
    overflow_count = 0;
  }
}

// ------------ Wi-Fi ------------
static const char* WIFI_SSID     = "NAME";
static const char* WIFI_PASSWORD = "PASS";
static const char* AP_SSID       = "ESP32-Flow";
static const char* AP_PASS       = "12345678";
static const char* MDNS_NAME     = "esp32flow";

// ------------ Authentication ------------
const char* www_username = "admin";
const char* www_password = "flowmeter";

// ------------ Power Management ------------
#define FLOW_TIMEOUT_MS 300000  // 5 minutes of no flow before entering light sleep
#define DEEP_SLEEP_ENABLED true
// Moved to shared globals section

// ------------ WebSocket ------------
WebSocketsServer webSocket(81);  // Function prototypes
void IRAM_ATTR pcnt_overflow_handler(void *arg);
void pcnt_init_unit();
void startWiFi();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
bool is_authenticated();
void getMetricsJSON(JsonDocument& doc);
void handleMetrics();
void handleRoot();
void setupWeb();
void enterLightSleep();
float calculateAdaptiveAlpha(float flow_rate);
void sampleOnce();

// ------------ Flow input ------------
static const gpio_num_t FLOW_GPIO = GPIO_NUM_34;

// We use the pulses-per-liter model with your calibration
#define USE_FREQ_MODEL 0
static const float PULSES_PER_LITER = 1319.0f;  // your calibration
// Optional ±5% trim. 1.00 = nominal, 0.95 = -5%, 1.05 = +5%.
static const float CAL_TRIM = 1.00f;

// If you ever switch to frequency model, set USE_FREQ_MODEL=1 and fill this:
static const float HZ_PER_LPM = 7.5f;

// Sampling
static const uint32_t SAMPLE_MS = 1000;    // 1 Hz updates

// PCNT glitch filter (0 disables). ~1000 ≈ 12.5 µs at 80 MHz APB.
static const uint16_t PCNT_GLITCH_FILTER = 0;

// Simple smoothing on the instantaneous flow to avoid jumpy gauge
static const float FLOW_EMA_ALPHA = 0.35f;  // 0..1 (higher = snappier)

// ------------ Shared Globals ------------
static pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;
WebServer server(80);

// Shared variables between cores (volatile for thread safety)
volatile float g_flow_lpm_f = 0.0f;   // EMA-smoothed float (internal)
volatile float g_total_liters_f = 0.0f;
volatile float g_hz_f = 0.0f;
volatile unsigned long last_flow_detected = 0;
volatile bool flow_active = false;

unsigned long last_sample_ms = 0;

// ------------ PCNT ------------
void pcnt_init_unit() {
  pinMode((int)FLOW_GPIO, INPUT_PULLUP); // Enable internal pull-up for better noise immunity

  pcnt_config_t cfg = {
    .pulse_gpio_num = FLOW_GPIO,
    .ctrl_gpio_num = PCNT_PIN_NOT_USED,
    .lctrl_mode = PCNT_MODE_KEEP,
    .hctrl_mode = PCNT_MODE_KEEP,
    .pos_mode = PCNT_COUNT_INC,   // Count rising edge
    .neg_mode = PCNT_COUNT_DIS,   // Disable counting falling edge
    .counter_h_lim = 30000,       // Below INT16_MAX to detect potential overflow
    .counter_l_lim = -30000,      // Above INT16_MIN to detect potential underflow
    .unit = PCNT_UNIT_0,
    .channel = PCNT_CHANNEL_0
  };

  // Initialize PCNT unit
  if (pcnt_unit_config(&cfg) != ESP_OK) {
    Serial.println("PCNT config failed");
    return;
  }

  // Configure and enable glitch filter if needed
  if (PCNT_GLITCH_FILTER > 0) {
    pcnt_set_filter_value(PCNT_UNIT, PCNT_GLITCH_FILTER);
    pcnt_filter_enable(PCNT_UNIT);
  } else {
    pcnt_filter_disable(PCNT_UNIT);
  }

  // Clear and start counter
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  
  // Set up event on high limit (for overflow detection)
  pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);
  
  // Register ISR service
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_isr_handler, NULL);
  
  pcnt_counter_resume(PCNT_UNIT);
  
  Serial.println("PCNT unit initialized");
}

// ------------ Wi-Fi ------------
void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected: %s  IP: %s\n", WIFI_SSID, WiFi.localIP().toString().c_str());
  } else {
    Serial.println("WiFi STA failed, starting SoftAP...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("AP SSID: %s  PASS: %s  IP: %s\n",
                  AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  }

  if (MDNS.begin(MDNS_NAME)) {
    Serial.printf("mDNS: http://%s.local/\n", MDNS_NAME);
  } else {
    Serial.println("mDNS setup failed");
  }
}

// ------------ WebSocket Event Handler ------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Client disconnected\n", num);
      break;
      
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] New client connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      
      // Send current state on connect
      StaticJsonDocument<192> doc;
      getMetricsJSON(doc);
      String json;
      if (serializeJson(doc, json) == 0) {
        Serial.println("Error: Failed to serialize JSON for WebSocket");
        return;
      }
      
      if (!webSocket.sendTXT(num, json)) {
        Serial.println("Error: Failed to send WebSocket message");
      } else {
        Serial.println("Sent initial data to client");
      }
      break;
    }
      
    case WStype_TEXT:
      // Handle incoming WebSocket messages if needed
      // webSocket.sendTXT(num, payload, length); // Echo back for testing
      break;
      
    case WStype_ERROR:
      Serial.printf("[%u] WebSocket error\n", num);
      break;
      
    default:
      Serial.printf("[%u] Unhandled WebSocket event: %d\n", num, type);
      break;
  }
}

// ------------ Authentication Middleware ------------
bool is_authenticated() {
  if (!server.authenticate(www_username, www_password)) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

// ------------ Metrics JSON Generator ------------
void getMetricsJSON(JsonDocument& doc) {
  // Use millis() for precise timing
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  
  // Only update values if enough time has passed (throttle updates)
  if (now - lastUpdate >= 500) {  // 2 updates per second max
    lastUpdate = now;
    
    // Store values with millisecond precision
    doc[F("flow_lpm")] = g_flow_lpm_f;
    doc[F("hz")] = g_hz_f;
    doc[F("total_liters")] = g_total_liters_f;
    doc[F("t_ms")] = now;
  }
  
  // Always include these values (they don't change often)
  doc[F("flow_lpm_int")] = (int)lroundf(g_flow_lpm_f);
  doc[F("hz_int")] = (int)lroundf(g_hz_f);
  doc[F("total_liters_int")] = (int)lroundf(g_total_liters_f);
  doc[F("model")] = USE_FREQ_MODEL ? F("Hz_per_LPM") : F("Pulses_per_Liter");
  doc[F("ppl_int")] = (int)lroundf(PULSES_PER_LITER * CAL_TRIM);
  doc[F("overflow")] = overflow_detected;
  
  // Add system info
  doc[F("rssi")] = WiFi.RSSI();
  doc[F("uptime")] = now / 1000;  // uptime in seconds
}

// ------------ Web UI ------------
const char INDEX_HTML[] PROGMEM = R"=====(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Flow Monitor</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>
  :root{
    --bg:#0a0f14; --card:#0f1520; --edge:#1d2a3b; --ink:#eaf3ff; --muted:#9bb1c9;
    --glow:#5aa9ff; --glow2:#7cdb84; --warn:#ff7a7a; --ok:#7cdb84;
  }
  *{box-sizing:border-box}
  body{margin:0;background:radial-gradient(1200px 600px at 70% -10%,#0f1724 0%,#0a0f14 60%);
       color:var(--ink);font-family:Inter,system-ui,Segoe UI,Roboto,Helvetica,Arial,sans-serif;
       min-height:100vh;display:grid;place-items:center;padding:24px}
  .wrap{width:100%;max-width:880px}
  .card{background:linear-gradient(180deg,#111826 0%,#0f1520 100%);border:1px solid var(--edge);
        border-radius:16px;padding:24px;box-shadow:0 8px 16px rgba(0,0,0,0.2);max-width:800px;margin:0 auto;position:relative;overflow:hidden}
  .title{color:var(--ink);font-size:20px;font-weight:600;margin-bottom:12px;text-transform:uppercase;letter-spacing:1px}
  .pill{border:1px solid #284059;border-radius:999px;padding:4px 10px;font-size:12px;color:var(--muted)}
  .metrics{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:12px;margin:20px 0;text-align:left}
  @media(min-width:780px){.grid{grid-template-columns:1.2fr 1fr}}
  .metric{background:rgba(90,169,255,0.05);border-radius:8px;padding:12px;border:1px solid var(--edge);transition:all 0.2s ease}
  .gaugeWrap{display:grid;place-items:center;padding:10px}
  .big{display:flex;flex-direction:column;align-items:center;gap:2px;margin-top:6px}
  .big .value{font-size:56px;font-weight:800;letter-spacing:.5px}
  .big .unit{color:var(--muted);font-size:13px}
  .kv{display:flex;justify-content:space-between;color:var(--muted);font-size:14px;margin:8px 0}
  .kv b{color:var(--ink)}
  .status-warn{color:var(--warn) !important;}
  .foot{text-align:center;margin-top:24px;padding-top:16px;border-top:1px solid var(--edge);color:var(--muted);font-size:12px;opacity:0.7}
  .chart-container{height:180px;margin:16px 0;position:relative;background:rgba(0,0,0,0.1);border-radius:8px;padding:8px}
  .chart-panel {margin-top:20px;}
  /* Gauge */
  .time-interval-selector {
    text-align: right;
    margin-bottom: 16px;
  }
  .time-interval-selector label {
    color: var(--muted);
    margin-right: 8px;
    font-size: 14px;
  }
  .time-interval-selector select {
    background: var(--bg);
    border: 1px solid var(--edge);
    color: var(--ink);
    padding: 4px 8px;
    border-radius: 4px;
    font-size: 14px;
    outline: none;
  }
  .gauge-container{position:relative;width:100%;max-width:400px;margin:0 auto 24px;text-align:center}
  .track{stroke:#162233;stroke-width:18;fill:none;stroke-linecap:round}
  .bar{stroke:url(#grad);stroke-width:18;fill:none;stroke-linecap:round;transition:stroke-dashoffset .28s cubic-bezier(.2,.8,.2,1)}
  .tick{stroke:#24344a;stroke-width:2}
  .lbl{fill:var(--muted);font-size:12px}
  .centerDot{fill:url(#glowGrad)}
</style>
</head>
<body>
<div class="wrap">
  <div class="card">
  <div class="time-interval-selector">
    <label for="timeInterval">Time Range:</label>
    <select id="timeInterval">
      <option value="0.5s">30 Seconds</option>
      <option value="1m" selected>1 Minute</option>
      <option value="10m">10 Minutes</option>
      <option value="30m">30 Minutes</option>
    </select>
  </div>
  <div class="gauge-container">
      <div class="title">ESP32 Flow Monitor</div>
          <div class="status-container" style="display: flex; align-items: center; gap: 8px;">
            <div class="pill" id="status">Connecting…</div>
          </div>
    </div>

    <div class="grid">
      <div class="panel gaugeWrap">
        <svg class="gauge" viewBox="0 0 260 180" aria-label="Flow gauge">
          <defs>
            <linearGradient id="grad" x1="0%" y1="0%" x2="100%" y2="0%">
              <stop offset="0%"  stop-color="#5aa9ff"/>
              <stop offset="100%" stop-color="#7cdb84"/>
            </linearGradient>
            <radialGradient id="glowGrad" cx="50%" cy="50%" r="50%">
              <stop offset="0%" stop-color="#5aa9ff" stop-opacity=".9"/>
              <stop offset="100%" stop-color="#5aa9ff" stop-opacity="0"/>
            </radialGradient>
          </defs>

          <g transform="translate(130,140)">
            <path id="track" d="M -110 0 A 110 110 0 0 1 110 0" class="track"/>
            <path id="bar"   d="M -110 0 A 110 110 0 0 1 110 0" class="bar"
                  stroke-dasharray="346" stroke-dashoffset="346"/>
            <g id="ticks"></g>
            <text class="lbl" x="-110" y="20" text-anchor="start" id="minLbl">0</text>
            <text class="lbl" x="0" y="28" text-anchor="middle" id="midLbl">5</text>
            <text class="lbl" x="110" y="20" text-anchor="end" id="maxLbl">10</text>
            <circle r="26" class="centerDot"/>
          </g>
        </svg>
        <div class="big">
          <div class="value" id="flow">0</div>
          <div class="unit">L/min</div>
        </div>
      </div>

      <div class="panel">
        <div class="kv"><span>Frequency</span><b><span id="hz">0</span> Hz</b></div>
        <div class="kv"><span>Total volume</span><b><span id="tot">0</span> L</b></div>
        <div class="kv"><span>Last update</span><b id="ts">—</b></div>
        <div class="kv"><span>Model</span><b id="model">Pulses_per_Liter</b></div>
        <div class="kv"><span>Calibration</span><b><span id="ppl">1319</span> PPL</b></div>
        <div class="kv"><span>Status</span><b id="statusText">Normal</b></div>
      </div>
    </div>

    <div class="panel chart-panel">
      <div class="kv"><span>Flow Rate History (Last 60s)</span></div>
      <div class="chart-container">
        <canvas id="flowChart"></canvas>
      </div>
    </div>

    <div class="foot"> made by dvir </div>
  </div>
</div>

<script>
  // Configuration
  var MAX_LPM = 10; // full-scale; change if your flow goes higher
  var WS_RECONNECT_DELAY = 2000; // ms between WebSocket reconnection attempts
  
  // Time intervals in milliseconds
  var INTERVALS = {
    '0.5s': 500,
    '1m': 60000,
    '10m': 600000,
    '30m': 1800000
  };
  var currentInterval = '1m';
  var maxDataPoints = 120; // Default for 1 minute at 0.5s intervals
  
  // Initialize UI
  document.getElementById('midLbl').textContent = Math.round(MAX_LPM/2);
  document.getElementById('maxLbl').textContent = MAX_LPM;
  
  // Set up interval selector
  var intervalSelect = document.getElementById('timeInterval');
  
  // Define updateChartInterval before using it
  var updateChartInterval = function(interval) {
    currentInterval = interval;
    var intervalMs = INTERVALS[interval];
    
    // Calculate number of data points based on interval
    if (interval === '0.5s') {
      maxDataPoints = 120; // 1 minute at 0.5s intervals
    } else {
      // For longer intervals, show more data points
      maxDataPoints = 60; // Show 60 points for all other intervals
    }
    
    // Reset chart data
    if (flowChart) {
      flowChart.data.labels = Array(maxDataPoints).fill('');
      flowChart.data.datasets[0].data = [];
      flowChart.update('none');
    }
  };
  
  // Add event listener after function is defined
  intervalSelect.addEventListener('change', function(e) {
    updateChartInterval(e.target.value);
  });
  
  // Initialize chart with default interval
  updateChartInterval(currentInterval);
  
  // Make it globally available for debugging
  window.updateChartInterval = updateChartInterval;
  
  // WebSocket connection
  var ws = null;
  var reconnectTimeout = null;
  var reconnectAttempts = 0;
  var maxReconnectAttempts = 5;
  var lastMessageTime = 0;
  var messageCount = 0;
  
  var connectWebSocket = function() {
    if (ws !== null) {
      try { ws.close(); } catch(e) { console.error('Error closing previous socket:', e); }
    }
    
    var protocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
    var host = window.location.host;
    var wsUrl = protocol + host + ':81';
    console.log('Connecting to WebSocket:', wsUrl);
    
    try {
      ws = new WebSocket(wsUrl);
      
      ws.onopen = function() {
        console.log('WebSocket connected');
        document.getElementById('status').textContent = 'Connected';
        document.getElementById('status').style.color = 'var(--ok)';
        clearTimeout(reconnectTimeout);
        reconnectAttempts = 0; // Reset reconnect attempts on successful connection
      };
      
      ws.onmessage = function(event) {
        try {
          console.log('WebSocket message received:', event.data);
          var data = JSON.parse(event.data);
          updateUI(data);
        } catch (e) {
          console.error('Error processing WebSocket message:', e);
        }
      };
      
      ws.onclose = function(event) {
        console.log('WebSocket disconnected. Code:', event.code, 'Reason:', event.reason);
        document.getElementById('status').textContent = 'Disconnected';
        document.getElementById('status').style.color = 'var(--warn)';
        
        // Try to reconnect after a delay with exponential backoff
        reconnectAttempts++;
        if (reconnectAttempts <= maxReconnectAttempts) {
          var delay = Math.min(1000 * Math.pow(2, reconnectAttempts), 30000); // Max 30s delay
          console.log('Reconnecting in', delay, 'ms (attempt', reconnectAttempts, 'of', maxReconnectAttempts, ')');
          clearTimeout(reconnectTimeout);
          reconnectTimeout = setTimeout(connectWebSocket, delay);
        } else {
          console.error('Max reconnection attempts reached. Please refresh the page.');
          document.getElementById('status').textContent = 'Connection failed - Please refresh';
        }
      };
      
      ws.onerror = function(error) {
        console.error('WebSocket error:', error);
        // The error event is followed by a close event, so we'll handle reconnection there
      };
    } catch (e) {
      console.error('Error creating WebSocket:', e);
      document.getElementById('status').textContent = 'Connection error';
      document.getElementById('status').style.color = 'var(--warn)';
    }
  };
  
  // Update UI with new data
  var updateUI = function(data) {
    try {
      messageCount++;
      lastMessageTime = Date.now();
      
      // Debug log every 10th message
      if (messageCount % 10 === 0) {
        console.log('Message #' + messageCount, data);
      }
      
      // Convert integer values to float for display
      var flowValue = data.flow_lpm_int / 1000.0;  // Assuming values are multiplied by 1000 on the ESP32
      var hzValue = data.hz_int / 1000.0;
      var totalValue = data.total_liters_int / 1000.0;
      
      // Update flow value display
      var flowEl = document.getElementById('flow');
      if (flowEl) {
        flowEl.textContent = flowValue.toFixed(2);
      }
      
      // Update chart if it exists
      if (typeof flowChart !== 'undefined' && flowChart) {
        if (flowChart.data.datasets[0].data.length >= maxDataPoints) {
          flowChart.data.datasets[0].data.shift();
          flowChart.data.labels.shift();
        }
        flowChart.data.datasets[0].data.push(flowValue);
        flowChart.data.labels.push('');
        
        // Only update chart at appropriate intervals
        var nowMs = Date.now();
        if (!window.lastUpdate || (nowMs - window.lastUpdate) >= 500) {
          flowChart.update('none');
          window.lastUpdate = nowMs;
        }
      }
    
      // Update gauge if it exists
      if (typeof setGauge === 'function') {
        setGauge(flowValue);
      }
      
      // Update metrics
      var hzEl = document.getElementById('hz');
      var totEl = document.getElementById('tot');
      var tsEl = document.getElementById('ts');
      var modelEl = document.getElementById('model');
      var pplEl = document.getElementById('ppl');
      
      if (hzEl) hzEl.textContent = hzValue.toFixed(1);
      if (totEl) totEl.textContent = totalValue.toFixed(3);
      if (tsEl) tsEl.textContent = new Date(data.t_ms).toLocaleTimeString();
      if (modelEl) modelEl.textContent = data.model || 'N/A';
      if (pplEl) pplEl.textContent = data.ppl_int || 'N/A';
      
      // Update status
      var statusEl = document.getElementById('statusText');
      if (statusEl) {
        if (data.overflow) {
          statusEl.textContent = 'Overflow!';
          statusEl.className = 'status-warn';
        } else {
          statusEl.textContent = 'Normal';
          statusEl.className = '';
        }
      }
      
      // Update connection status
      var statusEl = document.getElementById('status');
      if (statusEl && data.clients !== undefined) {
        statusEl.textContent = 'Connected (' + data.clients + ')';
        statusEl.style.color = 'var(--ok)';
      }
      
    } catch (e) {
      console.error('Error updating UI:', e);
    }
  };

  // Create gauge ticks
  var ticks = document.getElementById('ticks');
  for (var i = 0; i <= 20; i++) {
    var a = Math.PI - (Math.PI * i / 20);
    var r1 = 110, r2 = (i % 5 === 0) ? 96 : 102;
    var x1 = Math.cos(a)*r1, y1 = Math.sin(a)*r1;
    var x2 = Math.cos(a)*r2, y2 = Math.sin(a)*r2;
    var line = document.createElementNS("http://www.w3.org/2000/svg","line");
    line.setAttribute("x1",x1); line.setAttribute("y1",y1);
    line.setAttribute("x2",x2); line.setAttribute("y2",y2);
    line.setAttribute("class","tick");
    ticks.appendChild(line);
  }


  // Chart setup
  var flowChartCtx = document.getElementById('flowChart').getContext('2d');
  var flowChart = new Chart(flowChartCtx, {
    type: 'line',
    data: {
      labels: Array(maxDataPoints).fill(''),
      datasets: [{
        label: 'Flow Rate (L/min)',
        data: [],
        borderColor: '#5aa9ff',
        backgroundColor: 'rgba(90, 169, 255, 0.1)',
        borderWidth: 2,
        tension: 0.1,
        pointRadius: 0,
        fill: true,
        tension: 0.3
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: {
        duration: 0
      },
      scales: {
        y: {
          min: 0,
          max: MAX_LPM,
          grid: {
            color: 'rgba(36, 52, 74, 0.5)'
          },
          ticks: {
            color: '#9bb1c9'
          }
        },
        x: {
          display: false
        }
      },
      plugins: {
        legend: {
          display: false
        },
        tooltip: {
          mode: 'index',
          intersect: false,
          callbacks: {
            label: function(context) {
              return 'Flow: ' + context.parsed.y.toFixed(2) + ' L/min';
            }
          }
        }
      }
    }
  });

  // Initialize chart with zeros
  for (var i = 0; i < maxDataPoints; i++) {
    flowChart.data.datasets[0].data.push(0);
  }
  flowChart.update();

  // Gauge setup
  var arcLen = 346;
  var bar = document.getElementById('bar');
  var flowEl = document.getElementById('flow');
  var stEl = document.getElementById('status');

  var setGauge = function(lpm) {
    var v = Math.max(0, Math.min(MAX_LPM, lpm));
    var offset = arcLen * (1 - v / MAX_LPM);
    bar.setAttribute('stroke-dashoffset', offset.toFixed(1));
    
    // Update flow value with animation
    var currentFlow = parseFloat(flowEl.textContent) || 0;
    var diff = Math.abs(v - currentFlow);
    var duration = Math.min(300, diff * 50); // Faster for small changes
    
    if (duration > 20) {
      // Animate only significant changes
      flowEl.style.transition = `color ${duration/1000}s ease-out`;
      flowEl.style.color = v > currentFlow ? 'var(--glow2)' : 'var(--glow)';
      
      setTimeout(() => {
        flowEl.textContent = v.toString();
        flowEl.style.color = '';
      }, 50);
    } else {
      flowEl.textContent = v.toString();
    }
  }
  
  // Initialize
  setGauge(0);
  
  // Connection monitor
  setInterval(function() {
    // Show warning if no messages received in the last 3 seconds
    if (lastMessageTime > 0 && (Date.now() - lastMessageTime > 3000)) {
      var statusEl = document.getElementById('status');
      if (statusEl) {
        statusEl.textContent = 'No data (reconnecting...)';
        statusEl.style.color = 'var(--warn)';
      }
    }
  }, 1000);

  // Initialize WebSocket connection when page loads
  document.addEventListener('DOMContentLoaded', function() {
    console.log('DOM fully loaded, initializing WebSocket...');
    console.log('DOM loaded, initializing...');
    
    // Add manual reconnect button
    var statusEl = document.getElementById('status');
    var statusContainer = statusEl.parentElement;
    
    var reconnectBtn = document.createElement('button');
    reconnectBtn.textContent = 'Reconnect';
    reconnectBtn.style.marginLeft = '8px';
    reconnectBtn.style.padding = '2px 8px';
    reconnectBtn.style.fontSize = '12px';
    reconnectBtn.style.border = '1px solid var(--edge)';
    reconnectBtn.style.background = 'var(--card)';
    reconnectBtn.style.color = 'var(--ink)';
    reconnectBtn.style.borderRadius = '4px';
    reconnectBtn.style.cursor = 'pointer';
    
    reconnectBtn.onclick = function() {
      console.log('Manual reconnect requested');
      reconnectAttempts = 0;
      statusEl.textContent = 'Reconnecting...';
      statusEl.style.color = 'var(--warn)';
      connectWebSocket();
    };
    
    statusContainer.appendChild(reconnectBtn);
    
    // Initial connection
    console.log('Connecting WebSocket...');
    connectWebSocket();
  });
  
  // Handle page visibility changes
  document.addEventListener('visibilitychange', function() {
    if (!document.hidden && (!ws || ws.readyState === WebSocket.CLOSED)) {
      console.log('Page became visible, reconnecting WebSocket...');
      connectWebSocket();
    }
  });
</script>
</body>
</html>
)=====";

// JSON metrics endpoint
void handleMetrics() {
  if (!is_authenticated()) {
    return;
  }
  
  // Use a single buffer for the response
  static char response[192];
  StaticJsonDocument<192> doc;
  
  getMetricsJSON(doc);
  size_t len = serializeJson(doc, response);
  
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleRoot() {
  if (!is_authenticated()) {
    return;
  }
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void setupWeb() {
  // Setup HTTP routes with authentication
  server.on("/", HTTP_GET, handleRoot);
  server.on("/metrics", HTTP_GET, handleMetrics);
  
  // Enable CORS for all routes
  server.onNotFound([]() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(404, "text/plain", "Not found");
  });
  
  // Start HTTP server
  server.begin();
  
  // Configure and start WebSocket server
  webSocket.enableHeartbeat(15000, 3000, 2);
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  // Disable debug output for production
  #ifndef DEBUG
  webSocket.disableHeartbeat();
  #endif
  
  Serial.println("HTTP server started");
  Serial.println("WebSocket server started on ws://<ip>:81");
}

// ------------ Power Management ------------
void enterLightSleep() {
  if (!DEEP_SLEEP_ENABLED) return;
  
  Serial.println("Entering light sleep mode...");
  
  // Configure GPIO34 as wakeup source
  esp_sleep_enable_ext0_wakeup(FLOW_GPIO, HIGH); // Wake up on HIGH level
  
  // Disable WiFi and BT to save power
  WiFi.mode(WIFI_OFF);
  btStop();
  
  // Enter light sleep mode
  esp_light_sleep_start();
  
  // After wakeup
  Serial.println("Woke up from light sleep");
  
  // Reinitialize WiFi and WebSocket
  startWiFi();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

// ------------ Adaptive EMA Smoothing ------------
float calculateAdaptiveAlpha(float flow_rate) {
  // Base alpha on flow rate - faster response for higher flow rates
  if (flow_rate > 5.0f) return 0.5f;  // High flow - faster response
  if (flow_rate > 1.0f) return 0.35f; // Medium flow - balanced
  return 0.2f;                        // Low flow - more smoothing
}

// ------------ Sensor Task (Core 1) ------------
void sensorTask(void *pvParameters) {
  Serial.println("Sensor task started on core " + String(xPortGetCoreID()));
  
  // Initialize PCNT
  pcnt_init_unit();
  
  // Main sensor loop
  unsigned long last_sample_ms = 0;
  
  while (true) {
    unsigned long now = millis();
    
    // Sample at 1Hz (1000ms)
    if (now - last_sample_ms >= 1000) {
      last_sample_ms = now;
      
      // Critical section - read sensor data
      portENTER_CRITICAL(&mux);
      int16_t count = 0;
      pcnt_get_counter_value(PCNT_UNIT, &count);
      pcnt_counter_clear(PCNT_UNIT);
      portEXIT_CRITICAL(&mux);

      // Calculate flow rate (simplified for example)
      float hz = count;  // Since we're sampling over 1s
      float flow_rate = hz / PULSES_PER_LITER * 60.0f;  // Convert to L/min
      
      // Update shared variables
      portENTER_CRITICAL(&mux);
      g_hz_f = hz;
      g_flow_lpm_f = flow_rate;
      
      // Update total only if there's significant flow
      if (flow_rate > 0.1f) {
        g_total_liters_f += flow_rate / 60.0f;  // Convert L/min to L/s and add to total
        last_flow_detected = now;
        flow_active = true;
      } else if (flow_active && (now - last_flow_detected > 300000)) {  // 5 minutes
        flow_active = false;
      }
      portEXIT_CRITICAL(&mux);
    }
    
    // Small delay to prevent watchdog triggers
    delay(10);
  }
}

// ------------ Sampling (Legacy, kept for reference) ------------
void sampleOnce() {
  // This function is kept for reference but not used in dual-core mode
  // All sampling is now handled in the sensorTask() function
  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNIT, &count);
  
  // Check for overflow
  if (overflow_detected) {
    Serial.println("Warning: PCNT overflow detected!");
    overflow_detected = false;
  }
  
  pcnt_counter_clear(PCNT_UNIT);

  const float gate_s = SAMPLE_MS / 1000.0f;
  const float hz = count / gate_s;

  float liters_inc = 0.0f;
  float flow_now_lpm = 0.0f;

#if USE_FREQ_MODEL
  flow_now_lpm = (HZ_PER_LPM > 0) ? (hz / HZ_PER_LPM) : 0.0f;
  liters_inc   = flow_now_lpm * (gate_s / 60.0f);
#else
  const float ppl_eff = PULSES_PER_LITER * CAL_TRIM;   // apply ±5% trim if needed
  liters_inc   = (ppl_eff > 0) ? (count / ppl_eff) : 0.0f;
  flow_now_lpm = liters_inc * (60.0f / gate_s);        // instantaneous estimate
#endif

  // Update flow activity status
  if (flow_now_lpm > 0.1f) {  // Threshold to consider flow active
    last_flow_detected = millis();
    if (!flow_active) {
      flow_active = true;
      Serial.println("Flow detected - System active");
    }
  }

  // Adaptive EMA smoothing based on flow rate
  float alpha = calculateAdaptiveAlpha(flow_now_lpm);
  g_flow_lpm_f = (alpha * flow_now_lpm) + ((1.0f - alpha) * g_flow_lpm_f);
  
  // Only update total if we have valid flow
  if (flow_now_lpm > 0.05f) {  // Small threshold to ignore noise
    g_total_liters_f += liters_inc;
  }
  
  g_hz_f = hz;

  // Broadcast to all WebSocket clients if there's a change in flow
  static unsigned long lastBroadcast = 0;
  static float lastSentFlow = -1;
  
  // Always broadcast at least every 500ms if there are clients
  bool shouldBroadcast = (millis() - lastBroadcast >= 500 && webSocket.connectedClients() > 0);
  
  // Or broadcast immediately if flow changes significantly
  if (abs(g_flow_lpm_f - lastSentFlow) > 0.01f) {
    shouldBroadcast = true;
  }
  
  if (shouldBroadcast && webSocket.connectedClients() > 0) {
    StaticJsonDocument<256> doc;  // Increased size for additional fields
    
    // Add debug info
    doc["flow_lpm"] = g_flow_lpm_f;
    doc["hz"] = g_hz_f;
    doc["total_liters"] = g_total_liters_f;
    doc["t_ms"] = millis();
    doc["overflow"] = overflow_detected;
    doc["rssi"] = WiFi.RSSI();
    doc["clients"] = webSocket.connectedClients();
    doc["free_heap"] = ESP.getFreeHeap();
    
    String json;
    if (serializeJson(doc, json) > 0) {
      if (webSocket.broadcastTXT(json)) {
        lastSentFlow = g_flow_lpm_f;
        lastBroadcast = millis();
        
        // Debug output every 2 seconds
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 2000) {
          lastDebug = millis();
          Serial.print("WS Broadcast - Flow: ");
          Serial.print(g_flow_lpm_f);
          Serial.print(" L/min, Clients: ");
          Serial.println(webSocket.connectedClients());
          Serial.print("JSON: ");
          serializeJsonPretty(doc, Serial);
          Serial.println();
        }
      } else {
        Serial.println("Error: WebSocket broadcast failed");
      }
    } else {
      Serial.println("Error: Failed to serialize JSON");
    }
  }

  // Debug output - using F() macro for strings
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 1000) {
    last_debug = millis();
    int flow_i = (int)lroundf(g_flow_lpm_f);
    int hz_i   = (int)lroundf(g_hz_f);
    int tot_i  = (int)lroundf(g_total_liters_f);
    
    // Using separate print statements with F() to avoid large format string in RAM
    Serial.print(F("Flow: "));
    Serial.print(flow_i);
    Serial.print(F(" L/min | Freq: "));
    Serial.print(hz_i);
    Serial.print(F(" Hz | Total: "));
    Serial.print(tot_i);
    Serial.print(F(" L | Clients: "));
    Serial.println(webSocket.connectedClients());
  }
  
  // Check for inactivity timeout
  if (DEEP_SLEEP_ENABLED && flow_active && (millis() - last_flow_detected > FLOW_TIMEOUT_MS)) {
    flow_active = false;
    Serial.println("No flow detected - Entering power save mode");
    enterLightSleep();
  }
}

// ------------ Arduino ------------
void setup() {
  // Initialize serial
  Serial.begin(115200);
  delay(250);
  
  // Print startup banner using F() macro to store strings in flash
  Serial.println(F("\n================================"));
  Serial.println(F("  ESP32 Flow Meter Pro"));
  Serial.println(F("  Version: 2.0"));
  Serial.println(F("  Features:"));
  Serial.println(F("  - WebSocket Real-time Updates"));
  Serial.println(F("  - Adaptive EMA Smoothing"));
  Serial.println(F("  - Power Management"));
  Serial.println(F("  - Overflow Protection"));
  Serial.println(F("================================"));
  
  // Print configuration
#if USE_FREQ_MODEL
  Serial.printf("Mode: Frequency model  K=%.3f Hz/LPM\n", HZ_PER_LPM);
#else
  Serial.printf("Mode: Pulses-per-Liter  PPL=%.1f  Trim=%.2f\n", PULSES_PER_LITER, CAL_TRIM);
#endif
  Serial.printf("Sample window: %lu ms\n", (unsigned long)SAMPLE_MS);
  Serial.printf("Power save: %s\n", DEEP_SLEEP_ENABLED ? "Enabled" : "Disabled");
  if (DEEP_SLEEP_ENABLED) {
    Serial.printf("  Sleep after: %d ms of inactivity\n", FLOW_TIMEOUT_MS);
  }
  
  // Initialize components
  startWiFi();
  setupWeb();
  
  // Create sensor task on core 0 (Arduino runs on core 1 by default)
  xTaskCreatePinnedToCore(
    sensorTask,    // Task function
    "SensorTask",  // Name
    10000,         // Stack size (in words)
    NULL,          // Parameters
    1,             // Priority
    &SensorTask,   // Task handle
    0              // Core (0 = core 0, 1 = core 1)
  );
  
  if (SensorTask == NULL) {
    Serial.println("Failed to create sensor task!");
  }
  
  // Configure watchdog with optimized settings
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 3000,  // Reduced from 5000ms to 3000ms for faster recovery
    .idle_core_mask = 0, // Disable WDT for idle tasks
    .trigger_panic = true // Trigger panic on timeout
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);  // Add main task to WDT
  
  last_sample_ms = millis();
  last_flow_detected = millis();
  
  Serial.println("Initialization complete. System ready.");
}

void loop() {
  static unsigned long last_wdt_feed = 0;
  static unsigned long last_status = 0;
  unsigned long now = millis();
  
  // Handle web server and WebSocket clients
  server.handleClient();
  webSocket.loop();
  
  // Update WebSocket clients with latest data
  if (webSocket.connectedClients() > 0) {
    StaticJsonDocument<128> doc;
    
    // Critical section - access shared variables
    portENTER_CRITICAL(&mux);
    doc[F("flow_lpm_int")] = (int)lroundf(g_flow_lpm_f);
    doc[F("hz_int")] = (int)lroundf(g_hz_f);
    doc[F("total_liters_int")] = (int)lroundf(g_total_liters_f);
    doc[F("t_ms")] = now;
    doc[F("model")] = F("Pulses_per_Liter");
    doc[F("ppl_int")] = (int)lroundf(PULSES_PER_LITER * CAL_TRIM);
    doc[F("overflow")] = false;  // Overflow is now handled in the ISR
    portEXIT_CRITICAL(&mux);
    
    // Send to all connected clients
    String json;
    serializeJson(doc, json);
    webSocket.broadcastTXT(json);
  }
  
  // Status update every second
  if (now - last_status >= 1000) {
    last_status = now;
    
    // Critical section - access shared variables
    portENTER_CRITICAL(&mux);
    int flow_i = (int)lroundf(g_flow_lpm_f);
    int hz_i = (int)lroundf(g_hz_f);
    int tot_i = (int)lroundf(g_total_liters_f);
    portEXIT_CRITICAL(&mux);
    
    // Print status
    Serial.print(F("Flow: "));
    Serial.print(flow_i);
    Serial.print(F(" L/min | Freq: "));
    Serial.print(hz_i);
    Serial.print(F(" Hz | Total: "));
    Serial.print(tot_i);
    Serial.print(F(" L | Clients: "));
    Serial.println(webSocket.connectedClients());
    
    // Feed the watchdog
    esp_task_wdt_reset();
  }
  
  // Small delay to prevent watchdog triggers
  delay(1);
}
