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
unsigned long last_flow_detected = 0;
bool flow_active = false;

// ------------ WebSocket ------------
WebSocketsServer webSocket = WebSocketsServer(81);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

// ------------ Error Handling ------------
#define MAX_OVERFLOW_COUNT 10
uint32_t overflow_count = 0;
bool overflow_detected = false;

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

// ------------ Globals ------------
static pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;
WebServer server(80);

float g_flow_lpm_f = 0.0f;   // EMA-smoothed float (internal)
float g_total_liters_f = 0.0f;
float g_hz_f = 0.0f;

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
  pcnt_isr_handler_add(PCNT_UNIT, []() {
    overflow_detected = true;
    overflow_count++;
    if (overflow_count >= MAX_OVERFLOW_COUNT) {
      // Reset counter if too many overflows
      pcnt_counter_clear(PCNT_UNIT);
      overflow_count = 0;
    }
  }, NULL);
  
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
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      // Send current state on connect
      String json = getMetricsJSON();
      webSocket.sendTXT(num, json);
      break;
    }
    case WStype_TEXT:
      // Handle incoming WebSocket messages if needed
      break;
    default:
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
String getMetricsJSON() {
  StaticJsonDocument<256> doc;
  doc["flow_lpm_int"] = (int)lroundf(g_flow_lpm_f);
  doc["hz_int"] = (int)lroundf(g_hz_f);
  doc["total_liters_int"] = (int)lroundf(g_total_liters_f);
  doc["t_ms"] = millis();
  doc["model"] = USE_FREQ_MODEL ? "Hz_per_LPM" : "Pulses_per_Liter";
  doc["ppl_int"] = (int)lroundf(PULSES_PER_LITER * CAL_TRIM);
  doc["overflow"] = overflow_detected;
  
  String json;
  serializeJson(doc, json);
  return json;
}

// ------------ Web UI ------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 Flow Monitor</title>
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
        border-radius:18px;box-shadow:0 20px 40px rgba(0,0,0,.45);padding:22px}
  .head{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}
  .title{font-size:20px;font-weight:600;letter-spacing:.2px}
  .pill{border:1px solid #284059;border-radius:999px;padding:4px 10px;font-size:12px;color:var(--muted)}
  .grid{display:grid;gap:18px;grid-template-columns:1fr; margin-top:14px}
  @media(min-width:780px){.grid{grid-template-columns:1.2fr 1fr}}
  .panel{background:var(--card);border:1px solid var(--edge);border-radius:14px;padding:18px}
  .gaugeWrap{display:grid;place-items:center;padding:10px}
  .big{display:flex;flex-direction:column;align-items:center;gap:2px;margin-top:6px}
  .big .value{font-size:56px;font-weight:800;letter-spacing:.5px}
  .big .unit{color:var(--muted);font-size:13px}
  .kv{display:flex;justify-content:space-between;color:var(--muted);font-size:14px;margin:8px 0}
  .kv b{color:var(--ink)}
  .status-warn{color:var(--warn) !important;}
  .foot{margin-top:10px;color:var(--muted);font-size:12px;text-align:right}
  /* Gauge */
  .gauge{width:100%;max-width:420px;filter: drop-shadow(0 6px 18px rgba(90,169,255,.15));}
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
    <div class="head">
      <div class="title">ESP32 Flow Monitor</div>
      <div class="pill" id="status">Connecting…</div>
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

    <div class="foot"> made by dvir </div>
  </div>
</div>

<script>
  // Configuration
  const MAX_LPM = 10; // full-scale; change if your flow goes higher
  const WS_RECONNECT_DELAY = 2000; // ms between WebSocket reconnection attempts
  
  // Initialize UI
  document.getElementById('midLbl').textContent = Math.round(MAX_LPM/2);
  document.getElementById('maxLbl').textContent = MAX_LPM;
  
  // WebSocket connection
  let ws;
  let reconnectTimeout;
  
  function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss://' : 'ws://';
    const host = window.location.host;
    ws = new WebSocket(protocol + host + ':81');
    
    ws.onopen = function() {
      console.log('WebSocket connected');
      document.getElementById('status').textContent = 'Connected';
      document.getElementById('status').style.color = 'var(--ok)';
      clearTimeout(reconnectTimeout);
    };
    
    ws.onmessage = function(event) {
      const data = JSON.parse(event.data);
      updateUI(data);
    };
    
    ws.onclose = function() {
      console.log('WebSocket disconnected');
      document.getElementById('status').textContent = 'Reconnecting...';
      document.getElementById('status').style.color = 'var(--warn)';
      
      // Try to reconnect after a delay
      clearTimeout(reconnectTimeout);
      reconnectTimeout = setTimeout(connectWebSocket, WS_RECONNECT_DELAY);
    };
    
    ws.onerror = function(error) {
      console.error('WebSocket error:', error);
      ws.close(); // Will trigger onclose
    };
  }
  
  // Update UI with new data
  function updateUI(data) {
    // Update gauge
    setGauge(data.flow_lpm_int);
    
    // Update metrics
    document.getElementById('hz').textContent = data.hz_int;
    document.getElementById('tot').textContent = data.total_liters_int;
    document.getElementById('ts').textContent = new Date(data.t_ms).toLocaleTimeString();
    document.getElementById('model').textContent = data.model;
    document.getElementById('ppl').textContent = data.ppl_int;
    
    // Update status
    const statusEl = document.getElementById('statusText');
    if (data.overflow) {
      statusEl.textContent = 'Overflow Detected!';
      statusEl.className = 'status-warn';
    } else {
      statusEl.textContent = 'Normal';
      statusEl.className = '';
    }
  }

  // Create gauge ticks
  const ticks = document.getElementById('ticks');
  for (let i = 0; i <= 20; i++) {
    const a = Math.PI - (Math.PI * i / 20);
    const r1 = 110, r2 = (i % 5 === 0) ? 96 : 102;
    const x1 = Math.cos(a)*r1, y1 = Math.sin(a)*r1;
    const x2 = Math.cos(a)*r2, y2 = Math.sin(a)*r2;
    const line = document.createElementNS("http://www.w3.org/2000/svg","line");
    line.setAttribute("x1",x1); line.setAttribute("y1",y1);
    line.setAttribute("x2",x2); line.setAttribute("y2",y2);
    line.setAttribute("class","tick");
    ticks.appendChild(line);
  }

  // Gauge setup
  const arcLen = 346;
  const bar = document.getElementById('bar');
  const flowEl = document.getElementById('flow');
  const stEl = document.getElementById('status');

  function setGauge(lpmInt) {
    const v = Math.max(0, Math.min(MAX_LPM, lpmInt));
    const offset = arcLen * (1 - v / MAX_LPM);
    bar.setAttribute('stroke-dashoffset', offset.toFixed(1));
    
    // Update flow value with animation
    const currentFlow = parseFloat(flowEl.textContent) || 0;
    const diff = Math.abs(v - currentFlow);
    const duration = Math.min(300, diff * 50); // Faster for small changes
    
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
  
  // Connect WebSocket
  connectWebSocket();
  
  // Handle page visibility changes
  document.addEventListener('visibilitychange', () => {
    if (!document.hidden && (ws === undefined || ws.readyState === WebSocket.CLOSED)) {
      connectWebSocket();
    }
  });
</script>
</body>
</html>
)HTML";

// JSON metrics endpoint
void handleMetrics() {
  if (!is_authenticated()) {
    return;
  }
  server.send(200, "application/json", getMetricsJSON());
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
  
  // Start HTTP server
  server.begin();
  
  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
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

// ------------ Sampling ------------
void sampleOnce() {
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

  // Broadcast to all WebSocket clients
  if (webSocket.connectedClients() > 0) {
    String json = getMetricsJSON();
    webSocket.broadcastTXT(json);
  }

  // Debug output
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 1000) {
    last_debug = millis();
    int flow_i = (int)lroundf(g_flow_lpm_f);
    int hz_i   = (int)lroundf(g_hz_f);
    int tot_i  = (int)lroundf(g_total_liters_f);
    Serial.printf("Flow: %d L/min | Freq: %d Hz | Total: %d L | Clients: %d\n", 
                 flow_i, hz_i, tot_i, webSocket.connectedClients());
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
  
  // Print startup banner
  Serial.println("\n================================");
  Serial.println("  ESP32 Flow Meter Pro");
  Serial.println("  Version: 2.0");
  Serial.println("  Features:");
  Serial.println("  - WebSocket Real-time Updates");
  Serial.println("  - Adaptive EMA Smoothing");
  Serial.println("  - Power Management");
  Serial.println("  - Overflow Protection");
  Serial.println("================================");
  
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
  pcnt_init_unit();
  startWiFi();
  setupWeb();
  
  // Configure watchdog
  esp_task_wdt_init(10, true); // Enable panic so ESP32 restarts on watchdog timeout
  esp_task_wdt_add(NULL);      // Add current thread to WDT watch
  
  last_sample_ms = millis();
  last_flow_detected = millis();
  
  Serial.println("Initialization complete. System ready.");
}

void loop() {
  static unsigned long last_wdt_feed = 0;
  unsigned long now = millis();
  
  // Handle web server and WebSocket clients
  server.handleClient();
  webSocket.loop();
  
  // Handle sampling
  if (now - last_sample_ms >= SAMPLE_MS) {
    last_sample_ms = now;
    sampleOnce();
  }
  
  // Feed the watchdog periodically
  if (now - last_wdt_feed > 1000) {
    esp_task_wdt_reset();
    last_wdt_feed = now;
  }
  
  // Small delay to prevent watchdog triggers
  delay(1);
}
