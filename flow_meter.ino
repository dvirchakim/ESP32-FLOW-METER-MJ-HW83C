/*
  ESP32 Flow Monitor (MJ-HW83C on GPIO 34)
  - PCNT pulse counting
  - Integer metrics (flow L/min, Hz, total liters)
  - Modern web gauge (live JSON polling)
  - Serial Plotter prints integers for easy reading
  - Deep sleep with wake on button press (GPIO 0)

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
#include "driver/pcnt.h"
#include <ESPmDNS.h>
#include <esp_sleep.h>

// ------------ Wi-Fi ------------
static const char* WIFI_SSID     = "NAME";
static const char* WIFI_PASSWORD = "PASS";
static const char* AP_SSID       = "ESP32-Flow";
static const char* AP_PASS       = "12345678";
static const char* MDNS_NAME     = "esp32flow";

// ------------ Flow input ------------
static const gpio_num_t FLOW_GPIO = GPIO_NUM_34;

// ------------ Sleep button ------------
static const gpio_num_t SLEEP_BUTTON_GPIO = GPIO_NUM_0;
RTC_DATA_ATTR static int sleep_flag = 0;

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
  pinMode((int)FLOW_GPIO, INPUT); // sensor gives push-pull pulses per user

  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = FLOW_GPIO;
  cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
  cfg.pos_mode       = PCNT_COUNT_INC;
  cfg.neg_mode       = PCNT_COUNT_DIS;
  cfg.lctrl_mode     = PCNT_MODE_KEEP;
  cfg.hctrl_mode     = PCNT_MODE_KEEP;
  cfg.counter_h_lim  = 32767;
  cfg.counter_l_lim  = -32768;
  cfg.unit           = PCNT_UNIT;
  cfg.channel        = PCNT_CHANNEL_0;

  pcnt_unit_config(&cfg);

  if (PCNT_GLITCH_FILTER > 0) {
    pcnt_set_filter_value(PCNT_UNIT, PCNT_GLITCH_FILTER);
    pcnt_filter_enable(PCNT_UNIT);
  } else {
    pcnt_filter_disable(PCNT_UNIT);
  }

  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
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
      </div>
    </div>

    <div class="foot"> made by dvir </div>
  </div>
</div>

<script>
  const MAX_LPM = 10; // full-scale; change if your flow goes higher
  document.getElementById('midLbl').textContent = Math.round(MAX_LPM/2);
  document.getElementById('maxLbl').textContent = MAX_LPM;

  // ticks
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

  const arcLen = 346;
  const bar = document.getElementById('bar');
  const flowEl = document.getElementById('flow');
  const hzEl   = document.getElementById('hz');
  const totEl  = document.getElementById('tot');
  const tsEl   = document.getElementById('ts');
  const stEl   = document.getElementById('status');

  function setGauge(lpmInt) {
    const v = Math.max(0, Math.min(MAX_LPM, lpmInt));
    const offset = arcLen * (1 - v / MAX_LPM);
    bar.setAttribute('stroke-dashoffset', offset.toFixed(1));
    flowEl.textContent = v.toString();
  }

  async function poll() {
    try {
      const r = await fetch('/metrics', {cache:'no-store'});
      if(!r.ok) throw new Error(r.status);
      const j = await r.json();
      setGauge(j.flow_lpm_int);
      hzEl.textContent  = j.hz_int;
      totEl.textContent = j.total_liters_int;
      tsEl.textContent  = new Date(j.t_ms).toLocaleTimeString();
      stEl.textContent = 'Live';
      stEl.style.color = 'var(--ok)';
      document.getElementById('model').textContent = j.model;
      document.getElementById('ppl').textContent = j.ppl_int;
    } catch(e){
      stEl.textContent = 'Disconnected';
      stEl.style.color = 'var(--warn)';
    } finally {
      setTimeout(poll, 500);
    }
  }
  setGauge(0);
  poll();
</script>
</body>
</html>
)HTML";

// JSON metrics with integers only
void handleMetrics() {
#if USE_FREQ_MODEL
  const char* model = "Hz_per_LPM";
#else
  const char* model = "Pulses_per_Liter";
#endif

  // Build integer views
  int flow_lpm_int      = (int)lroundf(g_flow_lpm_f);
  int hz_int            = (int)lroundf(g_hz_f);
  int total_liters_int  = (int)lroundf(g_total_liters_f);
  int ppl_int           = (int)lroundf(PULSES_PER_LITER * CAL_TRIM);

  String json = "{";
  json += "\"flow_lpm_int\":" + String(flow_lpm_int) + ",";
  json += "\"hz_int\":" + String(hz_int) + ",";
  json += "\"total_liters_int\":" + String(total_liters_int) + ",";
  json += "\"t_ms\":" + String(millis()) + ",";
  json += "\"model\":\"" + String(model) + "\",";
  json += "\"ppl_int\":" + String(ppl_int);
  json += "}";
  server.send(200, "application/json", json);
}

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void setupWeb() {
  server.on("/", handleRoot);
  server.on("/metrics", handleMetrics);
  server.begin();
  Serial.println("HTTP server started");
}

// ------------ Sampling ------------
void sampleOnce() {
  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNIT, &count);
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

  // Exponential moving average for smoother gauge
  g_flow_lpm_f = (FLOW_EMA_ALPHA * flow_now_lpm) + ((1.0f - FLOW_EMA_ALPHA) * g_flow_lpm_f);
  g_total_liters_f += liters_inc;
  g_hz_f = hz;

  // Serial Plotter integers
  int flow_i = (int)lroundf(g_flow_lpm_f);
  int hz_i   = (int)lroundf(g_hz_f);
  int tot_i  = (int)lroundf(g_total_liters_f);
  Serial.printf("flow_lpm:%d\thz:%d\ttotal_liters:%d\n", flow_i, hz_i, tot_i);
}

// ------------ Arduino ------------
void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.println();

  // Check if woken from deep sleep
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    sleep_flag = 0; // reset flag on wake
    Serial.println("Woken from deep sleep");
  }
  Serial.println(F("ESP32 Flow Meter + Modern Integer Gauge"));
#if USE_FREQ_MODEL
  Serial.printf("Mode: Frequency model  K=%.3f Hz/LPM\n", HZ_PER_LPM);
#else
  Serial.printf("Mode: Pulses-per-Liter  PPL=%.1f  Trim=%.2f\n", PULSES_PER_LITER, CAL_TRIM);
#endif
  Serial.printf("Sample window: %lu ms\n", (unsigned long)SAMPLE_MS);

  pcnt_init_unit();
  pinMode(SLEEP_BUTTON_GPIO, INPUT_PULLUP);
  startWiFi();
  setupWeb();

  last_sample_ms = millis();
}

void loop() {
  // Check for sleep button press
  if (digitalRead(SLEEP_BUTTON_GPIO) == LOW && sleep_flag == 0) {
    sleep_flag = 1;
    Serial.println("Sleep button pressed, entering deep sleep...");
    esp_deep_sleep_enable_ext0_wakeup(SLEEP_BUTTON_GPIO, 0); // wake on low
    esp_deep_sleep_start();
  }

  server.handleClient();

  unsigned long now = millis();
  if (now - last_sample_ms >= SAMPLE_MS) {
    last_sample_ms = now;
    sampleOnce();
  }
}
