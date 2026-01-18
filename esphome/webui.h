#pragma once
#include "esphome.h"
#include "esp_task_wdt.h"
#include "esphome/components/web_server_base/web_server_base.h"

using namespace esphome;

void register_heizungs_ui() {
  auto *web = web_server_base::global_web_server_base;
  if (!web) {
    ESP_LOGW("heizui", "WebServerBase nicht gefunden – UI nicht registriert");
    return;
  }

  auto server_sp = web->get_server();
  if (!server_sp) {
    ESP_LOGW("heizui", "AsyncWebServer nicht verfügbar");
    return;
  }
  AsyncWebServer *server = server_sp.get();

  // ------------------------------------------------------------
  // 1) HTML-Oberfläche mit Tabs
  // ------------------------------------------------------------
  server->on("/ui", HTTP_GET, [](AsyncWebServerRequest *request) {
    const char html[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Heizungssteuerung</title>
  <style>
    body { font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; background:#f3f4f7; margin:0; }
    header { background:#1565c0; color:#fff; text-align:center; padding:1rem; font-size:1.25rem; }
    main { max-width:1000px; margin:0 auto; padding:1rem; }
    .tabs { display:flex; gap:.5rem; margin-bottom:1rem; flex-wrap:wrap; }
    .tab-btn {
      background:#e0e0e0; border:none; border-radius:10px; padding:.5rem 1rem; cursor:pointer;
    }
    .tab-btn.active { background:#fff; box-shadow:0 2px 6px rgba(0,0,0,.1); }
    .card { background:#fff; border-radius:12px; padding:1rem 1.5rem; margin:1rem 0; box-shadow:0 2px 8px rgba(0,0,0,.08); }
    .title { font-weight:600; color:#1976d2; margin-bottom:.5rem; }
    .grid { display:grid; gap:.7rem; grid-template-columns:repeat(auto-fit,minmax(140px,1fr)); }
    .btn { border:2px solid #ccc; border-radius:10px; padding:.6rem; text-align:center; background:#fafafa; cursor:pointer; }
    .btn.active { background:#1976d2; border-color:#1976d2; color:white; }
    .temp { font-size:1.4rem; font-weight:600; }
    .label { color:#666; font-size:0.9rem; }
    .flex { display:flex; justify-content:space-between; gap:1rem; align-items:center; }
    input[type=range] { width:100%; }
    footer { text-align:center; color:#888; font-size:.75rem; margin:1rem 0 2rem; }
  </style>
</head>
<body>
<header>🌡️ Heizungssteuerung (ESP)</header>
<main>
  <div class="tabs">
    <button class="tab-btn active" data-tab="overview" onclick="showTab('overview', this)">Übersicht</button>
    <button class="tab-btn" data-tab="curve" onclick="showTab('curve', this)">Heizkurve</button>
    <button class="tab-btn" data-tab="ww" onclick="showTab('ww', this)">Warmwasser</button>
    <button class="tab-btn" data-tab="diag" onclick="showTab('diag', this)">Diagnose</button>
  </div>

  <!-- Übersicht -->
  <section id="tab-overview">
    <div class="card">
      <div class="title">Betriebsmodus</div>
      <div class="grid">
        <div class="btn" id="btn-auto"   onclick="setMode('auto')">Auto</div>
        <div class="btn" id="btn-winter" onclick="setMode('winter')">Winter</div>
        <div class="btn" id="btn-summer" onclick="setMode('summer')">Sommer</div>
        <div class="btn" id="btn-pool"   onclick="setMode('pool')">Pool</div>
      </div>
      <div id="mode_info" style="margin-top:.7rem;color:#555;font-size:.9rem;">–</div>
    </div>

    <div class="card">
      <div class="title">Kerntemperaturen</div>
      <div class="grid" id="temps"></div>
    </div>

    <div class="card">
      <div class="title">Pumpen & Brenner</div>
      <div class="grid" id="actors"></div>
    </div>

    <div class="card">
      <div class="title">Mischerposition</div>
      <div id="mischer" style="font-size:1.6rem;font-weight:600;color:#1976d2;">-- %</div>
    </div>
  </section>

  <!-- Heizkurve -->
  <section id="tab-curve" style="display:none">
    <div class="card">
      <div class="title">Vorlauf</div>
      <div class="flex">
        <div>Ist: <b id="vl_ist">-- °C</b></div>
        <div>Soll (Kurve): <b id="vl_soll">-- °C</b></div>
      </div>
      <div style="margin-top:1rem;">
        <label>Ziel-Vorlauf (manuell):</label>
        <input type="range" id="target" min="20" max="70" step="0.5" oninput="updateTargetLabel(this.value)" onchange="sendTarget(this.value)">
        <div id="target_label">-- °C</div>
      </div>
    </div>
  </section>

  <!-- Warmwasser -->
  <section id="tab-ww" style="display:none">
    <div class="card">
      <div class="title">Warmwasser</div>
      <div class="flex">
        <div>Ist: <b id="ww_ist">-- °C</b></div>
        <div>Soll: <b id="ww_soll_label">-- °C</b></div>
      </div>
      <div style="margin-top:1rem;">
        <label>WW-Solltemperatur:</label>
        <input type="range" id="ww_soll" min="30" max="70" step="0.5" oninput="updateWwLabel(this.value)" onchange="sendWwTarget(this.value)">
      </div>
      <div style="margin-top:1rem;">
        <div class="btn" onclick="toggleActor('Warmwasserpumpe')">Warmwasserpumpe toggeln</div>
      </div>
    </div>
  </section>

  <!-- Diagnose -->
  <section id="tab-diag" style="display:none">
    <div class="card">
      <div class="title">Diagnose</div>
      <p>Letztes Update: <span id="timestamp">--:--:--</span></p>
      <p>Aktueller Modus: <span id="diag_mode">--</span></p>
      <p>Brenner: <span id="diag_brenner">--</span></p>
    </div>
  </section>

</main>
<footer>ESPHome WebUI – ähnlich deinem HA Dashboard</footer>

<script>
function showTab(id, btn) {
  ['overview','curve','ww','diag'].forEach(t => {
    document.getElementById('tab-'+t).style.display = (t === id) ? 'block' : 'none';
  });
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
  btn.classList.add('active');
}

async function load() {
  try {
    const r = await fetch('/ui_state');
    const d = await r.json();

    // Modus-Buttons
    ['auto','winter','summer','pool'].forEach(m => {
      const el = document.getElementById('btn-'+m);
      if (el) el.classList.toggle('active', d.mode === m);
    });
    document.getElementById('mode_info').textContent = "Aktiver Modus: " + d.mode.toUpperCase();

    // Temps
    const tbox = document.getElementById('temps');
    tbox.innerHTML = '';
    for (const k in d.temps) {
      const v = d.temps[k];
      tbox.innerHTML += `<div><div class="temp">${v.toFixed(1)}°C</div><div class="label">${k}</div></div>`;
    }

    // Actors
    const abox = document.getElementById('actors');
    abox.innerHTML = '';
    for (const k in d.actors) {
      const on = d.actors[k];
      abox.innerHTML += `<div class="btn ${on ? 'active':''}" onclick="toggleActor('${k}')">${k}</div>`;
    }

    // Mischer
    document.getElementById('mischer').textContent = d.mischer.toFixed(0) + " %";

    // Heizkurve-Tab
    document.getElementById('vl_ist').textContent = d.vl_ist.toFixed(1) + " °C";
    document.getElementById('vl_soll').textContent = d.vl_soll.toFixed(1) + " °C";
    document.getElementById('target').value = d.target;
    document.getElementById('target_label').textContent = d.target.toFixed(1) + " °C";

    // Warmwasser-Tab
    document.getElementById('ww_ist').textContent = d.ww_ist.toFixed(1) + " °C";
    document.getElementById('ww_soll').value = d.ww_soll;
    document.getElementById('ww_soll_label').textContent = d.ww_soll.toFixed(1) + " °C";

    // Diagnose
    document.getElementById('timestamp').textContent = new Date().toLocaleTimeString();
    document.getElementById('diag_mode').textContent = d.mode;
    document.getElementById('diag_brenner').textContent = d.actors["Brenner"] ? "AN" : "AUS";

  } catch(e) {
    console.log(e);
  }
}

function updateTargetLabel(v) {
  document.getElementById('target_label').textContent = v + " °C";
}
async function sendTarget(v) {
  await fetch('/ui_set_target?value='+v);
}

function updateWwLabel(v) {
  document.getElementById('ww_soll_label').textContent = parseFloat(v).toFixed(1) + " °C";
}
async function sendWwTarget(v) {
  await fetch('/ui_set_ww_target?value='+v);
}

async function setMode(m) {
  await fetch('/ui_set_mode?value='+m);
  load();
}

async function toggleActor(name) {
  await fetch('/ui_toggle_actor?value='+encodeURIComponent(name));
  load();
}

setInterval(load, 5000);
load();
</script>
</body>
</html>
)HTML";
    request->send(200, "text/html", html);
  });

  // ------------------------------------------------------------
  // 2) JSON-State – nur IDs verwenden, die wir sicher kennen
  // ------------------------------------------------------------
  server->on("/ui_state", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;

    // Modus
    doc["mode"] = id(heizmodus_select).state.c_str();

    // Vorlauf
    doc["vl_ist"]  = id(vorlauftemperatur).state;
    doc["vl_soll"] = id(curve_vl_target).state;
    doc["target"]  = id(zieltemperatur);

    // Warmwasser
    doc["ww_ist"]  = id(warmwassertemperatur).state;
    doc["ww_soll"] = id(warmwasser_zieltemperatur);   // float bei dir aus FRAM geladen

    // Temperaturen
    JsonObject temps = doc["temps"].to<JsonObject>();
    temps["Vorlauf"]     = id(vorlauftemperatur).state;
    temps["Warmwasser"]  = id(warmwassertemperatur).state;
    temps["Kessel"]      = id(kesseltemperatur).state;
    temps["Rücklauf"]    = id(ruecklauftemperatur).state;           // in deinem Log vorhanden

    // Aktoren
    JsonObject actors = doc["actors"].to<JsonObject>();
    actors["Heizkreispumpe"]  = id(heizkreispumpe).state;
    actors["Warmwasserpumpe"] = id(warmwasserpumpe).state;
    actors["Brenner"]         = id(brennerbetrieb_bz).state;  // read-only

    // Mischer
    doc["mischer"] = id(mischer_position);

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  // ------------------------------------------------------------
  // 3) Endpunkte für Aktionen
  // ------------------------------------------------------------
  server->on("/ui_set_mode", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("value")) { request->send(400, "text/plain", "missing value"); return; }
    String v = request->getParam("value")->value();
    id(heizmodus_select).publish_state(v.c_str());
    request->send(200, "text/plain", "ok");
  });

  server->on("/ui_set_target", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("value")) { request->send(400, "text/plain", "missing value"); return; }
    float v = request->getParam("value")->value().toFloat();
    id(zieltemperatur) = v;
    request->send(200, "text/plain", "ok");
  });

  server->on("/ui_set_ww_target", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("value")) { request->send(400, "text/plain", "missing value"); return; }
    float v = request->getParam("value")->value().toFloat();
    id(warmwasser_zieltemperatur) = v;
    request->send(200, "text/plain", "ok");
  });

  server->on("/ui_toggle_actor", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("value")) { request->send(400, "text/plain", "missing value"); return; }
    String name = request->getParam("value")->value();
    if (name == "Heizkreispumpe") {
      id(heizkreispumpe).toggle();
    } else if (name == "Warmwasserpumpe") {
      id(warmwasserpumpe).toggle();
    }
    // Brenner ist binary_sensor -> nicht toggeln
    request->send(200, "text/plain", "ok");
  });

  ESP_LOGI("heizui", "Erweitertes WebUI mit Tabs unter /ui aktiviert");
}
