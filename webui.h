#pragma once
#include "esphome.h"
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

  // eigene Seite unter /ui
  server->on("/ui", HTTP_GET, [](AsyncWebServerRequest *request) {
    const char html[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Heizungssteuerung</title>
  <style>
    body { font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; background:#f4f6f8; margin:0; }
    header { background:#1565c0; color:#fff; padding:1rem; text-align:center; font-size:1.3rem; }
    main { max-width:1000px; margin:0 auto; padding:1rem; }
    .card { background:#fff; border-radius:14px; padding:1rem 1.5rem; margin:1rem 0; box-shadow:0 2px 8px rgba(0,0,0,.08); }
    .title { font-weight:600; margin-bottom:.5rem; color:#1976d2; }
    .grid { display:grid; gap:.7rem; grid-template-columns:repeat(auto-fit,minmax(140px,1fr)); }
    .btn { border:2px solid #ddd; border-radius:10px; padding:.6rem; text-align:center; background:#f9f9f9; cursor:pointer; }
    .btn.active { background:#1976d2; border-color:#1976d2; color:white; }
    .temp { font-size:1.5rem; font-weight:600; }
    .label { color:#666; }
  </style>
</head>
<body>
<header>🌡️ Heizungssteuerung (ESP)</header>
<main>
  <div class="card">
    <div class="title">Betriebsmodus</div>
    <div class="grid">
      <div class="btn" id="btn-auto"   onclick="setMode('auto')">Auto</div>
      <div class="btn" id="btn-winter" onclick="setMode('winter')">Winter</div>
      <div class="btn" id="btn-summer" onclick="setMode('summer')">Sommer</div>
      <div class="btn" id="btn-pool"   onclick="setMode('pool')">Pool</div>
    </div>
  </div>

  <div class="card">
    <div class="title">Temperaturen</div>
    <div class="grid" id="temps"></div>
  </div>

  <div class="card">
    <div class="title">Pumpen & Brenner</div>
    <div class="grid" id="actors"></div>
  </div>

  <div class="card">
    <div class="title">Mischer</div>
    <div id="mischer">-- %</div>
  </div>
</main>

<script>
async function load() {
  try {
    const r = await fetch('/ui_state');
    const d = await r.json();
    ['auto','winter','summer','pool'].forEach(m => {
      const el = document.getElementById('btn-'+m);
      if (!el) return;
      el.classList.toggle('active', d.mode === m);
    });

    const tbox = document.getElementById('temps');
    tbox.innerHTML = '';
    for (const k in d.temps) {
      const v = d.temps[k];
      tbox.innerHTML += `<div><div class="temp">${v.toFixed(1)}°C</div><div class="label">${k}</div></div>`;
    }

    const abox = document.getElementById('actors');
    abox.innerHTML = '';
    for (const k in d.actors) {
      const on = d.actors[k];
      abox.innerHTML += `<div class="btn ${on ? 'active' : ''}">${k}</div>`;
    }

    document.getElementById('mischer').innerText = d.mischer.toFixed(0) + " %";
  } catch(e) {
    console.log(e);
  }
}

async function setMode(m) {
  await fetch('/ui_set_mode?value='+m);
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

  // JSON-State Endpoint
  server->on("/ui_state", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;

    // IDs anpassen an dein System!
    doc["mode"] = id(heizmodus_select).state.c_str();

    JsonObject temps = doc["temps"].to<JsonObject>();
    temps["Außen"] = id(outdoor).state;
    temps["Vorlauf"] = id(vorlauftemperatur).state;
    temps["Warmwasser"] = id(warmwassertemperatur).state;
    temps["Kessel"] = id(kesseltemperatur).state;

    JsonObject actors = doc["actors"].to<JsonObject>();
    actors["Heizkreispumpe"] = id(heizkreispumpe).state;
    actors["Warmwasserpumpe"] = id(warmwasserpumpe).state;
    actors["Brenner"] = id(brennerbetrieb_bz).state;

    doc["mischer"] = id(mischer_position);   // float, kein .state

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  // Modus setzen
  server->on("/ui_set_mode", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("value")) {
      request->send(400, "text/plain", "missing value");
      return;
    }
    String v = request->getParam("value")->value();
    id(heizmodus_select).publish_state(v.c_str());
    request->send(200, "text/plain", "ok");
  });

  ESP_LOGI("heizui", "Erweiterte Weboberfläche unter /ui registriert");
}