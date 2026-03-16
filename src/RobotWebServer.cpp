#include "RobotWebServer.h"
#include "config.h"
#include <WebServer.h>
#include <WiFi.h>

// ── Internal state ──
static WebServer server(80);
static RobotWeb::Command _currentCmd = RobotWeb::CMD_NONE;
static RobotWeb::TableParams _tableParams = {0, 0, false};

// Telemetry
static float _tObs, _tYaw, _tDist;
static bool _tEdgeL, _tEdgeR, _tEdgeRL, _tEdgeRR;
static long _tTicksL, _tTicksR;
static int _tPass, _tTotal;
static String _tMode = "IDLE";

// ── HTML / CSS / JS ──
static const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>TableBot Control</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;700&display=swap');
  *{margin:0;padding:0;box-sizing:border-box}
  body{
    font-family:'Inter',sans-serif;
    background:#0f0f1a;
    color:#e0e0e0;
    min-height:100vh;
    display:flex;flex-direction:column;align-items:center;
    padding:16px;
  }
  h1{
    font-size:1.6rem;font-weight:700;
    background:linear-gradient(135deg,#6ee7b7,#3b82f6);
    -webkit-background-clip:text;-webkit-text-fill-color:transparent;
    margin-bottom:12px;
  }
  .card{
    background:rgba(255,255,255,.06);
    border:1px solid rgba(255,255,255,.08);
    border-radius:16px;
    padding:20px;
    width:100%;max-width:420px;
    margin-bottom:14px;
    backdrop-filter:blur(12px);
  }
  .card h2{font-size:.95rem;font-weight:600;color:#94a3b8;margin-bottom:12px;text-transform:uppercase;letter-spacing:.08em}

  /* D-Pad */
  .dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;max-width:220px;margin:0 auto}
  .dpad button{
    width:64px;height:56px;border:none;border-radius:12px;
    font-size:1.3rem;cursor:pointer;
    background:linear-gradient(145deg,#1e293b,#334155);
    color:#e2e8f0;
    box-shadow:0 4px 14px rgba(0,0,0,.4);
    transition:transform .1s,box-shadow .15s;
    display:flex;align-items:center;justify-content:center;
  }
  .dpad button:active{transform:scale(.93);box-shadow:0 2px 6px rgba(0,0,0,.6)}
  .dpad .empty{visibility:hidden}
  .stop-btn{
    background:linear-gradient(145deg,#dc2626,#991b1b)!important;
    font-weight:700;font-size:.85rem;
  }

  /* Auto form */
  .auto-form{display:flex;flex-direction:column;gap:10px}
  .auto-form label{font-size:.82rem;color:#94a3b8}
  .auto-form input{
    background:#1e293b;border:1px solid #334155;border-radius:8px;
    padding:8px 12px;color:#e2e8f0;font-size:.95rem;width:100%;
  }
  .btn-row{display:flex;gap:8px;margin-top:4px}
  .btn-start{
    flex:1;padding:10px;border:none;border-radius:10px;cursor:pointer;
    font-weight:600;font-size:.9rem;
    background:linear-gradient(135deg,#059669,#10b981);color:#fff;
    transition:opacity .15s;
  }
  .btn-start:hover{opacity:.85}
  .btn-stop-auto{
    flex:1;padding:10px;border:none;border-radius:10px;cursor:pointer;
    font-weight:600;font-size:.9rem;
    background:linear-gradient(135deg,#dc2626,#ef4444);color:#fff;
    transition:opacity .15s;
  }

  /* Telemetry */
  .telem{display:grid;grid-template-columns:1fr 1fr;gap:6px 14px}
  .telem .item{display:flex;justify-content:space-between;font-size:.82rem}
  .telem .label{color:#64748b}
  .telem .val{font-weight:600;font-variant-numeric:tabular-nums}
  .edge-warn{color:#f87171;font-weight:700}
  .mode-badge{
    display:inline-block;padding:3px 10px;border-radius:20px;
    font-size:.75rem;font-weight:700;text-transform:uppercase;
    letter-spacing:.06em;
  }
  .mode-MANUAL{background:#3b82f6;color:#fff}
  .mode-CLEANING{background:#10b981;color:#fff}
  .mode-AUTO{background:#10b981;color:#fff}
  .mode-IDLE{background:#475569;color:#cbd5e1}
  .mode-EDGE{background:#f87171;color:#fff}
  .mode-OBSTACLE{background:#f59e0b;color:#1e293b}
  .mode-TURNING{background:#a78bfa;color:#fff}
  .mode-ADVANCE{background:#38bdf8;color:#1e293b}
  .mode-DONE{background:#22c55e;color:#fff}

  /* Progress bar */
  .progress-bar{
    height:8px;border-radius:4px;background:#1e293b;
    overflow:hidden;margin-top:8px;
  }
  .progress-fill{
    height:100%;border-radius:4px;
    background:linear-gradient(90deg,#10b981,#6ee7b7);
    transition:width .5s ease;
  }
</style>
</head>
<body>
<h1>&#x1F9F9; TableBot Control</h1>

<!-- Manual D-Pad -->
<div class="card">
  <h2>Manual Control</h2>
  <div class="dpad">
    <div class="empty"></div>
    <button onclick="cmd('fwd')">&#x25B2;</button>
    <div class="empty"></div>
    <button onclick="cmd('left')">&#x25C0;</button>
    <button class="stop-btn" onclick="cmd('stop')">STOP</button>
    <button onclick="cmd('right')">&#x25B6;</button>
    <div class="empty"></div>
    <button onclick="cmd('bwd')">&#x25BC;</button>
    <div class="empty"></div>
  </div>
</div>

<!-- Auto Mode -->
<div class="card">
  <h2>Automatic Cleaning</h2>
  <div class="auto-form">
    <label>Table Length (cm)
      <input id="tLen" type="number" placeholder="e.g. 60" min="10">
    </label>
    <label>Table Width (cm)
      <input id="tWid" type="number" placeholder="e.g. 45" min="10">
    </label>
    <div class="btn-row">
      <button class="btn-start" onclick="startAuto()">&#x25B6; Start Cleaning</button>
      <button class="btn-stop-auto" onclick="cmd('auto_stop')">&#x23F9; Stop</button>
    </div>
  </div>
</div>

<!-- Telemetry -->
<div class="card" id="telemCard">
  <h2>Telemetry <span id="mBadge" class="mode-badge mode-IDLE">IDLE</span></h2>
  <div class="telem">
    <div class="item"><span class="label">Pass</span><span class="val" id="vPass">—</span></div>
    <div class="item"><span class="label">Obstacle</span><span class="val" id="vO">—</span></div>
    <div class="item"><span class="label">Gyro Yaw</span><span class="val" id="vYaw">—</span></div>
    <div class="item"><span class="label">Enc Dist</span><span class="val" id="vDist">—</span></div>
    <div class="item"><span class="label">Front L</span><span class="val" id="vEL">—</span></div>
    <div class="item"><span class="label">Front R</span><span class="val" id="vER">—</span></div>
    <div class="item"><span class="label">Rear L</span><span class="val" id="vRL">—</span></div>
    <div class="item"><span class="label">Rear R</span><span class="val" id="vRR">—</span></div>
    <div class="item"><span class="label">Ticks L/R</span><span class="val" id="vTicks">—</span></div>
  </div>
  <div class="progress-bar"><div class="progress-fill" id="pBar" style="width:0%"></div></div>
</div>

<script>
function cmd(c){fetch('/cmd?c='+c).catch(()=>{});}
function startAuto(){
  var l=document.getElementById('tLen').value;
  var w=document.getElementById('tWid').value;
  if(!l||!w){alert('Enter table dimensions');return;}
  fetch('/auto?l='+l+'&w='+w).catch(()=>{});
}
function poll(){
  fetch('/telemetry').then(r=>r.json()).then(d=>{
    try{
      document.getElementById('vPass').textContent=d.pass+' / '+d.total;
      document.getElementById('vYaw').textContent=(d.yaw!=null?d.yaw.toFixed(1):'?')+' °';
      document.getElementById('vO').textContent=(d.o!=null?d.o.toFixed(1):'?')+' cm';
      var elL=document.getElementById('vEL');
      var elR=document.getElementById('vER');
      var rlL=document.getElementById('vRL');
      var rlR=document.getElementById('vRR');
      elL.textContent=d.el?'⚠ EDGE':'OK';
      elR.textContent=d.er?'⚠ EDGE':'OK';
      rlL.textContent=d.rl?'⚠ EDGE':'OK';
      rlR.textContent=d.rr?'⚠ EDGE':'OK';
      elL.className='val'+(d.el?' edge-warn':'');
      elR.className='val'+(d.er?' edge-warn':'');
      rlL.className='val'+(d.rl?' edge-warn':'');
      rlR.className='val'+(d.rr?' edge-warn':'');
      document.getElementById('vDist').textContent=(d.dist!=null?d.dist.toFixed(1):'?')+' cm';
      document.getElementById('vTicks').textContent=(d.tl!=null?d.tl:'?')+' / '+(d.tr!=null?d.tr:'?');
      var pct=d.total>0?Math.round((d.pass/d.total)*100):0;
      document.getElementById('pBar').style.width=pct+'%';
      var b=document.getElementById('mBadge');
      b.textContent=d.m;
      b.className='mode-badge mode-'+d.m;
    }catch(e){console.error('Telemetry error:',e);}
  }).catch(()=>{});
}
setInterval(poll,500);
</script>
</body>
</html>
)rawliteral";

// ── Route handlers ──
static void handleRoot() { server.send(200, "text/html", PAGE_HTML); }

static void handleCmd() {
  String c = server.arg("c");
  if (c == "fwd")
    _currentCmd = RobotWeb::CMD_FWD;
  else if (c == "bwd")
    _currentCmd = RobotWeb::CMD_BWD;
  else if (c == "left")
    _currentCmd = RobotWeb::CMD_LEFT;
  else if (c == "right")
    _currentCmd = RobotWeb::CMD_RIGHT;
  else if (c == "stop")
    _currentCmd = RobotWeb::CMD_STOP;
  else if (c == "auto_stop")
    _currentCmd = RobotWeb::CMD_AUTO_STOP;
  else if (c == "ai_stop")
    _currentCmd = RobotWeb::CMD_AI_STOP;
  else if (c == "ai_resume")
    _currentCmd = RobotWeb::CMD_AI_RESUME;
  server.send(200, "text/plain", "OK");
}

static void handleAuto() {
  // Reactive mode: no table dimensions needed
  // kept for compatibility but params are ignored
  _tableParams.lengthCm = server.arg("l").toFloat();
  _tableParams.widthCm = server.arg("w").toFloat();
  _tableParams.valid = true; // reactive mode always valid
  _currentCmd = RobotWeb::CMD_AUTO_START;
  server.send(200, "text/plain", "OK");
}

static void handleTelemetry() {
  String json = "{";
  json += "\"o\":" + String(_tObs, 1) + ",";
  json += "\"el\":" + String(_tEdgeL ? "true" : "false") + ",";
  json += "\"er\":" + String(_tEdgeR ? "true" : "false") + ",";
  json += "\"rl\":" + String(_tEdgeRL ? "true" : "false") + ",";
  json += "\"rr\":" + String(_tEdgeRR ? "true" : "false") + ",";
  json += "\"yaw\":" + String(_tYaw, 1) + ",";
  json += "\"dist\":" + String(_tDist, 1) + ",";
  json += "\"tl\":" + String(_tTicksL) + ",";
  json += "\"tr\":" + String(_tTicksR) + ",";
  json += "\"pass\":" + String(_tPass) + ",";
  json += "\"total\":" + String(_tTotal) + ",";
  json += "\"m\":\"" + _tMode + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// ── Public API ──
void RobotWeb::begin() {
  WiFi.mode(WIFI_AP);
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(WIFI_SSID, WIFI_PASS, 1, 0, 4);
  Serial.print("[WiFi] AP started – IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.on("/auto", handleAuto);
  server.on("/telemetry", handleTelemetry);
  server.begin();
}

void RobotWeb::handle() { server.handleClient(); }
RobotWeb::Command RobotWeb::getCommand() { return _currentCmd; }
void RobotWeb::clearCommand() { _currentCmd = CMD_NONE; }
RobotWeb::TableParams RobotWeb::getTableParams() { return _tableParams; }

void RobotWeb::setTelemetry(float obstacleCm, bool edgeL, bool edgeR,
                            bool edgeRL, bool edgeRR,
                            float yaw, float distCm,
                            long ticksL, long ticksR,
                            int currentPass, int totalPasses,
                            const char *mode) {
  _tObs = obstacleCm;
  _tEdgeL = edgeL;
  _tEdgeR = edgeR;
  _tEdgeRL = edgeRL;
  _tEdgeRR = edgeRR;
  _tYaw = yaw;
  _tDist = distCm;
  _tTicksL = ticksL;
  _tTicksR = ticksR;
  _tPass = currentPass;
  _tTotal = totalPasses;
  _tMode = mode;
}
