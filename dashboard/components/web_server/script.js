const $ = id => document.getElementById(id);

const consoleOut = $("console-out");
const consoleIn  = $("console-in");

function consoleLog(text) {
  consoleOut.textContent += text + "\n";
  consoleOut.scrollTop = consoleOut.scrollHeight;
}

function rows(obj) {
  return Object.entries(obj).map(([k, v]) =>
    `<div class="row"><span>${k}</span><span>${v}</span></div>`).join("");
}

function ledColor(led) {
  if (!led) return "#000";
  const m = Math.max(led.r, led.g, led.b);
  if (m === 0) return "#000";          // off
  const k = 255 / m;                   // scale brightest channel to full
  return `rgb(${Math.round(led.r * k)},${Math.round(led.g * k)},${Math.round(led.b * k)})`;
}

function render(d) {
  // IR receivers with simple bars (assumes ~0..4095 ADC range)
  $("ir").innerHTML = (d.ir || []).map((v, i) =>
    `<div class="row"><span>IR${i + 1}</span><span>${v}</span></div>
     <div class="bar"><div style="width:${Math.min(100, v / 40.95)}%"></div></div>`).join("");

  $("gyro").innerHTML =
    `<div class="row"><span>Z (°/s)</span><span>${d.gyro ? d.gyro.Z : "—"}</span></div>
     <div class="row"><span>Yaw (°)</span><span>${d.Yaw ?? "—"}</span></div>`;

  $("pose").innerHTML   = d.pose  ? rows(d.pose)  : "";
  $("odom").innerHTML   = d.odom  ? rows(d.odom)  : "";
  $("speed").innerHTML  = d.speed ? rows(d.speed) : "";
  $("motion").innerHTML =
    `<div class="row"><span>Linear (m/s)</span><span>${d.v ?? "—"}</span></div>
     <div class="row"><span>Angular (°/s)</span><span>${d.w ?? "—"}</span></div>`;

  if (d.btn) {
    $("btn").innerHTML = Object.entries(d.btn).map(([k, v]) =>
      `<div class="row"><span>${k}</span>
       <span class="pill ${v ? 'on' : 'off'}">${v ? 'PRESSED' : '—'}</span></div>`).join("");
  }
  if (d.dip !== undefined) {
    $("dip").innerHTML = [0, 1, 2].map(i =>
      `<div class="row"><span>DIP${i + 1}</span>
       <span class="pill ${(d.dip >> i) & 1 ? 'on' : 'off'}">${(d.dip >> i) & 1 ? 'ON' : 'OFF'}</span></div>`).join("");
  }
  if (d.led) {
    $("led").style.background = ledColor(d.led);
  }
}

async function poll() {
  try {
    const r = await fetch("/telemetry");
    const d = await r.json();
    render(d);
    $("status").textContent = "● live";
    $("status").style.color = "#2ea043";
  } catch (e) {
    $("status").textContent = "● nema podataka";
    $("status").style.color = "#d29922";
  }
}

async function sendCommand(cmd) {
  try {
    await fetch("/command", { method: "POST", body: cmd });
    consoleLog("> " + cmd);
    return true;
  } catch (e) {
    consoleLog("! slanje nije uspjelo: " + cmd);
    return false;
  }
}

const cmdHistory = [];
let histIdx = -1;
let histDraft = "";

consoleIn.addEventListener("keydown", (e) => {
  if (e.key === "Enter" && consoleIn.value.trim()) {
    const cmd = consoleIn.value.trim();
    consoleIn.value = "";
    if (cmdHistory[cmdHistory.length - 1] !== cmd) cmdHistory.push(cmd);
    histIdx = -1;

    if (cmd.toLowerCase() === "clear")
      consoleOut.textContent = "";
    else
      sendCommand(cmd.toLowerCase());
  }
  else if (e.key === "ArrowUp") {
    e.preventDefault();                      // keep cursor from jumping to start
    if (cmdHistory.length === 0) return;
    if (histIdx === -1) {
      histDraft = consoleIn.value;           // remember unfinished input
      histIdx = cmdHistory.length - 1;
    } else if (histIdx > 0) {
      histIdx--;
    }
    consoleIn.value = cmdHistory[histIdx];
  }
  else if (e.key === "ArrowDown") {
    e.preventDefault();
    if (histIdx === -1) return;
    histIdx++;
    if (histIdx >= cmdHistory.length) {
      histIdx = -1;
      consoleIn.value = histDraft;           // back to the unfinished input
    } else {
      consoleIn.value = cmdHistory[histIdx];
    }
  }
});

$("btn-reset-odom").addEventListener("click", () => sendCommand("reset_odom"));

setInterval(poll, 50);
poll();