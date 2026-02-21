const axisRadios = Array.from(document.querySelectorAll('input[name="axis"]'));
const modeToggle = document.getElementById('modeToggle');
const target = document.getElementById('target');
const targetv = document.getElementById('targetv');
const speed = document.getElementById('speed');
const acc = document.getElementById('acc');
const speedv = document.getElementById('speedv');
const accv = document.getElementById('accv');
const profileInfo = document.getElementById('profileInfo');
const saveSlotBtn = document.getElementById('saveSlot');
const saveNamesBtn = document.getElementById('saveNames');
const slotInfo = document.getElementById('slotInfo');
const logEl = document.getElementById('log');
const label1 = document.getElementById('label1');
const label2 = document.getElementById('label2');
const name1 = document.getElementById('name1');
const name2 = document.getElementById('name2');
const state1 = document.getElementById('state1');
const state2 = document.getElementById('state2');
const singleTarget = document.getElementById('singleTarget');
const dualTarget = document.getElementById('dualTarget');
const manualSetup = document.getElementById('manualSetup');
const manualJog = document.getElementById('manualJog');
const manualLimits = document.getElementById('manualLimits');
const singleSlots = document.getElementById('singleSlots');
const dualSection = document.getElementById('dualSection');
const dualName = document.getElementById('dualName');
const dualP1 = document.getElementById('dualP1');
const dualP2 = document.getElementById('dualP2');
const dualInfo = document.getElementById('dualInfo');
const saveDual = document.getElementById('saveDual');
const releaseBtn = document.getElementById('releaseBtn');
const setMiddleBtn = document.getElementById('setMiddleBtn');
const middleBtn = document.getElementById('middleBtn');
const stopBtn = document.getElementById('stopBtn');
const torqueOnBtn = document.getElementById('torqueOnBtn');
const recoverBtn = document.getElementById('recoverBtn');
const stopAllBtn = document.getElementById('stopAllBtn');
const jogStep = document.getElementById('jogStep');
const jogMinusBtn = document.getElementById('jogMinusBtn');
const jogPlusBtn = document.getElementById('jogPlusBtn');
const manualMin = document.getElementById('manualMin');
const manualMax = document.getElementById('manualMax');
const saveManualLimitsBtn = document.getElementById('saveManualLimitsBtn');

const slotButtons = Array.from(document.querySelectorAll('.slot-btn'));
const dualButtons = Array.from(document.querySelectorAll('.dual-btn'));

const modeOrder = ['manual', 'saved', 'dual'];
let modeIndex = 0;
let selectedSlot = 1;
let selectedDualSlot = 1;

const saved = {
  1: [0, 12.5, 25, 37.5, 50, 62.5, 75, 87.5, 100],
  2: [0, 12.5, 25, 37.5, 50, 62.5, 75, 87.5, 100],
};
const dualPresets = Array.from({ length: 9 }, (_, idx) => ({
  name: `Pose ${idx + 1}`,
  p1: 50.0,
  p2: 50.0,
}));
const commandSummary = { 1: 'n/a', 2: 'n/a' };
const liveSummary = { 1: 'n/a', 2: 'n/a' };
const livePercent = { 1: null, 2: null };
const manualLimitById = {
  1: { min: 0.0, max: 100.0 },
  2: { min: 0.0, max: 100.0 },
};
const motorProfile = {
  id: 'waveshare_st3215',
  name: 'Waveshare ST3215',
  speed_max: 3073,
  acc_max: 150,
};
const motorHealth = {
  1: { level: 'unknown', label: 'Unknown', detail: 'waiting for telemetry' },
  2: { level: 'unknown', label: 'Unknown', detail: 'waiting for telemetry' },
};

function log(line) {
  const ts = new Date().toISOString();
  const next = `[${ts}] ${line}\n${logEl.textContent}`;
  const lines = next.split('\n').slice(0, 320);
  logEl.textContent = lines.join('\n');
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function parsePercentInput(inputEl, fallback) {
  const raw = String(inputEl.value ?? '').trim();
  if (raw === '') {
    return clamp(Number(fallback), 0, 100);
  }
  const parsed = Number(raw);
  if (Number.isNaN(parsed)) {
    return clamp(Number(fallback), 0, 100);
  }
  return clamp(parsed, 0, 100);
}

function clampByAxisLimits(id, value) {
  const lim = manualLimitById[id] || { min: 0.0, max: 100.0 };
  return clamp(Number(value), Number(lim.min), Number(lim.max));
}

function normalizeLimits(minValue, maxValue) {
  let lo = clamp(Number(minValue), 0, 100);
  let hi = clamp(Number(maxValue), 0, 100);
  if (hi < lo) {
    const t = lo;
    lo = hi;
    hi = t;
  }
  return { min: lo, max: hi };
}

function classifyHealth(success, message) {
  if (success) {
    return { level: 'ok', label: 'OK', detail: 'telemetry valid' };
  }

  const text = String(message || '').toLowerCase();
  if (text.includes('overload')) {
    return { level: 'fault', label: 'FAULT', detail: 'overload' };
  }
  if (text.includes('overheat')) {
    return { level: 'fault', label: 'FAULT', detail: 'overheat' };
  }
  if (text.includes('input voltage') || text.includes('voltage')) {
    return { level: 'fault', label: 'FAULT', detail: 'voltage' };
  }
  if (text.includes('overele') || text.includes('overcurrent') || text.includes('current')) {
    return { level: 'fault', label: 'FAULT', detail: 'current' };
  }
  if (text.includes('angle')) {
    return { level: 'fault', label: 'FAULT', detail: 'angle' };
  }
  if (
    text.includes('no status packet')
    || text.includes('timeout')
    || text.includes('unavailable')
    || text.includes('not active')
    || text.includes('proxy')
  ) {
    return { level: 'offline', label: 'OFFLINE', detail: 'no telemetry' };
  }
  return { level: 'warn', label: 'WARN', detail: String(message || 'communication issue') };
}

function setMotorHealth(id, success, message) {
  if (!(id in motorHealth)) return;
  motorHealth[id] = classifyHealth(Boolean(success), String(message || ''));
}

function modeState() {
  return modeOrder[modeIndex];
}

function setMotorName(id, name) {
  const safe = String(name || '').trim() || `Motor ${id}`;
  if (id === 1) {
    label1.textContent = `${safe} (ID 1)`;
    name1.value = safe;
  } else if (id === 2) {
    label2.textContent = `${safe} (ID 2)`;
    name2.value = safe;
  }
}

function axisId() {
  const checked = axisRadios.find((r) => r.checked);
  return Number(checked ? checked.value : 1);
}

function updateTargetOutput() {
  targetv.value = Number(target.value).toFixed(1);
}

function speedRawFromPct() {
  const pct = clamp(Number(speed.value), 0, 100);
  const raw = Math.round((pct / 100.0) * Number(motorProfile.speed_max || 3073));
  return Math.max(1, raw);
}

function accRawFromPct() {
  const pct = clamp(Number(acc.value), 0, 100);
  const raw = Math.round((pct / 100.0) * Number(motorProfile.acc_max || 150));
  return Math.max(1, raw);
}

function updateMotionOutputs() {
  const speedPct = clamp(Number(speed.value), 0, 100);
  const accPct = clamp(Number(acc.value), 0, 100);
  speedv.value = `${speedPct.toFixed(0)}% (${speedRawFromPct()})`;
  accv.value = `${accPct.toFixed(0)}% (${accRawFromPct()})`;
  profileInfo.textContent = `Profile: ${motorProfile.name} | max speed ${motorProfile.speed_max}, max acc ${motorProfile.acc_max}`;
}

function updateModeToggle() {
  const mode = modeState();
  if (mode === 'manual') {
    modeToggle.textContent = 'Mode: Manual';
  } else if (mode === 'saved') {
    modeToggle.textContent = 'Mode: Saved Slot';
  } else {
    modeToggle.textContent = 'Mode: Dual Preset';
  }
}

function updateModeVisibility() {
  const mode = modeState();
  singleTarget.hidden = mode === 'dual';
  dualTarget.hidden = mode !== 'dual';
  manualSetup.hidden = mode !== 'manual';
  manualJog.hidden = mode !== 'manual';
  manualLimits.hidden = mode !== 'manual';
  singleSlots.hidden = mode !== 'saved';
  dualSection.hidden = mode !== 'dual';
}

function updateSlotUI() {
  const id = axisId();
  slotButtons.forEach((btn) => {
    const slot = Number(btn.dataset.slot);
    const val = saved[id][slot - 1];
    btn.textContent = `S${slot}: ${val.toFixed(1)}%`;
    btn.classList.toggle('active', slot === selectedSlot);
  });
  slotInfo.textContent = `Selected slot: ${selectedSlot} (ID ${id} = ${saved[id][selectedSlot - 1].toFixed(1)}%)`;
}

function updateDualUI() {
  dualButtons.forEach((btn) => {
    const slot = Number(btn.dataset.dualSlot);
    const p = dualPresets[slot - 1];
    btn.textContent = `D${slot} ${p.name}\nID1 ${p.p1.toFixed(1)}% / ID2 ${p.p2.toFixed(1)}%`;
    btn.classList.toggle('active', slot === selectedDualSlot);
  });

  const preset = dualPresets[selectedDualSlot - 1];
  dualName.value = preset.name;
  dualP1.value = preset.p1.toFixed(1);
  dualP2.value = preset.p2.toFixed(1);
  dualInfo.textContent = `Selected dual preset: ${selectedDualSlot} (${preset.name})`;
}

function renderStateBox(id) {
  const health = motorHealth[id] || { level: 'unknown', label: 'Unknown', detail: '' };
  const detail = String(health.detail || '').trim();
  const statusLine = detail ? `${health.label} (${detail})` : `${health.label}`;
  const text = `ID${id}\nStatus: ${statusLine}\nCommand: ${commandSummary[id]}\nLive: ${liveSummary[id]}`;
  let el = null;
  if (id === 1) {
    el = state1;
  } else if (id === 2) {
    el = state2;
  }
  if (el) {
    el.textContent = text;
    el.classList.remove('state-ok', 'state-warn', 'state-fault', 'state-offline', 'state-unknown');
    el.classList.add(`state-${health.level}`);
  }
}

function applySlotToTarget() {
  const id = axisId();
  const val = saved[id][selectedSlot - 1];
  target.value = String(val);
  updateTargetOutput();
}

function applyManualBoundsUI() {
  const id = axisId();
  const lim = manualLimitById[id] || { min: 0.0, max: 100.0 };
  const lo = clamp(Number(lim.min), 0, 100);
  const hi = clamp(Number(lim.max), lo, 100);
  target.min = String(lo);
  target.max = String(hi);
  manualMin.value = lo.toFixed(1);
  manualMax.value = hi.toFixed(1);
  target.value = String(clampByAxisLimits(id, Number(target.value)));
  updateTargetOutput();
}

function sendMoveCommand(sourceTag) {
  if (ws.readyState !== WebSocket.OPEN) {
    log(`skip send (${sourceTag}): websocket not connected`);
    return;
  }
  const mode = modeState();
  if (mode === 'dual') {
    const payload = {
      type: 'move_dual',
      slot: selectedDualSlot,
      speed: speedRawFromPct(),
      acc: accRawFromPct(),
    };
    ws.send(JSON.stringify(payload));
    log(`tx (${sourceTag}) ${JSON.stringify(payload)}`);
    return;
  }

  const id = axisId();
  const payload = {
    type: 'move',
    id,
    mode,
    speed: speedRawFromPct(),
    acc: accRawFromPct(),
  };
  if (mode === 'saved') {
    payload.slot = selectedSlot;
  } else {
    payload.percent = clampByAxisLimits(id, Number(target.value));
  }
  ws.send(JSON.stringify(payload));
  log(`tx (${sourceTag}) ${JSON.stringify(payload)}`);
}

function sendControlCommand(command) {
  if (ws.readyState !== WebSocket.OPEN) {
    log(`skip control (${command}): websocket not connected`);
    return;
  }
  const payload = {
    type: 'control',
    id: axisId(),
    command: String(command),
  };
  ws.send(JSON.stringify(payload));
  log(`tx ${JSON.stringify(payload)}`);
}

function sendStopAll() {
  if (ws.readyState !== WebSocket.OPEN) {
    log('skip stop_all: websocket not connected');
    return;
  }
  const payload = { type: 'stop_all' };
  ws.send(JSON.stringify(payload));
  log(`tx ${JSON.stringify(payload)}`);
}

function sendManualLimits(id, minValue, maxValue) {
  if (ws.readyState !== WebSocket.OPEN) {
    log('skip save manual limits: websocket not connected');
    return;
  }
  const lim = normalizeLimits(minValue, maxValue);
  const payload = {
    type: 'set_manual_limits',
    id: Number(id),
    min: lim.min,
    max: lim.max,
  };
  ws.send(JSON.stringify(payload));
  log(`tx ${JSON.stringify(payload)}`);
}

function getJogBasePercent(id) {
  const live = livePercent[id];
  if (Number.isFinite(live)) {
    return clampByAxisLimits(id, Number(live));
  }
  return clampByAxisLimits(id, Number(target.value));
}

target.addEventListener('input', updateTargetOutput);
target.addEventListener('change', () => {
  if (modeState() === 'manual') {
    sendMoveCommand('manual_target_change');
  }
});
speed.addEventListener('input', updateMotionOutputs);
acc.addEventListener('input', updateMotionOutputs);

axisRadios.forEach((radio) => {
  radio.addEventListener('change', () => {
    updateSlotUI();
    applyManualBoundsUI();
    if (modeState() === 'saved') {
      applySlotToTarget();
    }
  });
});

modeToggle.addEventListener('click', () => {
  modeIndex = (modeIndex + 1) % modeOrder.length;
  updateModeToggle();
  updateModeVisibility();
  if (modeState() === 'saved') {
    applySlotToTarget();
  }
  if (modeState() === 'dual') {
    updateDualUI();
  }
  if (modeState() === 'manual') {
    applyManualBoundsUI();
  }
});

slotButtons.forEach((btn) => {
  btn.addEventListener('click', () => {
    selectedSlot = Number(btn.dataset.slot);
    updateSlotUI();
    if (modeState() === 'saved') {
      applySlotToTarget();
      sendMoveCommand('saved_slot_click');
    }
  });
});

dualButtons.forEach((btn) => {
  btn.addEventListener('click', () => {
    selectedDualSlot = Number(btn.dataset.dualSlot);
    updateDualUI();
    if (modeState() === 'dual') {
      sendMoveCommand('dual_slot_click');
    }
  });
});

const wsProto = location.protocol === 'https:' ? 'wss' : 'ws';
const ws = new WebSocket(`${wsProto}://${location.host}/ws`);

ws.onopen = () => log('websocket connected');
ws.onclose = () => log('websocket closed');
ws.onerror = () => log('websocket error');
ws.onmessage = (event) => {
  let msg = null;
  try {
    msg = JSON.parse(event.data);
  } catch (_err) {
    log(`rx ${event.data}`);
    return;
  }
  if (msg.type !== 'live') {
    log(`rx ${event.data}`);
  }

  try {

    if (msg.type === 'hello') {
      if (msg.motor_profile && typeof msg.motor_profile === 'object') {
        const p = msg.motor_profile;
        motorProfile.id = String(p.id || motorProfile.id);
        motorProfile.name = String(p.name || motorProfile.name);
        motorProfile.speed_max = Math.max(1, Number(p.speed_max || motorProfile.speed_max));
        motorProfile.acc_max = Math.max(1, Number(p.acc_max || motorProfile.acc_max));
        updateMotionOutputs();
      }
      if (msg.names) {
        Object.entries(msg.names).forEach(([idStr, value]) => {
          setMotorName(Number(idStr), value);
        });
      }
      if (msg.positions) {
        Object.entries(msg.positions).forEach(([idStr, arr]) => {
          const id = Number(idStr);
          if (!saved[id] || !Array.isArray(arr)) {
            return;
          }
          for (let i = 0; i < Math.min(9, arr.length); i += 1) {
            saved[id][i] = clamp(Number(arr[i]), 0, 100);
          }
        });
      }
      if (msg.dual_presets) {
        Object.entries(msg.dual_presets).forEach(([slotStr, preset]) => {
          const slot = Number(slotStr);
          if (slot < 1 || slot > 9 || !preset) return;
          dualPresets[slot - 1] = {
            name: String(preset.name || `Pose ${slot}`),
            p1: clamp(Number(preset.p1 ?? 50), 0, 100),
            p2: clamp(Number(preset.p2 ?? 50), 0, 100),
          };
        });
      }
      if (msg.manual_limits) {
        Object.entries(msg.manual_limits).forEach(([idStr, lim]) => {
          const id = Number(idStr);
          if (!(id in manualLimitById) || !lim) return;
          const normalized = normalizeLimits(Number(lim.min), Number(lim.max));
          manualLimitById[id] = normalized;
        });
      }
      updateSlotUI();
      updateDualUI();
      applyManualBoundsUI();
      return;
    }

    if (msg.type === 'name') {
      setMotorName(Number(msg.id), msg.name);
      return;
    }

    if (msg.type === 'names' && msg.names) {
      Object.entries(msg.names).forEach(([idStr, value]) => {
        setMotorName(Number(idStr), value);
      });
      return;
    }

    if (msg.type === 'slot') {
      const id = Number(msg.id);
      const slot = Number(msg.slot);
      if (saved[id] && slot >= 1 && slot <= 9) {
        saved[id][slot - 1] = clamp(Number(msg.percent), 0, 100);
        updateSlotUI();
      }
      return;
    }

    if (msg.type === 'dual_slot') {
      const slot = Number(msg.slot);
      if (slot >= 1 && slot <= 9) {
        dualPresets[slot - 1] = {
          name: String(msg.name || `Pose ${slot}`),
          p1: clamp(Number(msg.p1 ?? 50), 0, 100),
          p2: clamp(Number(msg.p2 ?? 50), 0, 100),
        };
        updateDualUI();
      }
      return;
    }

    if (msg.type === 'manual_limits') {
      const id = Number(msg.id);
      if (id in manualLimitById) {
        const normalized = normalizeLimits(Number(msg.min), Number(msg.max));
        manualLimitById[id] = normalized;
        if (axisId() === id) {
          applyManualBoundsUI();
        }
      }
      return;
    }

    if (msg.type === 'state') {
      const id = Number(msg.id);
      commandSummary[id] = `${Number(msg.percent).toFixed(1)}% (target=${Number(msg.target_position)} actual=${Number(msg.actual_position)} ok=${Boolean(msg.success)})`;
      if (!Boolean(msg.success)) {
        setMotorHealth(id, false, String(msg.message || 'command failed'));
      }
      renderStateBox(id);
      if (id === 1 || id === 2) setMotorName(id, msg.name);
      return;
    }

    if (msg.type === 'live') {
      const id = Number(msg.id);
      const success = Boolean(msg.success);
      const rawPos = Number(msg.position);
      const percent = Number(msg.percent);
      if (success && Number.isFinite(percent)) {
        livePercent[id] = percent;
        liveSummary[id] = `${percent.toFixed(1)}% (raw=${rawPos})`;
        setMotorHealth(id, true, 'ok');
      } else {
        livePercent[id] = null;
        const detail = String(msg.message || 'no response');
        liveSummary[id] = `unavailable (${detail})`;
        setMotorHealth(id, false, detail);
      }
      renderStateBox(id);
      return;
    }

    if (msg.type === 'dual_result') {
      const text = `dual slot=${msg.slot} name=${msg.name} success=${Boolean(msg.success)}`;
      log(text);
      return;
    }

    if (msg.type === 'control_result') {
      const id = Number(msg.id);
      const text = `control id=${id} cmd=${msg.command} success=${Boolean(msg.success)} msg=${msg.message}`;
      log(text);
      if (id === 1 || id === 2) {
        commandSummary[id] = `setup ${String(msg.command)} (ok=${Boolean(msg.success)})`;
        if (!Boolean(msg.success)) {
          setMotorHealth(id, false, String(msg.message || 'control failed'));
        }
        renderStateBox(id);
      }
      return;
    }

    if (msg.type === 'stop_all_queued') {
      const cancelledGoals = Number(msg.cancelled_goals || 0);
      const issuedStops = Number(msg.issued_stop_commands || 0);
      log(`stop_all queued cancelled_goals=${cancelledGoals} issued_stop_commands=${issuedStops}`);
      return;
    }
  } catch (_err) {
    // keep raw logs
  }
};

saveNamesBtn.addEventListener('click', () => {
  if (ws.readyState !== WebSocket.OPEN) {
    log('skip save names: websocket not connected');
    return;
  }
  const payload = {
    type: 'set_names',
    names: {
      '1': String(name1.value || '').trim(),
      '2': String(name2.value || '').trim(),
    },
  };
  ws.send(JSON.stringify(payload));
  log(`tx ${JSON.stringify(payload)}`);
});

saveSlotBtn.addEventListener('click', () => {
  const id = axisId();
  const percent = clamp(Number(target.value), 0, 100);
  const payload = { type: 'set_slot', id, slot: selectedSlot, percent };
  ws.send(JSON.stringify(payload));
  log(`tx ${JSON.stringify(payload)}`);
});

saveDual.addEventListener('click', () => {
  if (ws.readyState !== WebSocket.OPEN) {
    log('skip save dual preset: websocket not connected');
    return;
  }
  const current = dualPresets[selectedDualSlot - 1];
  const fallbackP1 = Number.isFinite(livePercent[1]) ? livePercent[1] : current.p1;
  const fallbackP2 = Number.isFinite(livePercent[2]) ? livePercent[2] : current.p2;
  const name = String(dualName.value || '').trim() || `Pose ${selectedDualSlot}`;
  const p1 = parsePercentInput(dualP1, fallbackP1);
  const p2 = parsePercentInput(dualP2, fallbackP2);
  const payload = {
    type: 'set_dual_slot',
    slot: selectedDualSlot,
    name,
    p1,
    p2,
  };
  dualPresets[selectedDualSlot - 1] = { name, p1, p2 };
  updateDualUI();
  ws.send(JSON.stringify(payload));
  log(`tx ${JSON.stringify(payload)}`);
});

releaseBtn.addEventListener('click', () => sendControlCommand('release'));
setMiddleBtn.addEventListener('click', () => sendControlCommand('set_middle'));
middleBtn.addEventListener('click', () => sendControlCommand('middle'));
stopBtn.addEventListener('click', () => sendControlCommand('stop'));
torqueOnBtn.addEventListener('click', () => sendControlCommand('torque_on'));
recoverBtn.addEventListener('click', () => sendControlCommand('recover'));
stopAllBtn.addEventListener('click', () => sendStopAll());

jogMinusBtn.addEventListener('click', () => {
  const id = axisId();
  const step = clamp(Number(jogStep.value), 0.1, 20.0);
  jogStep.value = step.toFixed(1);
  const base = getJogBasePercent(id);
  const next = clampByAxisLimits(id, base - step);
  target.value = String(next);
  updateTargetOutput();
  sendMoveCommand('manual_jog_minus');
});

jogPlusBtn.addEventListener('click', () => {
  const id = axisId();
  const step = clamp(Number(jogStep.value), 0.1, 20.0);
  jogStep.value = step.toFixed(1);
  const base = getJogBasePercent(id);
  const next = clampByAxisLimits(id, base + step);
  target.value = String(next);
  updateTargetOutput();
  sendMoveCommand('manual_jog_plus');
});

saveManualLimitsBtn.addEventListener('click', () => {
  const id = axisId();
  const normalized = normalizeLimits(Number(manualMin.value), Number(manualMax.value));
  manualLimitById[id] = normalized;
  applyManualBoundsUI();
  sendManualLimits(id, normalized.min, normalized.max);
});

updateTargetOutput();
updateMotionOutputs();
updateModeToggle();
updateModeVisibility();
updateSlotUI();
updateDualUI();
applyManualBoundsUI();
renderStateBox(1);
renderStateBox(2);
