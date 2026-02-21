const axisRadios = Array.from(document.querySelectorAll('input[name="axis"]'));
const modeToggle = document.getElementById('modeToggle');
const target = document.getElementById('target');
const targetv = document.getElementById('targetv');
const speed = document.getElementById('speed');
const acc = document.getElementById('acc');
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
const singleSlots = document.getElementById('singleSlots');
const dualSection = document.getElementById('dualSection');
const dualName = document.getElementById('dualName');
const dualP1 = document.getElementById('dualP1');
const dualP2 = document.getElementById('dualP2');
const dualInfo = document.getElementById('dualInfo');
const saveDual = document.getElementById('saveDual');

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
  const text = `ID${id}\nCommand: ${commandSummary[id]}\nLive: ${liveSummary[id]}`;
  if (id === 1) {
    state1.textContent = text;
  } else if (id === 2) {
    state2.textContent = text;
  }
}

function applySlotToTarget() {
  const id = axisId();
  const val = saved[id][selectedSlot - 1];
  target.value = String(val);
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
      speed: Number(speed.value || 120),
      acc: Number(acc.value || 10),
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
    speed: Number(speed.value || 120),
    acc: Number(acc.value || 10),
  };
  if (mode === 'saved') {
    payload.slot = selectedSlot;
  } else {
    payload.percent = clamp(Number(target.value), 0, 100);
  }
  ws.send(JSON.stringify(payload));
  log(`tx (${sourceTag}) ${JSON.stringify(payload)}`);
}

target.addEventListener('input', updateTargetOutput);
target.addEventListener('change', () => {
  if (modeState() === 'manual') {
    sendMoveCommand('manual_target_change');
  }
});

axisRadios.forEach((radio) => {
  radio.addEventListener('change', () => {
    updateSlotUI();
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
      updateSlotUI();
      updateDualUI();
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

    if (msg.type === 'state') {
      const id = Number(msg.id);
      commandSummary[id] = `${Number(msg.percent).toFixed(1)}% (target=${Number(msg.target_position)} actual=${Number(msg.actual_position)} ok=${Boolean(msg.success)})`;
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
      } else {
        livePercent[id] = null;
        liveSummary[id] = `unavailable (${String(msg.message || 'no response')})`;
      }
      renderStateBox(id);
      return;
    }

    if (msg.type === 'dual_result') {
      const text = `dual slot=${msg.slot} name=${msg.name} success=${Boolean(msg.success)}`;
      log(text);
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

updateTargetOutput();
updateModeToggle();
updateModeVisibility();
updateSlotUI();
updateDualUI();
renderStateBox(1);
renderStateBox(2);
