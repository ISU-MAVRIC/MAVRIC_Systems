const socket = io();

// ── State ─────────────────────────────────────────────────────────────────────

const keys = new Set();
let gamepadIndex = null;
let speedMult = 1.0;

// Last emitted command snapshot for change-detection
let lastType = null;   // 'drive' | 'pivot' | 'stop'
let lastThrottle = null;
let lastTurn = null;
let lastPivotSide = null;

// ── DOM refs ──────────────────────────────────────────────────────────────────

const elStatus    = document.getElementById('status-dot');
const elStatusTxt = document.getElementById('status-txt');
const elSpeed     = document.getElementById('speed-val');
const elThrottle  = document.getElementById('throttle-val');
const elTurn      = document.getElementById('turn-val');
const elLeftRpm   = document.getElementById('left-rpm');
const elRightRpm  = document.getElementById('right-rpm');
const elMaxVel    = document.getElementById('max-vel');
const elMaxRpm    = document.getElementById('max-rpm');
const elApply     = document.getElementById('apply-btn');
const elCfgStatus = document.getElementById('cfg-status');

// ── Connection ────────────────────────────────────────────────────────────────

socket.on('connect', () => {
    elStatus.style.background = '#4caf50';
    elStatusTxt.textContent = 'Connected';
});

socket.on('disconnect', () => {
    elStatus.style.background = '#f44336';
    elStatusTxt.textContent = 'Disconnected';
    keys.clear();
    updateDisplay(0, 0);
});

socket.on('rpm_update', (data) => {
    elLeftRpm.textContent  = data.left.toFixed(1);
    elRightRpm.textContent = data.right.toFixed(1);
});

socket.on('config_applied', (data) => {
    elCfgStatus.textContent = `Applied: ${data.max_velocity} m/s, ${data.max_rpm} RPM`;
    elCfgStatus.style.color = '#4caf50';
});

socket.on('config_error', (data) => {
    elCfgStatus.textContent = `Error: ${data.error}`;
    elCfgStatus.style.color = '#f44336';
});

// ── Keyboard ──────────────────────────────────────────────────────────────────

const SPEED_MAP = { '1': 0.1, '2': 0.2, '3': 0.3, '4': 0.4, '5': 0.5,
                    '6': 0.6, '7': 0.7, '8': 0.8, '9': 0.9, '0': 1.0 };

document.addEventListener('keydown', (e) => {
    const k = e.key.toLowerCase();
    keys.add(k);
    if (SPEED_MAP[e.key] !== undefined) {
        speedMult = SPEED_MAP[e.key];
        elSpeed.textContent = Math.round(speedMult * 100) + '%';
    }
    e.preventDefault();
});

document.addEventListener('keyup', (e) => {
    keys.delete(e.key.toLowerCase());
});

// ── Gamepad ───────────────────────────────────────────────────────────────────

window.addEventListener('gamepadconnected', (e) => {
    gamepadIndex = e.gamepad.index;
});

window.addEventListener('gamepaddisconnected', () => {
    gamepadIndex = null;
});

// ── Safety: stop when page loses focus ───────────────────────────────────────

function safeStop() {
    keys.clear();
    socket.emit('stop', {});
    lastType = 'stop';
    updateDisplay(0, 0);
}

window.addEventListener('blur', safeStop);
document.addEventListener('visibilitychange', () => {
    if (document.hidden) safeStop();
});

// ── Heartbeat (keeps server watchdog alive) ───────────────────────────────────

setInterval(() => socket.emit('heartbeat', {}), 100);

// ── Helpers ───────────────────────────────────────────────────────────────────

const DEADBAND = 0.05;

function applyDeadband(v) {
    return Math.abs(v) < DEADBAND ? 0 : v;
}

function updateDisplay(throttle, turn) {
    elThrottle.textContent = throttle.toFixed(2);
    elTurn.textContent     = turn.toFixed(2);
}

// ── Main game loop ────────────────────────────────────────────────────────────

setInterval(() => {
    // 1. Gamepad reading (raw, before speed scaling)
    let throttle = 0;
    let turn = 0;

    if (gamepadIndex !== null) {
        const gp = navigator.getGamepads()[gamepadIndex];
        if (gp) {
            throttle = -gp.axes[1];  // left stick Y: up = positive
            turn     =  gp.axes[0];  // left stick X: right = positive
        }
    }

    // 2. Keyboard overrides gamepad when any movement key is held
    const kbActive = keys.has('w') || keys.has('s') || keys.has('a') || keys.has('d');
    if (kbActive) {
        throttle = (keys.has('w') ? 1 : 0) - (keys.has('s') ? 1 : 0);
        turn     = (keys.has('d') ? 1 : 0) - (keys.has('a') ? 1 : 0);
    }

    // 3. Apply speed multiplier
    throttle *= speedMult;
    turn     *= speedMult;

    // 4. Deadband
    throttle = applyDeadband(throttle);
    turn     = applyDeadband(turn);

    // 5. Pivot mode: Q or E while throttle is non-zero
    if ((keys.has('q') || keys.has('e')) && throttle !== 0) {
        const side = keys.has('q') ? 'right' : 'left';
        if (lastType !== 'pivot' || lastPivotSide !== side || lastThrottle !== throttle) {
            socket.emit('pivot', { side, rate: throttle });
            lastType      = 'pivot';
            lastPivotSide = side;
            lastThrottle  = throttle;
        }
        updateDisplay(throttle, 0);
        return;
    }

    // 6. Normal arcade drive
    if (throttle === 0 && turn === 0) {
        if (lastType !== 'stop') {
            socket.emit('stop', {});
            lastType = 'stop';
        }
    } else if (throttle !== lastThrottle || turn !== lastTurn || lastType !== 'drive') {
        socket.emit('drive', { throttle, turn });
        lastType     = 'drive';
        lastThrottle = throttle;
        lastTurn     = turn;
    }

    updateDisplay(throttle, turn);
}, 50);

// ── Config apply ──────────────────────────────────────────────────────────────

elApply.addEventListener('click', () => {
    elCfgStatus.textContent = 'Applying…';
    elCfgStatus.style.color = '#aaa';
    socket.emit('update_config', {
        max_velocity: parseFloat(elMaxVel.value),
        max_rpm:      parseFloat(elMaxRpm.value),
    });
});
