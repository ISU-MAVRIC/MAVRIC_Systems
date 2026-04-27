function createControllerSocket() {
    const handlers = new Map();
    let ws = null;
    let reconnectTimer = null;
    let reconnectDelay = 250;
    let manuallyClosed = false;

    function fire(type, data) {
        (handlers.get(type) || []).forEach((handler) => handler(data));
    }

    function websocketUrl() {
        const scheme = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        return `${scheme}//${window.location.host}/ws`;
    }

    function scheduleReconnect() {
        if (manuallyClosed || reconnectTimer) return;
        reconnectTimer = window.setTimeout(() => {
            reconnectTimer = null;
            connect();
        }, reconnectDelay);
        reconnectDelay = Math.min(reconnectDelay * 1.6, 3000);
    }

    function connect() {
        ws = new WebSocket(websocketUrl());

        ws.addEventListener('open', () => {
            reconnectDelay = 250;
            fire('connect', {});
        });

        ws.addEventListener('message', (event) => {
            let message;
            try {
                message = JSON.parse(event.data);
            } catch {
                return;
            }
            if (!message.type) return;
            fire(message.type, message);
        });

        ws.addEventListener('close', () => {
            fire('disconnect', {});
            scheduleReconnect();
        });

        ws.addEventListener('error', () => {
            if (ws) ws.close();
        });
    }

    connect();

    return {
        on(type, handler) {
            if (!handlers.has(type)) handlers.set(type, []);
            handlers.get(type).push(handler);
        },
        emit(type, payload = {}) {
            if (!ws || ws.readyState !== WebSocket.OPEN) return;
            ws.send(JSON.stringify({ type, ...payload }));
        },
        close() {
            manuallyClosed = true;
            if (reconnectTimer) window.clearTimeout(reconnectTimer);
            if (ws) ws.close();
        },
    };
}

const socket = createControllerSocket();

// ── Mobile detection ──────────────────────────────────────────────────────────
const isMobile = ('ontouchstart' in window) || (navigator.maxTouchPoints > 0);

// ── Shared state ──────────────────────────────────────────────────────────────
const keys = new Set();
let gamepadIndex = null;
let speedMult = 1.0;

let lastType = null;
let lastThrottle = null;
let lastTurn = null;
let lastPivotSide = null;

// Mobile touch state
let joyActive = false;
let touchThrottle = 0;
let touchTurn = 0;
let touchPivotSide = null;
const JOY_RADIUS = 68; // px — zone radius (100) minus knob radius (32)

// ── Multi-element updaters (desktop + mobile share class-based elements) ───────

function setStatus(color, text) {
    document.querySelectorAll('.js-status-dot').forEach(el => el.style.background = color);
    document.querySelectorAll('.js-status-txt').forEach(el => el.textContent = text);
}

function setRpm(left, right) {
    document.querySelectorAll('.js-left-rpm').forEach(el  => el.textContent = left.toFixed(1));
    document.querySelectorAll('.js-right-rpm').forEach(el => el.textContent = right.toFixed(1));
}

function setSpeedDisplay(pct) {
    document.querySelectorAll('.js-speed-val').forEach(el => el.textContent = pct);
}

function updateDisplay(throttle, turn) {
    document.querySelectorAll('.js-throttle-val').forEach(el => el.textContent = throttle.toFixed(2));
    document.querySelectorAll('.js-turn-val').forEach(el     => el.textContent = turn.toFixed(2));
}

// ── Socket events ─────────────────────────────────────────────────────────────

socket.on('connect', () => setStatus('#4caf50', 'Connected'));

socket.on('disconnect', () => {
    setStatus('#f44336', 'Disconnected');
    keys.clear();
    resetJoy();
    updateDisplay(0, 0);
});

socket.on('rpm_update', (data) => setRpm(data.left, data.right));

socket.on('config_applied', (data) => {
    const msg = `Applied: ${data.max_velocity} m/s, ${data.max_rpm} RPM`;
    const cfgStatus = document.getElementById('cfg-status');
    cfgStatus.textContent = msg;
    cfgStatus.style.color = '#4caf50';
    const d = document.getElementById('cfg-status-desktop');
    if (d) { d.textContent = msg; d.style.color = '#4caf50'; }
});

socket.on('config_error', (data) => {
    const cfgStatus = document.getElementById('cfg-status');
    cfgStatus.textContent = `Error: ${data.error}`;
    cfgStatus.style.color = '#f44336';
});

// ── Keyboard ──────────────────────────────────────────────────────────────────

const SPEED_MAP = { '1':0.1,'2':0.2,'3':0.3,'4':0.4,'5':0.5,
                    '6':0.6,'7':0.7,'8':0.8,'9':0.9,'0':1.0 };

document.addEventListener('keydown', (e) => {
    keys.add(e.key.toLowerCase());
    if (SPEED_MAP[e.key] !== undefined) {
        speedMult = SPEED_MAP[e.key];
        setSpeedDisplay(Math.round(speedMult * 100));
        const slider = document.getElementById('speed-slider');
        if (slider) slider.value = Math.round(speedMult * 100);
    }
    e.preventDefault();
});

document.addEventListener('keyup', (e) => keys.delete(e.key.toLowerCase()));

// ── Gamepad ───────────────────────────────────────────────────────────────────

window.addEventListener('gamepadconnected',    (e) => { gamepadIndex = e.gamepad.index; });
window.addEventListener('gamepaddisconnected', ()  => { gamepadIndex = null; });

// ── Safety: stop on focus loss ────────────────────────────────────────────────

function safeStop() {
    keys.clear();
    touchPivotSide = null;
    resetJoy();
    socket.emit('stop', {});
    lastType = 'stop';
    updateDisplay(0, 0);
}

window.addEventListener('blur', safeStop);
document.addEventListener('visibilitychange', () => { if (document.hidden) safeStop(); });

// ── Heartbeat (keeps server watchdog alive) ───────────────────────────────────

setInterval(() => socket.emit('heartbeat', {}), 100);

// ── Deadband ──────────────────────────────────────────────────────────────────

const DEADBAND = 0.05;
function applyDeadband(v) { return Math.abs(v) < DEADBAND ? 0 : v; }

// ── Joystick helpers ──────────────────────────────────────────────────────────

const joyKnob = document.getElementById('joy-knob');

function resetJoy() {
    joyActive = false;
    touchThrottle = 0;
    touchTurn = 0;
    if (joyKnob) {
        joyKnob.style.transform = 'translate(-50%, -50%)';
        joyKnob.classList.remove('active');
    }
}

// ── Mobile input setup ────────────────────────────────────────────────────────

if (isMobile) {
    const joyZone = document.getElementById('joy-zone');

    function moveJoy(cx, cy) {
        const r = joyZone.getBoundingClientRect();
        let dx = cx - (r.left + r.width  / 2);
        let dy = cy - (r.top  + r.height / 2);
        const dist = Math.hypot(dx, dy);
        if (dist > JOY_RADIUS) { dx = dx / dist * JOY_RADIUS; dy = dy / dist * JOY_RADIUS; }
        touchTurn     =  dx / JOY_RADIUS;
        touchThrottle = -dy / JOY_RADIUS;
        joyKnob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
    }

    joyZone.addEventListener('touchstart', (e) => {
        e.preventDefault();
        joyActive = true;
        joyKnob.classList.add('active');
        moveJoy(e.changedTouches[0].clientX, e.changedTouches[0].clientY);
    }, { passive: false });

    joyZone.addEventListener('touchmove', (e) => {
        e.preventDefault();
        if (!joyActive) return;
        moveJoy(e.changedTouches[0].clientX, e.changedTouches[0].clientY);
    }, { passive: false });

    joyZone.addEventListener('touchend',    (e) => { e.preventDefault(); resetJoy(); }, { passive: false });
    joyZone.addEventListener('touchcancel', (e) => { e.preventDefault(); resetJoy(); }, { passive: false });

    // Pivot buttons: hold = pivot, release = stop pivoting
    function attachPivot(id, side) {
        const el = document.getElementById(id);
        el.addEventListener('touchstart',  (e) => { e.preventDefault(); touchPivotSide = side; }, { passive: false });
        el.addEventListener('touchend',    (e) => { e.preventDefault(); touchPivotSide = null; }, { passive: false });
        el.addEventListener('touchcancel', (e) => { e.preventDefault(); touchPivotSide = null; }, { passive: false });
    }
    attachPivot('pivot-l', 'right'); // pivot left  = right side stationary
    attachPivot('pivot-r', 'left');  // pivot right = left  side stationary

    // Stop button
    document.getElementById('stop-btn').addEventListener('touchstart', (e) => {
        e.preventDefault();
        touchPivotSide = null;
        resetJoy();
        socket.emit('stop', {});
        lastType = 'stop';
        updateDisplay(0, 0);
    }, { passive: false });

    // Speed slider
    document.getElementById('speed-slider').addEventListener('input', (e) => {
        speedMult = parseInt(e.target.value) / 100;
        setSpeedDisplay(Math.round(speedMult * 100));
    });

    // Settings modal toggle
    document.getElementById('m-cfg-btn').addEventListener('click', () => {
        document.getElementById('settings-modal').classList.remove('hidden');
    });
}

// Desktop settings button
const desktopSettingsBtn = document.getElementById('desktop-settings-btn');
if (desktopSettingsBtn) {
    desktopSettingsBtn.addEventListener('click', () => {
        document.getElementById('settings-modal').classList.remove('hidden');
    });
}

document.getElementById('settings-close-btn').addEventListener('click', () => {
    document.getElementById('settings-modal').classList.add('hidden');
});

// ── Config apply ──────────────────────────────────────────────────────────────

document.getElementById('apply-btn').addEventListener('click', () => {
    const cfgStatus = document.getElementById('cfg-status');
    cfgStatus.textContent = 'Applying…';
    cfgStatus.style.color = '#aaa';
    socket.emit('update_config', {
        max_velocity: parseFloat(document.getElementById('max-vel').value),
        max_rpm:      parseFloat(document.getElementById('max-rpm').value),
    });
});

// ── Main game loop ────────────────────────────────────────────────────────────

setInterval(() => {
    let throttle = 0;
    let turn = 0;

    if (isMobile && joyActive) {
        // Touch joystick has priority on mobile; scale by speedMult
        throttle = touchThrottle * speedMult;
        turn     = touchTurn     * speedMult;
    } else {
        // Gamepad (raw)
        if (gamepadIndex !== null) {
            const gp = navigator.getGamepads()[gamepadIndex];
            if (gp) { throttle = -gp.axes[1]; turn = gp.axes[0]; }
        }
        // Keyboard overrides gamepad
        if (keys.has('w') || keys.has('s') || keys.has('a') || keys.has('d')) {
            throttle = (keys.has('w') ? 1 : 0) - (keys.has('s') ? 1 : 0);
            turn     = (keys.has('d') ? 1 : 0) - (keys.has('a') ? 1 : 0);
        }
        throttle *= speedMult;
        turn     *= speedMult;
    }

    throttle = applyDeadband(throttle);
    turn     = applyDeadband(turn);

    // Pivot: mobile touch buttons take priority, then keyboard Q/E
    const pivotSide = touchPivotSide
        || (keys.has('q') ? 'right' : keys.has('e') ? 'left' : null);

    if (pivotSide && throttle !== 0) {
        if (lastType !== 'pivot' || lastPivotSide !== pivotSide || lastThrottle !== throttle) {
            socket.emit('pivot', { side: pivotSide, rate: throttle });
            lastType      = 'pivot';
            lastPivotSide = pivotSide;
            lastThrottle  = throttle;
        }
        updateDisplay(throttle, 0);
        return;
    }

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
