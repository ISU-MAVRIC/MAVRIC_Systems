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
let avoidanceEnabled = false;

// Mobile touch state
let joyActive = false;
let touchThrottle = 0;
let touchTurn = 0;

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

function setCenterDistance(centerM) {
    const text = (centerM === null || centerM === undefined) ? '--' : Number(centerM).toFixed(2);
    document.querySelectorAll('.js-center-distance').forEach(el => el.textContent = text);
}

function formatBand(band) {
    if (!band) return '--';
    const value = band.min_m;
    if (value === null || value === undefined) return '--';
    return Number(value).toFixed(2);
}

function depthHealthColor(health) {
    if (health === null || health === undefined) return '#555';
    if (health >= 0.5) return '#4caf50';
    if (health >= 0.2) return '#ffb74d';
    return '#f44336';
}

function setCorridor(data) {
    document.querySelectorAll('.js-band-left').forEach(el   => el.textContent = formatBand(data.left));
    document.querySelectorAll('.js-band-center').forEach(el => el.textContent = formatBand(data.center));
    document.querySelectorAll('.js-band-right').forEach(el  => el.textContent = formatBand(data.right));
    const color = depthHealthColor(data.depth_health);
    const title = (data.depth_health === null || data.depth_health === undefined)
        ? 'Depth health: unknown'
        : `Depth health: ${(data.depth_health * 100).toFixed(0)}% worst-band valid pixels`;
    document.querySelectorAll('.js-depth-health-dot').forEach(el => {
        el.style.background = color;
        el.title = title;
    });
}

function setCameraStatus(data) {
    const label = data.simulated ? `Simulated: ${data.message}` : data.message;
    document.querySelectorAll('.js-camera-status').forEach(el => {
        el.textContent = label;
        el.classList.toggle('warn', data.simulated || !data.avoidance_usable);
    });
    document.querySelectorAll('.js-avoidance-toggle').forEach(el => {
        el.disabled = !data.avoidance_usable;
        if (!data.avoidance_usable) el.checked = false;
    });
}

function setAvoidanceStatus(data) {
    avoidanceEnabled = data.enabled;
    document.querySelectorAll('.js-avoidance-toggle').forEach(el => {
        el.checked = data.enabled;
    });
    document.querySelectorAll('.js-avoidance-state').forEach(el => {
        el.textContent = `${data.state}: ${data.reason}`;
    });
    const pivotText = data.pivot_side ? data.pivot_side : '—';
    document.querySelectorAll('.js-avoidance-pivot').forEach(el => {
        el.textContent = pivotText;
    });
    const attemptsText = (data.attempts === undefined || data.attempts === null)
        ? '0'
        : String(data.attempts);
    document.querySelectorAll('.js-avoidance-attempts').forEach(el => {
        el.textContent = attemptsText;
    });
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
socket.on('camera_status', setCameraStatus);
socket.on('distance_update', (data) => setCenterDistance(data.center_m));
socket.on('corridor_update', setCorridor);
socket.on('avoidance_status', setAvoidanceStatus);

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

function isEditableTarget(target) {
    return target instanceof HTMLInputElement
        || target instanceof HTMLTextAreaElement
        || target instanceof HTMLSelectElement
        || target.isContentEditable;
}

document.addEventListener('keydown', (e) => {
    if (isEditableTarget(e.target)) return;

    keys.add(e.key.toLowerCase());
    if (SPEED_MAP[e.key] !== undefined) {
        speedMult = SPEED_MAP[e.key];
        setSpeedDisplay(Math.round(speedMult * 100));
        const slider = document.getElementById('speed-slider');
        if (slider) slider.value = Math.round(speedMult * 100);
    }
    e.preventDefault();
});

document.addEventListener('keyup', (e) => {
    if (isEditableTarget(e.target)) return;
    keys.delete(e.key.toLowerCase());
});

// ── Gamepad ───────────────────────────────────────────────────────────────────

window.addEventListener('gamepadconnected',    (e) => { gamepadIndex = e.gamepad.index; });
window.addEventListener('gamepaddisconnected', ()  => { gamepadIndex = null; });

// ── Safety: stop on focus loss ────────────────────────────────────────────────

function safeStop() {
    keys.clear();
    resetJoy();
    socket.emit('stop', {});
    avoidanceEnabled = false;
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

function getJoyRadius(joyZone) {
    const zoneRect = joyZone.getBoundingClientRect();
    const knobRect = joyKnob.getBoundingClientRect();
    return Math.max(1, (Math.min(zoneRect.width, zoneRect.height) - Math.max(knobRect.width, knobRect.height)) / 2);
}

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
        const joyRadius = getJoyRadius(joyZone);
        let dx = cx - (r.left + r.width  / 2);
        let dy = cy - (r.top  + r.height / 2);
        const dist = Math.hypot(dx, dy);
        if (dist > joyRadius) { dx = dx / dist * joyRadius; dy = dy / dist * joyRadius; }
        touchTurn     =  dx / joyRadius;
        touchThrottle = -dy / joyRadius;
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

    // Stop button
    document.getElementById('stop-btn').addEventListener('touchstart', (e) => {
        e.preventDefault();
        resetJoy();
        socket.emit('stop', {});
        avoidanceEnabled = false;
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

function adjustStepper(el) {
    const input = document.getElementById(el.dataset.stepTarget);
    if (!input) return;
    const direction = Number(el.dataset.stepDir);
    const stepText = input.getAttribute('step') || '1';
    const step = Number(stepText);
    const min = Number(input.getAttribute('min') || Number.NEGATIVE_INFINITY);
    const max = Number(input.getAttribute('max') || Number.POSITIVE_INFINITY);
    const current = Number(input.value || input.defaultValue || 0);
    const decimals = stepText.includes('.') ? stepText.split('.')[1].length : 0;
    const next = Math.min(max, Math.max(min, current + direction * step));
    input.value = decimals > 0 ? next.toFixed(decimals) : String(Math.round(next));
    input.dispatchEvent(new Event('input', { bubbles: true }));
    input.dispatchEvent(new Event('change', { bubbles: true }));
}

document.querySelectorAll('.stepper-btn').forEach((el) => {
    el.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        adjustStepper(el);
    });

    el.addEventListener('click', (e) => {
        e.preventDefault();
    });
});

document.querySelectorAll('.js-avoidance-toggle').forEach((el) => {
    el.addEventListener('change', (e) => {
        avoidanceEnabled = e.target.checked;
        socket.emit('set_avoidance', { enabled: avoidanceEnabled });
        if (avoidanceEnabled) {
            keys.clear();
            resetJoy();
            lastType = 'avoidance';
            updateDisplay(0, 0);
        }
    });
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
    if (avoidanceEnabled) {
        updateDisplay(0, 0);
        return;
    }

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

    // Pivot: keyboard Q/E
    const pivotSide = keys.has('q') ? 'right' : keys.has('e') ? 'left' : null;

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
