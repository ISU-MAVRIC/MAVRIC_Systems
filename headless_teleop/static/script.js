// MAVRIC Rover Control Interface JavaScript

// Initialize Socket.IO connection
const socket = io();

// Track currently pressed keys
const pressedKeys = new Set();

// Status elements
const statusIndicator = document.getElementById('status-indicator');
const connectionStatus = document.getElementById('connection-status');
const activeKeysDisplay = document.getElementById('active-keys');
const emergencyStopBtn = document.getElementById('emergency-stop');

// Key mapping for special keys
const keyMap = {
    'CapsLock': 'capslock',
    'Shift': 'shift',
    'BracketLeft': '[',
    'BracketRight': ']',
};

// Valid control keys
const validKeys = new Set([
    'w', 'a', 's', 'd', 'q', 'e',  // Movement and rotation
    'y', 'h', 'u', 'j', 'i', 'k', 'c', 'v', 'z', 'x',  // Arm controls
    '[', ']',  // Claw
    'shift', 'capslock'  // Modifiers
]);

// Socket.IO event handlers
socket.on('connect', () => {
    console.log('Connected to server');
    statusIndicator.classList.add('connected');
    connectionStatus.textContent = 'Connected';
});

socket.on('disconnect', () => {
    console.log('Disconnected from server');
    statusIndicator.classList.remove('connected');
    connectionStatus.textContent = 'Disconnected';
    clearAllKeys();
});

socket.on('status', (data) => {
    console.log('Status:', data);
});

socket.on('key_state', (data) => {
    console.log('Key state update:', data);
    updateKeyDisplay(data.pressed_keys);
});

// Keyboard event handlers
document.addEventListener('keydown', (event) => {
    // Prevent default browser behavior for control keys
    if (validKeys.has(event.key.toLowerCase()) || 
        event.key === 'CapsLock' || 
        event.key === 'Shift' ||
        event.code === 'BracketLeft' ||
        event.code === 'BracketRight') {
        event.preventDefault();
    }

    // Handle ESC for emergency stop
    if (event.key === 'Escape') {
        event.preventDefault();
        emergencyStop();
        return;
    }

    // Map the key
    let key = event.key.toLowerCase();
    
    // Handle special keys
    if (keyMap[event.key]) {
        key = keyMap[event.key];
    } else if (keyMap[event.code]) {
        key = keyMap[event.code];
    }

    // Only process valid keys and avoid repeated keydown events
    if (validKeys.has(key) && !pressedKeys.has(key)) {
        pressedKeys.add(key);
        socket.emit('key_down', { key: key });
        updateKeyDisplay(Array.from(pressedKeys));
        highlightKey(key, true);
    }
});

document.addEventListener('keyup', (event) => {
    // Map the key
    let key = event.key.toLowerCase();
    
    // Handle special keys
    if (keyMap[event.key]) {
        key = keyMap[event.key];
    } else if (keyMap[event.code]) {
        key = keyMap[event.code];
    }

    // Only process valid keys
    if (validKeys.has(key) && pressedKeys.has(key)) {
        pressedKeys.delete(key);
        socket.emit('key_up', { key: key });
        updateKeyDisplay(Array.from(pressedKeys));
        highlightKey(key, false);
    }
});

// Handle window blur (lose focus) - release all keys
window.addEventListener('blur', () => {
    console.log('Window lost focus, releasing all keys');
    clearAllKeys();
});

// Emergency stop button
emergencyStopBtn.addEventListener('click', () => {
    emergencyStop();
});

// Functions
function updateKeyDisplay(keys) {
    if (keys.length === 0) {
        activeKeysDisplay.textContent = 'None';
    } else {
        activeKeysDisplay.textContent = keys.join(', ').toUpperCase();
    }
}

function highlightKey(key, active) {
    const keyElements = document.querySelectorAll(`[data-key="${key}"]`);
    keyElements.forEach(element => {
        if (active) {
            element.classList.add('active');
        } else {
            element.classList.remove('active');
        }
    });
}

function clearAllKeys() {
    // Clear local tracking
    const keysToRelease = Array.from(pressedKeys);
    pressedKeys.clear();
    
    // Remove all highlights
    keysToRelease.forEach(key => {
        highlightKey(key, false);
    });
    
    // Send emergency stop to server
    socket.emit('emergency_stop');
    updateKeyDisplay([]);
}

function emergencyStop() {
    console.log('EMERGENCY STOP ACTIVATED');
    clearAllKeys();
    
    // Visual feedback
    emergencyStopBtn.style.background = '#7f1d1d';
    setTimeout(() => {
        emergencyStopBtn.style.background = '';
    }, 500);
}

// Mouse/Touch support for key items (optional - for testing)
document.querySelectorAll('.key-item:not(.empty)').forEach(item => {
    const key = item.getAttribute('data-key');
    
    if (key) {
        // Mouse down / Touch start
        const handleStart = (e) => {
            e.preventDefault();
            if (!pressedKeys.has(key)) {
                pressedKeys.add(key);
                socket.emit('key_down', { key: key });
                updateKeyDisplay(Array.from(pressedKeys));
                highlightKey(key, true);
            }
        };
        
        // Mouse up / Touch end
        const handleEnd = (e) => {
            e.preventDefault();
            if (pressedKeys.has(key)) {
                pressedKeys.delete(key);
                socket.emit('key_up', { key: key });
                updateKeyDisplay(Array.from(pressedKeys));
                highlightKey(key, false);
            }
        };
        
        item.addEventListener('mousedown', handleStart);
        item.addEventListener('touchstart', handleStart);
        item.addEventListener('mouseup', handleEnd);
        item.addEventListener('touchend', handleEnd);
        item.addEventListener('mouseleave', handleEnd);
    }
});

// Prevent context menu on key items
document.querySelectorAll('.key-item').forEach(item => {
    item.addEventListener('contextmenu', (e) => e.preventDefault());
});

console.log('MAVRIC Rover Control Interface loaded');
