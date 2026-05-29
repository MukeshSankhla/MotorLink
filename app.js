/**
 * MotorLink - M0601 / DDSM115 Hub Motor Controller Logic
 * Communicates over browser Web Serial API.
 */

// ── State Management ──────────────────────────────────────────
let port = null;
let reader = null;
let readLoopActive = false;
let writeQueue = [];

// Polling and timing
let activePollInterval = null;
let lastWriteTime = 0;
let pollRateHz = 20; // 20Hz (50ms) — gives RS485 half-duplex bus time for RX responses
let activeMode = 'velocity'; // 'none', 'velocity', 'current', 'position', 'raw'

// Active Command Payloads
let currentVelocityRpm = 0;
let currentVelocityAccel = 1;
let currentTorqueRaw = 0;
let currentPositionRaw = 0;
let isPositionLoopActive = false;
let lastKnownPosDeg = 0;    // last parsed position mapped to 0-360
let positionZeroRef = 0;    // physical position in 32767 scale when switching to position loop
let lastKnownTemp = 0;      // retain temperature value since motor only sends it when stationary
let lastUnwrappedRawPos = null;
let cumulativeRotationDeg = 0;
let isFirstTelemetryFrame = true;
let lastSentCommandType = 0x64; // track last command type to differentiate 0x64 and 0x74 feedback

// Feedback telemetry data
let telemetryHistory = {
    speed: [], // { time: Date, val: number }
    current: [],
    position: [],
    temp: [],
    errors: [],
    timestamps: []
};
const MAX_CHART_POINTS = 300; // 60 seconds at 5 Hz (200ms query)
let lastFeedbackTime = 0;
let feedbackTimeoutCheckInterval = null;
let timeoutWarningShown = false; // debounce repeated timeout warnings

// UI elements and references
let activeTab = 'panel-velocity';
let currentProfile = 'fit1042'; // 'ddsm115' or 'fit1042'
let selectedMotorId = 0x01;
let scanActive = false;
let logCount = 0;

// Buffer for accumulating incoming serial bytes
let rxBuffer = new Uint8Array(0);

// ── Initialization ────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
    // Initialize icons
    if (window.lucide) {
        window.lucide.createIcons();
    }

    setupUIReferences();
    populateMotorIdSelector();
    setupEventListeners();
    checkBrowserSupport();
    updateUIConnectionState('disconnected');
});

// Check if browser supports Web Serial
function checkBrowserSupport() {
    const warning = document.getElementById('browser-warning');
    if (!('serial' in navigator)) {
        warning.classList.remove('hidden');
        document.getElementById('btn-connect').disabled = true;
    }
}

// References
let UI = {};
function setupUIReferences() {
    const elements = [
        'profile-selector', 'motor-id-selector',
        'btn-connect', 'status-dot', 'status-text',

        // Panels
        'panel-velocity', 'panel-current', 'panel-position', 'panel-id', 'panel-raw',
        'velocity-controls-sub', 'current-controls-sub', 'position-controls-sub',

        // Velocity controls
        'btn-vel-switch', 'vel-rpm-input', 'vel-rpm-slider', 'vel-accel-input',
        'btn-vel-rev', 'btn-vel-rev-50', 'btn-vel-fwd', 'btn-vel-fwd-50', 'btn-vel-start', 'btn-vel-stop', 'btn-vel-brake',

        // Current controls
        'btn-curr-switch', 'curr-val-input', 'curr-val-slider', 'curr-amps-display', 'btn-curr-stop',

        // Position controls
        'btn-pos-switch', 'pos-val-input', 'pos-val-slider', 'pos-deg-display',
        'btn-pos-stop', 'pos-dial-container', 'pos-dial-fill',
        'pos-dial-pointer', 'pos-dial-degrees', 'pos-dial-raw',

        // ID controls
        'btn-id-broadcast-scan', 'btn-id-full-scan', 'scan-progress-container',
        'scan-progress-status', 'scan-progress-percent', 'scan-progress-bar',
        'scan-results-list', 'new-id-input', 'btn-id-assign',

        // Raw controls
        'raw-frame-input', 'raw-frame-error', 'raw-frame-calculated',
        'raw-frame-repeat', 'raw-frame-repeat-options', 'raw-frame-count',
        'raw-frame-interval', 'btn-raw-send',

        // Telemetry
        'status-badge', 'mode-badge', 'telemetry-speed', 'telemetry-current', 'telemetry-position',
        'telemetry-position-raw', 'telemetry-temp', 'telemetry-error-badge', 'telemetry-error',

        // Charts & Gauges
        'combined-chart-svg', 'speed-chart-labels', 'current-chart-labels',
        'sparkline-speed-fill', 'sparkline-speed-path',
        'sparkline-current-fill', 'sparkline-current-path',
        'mini-pos-gauge-fill', 'mini-pos-gauge-dot',
        'mini-temp-gauge-fill', 'mini-temp-gauge-dot',

        // Logs
        'chk-collapse-polling', 'chk-auto-scroll', 'btn-clear-log', 'btn-export-log', 'log-container', 'log-count',

        // Estop
        'btn-estop', 'estop-overlay', 'btn-estop-ack',

        // Info Popup
        'btn-info-popup', 'info-popup-overlay', 'btn-close-info',

        // Lightbox
        'connection-img', 'lightbox-overlay', 'btn-close-lightbox', 'lightbox-img',

        // Motor visual status representation
        'motor-visual-container', 'motor-visual', 'motor-motion-text'
    ];

    elements.forEach(id => {
        const el = document.getElementById(id);
        UI[id] = el;
        if (!el) {
            console.warn(`[UI Init] Warning: Element with ID "${id}" was not found in the DOM.`);
        }
    });
}

function populateMotorIdSelector() {
    UI['motor-id-selector'].innerHTML = '';
    // Load from Session Storage if available
    const savedId = sessionStorage.getItem('motorLink_activeId');
    const initialId = savedId ? parseInt(savedId, 10) : 1;
    selectedMotorId = initialId;

    for (let i = 1; i <= 254; i++) {
        const opt = document.createElement('option');
        opt.value = i;
        const hex = i.toString(16).toUpperCase().padStart(2, '0');
        opt.textContent = `0x${hex} (${i})`;
        if (i === initialId) opt.selected = true;
        UI['motor-id-selector'].appendChild(opt);
    }
}

function setupEventListeners() {
    // Connect / Disconnect
    UI['btn-connect'].addEventListener('click', () => toggleConnection());

    // Profile Selection
    UI['profile-selector'].addEventListener('change', (e) => {
        currentProfile = e.target.value;
        logMsg(`[System] Protocol Profile changed to: ${e.target.value === 'fit1042' ? 'DFRobot FIT1042' : 'Waveshare DDSM115'}`, 'system');
        updateModeButtonStates();
    });

    // Motor ID selection
    UI['motor-id-selector'].addEventListener('change', (e) => {
        selectedMotorId = parseInt(e.target.value, 10);
        sessionStorage.setItem('motorLink_activeId', selectedMotorId);
        logMsg(`[System] Active Target Motor ID set to: 0x${selectedMotorId.toString(16).toUpperCase().padStart(2, '0')}`, 'system');
    });

    // Tab buttons switching
    document.querySelectorAll('.tab-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
            document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));

            btn.classList.add('active');
            const targetId = btn.getAttribute('data-target');
            if (UI[targetId]) UI[targetId].classList.add('active');
            activeTab = targetId;

            // Stop any active command loop when switching tabs to protect hardware
            stopActiveCommandLoop();

            // Centralized mode control UI state update
            updateControlModesState();
        });
    });

    // ── Velocity Controls ──
    UI['btn-vel-switch'].addEventListener('click', () => sendModeSwitch('velocity'));

    // Sync slider & numeric input
    UI['vel-rpm-slider'].addEventListener('input', (e) => {
        let val = parseInt(e.target.value, 10);
        UI['vel-rpm-input'].value = val;
        currentVelocityRpm = val;
    });

    UI['vel-rpm-input'].addEventListener('input', (e) => {
        let val = parseInt(e.target.value, 10);
        if (isNaN(val)) val = 0;
        val = Math.max(-330, Math.min(330, val));
        UI['vel-rpm-slider'].value = val;
        currentVelocityRpm = val;
    });

    UI['vel-accel-input'].addEventListener('input', (e) => {
        let val = parseInt(e.target.value, 10);
        if (isNaN(val)) val = 1;
        currentVelocityAccel = Math.max(0, Math.min(255, val));
    });

    // Presets
    UI['btn-vel-fwd'].addEventListener('click', () => {
        UI['vel-rpm-slider'].value = 100;
        UI['vel-rpm-input'].value = 100;
        currentVelocityRpm = 100;
        startCommandLoop('velocity');
    });

    UI['btn-vel-fwd-50'].addEventListener('click', () => {
        UI['vel-rpm-slider'].value = 50;
        UI['vel-rpm-input'].value = 50;
        currentVelocityRpm = 50;
        startCommandLoop('velocity');
    });

    UI['btn-vel-rev'].addEventListener('click', () => {
        UI['vel-rpm-slider'].value = -100;
        UI['vel-rpm-input'].value = -100;
        currentVelocityRpm = -100;
        startCommandLoop('velocity');
    });

    UI['btn-vel-rev-50'].addEventListener('click', () => {
        UI['vel-rpm-slider'].value = -50;
        UI['vel-rpm-input'].value = -50;
        currentVelocityRpm = -50;
        startCommandLoop('velocity');
    });

    // Start, Stop and Brake
    UI['btn-vel-start'].addEventListener('click', () => {
        startCommandLoop('velocity');
        logMsg(`[Velocity] Started motor at ${currentVelocityRpm} RPM.`, 'system');
    });

    UI['btn-vel-stop'].addEventListener('click', () => {
        stopActiveCommandLoop();
        sendVelocityCommand(0, currentVelocityAccel);
        logMsg(`[Velocity] Stopped polling. Sent 0 RPM command.`, 'warning');
    });

    UI['btn-vel-brake'].addEventListener('click', () => {
        stopActiveCommandLoop();
        sendBrakeCommandRepeatedly();
    });

    // ── Current Controls ──
    UI['btn-curr-switch'].addEventListener('click', () => sendModeSwitch('current'));

    // Sync slider & numeric input
    UI['curr-val-slider'].addEventListener('input', (e) => {
        let val = parseInt(e.target.value, 10);
        UI['curr-val-input'].value = val;
        updateCurrentAmpsDisplay(val);
    });

    UI['curr-val-input'].addEventListener('input', (e) => {
        let val = parseInt(e.target.value, 10);
        if (isNaN(val)) val = 0;
        val = Math.max(-32767, Math.min(32767, val));
        UI['curr-val-slider'].value = val;
        updateCurrentAmpsDisplay(val);
    });

    // Presets for current
    document.querySelectorAll('.btn-preset-curr').forEach(btn => {
        btn.addEventListener('click', (e) => {
            let val = parseInt(btn.getAttribute('data-value'), 10);

            // Safety confirmation dialogue for high currents (above +-25000)
            if (Math.abs(val) > 25000) {
                const conf = confirm(`WARNING: A current of ${val} (~${(val * 8 / 32767).toFixed(2)}A) is very high and can quickly overheat the winding or trigger overcurrent protection. Do you want to proceed?`);
                if (!conf) return;
            }

            document.querySelectorAll('.btn-preset-curr').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');

            UI['curr-val-slider'].value = val;
            UI['curr-val-input'].value = val;
            updateCurrentAmpsDisplay(val);
            startCommandLoop('current');
        });
    });

    UI['btn-curr-stop'].addEventListener('click', () => {
        stopActiveCommandLoop();
        sendCurrentCommand(0);
        logMsg(`[Current] Stopped polling. Sent 0 current command.`, 'warning');
        document.querySelectorAll('.btn-preset-curr').forEach(b => b.classList.remove('active'));
    });

    // ── Position Controls ──
    UI['btn-pos-switch'].addEventListener('click', () => sendModeSwitch('position'));

    // Sync slider & numeric input
    UI['pos-val-slider'].addEventListener('input', (e) => {
        let val = parseInt(e.target.value, 10);
        UI['pos-val-input'].value = val;
        updatePositionDisplay(val);
    });

    UI['pos-val-input'].addEventListener('input', (e) => {
        let val = parseInt(e.target.value, 10);
        if (isNaN(val)) val = 0;
        val = Math.max(0, Math.min(32767, val));
        UI['pos-val-slider'].value = val;
        updatePositionDisplay(val);
    });

    // Angle Presets
    document.querySelectorAll('.btn-preset-pos').forEach(btn => {
        btn.addEventListener('click', () => {
            let val = parseInt(btn.getAttribute('data-value'), 10);
            document.querySelectorAll('.btn-preset-pos').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');

            UI['pos-val-slider'].value = val;
            UI['pos-val-input'].value = val;
            updatePositionDisplay(val);
        });
    });

    UI['btn-pos-stop'].addEventListener('click', () => {
        isPositionLoopActive = false;
        stopActiveCommandLoop();
        logMsg("[Position] Stop pressed. Halted loop.", "warning");
    });

    UI['btn-estop-ack'].addEventListener('click', () => {
        window.location.reload();
    });

    // Dial interactive mouse/touch event listeners
    initDialInteractions();

    // ── ID Manager Controls ──
    UI['btn-id-broadcast-scan'].addEventListener('click', () => runBroadcastScan());
    UI['btn-id-full-scan'].addEventListener('click', () => runFullScan());
    UI['btn-id-assign'].addEventListener('click', () => assignNewMotorId());

    // ── Raw Hex Controls ──
    UI['raw-frame-input'].addEventListener('input', (e) => validateAndCalculateRawFrame(e.target.value));
    UI['btn-raw-send'].addEventListener('click', () => sendRawFrameCommand());

    // ── Log Controls ──
    UI['btn-clear-log'].addEventListener('click', () => {
        UI['log-container'].innerHTML = '';
        logCount = 0;
        updateLogCountBadge();
    });

    UI['btn-export-log'].addEventListener('click', () => exportLogToCSV());

    // (Note: Chart buttons and toggle listeners removed as the Speed and Current charts are now permanently merged)

    // ── Emergency Stop Button ──
    UI['btn-estop'].addEventListener('click', () => triggerEmergencyStop());

    // ── Info Popup ──
    if (UI['btn-info-popup'] && UI['info-popup-overlay'] && UI['btn-close-info']) {
        UI['btn-info-popup'].addEventListener('click', () => {
            UI['info-popup-overlay'].classList.remove('hidden');
            setTimeout(() => {
                UI['info-popup-overlay'].children[0].classList.remove('scale-95');
                UI['info-popup-overlay'].children[0].classList.add('scale-100');
            }, 10);
        });
        
        const closeInfoPopup = () => {
            UI['info-popup-overlay'].children[0].classList.remove('scale-100');
            UI['info-popup-overlay'].children[0].classList.add('scale-95');
            setTimeout(() => {
                UI['info-popup-overlay'].classList.add('hidden');
            }, 300);
        };

        UI['btn-close-info'].addEventListener('click', closeInfoPopup);
        UI['info-popup-overlay'].addEventListener('click', (e) => {
            if (e.target === UI['info-popup-overlay']) closeInfoPopup();
        });
    }

    // ── Image Lightbox ──
    if (UI['connection-img'] && UI['lightbox-overlay'] && UI['btn-close-lightbox']) {
        UI['connection-img'].addEventListener('click', () => {
            UI['lightbox-overlay'].classList.remove('hidden');
            setTimeout(() => {
                UI['lightbox-img'].classList.remove('scale-95');
                UI['lightbox-img'].classList.add('scale-100');
            }, 10);
        });

        const closeLightbox = () => {
            UI['lightbox-img'].classList.remove('scale-100');
            UI['lightbox-img'].classList.add('scale-95');
            setTimeout(() => {
                UI['lightbox-overlay'].classList.add('hidden');
            }, 300);
        };

        UI['btn-close-lightbox'].addEventListener('click', closeLightbox);
        UI['lightbox-overlay'].addEventListener('click', (e) => {
            // Close if clicking outside the image
            if (e.target === UI['lightbox-overlay']) closeLightbox();
        });
    }

    // Keyboard Shortcuts (operable by one hand)
    window.addEventListener('keydown', (e) => {
        // Only run shortcuts if user isn't typing in an input box
        if (document.activeElement.tagName === 'INPUT' || document.activeElement.tagName === 'SELECT') {
            return;
        }

        const key = e.key.toUpperCase();
        if (key === ' ') { // Spacebar for emergency stop
            e.preventDefault();
            triggerEmergencyStop();
        } else if (key === 'F') { // F for quick forward
            if (activeTab === 'panel-velocity') {
                UI['btn-vel-fwd'].click();
            }
        } else if (key === 'B') { // B for quick reverse
            if (activeTab === 'panel-velocity') {
                UI['btn-vel-rev'].click();
            }
        } else if (key === 'S') { // S for stop
            if (activeTab === 'panel-velocity') {
                UI['btn-vel-stop'].click();
            } else if (activeTab === 'panel-current') {
                UI['btn-curr-stop'].click();
            } else if (activeTab === 'panel-position') {
                UI['btn-pos-stop'].click();
            }
        } else if (key === 'K') { // K for brake
            if (activeTab === 'panel-velocity') {
                UI['btn-vel-brake'].click();
            }
        }
    });

    // Window Navigation / Close Safety Safeguard
    window.addEventListener('beforeunload', () => {
        if (port && port.writable) {
            // Send switch to velocity, speed = 0, and Brake frame on exit
            const modeFrame = new Uint8Array([selectedMotorId, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02]);
            const zeroFrame = buildFrame(selectedMotorId, 0x64, [0, 0, 0, 0, 0, 0, 0]);
            const brakeFrame = buildFrame(selectedMotorId, 0x64, [0, 0, 0, 0, 0, 0xFF, 0]);

            try {
                const writer = port.writable.getWriter();
                writer.write(modeFrame);
                writer.write(zeroFrame);
                writer.write(brakeFrame);
                writer.releaseLock();
            } catch (e) { }
        }
    });
}

// ── Dial Angle Math Interactions ──────────────────────────────
function initDialInteractions() {
    const container = UI['pos-dial-container'];

    function updateDialFromCoords(clientX, clientY) {
        const rect = container.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;

        const dx = clientX - centerX;
        const dy = clientY - centerY;

        // Calculate angle in degrees from x-axis (CCW)
        // Adjust for -90 degrees rotation of SVG circle coordinate system
        let angleRad = Math.atan2(dy, dx);
        let angleDeg = angleRad * (180 / Math.PI);

        // Convert to 0 to 360 CCW starting from top
        angleDeg = (angleDeg + 90 + 360) % 360;

        // Convert degrees to 0-32767 raw range
        const rawVal = Math.round((angleDeg / 360) * 32767);

        UI['pos-val-slider'].value = rawVal;
        UI['pos-val-input'].value = rawVal;
        updatePositionDisplay(rawVal);
    }

    let isDragging = false;

    container.addEventListener('mousedown', (e) => {
        isDragging = true;
        updateDialFromCoords(e.clientX, e.clientY);
    });

    window.addEventListener('mousemove', (e) => {
        if (isDragging) {
            updateDialFromCoords(e.clientX, e.clientY);
        }
    });

    window.addEventListener('mouseup', () => {
        isDragging = false;
    });

    // Touch support
    container.addEventListener('touchstart', (e) => {
        isDragging = true;
        updateDialFromCoords(e.touches[0].clientX, e.touches[0].clientY);
    });

    window.addEventListener('touchmove', (e) => {
        if (isDragging) {
            updateDialFromCoords(e.touches[0].clientX, e.touches[0].clientY);
        }
    });

    window.addEventListener('touchend', () => {
        isDragging = false;
    });
}

// Update Position Display elements
function updatePositionDisplay(rawVal) {
    currentPositionRaw = rawVal;

    const deg = ((rawVal / 32767) * 360).toFixed(1);
    UI['pos-deg-display'].textContent = deg;
    UI['pos-dial-degrees'].textContent = `${Math.round(deg)}°`;
    UI['pos-dial-raw'].textContent = `Raw: ${rawVal}`;

    // Update SVG dial rendering (pointer and fill offset)
    const angleRad = deg * (Math.PI / 180);
    const px = 50 + 42 * Math.cos(angleRad);
    const py = 50 + 42 * Math.sin(angleRad);

    UI['pos-dial-pointer'].setAttribute('cx', px);
    UI['pos-dial-pointer'].setAttribute('cy', py);

    // Stroke dash offset (circumference of r=42 is ~263.89)
    const fillPercent = rawVal / 32767;
    const offset = 263.89 * (1 - fillPercent);
    UI['pos-dial-fill'].setAttribute('stroke-dashoffset', offset);

    // Immediately send position command if port is active and in position mode
    if (port && activeMode === 'position') {
        sendPositionCommand(rawVal);
    }
}

function applyCurrentSafetyLock(requestedVal) {
    if (!port) return requestedVal;

    const prevVal = currentTorqueRaw;

    // Check if crossing zero boundary
    if (prevVal > 1000 && requestedVal < 0) {
        logMsg(`[Safety Lock] Blocked sudden torque reversal! Sweep current near zero (under ±1000 raw counts) first. Clamped to 0.`, 'error');
        return 0;
    }

    if (prevVal < -1000 && requestedVal > 0) {
        logMsg(`[Safety Lock] Blocked sudden torque reversal! Sweep current near zero (under ±1000 raw counts) first. Clamped to 0.`, 'error');
        return 0;
    }

    return requestedVal;
}

function updateCurrentAmpsDisplay(rawVal) {
    const safeVal = applyCurrentSafetyLock(rawVal);

    if (safeVal !== rawVal) {
        UI['curr-val-slider'].value = safeVal;
        UI['curr-val-input'].value = safeVal;
        rawVal = safeVal;
    }

    currentTorqueRaw = rawVal;
    const amps = (rawVal * (8.0 / 32767.0)).toFixed(2);
    UI['curr-amps-display'].textContent = amps;
}

// ── CRC-8/Maxim Engine ────────────────────────────────────────
function crc8Maxim(bytes) {
    let crc = 0;
    for (const byte of bytes) {
        crc ^= byte;
        for (let i = 0; i < 8; i++) {
            crc = (crc & 0x01) ? ((crc >> 1) ^ 0x8C) : (crc >> 1);
        }
    }
    return crc;
}

function buildFrame(motorId, cmd, data7) {
    const frame = [motorId, cmd, ...data7.slice(0, 7)];
    while (frame.length < 9) {
        frame.push(0x00);
    }
    frame.push(crc8Maxim(frame));
    return new Uint8Array(frame);
}

// ── Connection Manager ────────────────────────────────────────
async function toggleConnection() {
    if (port) {
        // Disconnect
        await disconnectSerial();
    } else {
        // Connect
        await connectSerial();
    }
}

async function connectSerial() {
    if (!('serial' in navigator)) return;

    isFirstTelemetryFrame = true;
    updateUIConnectionState('connecting');
    logMsg('[System] Opening serial port chooser...', 'system');

    try {
        // Request port
        port = await navigator.serial.requestPort();

        // Open port
        await port.open({
            baudRate: 115200,
            dataBits: 8,
            stopBits: 1,
            parity: 'none',
            flowControl: 'none'
        });

        logMsg(`[System] Port opened successfully at 115200 8N1`, 'system');
        updateUIConnectionState('connected');

        // Start reading loop
        startReadLoop();

        // Trigger automated ID broadcast scan on connection (FR-CON-06)
        setTimeout(() => {
            logMsg('[System] Automated connection initialization: sending ID Broadcast Query...', 'system');
            sendBroadcastQueryFrame();
        }, 500);

        // Start feedback query loop in background (FR-FB-01)
        startFeedbackQueryTimer();

    } catch (e) {
        logMsg(`[System] Connection failed: ${e.message}`, 'error');
        updateUIConnectionState('disconnected');
        port = null;
    }
}

async function disconnectSerial() {
    logMsg('[System] Stopping motor and closing connection...', 'system');
    try {
        stopFeedbackQueryTimer();

        // Execute the exact same firm braking sequence as Emergency Stop
        await haltMotorFirmly();

        // 1. Signal read loop to stop
        readLoopActive = false;

        // 2. Cancel reader to unblock pending read()
        if (reader) {
            try {
                reader.cancel().catch(e => { });
            } catch (e) { }
            reader = null;
        }

        await sleep(100);

        // 3. Close the port
        if (port) {
            try {
                await port.close();
            } catch (e) {
                logMsg(`[System] Port close error (non-fatal): ${e.message}`, 'system');
            }
        }
    } catch (e) {
        logMsg(`[System] Disconnect error: ${e.message}`, 'error');
    } finally {
        port = null;
        reader = null;
        readLoopActive = false;
        rxBuffer = new Uint8Array(0);
        updateUIConnectionState('disconnected');
        logMsg('[System] Disconnected successfully. Refreshing page...', 'system');
        await sleep(500);
        window.location.reload();
    }
}

function updateUIConnectionState(state) {
    const dot = UI['status-dot'];
    const text = UI['status-text'];
    const connBtn = UI['btn-connect'];
    const motorContainer = UI['motor-visual-container'];
    const motorVisual = UI['motor-visual'];
    const motionText = UI['motor-motion-text'];

    dot.className = 'w-2.5 h-2.5 rounded-full ';

    // Reset motor container & image classes to theme defaults
    motorContainer.className = 'motor-visual-container ';
    motorVisual.className = 'motor-visual-img ';

    const elementsToDisable = [
        'btn-vel-switch', 'btn-curr-switch', 'btn-pos-switch',
        'btn-id-broadcast-scan', 'btn-id-full-scan', 'btn-id-assign',
        'btn-raw-send'
    ];

    if (state === 'connected') {
        dot.classList.add('bg-emerald-500', 'shadow-[0_0_10px_rgba(16,185,129,0.3)]');

        let portInfoStr = '';
        if (port) {
            const info = port.getInfo();
            if (info.usbVendorId) {
                portInfoStr = ` (USB VID: 0x${info.usbVendorId.toString(16).toUpperCase()})`;
            } else {
                portInfoStr = ` (COM Port)`;
            }
        }
        text.textContent = `Connected${portInfoStr}`;
        text.className = 'text-xs font-bold text-[#059669]';
        connBtn.className = 'bg-rose-600 hover:bg-rose-700 text-white font-semibold text-sm px-5 py-2.5 rounded-lg transition-all active:scale-95 flex items-center gap-2 shadow-sm';
        connBtn.innerHTML = `<i data-lucide="power-off" class="w-4 h-4"></i><span>Disconnect</span>`;

        motorContainer.classList.add('status-ring-connected');
        motorVisual.classList.add('motor-connected');
        motionText.textContent = 'Connected';

        elementsToDisable.forEach(id => {
            if (UI[id]) UI[id].disabled = false;
        });
        updateControlModesState();

        UI['status-badge'].textContent = 'Online';
        UI['status-badge'].className = 'text-[10px] font-bold px-2 py-0.5 rounded-full bg-emerald-100 text-emerald-700 border border-emerald-250';
        updateModeBadge(activeMode);
    } else if (state === 'connecting') {
        dot.classList.add('bg-amber-500', 'shadow-[0_0_10px_rgba(245,158,11,0.3)]');
        text.textContent = 'Connecting...';
        text.className = 'text-xs font-bold text-amber-600';
        connBtn.disabled = true;

        motorContainer.classList.add('status-ring-connecting');
        motorVisual.classList.add('motor-connecting');
        motionText.textContent = 'Connecting...';

        UI['status-badge'].textContent = 'Connecting';
        UI['status-badge'].className = 'text-[10px] font-bold px-2 py-0.5 rounded-full bg-amber-100 text-amber-750 border border-amber-250';
        UI['mode-badge'].classList.add('hidden');
    } else {
        // Disconnected / Error
        dot.classList.add('bg-slate-400');
        text.textContent = 'Disconnected';
        text.className = 'text-xs font-bold text-slate-500';
        connBtn.className = 'bg-blue-600 hover:bg-blue-700 text-white font-semibold text-sm px-5 py-2.5 rounded-lg transition-all active:scale-95 flex items-center gap-2 shadow-sm';
        connBtn.innerHTML = `<i data-lucide="power" class="w-4 h-4"></i><span>Connect</span>`;
        connBtn.disabled = false;

        if (state === 'error') {
            motorContainer.classList.add('status-ring-error');
            motorVisual.classList.add('motor-error');
            motionText.textContent = 'Error';
        } else {
            motorContainer.classList.add('status-ring-disconnected');
            motorVisual.classList.add('motor-disconnected');
            motionText.textContent = 'Offline';
        }

        // Remove spin animations
        motorVisual.classList.remove('motor-spin-cw', 'motor-spin-ccw');
        motorVisual.style.transform = 'none';

        elementsToDisable.forEach(id => {
            if (UI[id]) UI[id].disabled = true;
        });
        updateControlModesState();
        UI['status-badge'].textContent = 'Offline';
        UI['status-badge'].className = 'text-[10px] font-bold px-2 py-0.5 rounded-full bg-slate-100 text-slate-500 border border-slate-200';
        UI['mode-badge'].classList.add('hidden');
    }

    if (window.lucide) {
        window.lucide.createIcons();
    }
}

function updateModeBadge(modeName) {
    if (!UI['mode-badge']) return;
    let modeText = 'Unknown';
    UI['mode-badge'].classList.remove('hidden');
    if (modeName === 'velocity') {
        modeText = 'Velocity Loop';
        UI['mode-badge'].className = 'absolute top-3 right-3 text-[10px] font-bold px-2 py-0.5 rounded-full bg-blue-100 text-blue-700 border border-blue-200';
    } else if (modeName === 'current') {
        modeText = 'AC (Current)';
        UI['mode-badge'].className = 'absolute top-3 right-3 text-[10px] font-bold px-2 py-0.5 rounded-full bg-emerald-100 text-emerald-700 border border-emerald-200';
    } else if (modeName === 'position') {
        modeText = 'Position Loop';
        UI['mode-badge'].className = 'absolute top-3 right-3 text-[10px] font-bold px-2 py-0.5 rounded-full bg-indigo-100 text-indigo-700 border border-indigo-200';
    } else if (modeName === 'raw') {
        modeText = 'Raw Hex';
        UI['mode-badge'].className = 'absolute top-3 right-3 text-[10px] font-bold px-2 py-0.5 rounded-full bg-slate-100 text-slate-500 border border-slate-200';
    } else {
        modeText = 'Open Loop';
        UI['mode-badge'].className = 'absolute top-3 right-3 text-[10px] font-bold px-2 py-0.5 rounded-full bg-slate-100 text-slate-500 border border-slate-200';
    }
    UI['mode-badge'].textContent = modeText;
}

// ── Web Serial Read Worker ────────────────────────────────────
async function startReadLoop() {
    readLoopActive = true;

    while (port && port.readable && readLoopActive) {
        try {
            reader = port.readable.getReader();

            while (readLoopActive) {
                const { value, done } = await reader.read();
                if (done) {
                    break;
                }
                if (value && value.length > 0) {
                    handleIncomingSerialBytes(value);
                }
            }
        } catch (e) {
            logMsg(`[Read Loop Error] ${e.message}`, 'error');
            break;
        } finally {
            if (reader) {
                reader.releaseLock();
                reader = null;
            }
        }
    }
}

function handleIncomingSerialBytes(newBytes) {
    // Append to our buffer
    const combined = new Uint8Array(rxBuffer.length + newBytes.length);
    combined.set(rxBuffer, 0);
    combined.set(newBytes, rxBuffer.length);
    rxBuffer = combined;

    // Parse complete packets (10 bytes)
    let index = 0;
    while (index <= rxBuffer.length - 10) {
        // A valid response packet:
        // Index 0: ID
        // Index 1: Mode/CMD (0x74 or active control command 0x64 etc.)
        // We look for a valid CRC match on 10 bytes
        const potentialPacket = rxBuffer.slice(index, index + 10);
        const calcCrc = crc8Maxim(potentialPacket.slice(0, 9));

        if (calcCrc === potentialPacket[9]) {
            // Found a valid verified frame!
            parseIncomingFrame(potentialPacket);
            index += 10;
        } else {
            // Discard single byte and slide window
            index++;
        }
    }

    if (index > 0) {
        rxBuffer = rxBuffer.slice(index);
    }
}

// ── Parser & Telemetry Handler ────────────────────────────────
function parseIncomingFrame(bytes) {
    const motorId = bytes[0];
    const cmdType = bytes[1];

    // Hex string for logging
    const hexStr = Array.from(bytes).map(b => b.toString(16).toUpperCase().padStart(2, '0')).join(' ');

    // Format collapsing logs (FR-LOG-06)
    const isHighFreq = (cmdType === 0x64 || cmdType === 0x74);
    if (UI['chk-collapse-polling'].checked && isHighFreq) {
        // Do not flood log with repetitive polls, but count them in system
    } else {
        logMsg(`← RX [ID 0x${motorId.toString(16).toUpperCase().padStart(2, '0')}]: ${hexStr}`, 'received');
    }

    // Record feedback time for timeout checking — also resets warning flag
    lastFeedbackTime = Date.now();
    timeoutWarningShown = false;

    let shouldSyncPosTarget = false;
    // On initial connection, read the motor operating mode and switch UI mode to match
    if (isFirstTelemetryFrame) {
        isFirstTelemetryFrame = false;
        const detectedMode = bytes[1];
        let targetModeName = null;
        if (detectedMode === 0x01) targetModeName = 'current';
        else if (detectedMode === 0x02) targetModeName = 'velocity';
        else if (detectedMode === 0x03) targetModeName = 'position';

        if (targetModeName) {
            activeMode = targetModeName;

            // Switch tab panel visually
            let tabId = 'tab-vel';
            if (targetModeName === 'current') tabId = 'tab-curr';
            else if (targetModeName === 'position') {
                tabId = 'tab-pos';
                shouldSyncPosTarget = true;
            }

            const tabBtn = document.getElementById(tabId);
            if (tabBtn) {
                // Click the tab visually to activate panel
                document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
                document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
                tabBtn.classList.add('active');
                const targetPanelId = tabBtn.getAttribute('data-target');
                if (UI[targetPanelId]) UI[targetPanelId].classList.add('active');
                activeTab = targetPanelId;
            }

            // Synchronize states
            // 1. Current (Torque) - Invert to match CW-Positive orientation
            let rawCurrent = (bytes[2] << 8) | bytes[3];
            if (rawCurrent > 32767) rawCurrent -= 65536; // signed 16-bit
            rawCurrent = -rawCurrent; // Invert
            const currentAmps = (rawCurrent / 32767.0) * 8.0;

            // 2. Speed (Velocity) - Invert to match CW-Positive orientation
            let rawSpeed = (bytes[4] << 8) | bytes[5];
            if (rawSpeed > 32767) rawSpeed -= 65536; // signed 16-bit
            rawSpeed = -rawSpeed; // Invert

            updateModeButtonStates();
            updateControlModesState();
            updateModeBadge(targetModeName);

            logMsg(`[System] Initial connection: Detected active motor mode as ${targetModeName.toUpperCase()}. Synchronized dashboard control loop.`, 'system');
        }
    }

    // Parse data registers depending on feedback command structure (0x74 response)
    // 0: ID
    // 1: CMD/Mode
    // 2-3: Torque current (signed 16-bit, big-endian)
    // 4-5: Speed (signed 16-bit, big-endian)
    // 6: Position (0-255)
    // 7: Temperature (°C)
    // 8: Error Code
    // 9: CRC

    // 1. Current
    let rawCurrent = (bytes[2] << 8) | bytes[3];
    if (rawCurrent > 32767) rawCurrent -= 65536; // signed 16-bit conversion
    const currentAmps = rawCurrent * (8.0 / 32767.0);

    // 2. Speed (RPM)
    let rawSpeed = (bytes[4] << 8) | bytes[5];
    if (rawSpeed > 32767) rawSpeed -= 65536; // signed 16-bit

    let posDeg = 0;
    let rawPosStr = "";

    // Differentiate Protocol 1 (0x64) vs Protocol 2 (0x74) Feedback
    if (lastSentCommandType === 0x64) {
        // 0x64 Feedback: DATA[6]=Pos High, DATA[7]=Pos Low
        const pos16 = (bytes[6] << 8) | bytes[7];
        posDeg = (pos16 / 32767.0) * 360.0;
        rawPosStr = `(${pos16})`;
    } else {
        // 0x74 Feedback: DATA[6]=Winding Temp, DATA[7]=U8 Pos
        const tempCelsius = bytes[6];
        lastKnownTemp = tempCelsius;
        const pos8 = bytes[7];
        posDeg = (pos8 / 255.0) * 360.0;
        rawPosStr = `(${pos8})`;
    }

    lastKnownPosDeg = posDeg;

    if (shouldSyncPosTarget) {
        const currentPosTarget = Math.round((posDeg / 360.0) * 32767);
        UI['pos-val-slider'].value = currentPosTarget;
        UI['pos-val-input'].value = currentPosTarget;
        currentPositionRaw = currentPosTarget;
        updatePositionDisplay(currentPosTarget);
    }

    // Smooth unwrapped rotation computation based on posDeg
    if (lastUnwrappedRawPos === null) {
        lastUnwrappedRawPos = posDeg;
        cumulativeRotationDeg = posDeg;
    } else {
        let diffDeg = posDeg - lastUnwrappedRawPos;
        if (diffDeg > 180.0) {
            diffDeg -= 360.0;
        } else if (diffDeg < -180.0) {
            diffDeg += 360.0;
        }
        cumulativeRotationDeg += diffDeg;
        lastUnwrappedRawPos = posDeg;
    }

    // 5. Error Code
    const errorCode = bytes[8];

    // Update live indicators
    UI['telemetry-speed'].textContent = rawSpeed;
    UI['telemetry-current'].textContent = currentAmps.toFixed(2);
    UI['telemetry-position'].textContent = posDeg.toFixed(1);
    UI['telemetry-position-raw'].textContent = rawPosStr;
    UI['telemetry-temp'].textContent = lastKnownTemp;

    // Parse error string
    let errorStr = "00";
    if (errorCode !== 0) {
        const errors = [];
        if (errorCode & 0x01) errors.push("Sensor");
        if (errorCode & 0x02) errors.push("Overcurrent");
        if (errorCode & 0x04) errors.push("Phase Overcurrent");
        if (errorCode & 0x08) errors.push("Stall");
        if (errorCode & 0x10) errors.push("Troubleshooting");
        if (errors.length === 0) errors.push(`0x${errorCode.toString(16).toUpperCase().padStart(2, '0')}`);
        errorStr = errors.join(', ');
    }

    UI['telemetry-error'].textContent = errorStr;

    if (errorCode !== 0) {
        UI['telemetry-error-badge'].className = 'bg-rose-100 border border-rose-200 text-rose-700 font-mono text-[9px] px-1.5 py-0.5 rounded font-bold';
    } else {
        UI['telemetry-error-badge'].className = 'bg-emerald-100 border border-emerald-200 text-emerald-700 font-mono text-[9px] px-1.5 py-0.5 rounded font-bold';
    }

    // Dynamic Motor Image Rotation based on live speed/position feedback
    const motorVisual = UI['motor-visual'];
    const motionText = UI['motor-motion-text'];

    // Direct unwrapped positioning (REVERSED orientation as requested by user)
    motorVisual.style.transform = `rotate(${(cumulativeRotationDeg).toFixed(1)}deg)`;

    if (rawSpeed !== 0) {
        if (rawSpeed > 0) {
            motionText.textContent = `Spinning CCW (${rawSpeed} RPM)`;
        } else {
            motionText.textContent = `Spinning CW (${rawSpeed} RPM)`;
        }
    } else {
        motionText.textContent = `Stationary (${posDeg.toFixed(1)}°)`;
    }

    // Update charts data queues
    pushTelemetryToHistory(rawSpeed, currentAmps);

    // Update live mini visual widgets (sparklines, circular angle gauges, temperature meters)
    updateMiniVisualWidgets(rawSpeed, currentAmps, posDeg, lastKnownTemp);

    // Update active loop mode representation in light theme style
    updateModeBadge(activeMode);

    // Maintain Online status on the status-badge
    UI['status-badge'].textContent = 'Online';
    UI['status-badge'].className = 'text-[10px] font-bold px-2 py-0.5 rounded-full bg-emerald-100 text-emerald-700 border border-emerald-200';

    // If scanning, register found motor
    if (scanActive) {
        registerScannedMotor(motorId);
    }
}

// ── Serial Write Helpers ──────────────────────────────────────
let isWriting = false;
async function writeFrame(frameBytes) {
    if (!port || !port.writable) {
        logMsg('[System] Write failed: Port disconnected', 'error');
        return false;
    }

    // Wait for the stream lock to be released
    while (isWriting || port.writable.locked) {
        await sleep(2);
    }

    isWriting = true;
    try {
        const writer = port.writable.getWriter();
        await writer.write(frameBytes);
        writer.releaseLock();
        isWriting = false;

        lastWriteTime = Date.now();
        lastSentCommandType = frameBytes[1];

        // Log frame (collapsible)
        const motorId = frameBytes[0];
        const cmdType = frameBytes[1];
        const isHighFreq = (cmdType === 0x64 || cmdType === 0x74);

        if (UI['chk-collapse-polling'].checked && isHighFreq) {
            // Keep console uncluttered
        } else {
            const hexStr = Array.from(frameBytes).map(b => b.toString(16).toUpperCase().padStart(2, '0')).join(' ');
            logMsg(`→ TX [ID 0x${motorId.toString(16).toUpperCase().padStart(2, '0')}]: ${hexStr}`, 'sent');
        }
        return true;
    } catch (e) {
        isWriting = false;
        logMsg(`[Write Error] ${e.message}`, 'error');
        return false;
    }
}

// ── Mode Switch Command Builder ──────────────────────────────
async function sendModeSwitch(modeName) {
    if (!port) return;

    stopActiveCommandLoop();

    let modeVal = 0x02; // Velocity default
    if (modeName === 'current') modeVal = 0x01;
    if (modeName === 'position') modeVal = 0x03;

    logMsg(`[System] Switching to ${modeName.toUpperCase()} loop mode...`, 'warning');

    // Command structure for both FIT1042 and DDSM115 mode switches:
    // [ID, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, Mode]
    const modeFrame = new Uint8Array([
        selectedMotorId,
        0xA0,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        modeVal
    ]);

    // Send 5 times consecutive to guarantee reception
    for (let i = 0; i < 5; i++) {
        await writeFrame(modeFrame);
        await sleep(20);
    }

    activeMode = modeName;
    updateModeButtonStates();
    updateControlModesState();
    updateModeBadge(modeName);

    if (modeName === 'position') {
        const currentPosTarget = Math.round((lastKnownPosDeg / 360.0) * 32767);
        UI['pos-val-slider'].value = currentPosTarget;
        UI['pos-val-input'].value = currentPosTarget;
        updatePositionDisplay(currentPosTarget);
        logMsg(`[Position] Switched to Position loop mode. Target synced to current absolute angle.`, 'system');
    }

    logMsg(`[System] Switch command sequences completed.`, 'system');
}

function updateModeButtonStates() {
    // Reset active visual states
    if (UI['btn-vel-switch']) UI['btn-vel-switch'].className = 'btn-mode-switch';
    if (UI['btn-curr-switch']) UI['btn-curr-switch'].className = 'btn-mode-switch';
    if (UI['btn-pos-switch']) UI['btn-pos-switch'].className = 'btn-mode-switch';

    if (activeMode === 'velocity') {
        if (UI['btn-vel-switch']) UI['btn-vel-switch'].className = 'btn-mode-switch active';
    } else if (activeMode === 'current') {
        if (UI['btn-curr-switch']) UI['btn-curr-switch'].className = 'btn-mode-switch active';
    } else if (activeMode === 'position') {
        if (UI['btn-pos-switch']) UI['btn-pos-switch'].className = 'btn-mode-switch active';
    }
}

function updateControlModesState() {
    const isConnected = !!port;

    // 1. General tab panel disabling based on connection state
    const panels = ['panel-velocity', 'panel-current', 'panel-position', 'panel-id', 'panel-raw'];
    panels.forEach(id => {
        const el = UI[id];
        if (el) {
            if (!isConnected) {
                el.classList.add('ui-disabled');
            } else {
                el.classList.remove('ui-disabled');
            }
        }
    });

    // 2. Specific sub-controls disabling based on active mode
    const velSub = UI['velocity-controls-sub'];
    const currSub = UI['current-controls-sub'];
    const posSub = UI['position-controls-sub'];

    if (isConnected) {
        if (activeMode === 'velocity') {
            if (velSub) velSub.classList.remove('ui-disabled');
        } else {
            if (velSub) velSub.classList.add('ui-disabled');
        }

        if (activeMode === 'current') {
            if (currSub) currSub.classList.remove('ui-disabled');
        } else {
            if (currSub) currSub.classList.add('ui-disabled');
        }

        if (activeMode === 'position') {
            if (posSub) posSub.classList.remove('ui-disabled');
        } else {
            if (posSub) posSub.classList.add('ui-disabled');
        }
    } else {
        // If disconnected, disable all sub-controls too just in case
        if (velSub) velSub.classList.add('ui-disabled');
        if (currSub) currSub.classList.add('ui-disabled');
        if (posSub) posSub.classList.add('ui-disabled');
    }
}

// ── Control Loops (50Hz polling) ──────────────────────────────
function startCommandLoop(mode) {
    stopActiveCommandLoop();
    activeMode = mode;
    updateModeButtonStates();
    updateControlModesState();

    logMsg(`[Loop] Starting ${mode.toUpperCase()} control loop at ${pollRateHz}Hz`, 'system');

    const intervalMs = 1000 / pollRateHz;

    activePollInterval = setInterval(() => {
        if (activeMode === 'velocity') {
            sendVelocityCommand(currentVelocityRpm, currentVelocityAccel);
        } else if (activeMode === 'current') {
            sendCurrentCommand(currentTorqueRaw);
        } else if (activeMode === 'position') {
            sendPositionCommand(currentPositionRaw);
        }
    }, intervalMs);
}

function stopActiveCommandLoop() {
    if (activePollInterval) {
        clearInterval(activePollInterval);
        activePollInterval = null;
        logMsg(`[Loop] Stopped active control loop.`, 'system');
    }
}

// 1. Velocity Loop
function sendVelocityCommand(rpm, accel) {
    // Invert RPM to match CW-Positive orientation
    let cmdRpm = -rpm;

    // Clamp velocities client side to +-330 (NFR-SEC-01)
    cmdRpm = Math.max(-330, Math.min(330, cmdRpm));

    // Build RPM payload (INT16 Big Endian)
    const buffer = new ArrayBuffer(2);
    const view = new DataView(buffer);
    view.setInt16(0, cmdRpm, false); // Big endian
    const rpmBytes = new Uint8Array(buffer);

    // Frame layout: [ID, 0x64, RPM_H, RPM_L, 0, 0, Accel, 0, 0, CRC]
    const payload = [rpmBytes[0], rpmBytes[1], 0, 0, accel, 0, 0];
    const frame = buildFrame(selectedMotorId, 0x64, payload);
    writeFrame(frame);
}

// 2. Current Loop
function sendCurrentCommand(rawCurrent) {
    // Invert Current to match CW-Positive orientation
    let cmdCurrent = -rawCurrent;
    cmdCurrent = Math.max(-32767, Math.min(32767, cmdCurrent));

    const buffer = new ArrayBuffer(2);
    const view = new DataView(buffer);
    view.setInt16(0, cmdCurrent, false); // Big endian
    const currBytes = new Uint8Array(buffer);

    // Frame: [ID, 0x64, CURR_H, CURR_L, 0, 0, 0, 0, 0, CRC]
    const payload = [currBytes[0], currBytes[1], 0, 0, 0, 0, 0];
    const frame = buildFrame(selectedMotorId, 0x64, payload);
    writeFrame(frame);
}

// 3. Position Loop
function sendPositionCommand(rawPos) {
    let cmdPos = rawPos;
    cmdPos = Math.max(0, Math.min(32767, cmdPos));

    const buffer = new ArrayBuffer(2);
    const view = new DataView(buffer);
    view.setUint16(0, cmdPos, false); // Big endian
    const posBytes = new Uint8Array(buffer);

    // Frame: [ID, 0x64, POS_H, POS_L, 0, 0, 0, 0, 0, CRC]
    const payload = [posBytes[0], posBytes[1], 0, 0, 0, 0, 0];
    const frame = buildFrame(selectedMotorId, 0x64, payload);
    writeFrame(frame);
}

// 4. Electric Brake
async function sendBrakeCommandRepeatedly() {
    // Frame: [ID, 0x64, 0, 0, 0, 0, 0, 0xFF, 0, CRC] (from SRS Section 8)
    const frame = new Uint8Array([selectedMotorId, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xD1]);

    // Re-verify CRC check in case ID !== 0x01
    let frameToSend = frame;
    if (selectedMotorId !== 0x01) {
        frameToSend = buildFrame(selectedMotorId, 0x64, [0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00]);
    }

    logMsg(`[Brake] Sending electric brake frame 5 times...`, 'warning');
    for (let i = 0; i < 5; i++) {
        await writeFrame(frameToSend);
        await sleep(20);
    }
    logMsg(`[Brake] Braking completed.`, 'system');
}

// ── Telemetry Feedback Timer ──────────────────────────────────
let feedbackQueryTimer = null;
let feedbackIntervalMs = 200; // configurable

function startFeedbackQueryTimer() {
    stopFeedbackQueryTimer();

    // Background polling query (0x74)
    feedbackQueryTimer = setInterval(() => {
        // If we are already running a control command loop, that command itself returns feedback.
        // We only explicitly query 0x74 if no command was written in the last 150ms.
        if (Date.now() - lastWriteTime > 150) {
            sendFeedbackQuery();
        }
    }, feedbackIntervalMs);

    // Monitor connection response timeouts (NFR-REL-02)
    // Give 5 seconds grace period on initial connect before triggering timeout
    lastFeedbackTime = Date.now() + 3000;
    timeoutWarningShown = false;
    feedbackTimeoutCheckInterval = setInterval(() => {
        if (port && (Date.now() - lastFeedbackTime > 2000)) {
            // Only show warning once per timeout cycle
            if (!timeoutWarningShown) {
                timeoutWarningShown = true;
                logMsg('[Warning] Motor not responding... Check power and wiring.', 'warning');
                UI['status-badge'].textContent = 'No Response';
                UI['status-badge'].className = 'text-[10px] font-bold px-2 py-0.5 rounded-full bg-amber-100 text-amber-700 border border-amber-250';
                UI['mode-badge'].classList.add('hidden');
            }
        } else if (port) {
            // Reset warning flag when feedback resumes
            timeoutWarningShown = false;
        }
    }, 2000);
}

function stopFeedbackQueryTimer() {
    if (feedbackQueryTimer) {
        clearInterval(feedbackQueryTimer);
        feedbackQueryTimer = null;
    }
    if (feedbackTimeoutCheckInterval) {
        clearInterval(feedbackTimeoutCheckInterval);
        feedbackTimeoutCheckInterval = null;
    }
}

function sendFeedbackQuery() {
    // Frame: [ID, 0x74, 0, 0, 0, 0, 0, 0, 0, CRC]
    const frame = buildFrame(selectedMotorId, 0x74, [0, 0, 0, 0, 0, 0, 0]);
    writeFrame(frame);
}

// ── ID Management Engine ──────────────────────────────────────
let scannedIds = new Set();

function registerScannedMotor(motorId) {
    if (!scannedIds.has(motorId)) {
        scannedIds.add(motorId);
        updateDiscoveredMotorsList();
        logMsg(`[Scan] DISCOVERED Motor ID: 0x${motorId.toString(16).toUpperCase().padStart(2, '0')} (${motorId})`, 'received');
    }
}

function updateDiscoveredMotorsList() {
    const list = UI['scan-results-list'];
    list.innerHTML = '';

    if (scannedIds.size === 0) {
        list.innerHTML = '<li class="text-slate-500 italic py-1">No motors found</li>';
        return;
    }

    Array.from(scannedIds).sort((a, b) => a - b).forEach(id => {
        const li = document.createElement('li');
        const hex = id.toString(16).toUpperCase().padStart(2, '0');
        li.className = 'bg-blue-50 border border-blue-200 text-blue-700 font-bold px-3 py-1.5 rounded-lg cursor-pointer hover:bg-blue-100 transition-colors flex items-center gap-1';
        li.innerHTML = `<span>ID 0x${hex} (${id})</span>`;
        li.addEventListener('click', () => {
            // Select this motor ID
            selectedMotorId = id;
            UI['motor-id-selector'].value = id;
            sessionStorage.setItem('motorLink_activeId', id);
            logMsg(`[System] Switched active motor target to scanned ID: 0x${hex}`, 'system');
        });
        list.appendChild(li);
    });
}

function sendBroadcastQueryFrame() {
    // Frame: C8 64 00 00 00 00 00 00 00 DE (verbatim)
    const frame = new Uint8Array([0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE]);
    writeFrame(frame);
}

async function runBroadcastScan() {
    if (scanActive) return;

    logMsg('[Scan] Starting broadcast ID query...', 'system');
    scanActive = true;
    scannedIds.clear();
    updateDiscoveredMotorsList();

    // Toggle progress indicators
    UI['scan-progress-container'].classList.remove('hidden');
    UI['scan-progress-bar'].style.width = '100%';
    UI['scan-progress-percent'].textContent = '100%';
    UI['scan-progress-status'].textContent = 'Broadcasting ID query...';

    sendBroadcastQueryFrame();

    await sleep(600);

    UI['scan-progress-container'].classList.add('hidden');
    scanActive = false;
    logMsg(`[Scan] Broadcast scan finished. Discovered ${scannedIds.size} motors.`, 'system');
}

async function runFullScan() {
    if (scanActive) return;

    logMsg('[Scan] Starting full address polling scan (0x01 to 0xFE)...', 'warning');
    scanActive = true;
    scannedIds.clear();
    updateDiscoveredMotorsList();

    UI['scan-progress-container'].classList.remove('hidden');

    const totalIds = 254;
    for (let id = 1; id <= 254; id++) {
        if (!scanActive) break; // Allow cancel / interruption

        const pct = Math.round((id / totalIds) * 100);
        UI['scan-progress-bar'].style.width = `${pct}%`;
        UI['scan-progress-percent'].textContent = `${pct}%`;

        const hexStr = id.toString(16).toUpperCase().padStart(2, '0');
        UI['scan-progress-status'].textContent = `Polling ID 0x${hexStr} (${id}/${totalIds})...`;

        // Write feedback query directly to specific ID
        const frame = buildFrame(id, 0x74, [0, 0, 0, 0, 0, 0, 0]);
        await writeFrame(frame);

        // Timeout per ID: 100ms
        await sleep(100);
    }

    UI['scan-progress-container'].classList.add('hidden');
    scanActive = false;
    logMsg(`[Scan] Full scan completed. Found ${scannedIds.size} motor(s).`, 'system');
}

async function assignNewMotorId() {
    let newId = parseInt(UI['new-id-input'].value, 10);
    if (isNaN(newId) || newId < 1 || newId > 254) {
        alert('Invalid target ID. Please choose a value between 1 and 254 (0x01–0xFE).');
        return;
    }

    const hexTarget = newId.toString(16).toUpperCase().padStart(2, '0');
    const conf = confirm(`Are you sure you want to write a new ID (0x${hexTarget}) to the motor? Verify ONLY ONE motor is connected to the bus.`);
    if (!conf) return;

    logMsg(`[ID Write] Sending ID Set frame to target 0x${hexTarget} exactly 5 times consecutive...`, 'warning');

    // Command structure: AA 55 53 [ID] 00 00 00 00 00 CRC
    // CRC is standard Dallas CRC-8
    const frame = buildFrame(0xAA, 0x55, [0x53, newId, 0, 0, 0, 0, 0]);
    // Override first two elements since buildFrame takes (motorId, cmd)
    // buildFrame(0xAA, 0x55, [0x53, newId...]) makes frame: [0xAA, 0x55, 0x53, newId, 0, 0, 0, 0, 0, CRC]

    for (let i = 0; i < 5; i++) {
        await writeFrame(frame);
        await sleep(20);
    }

    logMsg(`[ID Write] Assignment frames sent. Re-run scan to verify change.`, 'system');

    // Add new option to selection dropdown
    selectedMotorId = newId;
    populateMotorIdSelector();
}

// ── Raw Frame Validator ───────────────────────────────────────
function validateAndCalculateRawFrame(inputVal) {
    // Strip whitespaces
    const clean = inputVal.replace(/\s+/g, '');

    // Check if clean is exactly 18 hex characters (9 bytes)
    const isHex = /^[0-9a-fA-F]{18}$/.test(clean);

    if (!isHex) {
        UI['raw-frame-error'].classList.remove('hidden');
        UI['btn-raw-send'].disabled = true;
        UI['raw-frame-calculated'].innerHTML = '<span class="text-slate-500 italic">Invalid input payload</span>';
        return null;
    }

    UI['raw-frame-error'].classList.add('hidden');
    UI['btn-raw-send'].disabled = false;

    // Convert to bytes array
    const bytes = [];
    for (let i = 0; i < 18; i += 2) {
        bytes.push(parseInt(clean.substr(i, 2), 16));
    }

    // Calculate CRC8
    const crc = crc8Maxim(bytes);
    bytes.push(crc);

    // Render
    const hexFormatted = bytes.map((b, idx) => {
        const str = b.toString(16).toUpperCase().padStart(2, '0');
        if (idx === 9) {
            return `<span class="text-emerald-400 font-bold underline">${str}</span>`;
        }
        return str;
    }).join(' ');

    UI['raw-frame-calculated'].innerHTML = hexFormatted;
    return new Uint8Array(bytes);
}

let rawSendTimer = null;
async function sendRawFrameCommand() {
    const frame = validateAndCalculateRawFrame(UI['raw-frame-input'].value);
    if (!frame) return;

    stopActiveCommandLoop();

    const isRepeat = UI['raw-frame-repeat'].checked;

    if (!isRepeat) {
        writeFrame(frame);
        logMsg('[Raw] Custom frame packet sent once.', 'system');
    } else {
        const count = parseInt(UI['raw-frame-count'].value, 10);
        const interval = parseInt(UI['raw-frame-interval'].value, 10);

        let sentCount = 0;
        logMsg(`[Raw] Starting raw frame loop. Interval ${interval}ms...`, 'warning');

        if (rawSendTimer) clearInterval(rawSendTimer);

        rawSendTimer = setInterval(() => {
            writeFrame(frame);
            sentCount++;

            if (count > 0 && sentCount >= count) {
                clearInterval(rawSendTimer);
                rawSendTimer = null;
                logMsg(`[Raw] Finished sending ${sentCount} repeating frames.`, 'system');
            }
        }, interval);
    }
}

// ── Emergency Stop Logic ──────────────────────────────────────
async function haltMotorFirmly() {
    stopActiveCommandLoop();
    if (rawSendTimer) {
        clearInterval(rawSendTimer);
        rawSendTimer = null;
    }

    if (!port || !port.writable) return;

    // Force velocity mode first to ensure brake works (0x02 = velocity mode)
    const modeFrame = new Uint8Array([selectedMotorId, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02]);
    for (let i = 0; i < 3; i++) {
        await writeFrame(modeFrame);
        await sleep(10);
    }

    // Force velocity to 0 and send electric brake frames 5 times consecutively
    const zeroFrame = buildFrame(selectedMotorId, 0x64, [0, 0, 0, 0, 0, 0, 0]);
    const brakeFrame = buildFrame(selectedMotorId, 0x64, [0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00]);

    for (let i = 0; i < 5; i++) {
        await writeFrame(zeroFrame);
        await sleep(10);
        await writeFrame(brakeFrame);
        await sleep(10);
    }
}

async function triggerEmergencyStop() {
    logMsg('[EMERGENCY STOP] HALTING MOTOR IMMEDIATELY!', 'error');

    await haltMotorFirmly();

    // Reset all controls to safety defaults
    UI['vel-rpm-slider'].value = 0;
    UI['vel-rpm-input'].value = 0;
    currentVelocityRpm = 0;

    UI['curr-val-slider'].value = 0;
    UI['curr-val-input'].value = 0;
    updateCurrentAmpsDisplay(0);

    logMsg('[EMERGENCY STOP] Command sequences completed. Loop controls reset.', 'error');

    // Reveal E-Stop Fullscreen Gradient Overlay Popup
    if (UI['estop-overlay']) {
        UI['estop-overlay'].classList.remove('hidden');
        setTimeout(() => {
            const childDiv = UI['estop-overlay'].querySelector('div');
            if (childDiv) {
                childDiv.classList.remove('scale-95');
                childDiv.classList.add('scale-100');
            }
        }, 50);
    }
}

// ── Custom SVG Plot Charting ──────────────────────────────────
function pushTelemetryToHistory(speed, current) {
    const now = new Date();

    telemetryHistory.speed.push(speed);
    telemetryHistory.current.push(current);
    telemetryHistory.timestamps.push(now);

    // Truncate
    if (telemetryHistory.speed.length > MAX_CHART_POINTS) {
        telemetryHistory.speed.shift();
        telemetryHistory.current.shift();
        telemetryHistory.timestamps.shift();
    }

    // Re-render active chart paths
    renderCharts();
}

function renderCharts() {
    // Render speed and current overlapping lines on the unified `#combined-chart-svg`
    const chartWidth = 480; // Bound horizontally between x=60 and x=540 (NFR-CH-04)

    // 1. Render Speed Path (Left axis: ±350 RPM -> y=20 to y=200)
    if (telemetryHistory.speed.length > 0) {
        const points = [];
        const total = telemetryHistory.speed.length;

        for (let i = 0; i < total; i++) {
            const val = telemetryHistory.speed[i];
            const x = 60 + (i / (MAX_CHART_POINTS - 1)) * chartWidth;
            const y = 110 - (val / 350) * -90;
            const clampedY = Math.max(20, Math.min(200, y));
            points.push(`${x.toFixed(1)},${clampedY.toFixed(1)}`);
        }

        if (points.length > 0) {
            const pathEl = document.getElementById('chart-speed-path');
            if (pathEl) pathEl.setAttribute('d', `M ${points.join(' L ')}`);
        }
    }

    // 2. Render Current Path (Right axis: ±8.0 A -> y=20 to y=200)
    if (telemetryHistory.current.length > 0) {
        const points = [];
        const total = telemetryHistory.current.length;

        for (let i = 0; i < total; i++) {
            const val = telemetryHistory.current[i];
            const x = 60 + (i / (MAX_CHART_POINTS - 1)) * chartWidth;
            const y = 110 - (val / 8.0) * -90;
            const clampedY = Math.max(20, Math.min(200, y));
            points.push(`${x.toFixed(1)},${clampedY.toFixed(1)}`);
        }

        if (points.length > 0) {
            const pathEl = document.getElementById('chart-current-path');
            if (pathEl) pathEl.setAttribute('d', `M ${points.join(' L ')}`);
        }
    }
}

// ── Live Mini Visual Sparklines & Gauges ──────────────────────
function updateMiniVisualWidgets(rawSpeed, currentAmps, posDeg, tempCelsius) {
    // 1. Speed Sparkline SVG path generation (last 40 data points)
    const speedHistory = telemetryHistory.speed.slice(-40);
    if (speedHistory.length > 0) {
        const total = speedHistory.length;
        const points = [];
        for (let i = 0; i < total; i++) {
            const val = speedHistory[i];
            const x = total > 1 ? (i / (total - 1)) * 200 : 0;
            // Center y=60, bounds max ±350 RPM, visual margins keep plot between y=10 and y=110
            const y = Math.max(10, Math.min(110, 60 - (val / 350) * 50));
            points.push(`${x.toFixed(1)},${y.toFixed(1)}`);
        }
        const linePath = `M ${points.join(' L ')}`;
        if (UI['sparkline-speed-path']) UI['sparkline-speed-path'].setAttribute('d', linePath);
        if (UI['sparkline-speed-fill']) {
            UI['sparkline-speed-fill'].setAttribute('d', `${linePath} L 200,120 L 0,120 Z`);
        }
    }

    // 2. Current Sparkline SVG path generation (last 40 data points)
    const currentHistory = telemetryHistory.current.slice(-40);
    if (currentHistory.length > 0) {
        const total = currentHistory.length;
        const points = [];
        for (let i = 0; i < total; i++) {
            const val = currentHistory[i];
            const x = total > 1 ? (i / (total - 1)) * 200 : 0;
            // Center y=60, bounds max ±8.0 A, visual margins keep plot between y=10 and y=110
            const y = Math.max(10, Math.min(110, 60 - (val / 8.0) * 50));
            points.push(`${x.toFixed(1)},${y.toFixed(1)}`);
        }
        const linePath = `M ${points.join(' L ')}`;
        if (UI['sparkline-current-path']) UI['sparkline-current-path'].setAttribute('d', linePath);
        if (UI['sparkline-current-fill']) {
            UI['sparkline-current-fill'].setAttribute('d', `${linePath} L 200,120 L 0,120 Z`);
        }
    }

    // 3. Position Mini Dial Circular Gauge tracking
    if (UI['mini-pos-gauge-fill'] && UI['mini-pos-gauge-dot']) {
        const fillPercent = Math.max(0, Math.min(1, posDeg / 360));
        // Stroke circumference of r=40 is ~251.3 counts
        const offset = 251.3 * (1 - fillPercent);
        UI['mini-pos-gauge-fill'].setAttribute('stroke-dashoffset', offset);

        // Glow pointer dot handle coordinates mapping (radius=40, center cx=50, cy=50)
        const angleRad = posDeg * (Math.PI / 180);
        const dx = 50 + 40 * Math.cos(angleRad);
        const dy = 50 + 40 * Math.sin(angleRad);
        UI['mini-pos-gauge-dot'].setAttribute('cx', dx);
        UI['mini-pos-gauge-dot'].setAttribute('cy', dy);
    }

    // 4. Winding Temperature Semi-Circular Gauge tracking (0 to 80°C boundary limits)
    if (UI['mini-temp-gauge-fill'] && UI['mini-temp-gauge-dot']) {
        const fillPercent = Math.max(0, Math.min(1, tempCelsius / 80));
        // Stroke arc length of semi-circle of r=40 is ~125.66 counts
        const offset = 125.66 * (1 - fillPercent);
        UI['mini-temp-gauge-fill'].setAttribute('stroke-dashoffset', offset);

        // Pointer dot handle coordinates mapping (radius=40, center cx=50, cy=50)
        const angleRad = Math.PI - (fillPercent * Math.PI);
        const dx = 50 + 40 * Math.cos(angleRad);
        const dy = 50 - 40 * Math.sin(angleRad);
        UI['mini-temp-gauge-dot'].setAttribute('cx', dx);
        UI['mini-temp-gauge-dot'].setAttribute('cy', dy);
    }
}

// ── Log Viewer Console ────────────────────────────────────────
function logMsg(message, type = 'system') {
    logCount++;
    updateLogCountBadge();

    const container = UI['log-container'];
    const line = document.createElement('div');
    line.className = `log-line ${type}`;

    const now = new Date();
    const timeStr = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}.${now.getMilliseconds().toString().padStart(3, '0')}`;

    line.innerHTML = `<span class="text-slate-500 font-semibold select-none">[${timeStr}]</span> ${message}`;
    container.appendChild(line);

    // Auto-scroll terminal if enabled
    if (UI['chk-auto-scroll'] && UI['chk-auto-scroll'].checked) {
        container.scrollTop = container.scrollHeight;
    }

    // Limit log lines inside HTML to 300 to prevent browser slowdown
    if (container.children.length > 300) {
        container.removeChild(container.firstChild);
    }
}

function updateLogCountBadge() {
    UI['log-count'].textContent = `${logCount} entries`;
}

function exportLogToCSV() {
    const lines = [];
    // Collect log text content
    const items = UI['log-container'].querySelectorAll('.log-line');

    if (items.length === 0) {
        alert('Console log is empty. Nothing to export.');
        return;
    }

    lines.push('Timestamp,Type,Content');

    items.forEach(el => {
        const text = el.innerText;
        // Parse: [HH:MM:SS.mmm] content
        const matches = text.match(/^\[(.*?)\] (.*)/);
        if (matches) {
            const time = matches[1];
            const content = matches[2].replace(/"/g, '""'); // escape quotes
            let type = 'system';
            if (el.classList.contains('sent')) type = 'TX';
            if (el.classList.contains('received')) type = 'RX';
            if (el.classList.contains('error')) type = 'Error';
            if (el.classList.contains('warning')) type = 'Warning';

            lines.push(`"${time}","${type}","${content}"`);
        }
    });

    const csvContent = "data:text/csv;charset=utf-8," + lines.join("\n");
    const encodedUri = encodeURI(csvContent);
    const link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    link.setAttribute("download", `motorlink_serial_log_${Date.now()}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);

    logMsg('[System] Exported console session log as CSV.', 'system');
}

// ── Utility Helper Functions ──────────────────────────────────
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}
