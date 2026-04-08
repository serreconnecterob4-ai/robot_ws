// =======================================================================
// 3. LOGIQUE DE CONTRÔLE
// =======================================================================



function publishDriveCommand(linearX, angularZ) {
    const hasManualMotion = (linearX !== 0.0 || angularZ !== 0.0);
    let justPausedMission = false;

    // Si mission en cours, un mouvement manuel demande une PAUSE (et non un cancel).
    if (missionActive && !missionPaused && hasManualMotion) {
        if (typeof missionCancelPub !== 'undefined') {
            missionCancelPub.publish(new ROSLIB.Message({ data: 'pause' }));
            justPausedMission = true;
        }

        missionPaused = true;
        if (typeof setEmergencyButtonPausedState === 'function') {
            setEmergencyButtonPausedState(true);
        }

        const info = document.getElementById('trajInfo');
        if (info) info.innerText = '⏸ Mission en pause (contrôle manuel actif)';

        if (typeof showToast === 'function') {
            showToast('⏸️ Mission mise en pause pour contrôle manuel.', 'warn');
        }
        logEvent('Mission mise en pause par commande manuelle', 'warn');
        console.log('Mission mise en pause par mouvement manuel.');
    }

    const msg = new ROSLIB.Message({
        linear: { x: linearX, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angularZ }
    });

    cmdVelPub.publish(msg);

    // Le premier cmd peut arriver juste avant la prise en compte de la pause côté gate.
    // On republie une fois, très court délai, pour garantir la reprise de mouvement.
    if (justPausedMission && hasManualMotion) {
        setTimeout(() => {
            cmdVelPub.publish(msg);
        }, 120);
    }
}

function updateSpeed(val) {
    robotGlobalSpeed = parseInt(val) / 100.0;
    document.getElementById('speedVal').innerText = val + '%';
    localStorage.setItem('robotGlobalSpeed', val);
    logEvent(`Vitesse robot: ${val}%`, 'info');
}

// --- ROBOT (ZQSD) ---
const robotSpeedMultiplier = 3.00; // Ajustement pour compenser les écarts de vitesse
let robotGlobalSpeed = 0.50; // Valeur par défaut à 50%
let robotTurningSpeed = 1.40 * 0.93;

function sendCmd(direction) {
    let linearX = 0.0;
    let angularZ = 0.0;

    if (direction === 'up') linearX = 0.19 * robotGlobalSpeed*robotSpeedMultiplier;
    if (direction === 'down') linearX = -0.19 * robotGlobalSpeed*robotSpeedMultiplier;
    if (direction === 'left') angularZ = robotTurningSpeed * robotGlobalSpeed;
    if (direction === 'right') angularZ = -robotTurningSpeed * robotGlobalSpeed;

    publishDriveCommand(linearX, angularZ);
    logEvent(`Robot: ${direction}`, 'info');
}

function sendCmdFromKeyboardState() {
    const up = !!keyState['z'];
    const down = !!keyState['s'];
    const left = !!keyState['q'];
    const right = !!keyState['d'];

    let linearX = 0.0;
    let angularZ = 0.0;

    if (up && !down) linearX = 0.19 * robotGlobalSpeed*robotSpeedMultiplier;
    else if (down && !up) linearX = -0.19 * robotGlobalSpeed*robotSpeedMultiplier;

    if (left && !right) angularZ = robotTurningSpeed * robotGlobalSpeed;
    else if (right && !left) angularZ = -robotTurningSpeed * robotGlobalSpeed;

    publishDriveCommand(linearX, angularZ);
}

// --- PTZ (OKLM) ---
let ptzHoldInterval = null;

function publishPtzPoint(direction) {
    // Utilisation de Point (x, y)
    const point = new ROSLIB.Message({
        x: 0.0,
        y: 0.0,
        z: 0.0
    });

    if (direction === 'up')    point.x = 1.0;
    if (direction === 'down')  point.x = -1.0;
    if (direction === 'right') point.y = 1.0;
    if (direction === 'left')  point.y = -1.0;

    ptzPub.publish(point);
}

function sendPtz(direction) {
    console.log('sendPtz() appelé avec direction:', direction);
    logEvent(`Caméra PTZ: ${direction}`, 'info');

    // Stop: couper la répétition puis envoyer l'arrêt plusieurs fois
    if (direction === 'stop') {
        if (ptzHoldInterval) {
            clearInterval(ptzHoldInterval);
            ptzHoldInterval = null;
        }
        publishPtzPoint('stop');
        setTimeout(() => publishPtzPoint('stop'), 70);
        setTimeout(() => publishPtzPoint('stop'), 150);
        return;
    }

    // Direction maintenue: publier tout de suite puis en continu
    publishPtzPoint(direction);
    if (ptzHoldInterval) {
        clearInterval(ptzHoldInterval);
    }
    ptzHoldInterval = setInterval(() => publishPtzPoint(direction), 120);
}

// --- CLAVIER ---
const keyState = {};
const keyToButton = {};

document.querySelectorAll('.dpad button').forEach((button) => {
    const hint = button.querySelector('.key-hint');
    if (!hint) return;
    const key = hint.textContent.trim().toLowerCase();
    if (key) keyToButton[key] = button;
});

document.addEventListener('keydown', (event) => {
    const key = event.key.toLowerCase();
    if (keyState[key]) return;
    keyState[key] = true;

    if (keyToButton[key]) keyToButton[key].classList.add('active-key');
    if (['z','q','s','d','o','k','l','m'].includes(key)) logEvent(`Clavier: ${key.toUpperCase()}`, 'info');

    if (['z','q','s','d'].includes(key)) sendCmdFromKeyboardState();

    if (key === 'o') sendPtz('up');
    if (key === 'l') sendPtz('down');
    if (key === 'k') sendPtz('left');
    if (key === 'm') sendPtz('right');
});

document.addEventListener('keyup', (event) => {
    const key = event.key.toLowerCase();
    keyState[key] = false;

    if (keyToButton[key]) keyToButton[key].classList.remove('active-key');

    if (['z','q','s','d'].includes(key)) sendCmdFromKeyboardState();
    if (['o','k','l','m'].includes(key)) sendPtz('stop');
});

// =======================================================================
// 4. AUTRES FONCTIONS
// =======================================================================

function updateZoom(val) {
    const elem = document.getElementById('zoomVal');
    const elemModal = document.getElementById('zoomValModal');
    if (elem) elem.innerText = val + '%';
    if (elemModal) elemModal.innerText = val + '%';
    localStorage.setItem('zoomValue', val);
    zoomPub.publish(new ROSLIB.Message({ data: parseFloat(val) }));
    logEvent(`Zoom optique: ${val}%`, 'info');
}

function updateFocus(val) {
    const elemModal = document.getElementById('focusValModal');
    const focusPos = Math.max(0, Math.min(28, parseInt(val)));

    setAutofocusState(false, { publish: true });
    if (elemModal) elemModal.innerText = focusPos;
    logEvent(`Focus manuel: ${focusPos}`, 'info');
    localStorage.setItem('focusValue', String(focusPos));
    focusPub.publish(new ROSLIB.Message({ data: focusPos }));
}

let autofocusEnabled = true;

function setAutofocusState(enabled, options = {}) {
    autofocusEnabled = enabled;
    localStorage.setItem('autofocusEnabled', enabled ? '1' : '0');

    const btn = document.getElementById('btnAutofocus');
    if (btn) {
        btn.textContent = enabled ? '🎯 Autofocus: ON' : '🎯 Autofocus: OFF';
        btn.classList.toggle('active', enabled);
    }

    const focusSliderModal = document.getElementById('focusSliderModal');
    if (focusSliderModal) {
        focusSliderModal.disabled = enabled;
    }

    if (enabled && options.publish) {
        const focusValModal = document.getElementById('focusValModal');
        if (focusValModal) focusValModal.innerText = 'Auto';
        localStorage.setItem('focusValue', '0');
        autofocusPub.publish(new ROSLIB.Message({ data: true }));
    } else if (!enabled && options.publish) {
        autofocusPub.publish(new ROSLIB.Message({ data: false }));
    }
}

function toggleAutofocus() {
    setAutofocusState(!autofocusEnabled, { publish: true });
}

function updateArmSpeed(val) {
    const elem = document.getElementById('armSpeedVal');
    if (elem) elem.innerText = val + '%';
    localStorage.setItem('armSpeed', val);
    armSpeedPub.publish(new ROSLIB.Message({ data: parseFloat(val) }));
    logEvent(`Vitesse bras: ${val}%`, 'info');
}

function updateArmPos(val) {
    const elem = document.getElementById('armPosVal');
    if (elem) elem.innerText = val + '%';
    localStorage.setItem('armPosition', val);
    armPosPub.publish(new ROSLIB.Message({ data: parseFloat(val) }));
    logEvent(`Position bras: ${val}%`, 'info');
}

function updateRobotVolume(val) {
    const elem = document.getElementById('robotVolumeVal');
    if (elem) elem.innerText = val + '%';
    localStorage.setItem('robotVolume', val);
    robotVolumePub.publish(new ROSLIB.Message({ data: parseFloat(val) }));
    logEvent(`Volume robot: ${val}%`, 'info');
}

function triggerAlert() {
    const btn = document.getElementById('btnAlert');
    alertPub.publish(new ROSLIB.Message({ data: true }));
    if (btn) {
        btn.classList.add('active');
        btn.textContent = '🚨 Alerte Caméra: TEST';
    }
    logEvent('Alerte caméra: TEST', 'warning');

    setTimeout(() => {
        alertPub.publish(new ROSLIB.Message({ data: false }));
        if (btn) {
            btn.classList.remove('active');
            btn.textContent = '🚨 Alerte Caméra';
        }
    }, 1000);
}

// Gestion Lampe et Micro
let lampActive = false;
let micActive = false;

function toggleLamp() {
    const btn = document.getElementById('btnLamp');
    lampActive = !lampActive;
    
    console.log('toggleLamp() appelé, lampActive =', lampActive);
    
    // Publier sur /camera/light
    const lightMsg = new ROSLIB.Message({ data: lampActive });
    lightPub.publish(lightMsg);
    console.log('Message publié sur /camera/light:', lightMsg);
    
    if (lampActive) {
        btn.classList.add('active');
        logEvent('💡 Lampe IR: activée', 'success');
    } else {
        btn.classList.remove('active');
        logEvent('💡 Lampe IR: désactivée', 'warn');
    }
}

function toggleMic() {
    const btn = document.getElementById('btnMic');
    micActive = !micActive;
    if (micActive) {
        btn.classList.add('active');
        logEvent('Micro: activé', 'success');
    } else {
        btn.classList.remove('active');
        logEvent('Micro: désactivé', 'warn');
    }
}
