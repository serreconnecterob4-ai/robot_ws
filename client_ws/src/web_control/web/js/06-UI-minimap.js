// =======================================================================
// GESTION DE LA MODALE REGLAGES
// =======================================================================

function toggleSettings() {
    const modal = document.getElementById('settingsModal');
    // Si c'est affiché (block), on cache (none), sinon on affiche
    modal.style.display = (modal.style.display === 'block') ? 'none' : 'block';
    logEvent(`Réglages: ${modal.style.display === 'block' ? 'ouverts' : 'fermés'}`, 'info');
}

// Gestion du mode sombre
function toggleDarkMode() {
    document.body.classList.toggle('dark-mode');
    
    // Sauvegarder la préférence
    const isDarkMode = document.body.classList.contains('dark-mode');
    localStorage.setItem('darkMode', isDarkMode ? 'enabled' : 'disabled');
    
    // Changer l'icône
    const btn = document.getElementById('btnDarkMode');
    if (btn) {
        btn.textContent = isDarkMode ? '☀️' : '🌙';
    }
    logEvent(`Mode sombre: ${isDarkMode ? 'activé' : 'désactivé'}`, 'info');
}

function setMapModeLockedVisual(locked) {
    const btn = document.getElementById('mapModeBtn');
    if (!btn) return;
    const icon = document.getElementById('mapModeIcon');

    btn.setAttribute('aria-pressed', locked ? 'true' : 'false');
    if (locked) {
        btn.style.background = 'rgba(196,50,50,0.9)';
    } else {
        btn.style.background = 'rgba(0,0,0,0.45)';
    }

    if (icon) {
        icon.querySelectorAll('circle, path').forEach(el => el.setAttribute('stroke', '#fff'));
        const dot = icon.querySelector('circle[r="3"]');
        if (dot) dot.setAttribute('fill', '#fff');
    }
}

// Charger la préférence au démarrage
window.addEventListener('DOMContentLoaded', () => {
    const darkMode = localStorage.getItem('darkMode');
    if (darkMode === 'enabled') {
        document.body.classList.add('dark-mode');
        const btn = document.getElementById('btnDarkMode');
        if (btn) btn.textContent = '☀️';
    }
    logEvent('Interface chargée', 'success');

    // Charger les valeurs sauvegardées des curseurs
    const savedSpeed = localStorage.getItem('robotSpeed');
    if (savedSpeed !== null) {
        const speedSlider = document.getElementById('speedSlider');
        if (speedSlider) {
            speedSlider.value = savedSpeed;
            updateSpeed(savedSpeed);
        }
    }

    const savedZoom = localStorage.getItem('zoomValue');
    if (savedZoom !== null) {
        const zoomSlider = document.getElementById('zoomSlider');
        const zoomSliderModal = document.getElementById('zoomSliderModal');
        if (zoomSlider) zoomSlider.value = savedZoom;
        if (zoomSliderModal) zoomSliderModal.value = savedZoom;
        const zoomVal = document.getElementById('zoomVal');
        const zoomValModal = document.getElementById('zoomValModal');
        if (zoomVal) zoomVal.innerText = savedZoom + '%';
        if (zoomValModal) zoomValModal.innerText = savedZoom + '%';
    }

    const savedFocus = localStorage.getItem('focusValue') || '0';
    const focusSliderModal = document.getElementById('focusSliderModal');
    const normalizedFocus = Math.max(0, Math.min(28, parseInt(savedFocus)) || 0);
    if (focusSliderModal) focusSliderModal.value = normalizedFocus;
    const focusValModal = document.getElementById('focusValModal');
    if (focusValModal) {
        if (normalizedFocus === 0) {
            focusValModal.innerText = 'Auto';
        } else {
            focusValModal.innerText = normalizedFocus;
        }
    }

    const savedAutofocus = localStorage.getItem('autofocusEnabled');
    const autofocusFromStorage = savedAutofocus === null ? (normalizedFocus === 0) : savedAutofocus === '1';
    setAutofocusState(autofocusFromStorage, { publish: false });
    if (!autofocusFromStorage && normalizedFocus !== 0) {
        const focusValModal = document.getElementById('focusValModal');
        if (focusValModal) focusValModal.innerText = normalizedFocus;
    }

    const savedArmSpeed = localStorage.getItem('armSpeed') || '50';
    const armSpeedSlider = document.getElementById('armSpeedSlider');
    if (armSpeedSlider) armSpeedSlider.value = savedArmSpeed;
    const armSpeedVal = document.getElementById('armSpeedVal');
    if (armSpeedVal) armSpeedVal.innerText = savedArmSpeed + '%';

    // Priorite a la nouvelle cle en cm; fallback sur ancienne cle en %.
    const savedArmPosCm = localStorage.getItem('armPositionCm');
    const savedArmPosLegacyPct = localStorage.getItem('armPosition');

    let restoredArmCm = 17.0;
    if (savedArmPosCm !== null) {
        restoredArmCm = Math.max(17.0, Math.min(87.0, Number(savedArmPosCm) || 17.0));
    } else if (savedArmPosLegacyPct !== null) {
        restoredArmCm = percentToHeight(Number(savedArmPosLegacyPct));
    }

    const armPosSlider = document.getElementById('armPosSlider');
    if (armPosSlider) armPosSlider.value = String(restoredArmCm);
    const armPosVal = document.getElementById('armPosVal');
    if (armPosVal) armPosVal.innerText = `${restoredArmCm.toFixed(1)} cm`;

    const savedRobotVolume = localStorage.getItem('robotVolume') || '50';
    const robotVolumeSlider = document.getElementById('robotVolumeSlider');
    if (robotVolumeSlider) robotVolumeSlider.value = savedRobotVolume;
    const robotVolumeVal = document.getElementById('robotVolumeVal');
    if (robotVolumeVal) robotVolumeVal.innerText = savedRobotVolume + '%';

    const navButton = document.querySelector('.nav-link-button');
    if (navButton) {
        navButton.addEventListener('click', () => {
            logEvent('Ouverture page navigation', 'info');
        });
    }

    // Mise en cache des références DOM fréquemment utilisées (évite getElementById à ~60 Hz)
    _mapAreaEl     = document.getElementById('mapArea');
    _robotGroupEl  = document.getElementById('robotGroup');
    _robotHaloEl   = document.getElementById('robotHalo');
    _robotRippleEl = document.getElementById('robotRipple');
    _robotDotEl    = document.getElementById('robotDot');
    if (_robotDotEl && !_robotDotDefaultFill) {
        _robotDotDefaultFill = _robotDotEl.getAttribute('fill') || '#FFD700';
    }
    const mapArea = _mapAreaEl;
    if (mapArea) {
        mapArea.addEventListener('click', handleMapClick);
        // Initialiser le panning de l'image de fond (drag to pan)
        try {
            initMapPanning(mapArea);
            // Pré-chauffe le cache de taille bg (évite toute latence au 1er message ROS)
            getDisplayedBgSize(mapArea).catch(() => {});
            // Setup map mode toggle button
            const btn = document.getElementById('mapModeBtn');
            if (btn) {
                // initial visual state: pan (unlocked)
                btn.setAttribute('aria-pressed', 'false');
                setMapModeLockedVisual(false);

                btn.addEventListener('click', async () => {
                    const controller = mapArea._panController;
                    if (!controller) return;
                    if (controller.mode === 'pan') {
                        if (_odometryOffline) {
                            controller.enable();
                            controller.mode = 'pan';
                            setMapModeLockedVisual(false);
                            btn.title = 'Mode: Pan (clic pour Lock)';
                            showToast('⚠️ Odométrie hors ligne: mode Lock indisponible', 'warn');
                            logEvent('Mode Lock refusé: odométrie hors ligne', 'warn');
                            return;
                        }
                        // switch to lock: prefer last known robot pixel, fallback to dataset or 100,100
                        controller.disable();
                        let lx = 100, ly = 100;
                        if (lastKnownRobotPixel && isFinite(lastKnownRobotPixel.x) && isFinite(lastKnownRobotPixel.y)) {
                            lx = lastKnownRobotPixel.x;
                            ly = lastKnownRobotPixel.y;
                        } else if (mapArea.dataset.lockPixel) {
                            const parts = mapArea.dataset.lockPixel.split(',');
                            lx = parseFloat(parts[0]) || lx;
                            ly = parseFloat(parts[1]) || ly;
                        }
                        await controller.lockToPixel(lx, ly);
                        // remember which pixel we're locked to
                        mapArea.dataset.lockPixel = `${lx},${ly}`;
                        controller.mode = 'lock';
                        setMapModeLockedVisual(true);
                        btn.title = 'Mode: Lock (clic pour Pan)';
                        logEvent(`Carte: mode Lock activé (locked to ${Math.round(lx)},${Math.round(ly)})`, 'info');
                    } else {
                        controller.enable();
                        controller.mode = 'pan';
                        setMapModeLockedVisual(false);
                        btn.title = 'Mode: Pan (clic pour Lock)';
                        logEvent('Carte: mode Pan activé', 'info');
                    }
                });
            }
        } catch (e) {
            console.warn('initMapPanning failed', e);
        }
    }
});

// Initialisation du panning pour la div mapArea
function initMapPanning(mapArea) {
    if (!mapArea) return;

    // state
    if (!mapArea.dataset.bgOffsetX) mapArea.dataset.bgOffsetX = '0';
    if (!mapArea.dataset.bgOffsetY) mapArea.dataset.bgOffsetY = '0';
    let offsetX = parseFloat(mapArea.dataset.bgOffsetX) || 0;
    let offsetY = parseFloat(mapArea.dataset.bgOffsetY) || 0;
    mapArea.style.backgroundPosition = `${offsetX}px ${offsetY}px`;
    mapArea.style.cursor = 'grab';

    let dragging = false;
    let startX = 0, startY = 0;
    let moved = false;
    let pointerId = null;
    let dragPrevTransition = null;

    // named handlers so we can add/remove
    function onPointerDown(ev) {
        // ignore pointerdown originating from the mode button so clicks work
        try {
            if (ev.target && ev.target.closest && ev.target.closest('#mapModeBtn')) return;
        } catch (e) {}
        if (ev.button && ev.button !== 0) return;
        // Resynchroniser les offsets locaux avec l'etat global (peut changer via auto-zoom).
        offsetX = parseFloat(mapArea.dataset.bgOffsetX) || 0;
        offsetY = parseFloat(mapArea.dataset.bgOffsetY) || 0;
        dragging = true; moved = false; pointerId = ev.pointerId;
        startX = ev.clientX; startY = ev.clientY;
        // Pendant le grab, désactiver les transitions pour éviter le décalage background vs overlays.
        dragPrevTransition = mapArea.style.transition;
        mapArea.style.transition = 'none';
        try { mapArea.setPointerCapture(pointerId); } catch(e){}
        mapArea.style.cursor = 'grabbing';
        ev.preventDefault();
    }

    function onPointerMove(ev) {
        if (!dragging || ev.pointerId !== pointerId) return;
        const dx = ev.clientX - startX;
        const dy = ev.clientY - startY;
        const newX = offsetX + dx;
        const newY = offsetY + dy;
        mapArea.style.backgroundPosition = `${newX}px ${newY}px`;
        mapArea.dataset.bgOffsetX = String(newX);  // Sync dataset for robot reading
        mapArea.dataset.bgOffsetY = String(newY);
        moved = Math.abs(dx) > 2 || Math.abs(dy) > 2;
        
        // Redraw trajectoire et point robot en temps réel
        trajectoryOffsetX = newX;
        trajectoryOffsetY = newY;
        updateTrajectoryDisplay(mapArea).catch(() => {});
        if (lastKnownRobotPixel) {
            updateRobotDotOnMap(lastKnownRobotPixel.x, lastKnownRobotPixel.y, _currentlyTakingPhoto).catch(() => {});
        }
        
        ev.preventDefault(  );
    }

    function onEnd(ev) {
        if (!dragging || ev.pointerId !== pointerId) return;
        const dx = ev.clientX - startX;
        const dy = ev.clientY - startY;
        offsetX += dx; offsetY += dy;
        mapArea.dataset.bgOffsetX = String(offsetX);
        mapArea.dataset.bgOffsetY = String(offsetY);
        trajectoryOffsetX = offsetX;  // Sync trajectory offset
        trajectoryOffsetY = offsetY;
        dragging = false; pointerId = null;
        // Restaurer les transitions (auto-zoom smooth) après le drag.
        mapArea.style.transition = dragPrevTransition || '';
        dragPrevTransition = null;
        try { mapArea.releasePointerCapture(ev.pointerId); } catch(e){}
        mapArea.style.cursor = 'grab';
        // Repositionner le dot robot et redraw trajectoire
        if (lastKnownRobotPixel) {
            updateRobotDotOnMap(lastKnownRobotPixel.x, lastKnownRobotPixel.y, false);
        }
        updateTrajectoryDisplay(mapArea);
        ev.preventDefault();
    }

    function onClickPrevent(ev) {
        if (moved) { ev.stopImmediatePropagation(); ev.preventDefault(); moved = false; }
    }

    async function onDblClick(ev) {
        if (typeof recenterMapAreaToDefaultPosition === 'function') {
            await recenterMapAreaToDefaultPosition(mapArea);
            offsetX = parseFloat(mapArea.dataset.bgOffsetX) || 0;
            offsetY = parseFloat(mapArea.dataset.bgOffsetY) || 0;
            mapArea.style.cursor = 'grab';
            logEvent('Carte recentrée', 'info');
            if (lastKnownRobotPixel) {
                updateRobotDotOnMap(lastKnownRobotPixel.x, lastKnownRobotPixel.y, false);
            }
            updateTrajectoryDisplay(mapArea);
            ev.preventDefault();
            return;
        }

        offsetX = 0; offsetY = 0;
        mapArea.dataset.bgOffsetX = '0'; mapArea.dataset.bgOffsetY = '0';
        mapArea.style.backgroundPosition = `${offsetX}px ${offsetY}px`;
        mapArea.style.cursor = 'grab';
        logEvent('Carte recentrée', 'info');
        trajectoryOffsetX = 0;
        trajectoryOffsetY = 0;
        if (lastKnownRobotPixel) {
            updateRobotDotOnMap(lastKnownRobotPixel.x, lastKnownRobotPixel.y, false);
        }
        updateTrajectoryDisplay(mapArea);
        ev.preventDefault();
    }

    const attach = () => {
        mapArea.addEventListener('pointerdown', onPointerDown);
        mapArea.addEventListener('pointermove', onPointerMove);
        mapArea.addEventListener('pointerup', onEnd);
        mapArea.addEventListener('pointercancel', onEnd);
        mapArea.addEventListener('click', onClickPrevent, true);
        mapArea.addEventListener('dblclick', onDblClick);
    };

    const detach = () => {
        mapArea.removeEventListener('pointerdown', onPointerDown);
        mapArea.removeEventListener('pointermove', onPointerMove);
        mapArea.removeEventListener('pointerup', onEnd);
        mapArea.removeEventListener('pointercancel', onEnd);
        mapArea.removeEventListener('click', onClickPrevent, true);
        mapArea.removeEventListener('dblclick', onDblClick);
    };

    // use the global getDisplayedBgSize(mapArea) helper instead

    // lockToPixel: centers given image pixel (px,py) in the mapArea
    async function lockToPixel(px, py) {
        const disp = await getDisplayedBgSize(mapArea);
        const scaleX = disp.scaleX || 1;
        const scaleY = disp.scaleY || 1;
        const centerX = Math.round(mapArea.clientWidth / 2);
        const centerY = Math.round(mapArea.clientHeight / 2);
        const imagePixelX = px * scaleX;
        const imagePixelY = py * scaleY;
        const newOffsetX = centerX - imagePixelX;
        const newOffsetY = centerY - imagePixelY;
        offsetX = newOffsetX; offsetY = newOffsetY;
        mapArea.dataset.bgOffsetX = String(offsetX);
        mapArea.dataset.bgOffsetY = String(offsetY);
        mapArea.style.backgroundPosition = `${offsetX}px ${offsetY}px`;
        // Update trajectory offsets in lock mode
        trajectoryOffsetX = offsetX;
        trajectoryOffsetY = offsetY;
        await updateTrajectoryDisplay(mapArea);
        // Mettre à jour le point du robot après changement d'offset
        if (lastKnownRobotPixel) {
            await updateRobotDotOnMap(lastKnownRobotPixel.x, lastKnownRobotPixel.y, _currentlyTakingPhoto);
        }
    }

    // create controller and attach to element
    const controller = {
        enable() { attach(); },
        disable() { detach(); },
        lockToPixel,
        mode: 'pan'
    };

    // start enabled
    attach();
    mapArea._panController = controller;

    // When in lock mode, ensure recenter on resize
    const ro = new ResizeObserver(async () => {
        // Invalider le cache de taille d'image affichée (dimensions conteneur changées)
        _bgSizeCache   = { key: '', value: null };
        _bgSizePending = null;  // Abandonner tout chargement en cours (taille obsolète)
        if (mapArea._panController && mapArea._panController.mode === 'lock') {
            const parts = (mapArea.dataset.lockPixel || '100,100').split(',');
            const lx = parseFloat(parts[0]) || 100;
            const ly = parseFloat(parts[1]) || 100;
            await mapArea._panController.lockToPixel(lx, ly);
        }
        // Repositionner le dot robot et redraw trajectoire après resize/recalcul du cache
        if (lastKnownRobotPixel) {
            updateRobotDotOnMap(lastKnownRobotPixel.x, lastKnownRobotPixel.y, false);
        }
        await updateTrajectoryDisplay(mapArea);
    });
    ro.observe(mapArea);
    mapArea._panController._resizeObserver = ro;
}

// Fermer la modale si on clique en dehors de la boîte (sur le fond gris)
window.onclick = function(event) {
    const modal = document.getElementById('settingsModal');
    if (event.target == modal) {
        modal.style.display = "none";
    }
};

// =======================================================================
// TOAST NOTIFICATIONS
// =======================================================================

function showToast(message, type = 'info') {
    let container = document.getElementById('toast-container');
    if (!container) {
        container = document.createElement('div');
        container.id = 'toast-container';
        container.style.cssText = `
            position: fixed; top: 80px; right: 20px; z-index: 9999;
            display: flex; flex-direction: column; gap: 8px; pointer-events: none;
        `;
        document.body.appendChild(container);
    }

    const toast = document.createElement('div');
    const colors = { error: '#e74c3c', success: '#2ecc71', warn: '#f39c12', info: '#3498db' };
    toast.style.cssText = `
        background: ${colors[type] || colors.info}; color: white;
        padding: 12px 18px; border-radius: 8px; font-size: 0.95rem;
        box-shadow: 0 4px 12px rgba(0,0,0,0.4); opacity: 0;
        transition: opacity 0.3s ease; max-width: 320px; pointer-events: auto;
    `;
    toast.textContent = message;
    container.appendChild(toast);

    // Apparition
    requestAnimationFrame(() => { toast.style.opacity = '1'; });

    // Disparition après 3s
    setTimeout(() => {
        toast.style.opacity = '0';
        setTimeout(() => toast.remove(), 300);
    }, 3000);
}