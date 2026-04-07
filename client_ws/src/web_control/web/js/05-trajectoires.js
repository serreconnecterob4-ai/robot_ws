// =======================================================================
// TRAJETS : liste, chargement, dessin (mise à l'échelle simple)
// =======================================================================


const originPixel = { x: 160.1, y: 1104.8 };
const metersPerPixel = 2.6617 / 100;
const origin_map_size = { width: 9966, height: 2622 };
const map_size = { width: 1661, height: 437 };
const minimap_meters_per_pixel = metersPerPixel * (origin_map_size.width / map_size.width);
const thetaDegrees = 76.681;


// Gestion Mission
let missionActive = false;
let missionPaused = false;
let currentTrajectoryData = null;
let activeMissionRequestId = null;

// Ratios carte (mis à jour à chaque chargement de trajet)
let mapRatioX = 1, mapRatioY = 1;
let mapOriginalW = 1, mapOriginalH = 1;
// Dernière position pixel connue du robot (mise à jour par mission feedback ou odométrie)
let lastKnownRobotPixel = {x:-0 ,y:-0};
let lastKnownRobotPosition = { x: -0, y: -0 };  // en mètres, mis à jour par feedback de mission
let startPoint_inpixel = {pixel_x: 0, pixel_y:0};
let robot_orientation = 0;
let covariance_radius = 3;
// Index du waypoint actuel pendant une mission
let idx = -1;
let currentWaypointIndex = -1;
// Trajectoire brute pour redraw avec offsets dynamiques
let currentTrajectoryRawPoints = null;  // { startPoint, trajectoryPoints, originalW, originalH }
let trajectoryOffsetX = 0;
let trajectoryOffsetY = 0;
// Cache pour getDisplayedBgSize – invalidé sur resize (évite de recharger l'image à ~50 Hz)
let _bgSizeCache = { key: '', value: null };
// Verrou anti-chargement concurrent de l'image de fond (évite plusieurs new Image() en parallèle)
let _bgSizePending = null;

// Auto-zoom carte autour d'une zone d'interet (sans changer la taille visuelle de mapArea)
const AUTO_ZOOM_TARGET_METERS = { x: -3.582, y: 15.750 };
const AUTO_ZOOM_RADIUS_METERS = 7;
const AUTO_ZOOM_FACTOR = 3.0;
const AUTO_ZOOM_TRANSITION_MS = 380;
let _mapAutoZoomActive = false;
let _mapAutoZoomBaseSize = null;
let _mapAreaInitialInlineBgSize = null;
let _mapAreaInitialInlineTransition = null;
let _mapZoomTransitionToken = 0;
let _mapAutoZoomRestoreSizeTimer = null;
let _mapAutoZoomLockRecenterTimer = null;

const LAST_MISSION_COOKIE_NAME = 'lastMissionTrajectoryName';

function setCookie(name, value, days) {
    const maxAge = Math.max(0, Math.floor(days * 24 * 60 * 60));
    document.cookie = `${name}=${encodeURIComponent(value)}; path=/; max-age=${maxAge}; SameSite=Lax`;
}

function getCookie(name) {
    const prefix = `${name}=`;
    const cookies = document.cookie ? document.cookie.split(';') : [];
    for (const rawCookie of cookies) {
        const cookie = rawCookie.trim();
        if (cookie.startsWith(prefix)) {
            return decodeURIComponent(cookie.substring(prefix.length));
        }
    }
    return '';
}

function saveLastMissionTrajectoryName(filename) {
    if (!filename) return;
    setCookie(LAST_MISSION_COOKIE_NAME, filename, 2);
}

function clearLastMissionTrajectoryName() {
    setCookie(LAST_MISSION_COOKIE_NAME, '', 0);
}

function getLastMissionTrajectoryName() {
    return getCookie(LAST_MISSION_COOKIE_NAME);
}

function unloadCurrentTrajectory(options = {}) {
    const {
        clearSelection = true,
        clearCookie = false,
        clearInfo = false,
    } = options;

    currentTrajectoryData = null;
    currentTrajectoryRawPoints = null;
    currentWaypointIndex = -1;
    idx = -1;

    const polyline = document.getElementById('displayPath');
    const passedPath = document.getElementById('passedPath');
    const startCircle = document.getElementById('startCircle');
    const waypointsGroup = document.getElementById('waypointsGroup');
    const robotToMissionLink = document.getElementById('robotToMissionLink');

    if (polyline) polyline.setAttribute('points', '');
    if (passedPath) passedPath.setAttribute('points', '');
    if (startCircle) startCircle.style.display = 'none';
    if (waypointsGroup) waypointsGroup.innerHTML = '';
    if (robotToMissionLink) robotToMissionLink.setAttribute('display', 'none');

    if (clearSelection) {
        const select = document.getElementById('trajSelect');
        if (select) select.value = '';
    }

    if (clearCookie && typeof clearLastMissionTrajectoryName === 'function') {
        clearLastMissionTrajectoryName();
    }

    if (clearInfo) {
        const info = document.getElementById('trajInfo');
        if (info) info.innerText = 'Aucun trajet chargé';
    }
}

async function loadTrajectoryByFilename(filename, options = {}) {
    const { silent = false, updateSelection = false } = options;
    const info = document.getElementById('trajInfo');

    console.info('[TRAJECTORY] loadTrajectoryByFilename:start', {
        filename,
        silent,
        updateSelection
    });

    if (!filename) {
        throw new Error('Aucun nom de trajet fourni');
    }

    if (!silent && info) info.innerText = '⏳ Chargement...';

    const url = `trajectories/${filename}`;
    console.info('[TRAJECTORY] fetch:start', { url });
    const response = await fetch(url);
    console.info('[TRAJECTORY] fetch:status', { url, ok: response.ok, status: response.status });
    if (!response.ok) {
        throw new Error(`Fichier non trouvé (${response.status})`);
    }

    const data = await response.json();
    console.info('[TRAJECTORY] json:loaded', {
        hasImage: !!data?.image,
        trajectoryCount: Array.isArray(data?.trajectory) ? data.trajectory.length : null
    });
    currentTrajectoryData = data;
    await drawTrajectoryOnMap(data);
    console.info('[TRAJECTORY] drawTrajectoryOnMap:done', { filename });

    if (updateSelection) {
        const select = document.getElementById('trajSelect');
        if (select) {
            const optionExists = Array.from(select.options).some(opt => opt.value === filename);
            if (optionExists) {
                select.value = filename;
            }
        }
    }

    if (!silent && info) {
        const nbPoints = data.trajectory ? data.trajectory.length : 0;
        info.innerText = `✅ Trajet chargé : ${nbPoints} points`;
    }
    if (!silent) {
        logEvent(`Trajet chargé: ${filename}`, 'success');
    }

    console.info('[TRAJECTORY] loadTrajectoryByFilename:success', { filename });

    return data;
}

async function tryLoadLastMissionTrajectoryFromCookie() {
    const filename = getLastMissionTrajectoryName();
    console.info('[TRAJECTORY][COOKIE] restore:attempt', {
        cookieName: LAST_MISSION_COOKIE_NAME,
        filename: filename || null
    });
    if (!filename) {
        console.warn('[TRAJECTORY][COOKIE] restore:skip - cookie vide');
        return false;
    }

    try {
        await loadTrajectoryByFilename(filename, { silent: true, updateSelection: true });
        console.info('[TRAJECTORY][COOKIE] restore:success', { filename });
        logEvent(`Trajet restauré depuis cookie: ${filename}`, 'info');
        return true;
    } catch (err) {
        console.warn('[TRAJECTORY][COOKIE] restore:failed', { filename, error: String(err) });
        logEvent(`Echec restauration trajet cookie: ${filename}`, 'warn');
        return false;
    }
}


function updateTrajectoryList(files) {
    const select = document.getElementById('trajSelect');
    const currentVal = select.value;
    const lastMissionVal = getLastMissionTrajectoryName();
    select.innerHTML = '<option value="">-- Choisir un trajet --</option>';
    files.forEach(file => {
        const opt = document.createElement('option');
        opt.value = file;
        opt.text = file;
        select.appendChild(opt);
    });
    const preferredVal = currentVal || lastMissionVal;
    if (preferredVal && files.includes(preferredVal)) {
        select.value = preferredVal;
    }
}

function loadSelectedTrajectory() {
    const filename = document.getElementById('trajSelect').value;

    if (!filename) {
        const info = document.getElementById('trajInfo');
        if (info) info.innerText = "Aucun trajet sélectionné";
        logEvent('Trajet non sélectionné', 'warn');
        return;
    }

    loadTrajectoryByFilename(filename)
        .catch(err => {
            const info = document.getElementById('trajInfo');
            console.error("Erreur chargement json:", err);
            if (info) info.innerText = "❌ Erreur lors du chargement";
            logEvent(`Erreur chargement trajet: ${filename}`, 'error');
        });
}

async function drawTrajectoryOnMap(data) {
    const mapArea = document.getElementById('mapArea');
    if (!mapArea) return;

    const originalW = data.image.width;
    const originalH = data.image.height;

    // Invalider le cache avant premier affichage (assure mesure correcte)
    _bgSizeCache = { key: '', value: null };
    _bgSizePending = null;
    const disp = await getDisplayedBgSize(mapArea);
    // Mémoriser pour positionner le rond robot depuis le feedback
    mapRatioX   = disp.w / originalW;
    mapRatioY   = disp.h / originalH;
    mapOriginalW = originalW;
    mapOriginalH = originalH;

    // Stocker les points bruts pour redraw dynamique
    let trajectoryPoints = [];
    if (data.trajectory && Array.isArray(data.trajectory)) {
        // Conserver les flags (photography, etc.) pour pouvoir les utiliser au redraw
        data.trajectory.forEach((p, pointIndex) => {
            const rawX = (typeof p.x === 'number') ? p.x : 0;
            const rawY = (typeof p.y === 'number') ? p.y : 0;
            const photography = p.photography !== undefined ? p.photography : false;
            trajectoryPoints.push({ x: rawX, y: rawY, photography });
        });
    }
    // Si une mission est active, tronquer les points déjà dépassés
    // if (missionActive && currentWaypointIndex + 1 <= trajectoryPoints.length) {
    //     trajectoryPoints = trajectoryPoints.slice(currentWaypointIndex + 1);
    // }
    if(_odometryOffline && !missionActive) {
        startPoint_inpixel = {pixel_x: data.startPoint.x , pixel_y: data.startPoint.y};
    }
    else {
        startPoint_inpixel = meters_to_pixels(lastKnownRobotPosition.x, lastKnownRobotPosition.y, originalW, originalH);
    }
    currentTrajectoryRawPoints = {
        // Si mission active, utiliser la position du robot; sinon utiliser le startPoint des données
        startPoint: startPoint_inpixel ,
        trajectoryPoints,
        originalW,
        originalH
    };
    // console.log('Points bruts de la trajectoire chargée :', currentTrajectoryRawPoints);
    // Syncer les offsets avec l'état actuel du mapArea (au lieu de les reset à 0)
    trajectoryOffsetX = parseFloat(mapArea.dataset.bgOffsetX) || 0;
    trajectoryOffsetY = parseFloat(mapArea.dataset.bgOffsetY) || 0;

    // Redraw avec offsets (initialement 0)
    await updateTrajectoryDisplay(mapArea);
}

// Redraw la trajectoire en appliquant échelle + offsets (appelé lors du pan, resize, etc.)
async function updateTrajectoryDisplay(mapArea) {
    if (!mapArea || !currentTrajectoryRawPoints) return;

    const polyline = document.getElementById('displayPath');
    const startCircle = document.getElementById('startCircle');
    const waypointsGroup = document.getElementById('waypointsGroup');
    if (!polyline || !startCircle || !waypointsGroup) return;

    
    const disp = await getDisplayedBgSize(mapArea);
    const displayW = disp.w;
    const displayH = disp.h;
    const { originalW, originalH, startPoint, trajectoryPoints } = currentTrajectoryRawPoints;
    let upcomingStr = '';
    let pointsStr = '';

    const ratioX = displayW / originalW;
    const ratioY = displayH / originalH;


    const start_px = startPoint.pixel_x * ratioX + trajectoryOffsetX;
    const start_py = startPoint.pixel_y * ratioY + trajectoryOffsetY;

    startCircle.setAttribute('cx', start_px);
    startCircle.setAttribute('cy', start_py);

    if(_odometryOffline && !missionActive) {

        pointsStr = `${start_px},${start_py}`;
    }
    startCircle.style.display = (missionActive) ? 'block' : 'none';

    // Vider le groupe des waypoints
    waypointsGroup.innerHTML = '';

    // Les segments passés ne sont affichés que pendant une mission réellement active.
    const shouldShowPassedPath = missionActive;

    // Pré-calculer toutes les coordonnées projetées pour la trajectoire complète
    const projected = (Array.isArray(trajectoryPoints) ? trajectoryPoints : []).map(p => ({
        px: p.x * ratioX + trajectoryOffsetX,
        py: p.y * ratioY + trajectoryOffsetY,
        photography: p.photography
    }));

    // currentWaypointIndex représente le waypoint en cours de ciblage.
    // On affiche en cyan à partir de cet index (pas +1), sinon on retire un segment en trop.
    const safeStart = (shouldShowPassedPath && typeof currentWaypointIndex === 'number' && !isNaN(currentWaypointIndex))
        ? Math.min(Math.max(0, currentWaypointIndex), projected.length)
        : 0;

    // Construire la chaîne pour les segments passés (gris) de manière à
    // placer le startPoint entre passé et à-venir.
    let passedStr = '';
    if (safeStart > 0 && projected.length > 0) {
        // Tracer une ligne de projected[0] jusqu'à projected[safeStart-1] (inclus), puis au startPoint
        passedStr = `${projected[0].px},${projected[0].py}`;
        // Ajouter tous les points passés jusqu'à l'index currentWaypointIndex (= safeStart - 1)
        for (let i = 1; i < safeStart && i < projected.length; i++) {
            passedStr += ` ${projected[i].px},${projected[i].py}`;
        }
        // Pendant la mission, raccorder la partie passée à la position robot pour garder un tracé visible.
        if (missionActive || (!missionActive && _odometryOffline)) {
            passedStr += ` ${start_px},${start_py}`;
        }
    } else {
        // aucun point passé — pas de ligne grise
        passedStr = '';
    }

    // console.log('[TRAJECTORY] safeStart=', safeStart, 'currentWaypointIndex=', currentWaypointIndex, 'missionActive=', missionActive);
    // console.log('[TRAJECTORY] passedStr=', passedStr);

    // Construire la chaîne pour les segments à venir (cyan) en commençant par startPoint
    for (let i = safeStart; i < projected.length; i++) {
        upcomingStr += ` ${projected[i].px},${projected[i].py}`;
    }

    // Mettre à jour les polylines
    const passedPath = document.getElementById('passedPath');
    let robotToMissionLink = document.getElementById('robotToMissionLink');
    if (!robotToMissionLink && polyline.parentNode) {
        robotToMissionLink = document.createElementNS('http://www.w3.org/2000/svg', 'line');
        robotToMissionLink.setAttribute('id', 'robotToMissionLink');
        robotToMissionLink.setAttribute('stroke', '#8de7ff');
        robotToMissionLink.setAttribute('stroke-width', '2');
        robotToMissionLink.setAttribute('stroke-opacity', '0.45');
        robotToMissionLink.setAttribute('display', 'none');
        polyline.parentNode.insertBefore(robotToMissionLink, polyline);
    }

    const hasLoadedMission = projected.length > 0;
    const robotOnline = (typeof _odometryOffline !== 'undefined') ? !_odometryOffline : false;
    if (robotToMissionLink) {
        const firstCheckpointPassed = missionActive && typeof currentWaypointIndex === 'number' && currentWaypointIndex >= 1;
        if (robotOnline && hasLoadedMission && !firstCheckpointPassed) {
            const firstPoint = projected[0];
            robotToMissionLink.setAttribute('x1', String(start_px));
            robotToMissionLink.setAttribute('y1', String(start_py));
            robotToMissionLink.setAttribute('x2', String(firstPoint.px));
            robotToMissionLink.setAttribute('y2', String(firstPoint.py));
            robotToMissionLink.setAttribute('display', 'block');
        } else {
            robotToMissionLink.setAttribute('display', 'none');
        }
    }

    // console.log('[TRAJECTORY] passedPath element exists?', !!passedPath);
    if (passedPath) {
        passedPath.setAttribute('points', passedStr);
        // console.log('[TRAJECTORY] passedPath points updated');
    } else {
        // console.warn('[TRAJECTORY] passedPath element not found!');
    }
    polyline.setAttribute('points', upcomingStr);

    // Dessiner les cercles des waypoints (uniquement pour les points affichés)
    const visiblePoints = projected.slice(safeStart);
    visiblePoints.forEach((p, index) => {
        const circle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
        circle.setAttribute('cx', p.px);
        circle.setAttribute('cy', p.py);
        circle.setAttribute('r', '4');
        if (p.photography == "yes") {
            circle.setAttribute('fill', '#ff9800');
            circle.setAttribute('stroke', '#ff6f00');
        } else {
            circle.setAttribute('fill', 'cyan');
            circle.setAttribute('stroke', '#0099cc');
        }
        circle.setAttribute('stroke-width', '1');
        circle.setAttribute('opacity', '0.8');
        waypointsGroup.appendChild(circle);
    });
}

async function updateMapAutoZoomByRobotPosition() {
    const mapArea = _mapAreaEl || document.getElementById('mapArea');
    if (!mapArea) return;

    const rx = lastKnownRobotPosition?.x;
    const ry = lastKnownRobotPosition?.y;
    if (!isFinite(rx) || !isFinite(ry)) return;

    const dx = rx - AUTO_ZOOM_TARGET_METERS.x;
    const dy = ry - AUTO_ZOOM_TARGET_METERS.y;
    const shouldZoom = Math.hypot(dx, dy) <= AUTO_ZOOM_RADIUS_METERS;

    if (shouldZoom === _mapAutoZoomActive) return;

    if (_mapAreaInitialInlineBgSize === null) {
        _mapAreaInitialInlineBgSize = mapArea.style.backgroundSize || '';
    }

    const controller = mapArea._panController;
    const lockMode = !!(controller && controller.mode === 'lock');

    const currentDisp = await getDisplayedBgSize(mapArea);
    const currentOffsetX = parseFloat(mapArea.dataset.bgOffsetX) || 0;
    const currentOffsetY = parseFloat(mapArea.dataset.bgOffsetY) || 0;

    if (_mapAreaInitialInlineTransition === null) {
        _mapAreaInitialInlineTransition = mapArea.style.transition || '';
    }

    // Verrouiller l'etat de depart en valeurs absolues pour eviter le saut visuel initial.
    mapArea.style.transition = 'none';
    mapArea.style.backgroundSize = `${currentDisp.w}px ${currentDisp.h}px`;
    mapArea.style.backgroundPosition = `${currentOffsetX}px ${currentOffsetY}px`;
    // Force le navigateur a appliquer cet etat avant de lancer la transition.
    void mapArea.offsetWidth;

    mapArea.style.transition = `background-size ${AUTO_ZOOM_TRANSITION_MS}ms ease, background-position ${AUTO_ZOOM_TRANSITION_MS}ms ease`;

    if (shouldZoom) {
        const baseDisp = currentDisp;
        _mapAutoZoomBaseSize = { w: baseDisp.w, h: baseDisp.h };

        const oldOffsetX = currentOffsetX;
        const oldOffsetY = currentOffsetY;
        const oldScaleX = baseDisp.scaleX || 1;
        const oldScaleY = baseDisp.scaleY || 1;
        const anchor = getMapZoomAnchorImagePixel(mapArea, baseDisp, oldOffsetX, oldOffsetY);

        const zoomedW = baseDisp.w * AUTO_ZOOM_FACTOR;
        const zoomedH = baseDisp.h * AUTO_ZOOM_FACTOR;
        const newScaleX = oldScaleX * AUTO_ZOOM_FACTOR;
        const newScaleY = oldScaleY * AUTO_ZOOM_FACTOR;
        let newOffsetX = oldOffsetX - anchor.x * (newScaleX - oldScaleX);
        let newOffsetY = oldOffsetY - anchor.y * (newScaleY - oldScaleY);
        if (lockMode && lastKnownRobotPixel && isFinite(lastKnownRobotPixel.x) && isFinite(lastKnownRobotPixel.y)) {
            const centerX = mapArea.clientWidth / 2;
            const centerY = mapArea.clientHeight / 2;
            newOffsetX = centerX - lastKnownRobotPixel.x * newScaleX;
            newOffsetY = centerY - lastKnownRobotPixel.y * newScaleY;
        }

        mapArea.style.backgroundSize = `${zoomedW}px ${zoomedH}px`;
        mapArea.style.backgroundPosition = `${newOffsetX}px ${newOffsetY}px`;
        mapArea.dataset.bgOffsetX = String(newOffsetX);
        mapArea.dataset.bgOffsetY = String(newOffsetY);
        trajectoryOffsetX = newOffsetX;
        trajectoryOffsetY = newOffsetY;
    } else {
        const zoomedDisp = currentDisp;
        const targetW = _mapAutoZoomBaseSize?.w || (zoomedDisp.w / AUTO_ZOOM_FACTOR);
        const targetH = _mapAutoZoomBaseSize?.h || (zoomedDisp.h / AUTO_ZOOM_FACTOR);
        const oldOffsetX = currentOffsetX;
        const oldOffsetY = currentOffsetY;
        const oldScaleX = zoomedDisp.scaleX || 1;
        const oldScaleY = zoomedDisp.scaleY || 1;
        const naturalW = oldScaleX > 0 ? (zoomedDisp.w / oldScaleX) : 1;
        const naturalH = oldScaleY > 0 ? (zoomedDisp.h / oldScaleY) : 1;
        const newScaleX = targetW / naturalW;
        const newScaleY = targetH / naturalH;
        const anchor = getMapZoomAnchorImagePixel(mapArea, zoomedDisp, oldOffsetX, oldOffsetY);
        let newOffsetX = oldOffsetX - anchor.x * (newScaleX - oldScaleX);
        let newOffsetY = oldOffsetY - anchor.y * (newScaleY - oldScaleY);
        if (lockMode && lastKnownRobotPixel && isFinite(lastKnownRobotPixel.x) && isFinite(lastKnownRobotPixel.y)) {
            const centerX = mapArea.clientWidth / 2;
            const centerY = mapArea.clientHeight / 2;
            newOffsetX = centerX - lastKnownRobotPixel.x * newScaleX;
            newOffsetY = centerY - lastKnownRobotPixel.y * newScaleY;
        }

        // Dezoomer vers une taille explicite en px (animable), puis restaurer le style par defaut.
        mapArea.style.backgroundSize = `${targetW}px ${targetH}px`;
        mapArea.style.backgroundPosition = `${newOffsetX}px ${newOffsetY}px`;
        mapArea.dataset.bgOffsetX = String(newOffsetX);
        mapArea.dataset.bgOffsetY = String(newOffsetY);
        trajectoryOffsetX = newOffsetX;
        trajectoryOffsetY = newOffsetY;

        if (_mapAutoZoomRestoreSizeTimer) {
            clearTimeout(_mapAutoZoomRestoreSizeTimer);
            _mapAutoZoomRestoreSizeTimer = null;
        }
        _mapAutoZoomRestoreSizeTimer = setTimeout(() => {
            if (!_mapAutoZoomActive) {
                mapArea.style.backgroundSize = _mapAreaInitialInlineBgSize;
                _bgSizeCache = { key: '', value: null };
                _bgSizePending = null;
            }
            _mapAutoZoomRestoreSizeTimer = null;
        }, AUTO_ZOOM_TRANSITION_MS + 40);

        _mapAutoZoomBaseSize = null;
    }

    _mapAutoZoomActive = shouldZoom;
    _bgSizeCache = { key: '', value: null };
    _bgSizePending = null;

    if (_mapAutoZoomLockRecenterTimer) {
        clearTimeout(_mapAutoZoomLockRecenterTimer);
        _mapAutoZoomLockRecenterTimer = null;
    }
    if (lockMode && controller && lastKnownRobotPixel && isFinite(lastKnownRobotPixel.x) && isFinite(lastKnownRobotPixel.y)) {
        _mapAutoZoomLockRecenterTimer = setTimeout(async () => {
            try {
                if (controller.mode === 'lock' && lastKnownRobotPixel && isFinite(lastKnownRobotPixel.x) && isFinite(lastKnownRobotPixel.y)) {
                    await controller.lockToPixel(lastKnownRobotPixel.x, lastKnownRobotPixel.y);
                }
            } catch (e) {
                console.warn('[AUTO ZOOM][LOCK] recenter final error:', e);
            } finally {
                _mapAutoZoomLockRecenterTimer = null;
            }
        }, AUTO_ZOOM_TRANSITION_MS + 50);
    }

    startSmoothMapTransitionSync(mapArea, AUTO_ZOOM_TRANSITION_MS + 80);
    await updateTrajectoryDisplay(mapArea);
    if (lastKnownRobotPixel && isFinite(lastKnownRobotPixel.x) && isFinite(lastKnownRobotPixel.y)) {
        await updateRobotDotOnMap(lastKnownRobotPixel.x, lastKnownRobotPixel.y, _currentlyTakingPhoto);
    }
}

function startSmoothMapTransitionSync(mapArea, durationMs) {
    const localToken = ++_mapZoomTransitionToken;
    const start = performance.now();

    const tick = () => {
        if (localToken !== _mapZoomTransitionToken) return;
        const elapsed = performance.now() - start;

        _bgSizeCache = { key: '', value: null };
        _bgSizePending = null;

        updateTrajectoryDisplay(mapArea).catch(() => {});
        if (lastKnownRobotPixel && isFinite(lastKnownRobotPixel.x) && isFinite(lastKnownRobotPixel.y)) {
            updateRobotDotOnMap(lastKnownRobotPixel.x, lastKnownRobotPixel.y, _currentlyTakingPhoto).catch(() => {});
        }

        if (elapsed < durationMs) {
            requestAnimationFrame(tick);
        }
    };

    requestAnimationFrame(tick);
}

function getMapZoomAnchorImagePixel(mapArea, disp, offsetX, offsetY) {
    if (lastKnownRobotPixel && isFinite(lastKnownRobotPixel.x) && isFinite(lastKnownRobotPixel.y) && !_odometryOffline) {
        return { x: lastKnownRobotPixel.x, y: lastKnownRobotPixel.y };
    }

    const scaleX = disp.scaleX || 1;
    const scaleY = disp.scaleY || 1;
    const centerScreenX = mapArea.clientWidth / 2;
    const centerScreenY = mapArea.clientHeight / 2;
    const centerImageX = (centerScreenX - offsetX) / scaleX;
    const centerImageY = (centerScreenY - offsetY) / scaleY;
    return { x: centerImageX, y: centerImageY };
}

async function recenterMapAreaToDefaultPosition(mapArea) {
    if (!mapArea) return;

    _bgSizeCache = { key: '', value: null };
    _bgSizePending = null;
    const disp = await getDisplayedBgSize(mapArea);
    const centeredOffsetX = (mapArea.clientWidth - disp.w) / 2;
    const centeredOffsetY = (mapArea.clientHeight - disp.h) / 2;

    mapArea.style.backgroundPosition = `${centeredOffsetX}px ${centeredOffsetY}px`;
    mapArea.dataset.bgOffsetX = String(centeredOffsetX);
    mapArea.dataset.bgOffsetY = String(centeredOffsetY);
    trajectoryOffsetX = centeredOffsetX;
    trajectoryOffsetY = centeredOffsetY;
}

function meters_to_pixels(x, y, imageSize_x , imageSize_y) {

    const ratio_conversion_width = origin_map_size.width / imageSize_x;
    const ratio_conversion_height = origin_map_size.height / imageSize_y;

    const thetaRad = thetaDegrees * Math.PI / 180;
    const cosTheta = Math.cos(thetaRad);
    const sinTheta = Math.sin(thetaRad);

    // ------ CALCULUS 

    const meters_y_inv = (y*(cosTheta/sinTheta)-x) / (sinTheta+cosTheta*cosTheta/sinTheta);
    const meters_x = (x + meters_y_inv*sinTheta) / cosTheta;

    const meters_y = - meters_y_inv;

    const dx = meters_x / metersPerPixel;
    const dy = meters_y / metersPerPixel;

    const converted_x = dx + originPixel.x;
    const converted_y = dy + originPixel.y;

    const original_x = converted_x / ratio_conversion_width;
    const original_y = converted_y / ratio_conversion_height;
    
    return { pixel_x: original_x, pixel_y: original_y};
}

// Retourne la taille affichée (en pixels) de l'image de fond de la div `mapArea`.
// Renvoie un objet { w, h, scaleX, scaleY }. Résultat mis en cache par dimensions du conteneur.
function getDisplayedBgSize(mapArea) {
    if (!mapArea) return Promise.resolve({ w: 0, h: 0, scaleX: 1, scaleY: 1 });
    const cacheKey = `${mapArea.clientWidth}x${mapArea.clientHeight}`;
    if (_bgSizeCache.key === cacheKey && _bgSizeCache.value) {
        return Promise.resolve(_bgSizeCache.value);
    }
    // Réutiliser le chargement en cours (évite plusieurs new Image() simultanées)
    if (_bgSizePending) return _bgSizePending;

    const cs = getComputedStyle(mapArea);
    const bg = cs.backgroundImage || '';
    const bgSize = (cs.backgroundSize || 'auto').trim();
    const m = bg.match(/url\(("|'|)(.*?)\1\)/);
    const src = m ? m[2] : null;
    if (!src) {
        const fallback = { w: mapArea.clientWidth, h: mapArea.clientHeight, scaleX: 1, scaleY: 1 };
        _bgSizeCache = { key: cacheKey, value: fallback };
        return Promise.resolve(fallback);
    }

    const containerW = mapArea.clientWidth;
    const containerH = mapArea.clientHeight;
    _bgSizePending = new Promise((resolve) => {
        const img = new Image();
        img.onload = () => {
            const naturalW = img.naturalWidth;
            const naturalH = img.naturalHeight;
            let dispW, dispH;
            if (bgSize === 'cover') {
                const scale = Math.max(containerW / naturalW, containerH / naturalH);
                dispW = naturalW * scale; dispH = naturalH * scale;
            } else if (bgSize === 'contain') {
                const scale = Math.min(containerW / naturalW, containerH / naturalH);
                dispW = naturalW * scale; dispH = naturalH * scale;
            } else if (bgSize === 'auto' || bgSize === 'auto auto') {
                dispW = naturalW; dispH = naturalH;
            } else {
                const parts = bgSize.split(/\s+/);
                dispW = parseBgDim(parts[0], containerW, naturalW);
                dispH = parseBgDim(parts[1] || 'auto', containerH, naturalH);
            }
            const result = { w: dispW, h: dispH, scaleX: dispW / naturalW, scaleY: dispH / naturalH };
            _bgSizeCache = { key: cacheKey, value: result };
            _bgSizePending = null;
            resolve(result);
        };
        img.onerror = () => {
            const fallback = { w: containerW, h: containerH, scaleX: 1, scaleY: 1 };
            _bgSizeCache = { key: cacheKey, value: fallback };
            _bgSizePending = null;
            resolve(fallback);
        };
        img.src = src;
    });
    return _bgSizePending;
}

/** Parse une dimension CSS de background-size en pixels absolus. */
function parseBgDim(val, containerSize, naturalSize) {
    if (!val || val === 'auto') return naturalSize;
    if (val.endsWith('px')) return parseFloat(val);
    if (val.endsWith('%')) return containerSize * parseFloat(val) / 100;
    return naturalSize;
}


/**
 * Place le rond jaune (+ halo bleu si prise de photo) sur la carte.
 * x/y sont dans le référentiel de la carte (mêmes coordonnées que gps_x/gps_y).
 */
async function updateRobotDotOnMap(pixelX, pixelY, isTakingPhoto) {
    const group   = _robotGroupEl  || document.getElementById('robotGroup');
    const mapArea = _mapAreaEl     || document.getElementById('mapArea');
    if (!group || !mapArea) return;

    const disp    = await getDisplayedBgSize(mapArea);
    const screenX = pixelX * (disp.scaleX || 1) + (parseFloat(mapArea.dataset.bgOffsetX) || 0);
    const screenY = pixelY * (disp.scaleY || 1) + (parseFloat(mapArea.dataset.bgOffsetY) || 0);

    group.setAttribute('transform', `translate(${screenX}, ${screenY})`);
    group.style.opacity = '1';

    const heading = document.getElementById('robotHeading');
    if (heading) {
        const headingDeg = (robot_orientation || 0) * 180 / Math.PI + 90;
        heading.setAttribute('transform', `rotate(${headingDeg} 0 0)`);
    }

    const covCircle = document.getElementById('robotCovariance');
    if (covCircle) {
        let covRadiusPx = 3;
        if (isFinite(covariance_radius) && covariance_radius > 0 &&
            isFinite(minimap_meters_per_pixel) && minimap_meters_per_pixel > 0) {
            const imageRadiusPx = covariance_radius / minimap_meters_per_pixel;
            const scale = ((disp.scaleX || 1) + (disp.scaleY || 1)) / 2;
            covRadiusPx = Math.max(3, imageRadiusPx * scale);
        }
        covCircle.setAttribute('r', String(covRadiusPx));
    }

    // Halo de respiration + anneau d'expansion : visibles uniquement lors d'une prise de photo
    const halo   = _robotHaloEl   || document.getElementById('robotHalo');
    const ripple = _robotRippleEl || document.getElementById('robotRipple');
    if (isTakingPhoto) {
        halo?.classList.add('pulsing');
        ripple?.classList.add('pulsing');
    } else {
        halo?.classList.remove('pulsing');
        ripple?.classList.remove('pulsing');
    }
}

/** Cache le rond robot (fin/annulation de mission). */
function hideRobotDot() {
    const group = _robotGroupEl || document.getElementById('robotGroup');
    if (group) {
        group.style.opacity = '0';
        (_robotHaloEl   || document.getElementById('robotHalo'))  ?.classList.remove('pulsing');
        (_robotRippleEl || document.getElementById('robotRipple'))?.classList.remove('pulsing');
    }
}

