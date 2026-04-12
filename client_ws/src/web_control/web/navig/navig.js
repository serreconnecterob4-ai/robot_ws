// --- Cookie utility functions ---
function setCookie(name, value, days) {
    const expires = new Date();
    expires.setTime(expires.getTime() + days * 24 * 60 * 60 * 1000);
    document.cookie = `${name}=${value};expires=${expires.toUTCString()};path=/`;
}

function getCookie(name) {
    const nameEQ = name + '=';
    const ca = document.cookie.split(';');
    for (let i = 0; i < ca.length; i++) {
        let c = ca[i];
        while (c.charAt(0) === ' ') c = c.substring(1, c.length);
        if (c.indexOf(nameEQ) === 0) return c.substring(nameEQ.length, c.length);
    }
    return null;
}

document.addEventListener('DOMContentLoaded', () => {
    // --- DOM Elements ---
    const mapContainer = document.getElementById('map-container');
    const mapWorld = document.getElementById('map-world');
    const mapImage = document.getElementById('map-image');
    const mapCostmapOverlay = document.getElementById('map-costmap-overlay');
    const trajectoryLayer = document.getElementById('trajectory-lines');
    const trajectoryPath = document.getElementById('trajectory-path');
    const robotGroup = document.getElementById('robot-group');
    const robotDot = document.getElementById('robot-dot');
    const robotHalo = document.getElementById('robot-halo');
    const robotRipple = document.getElementById('robot-ripple');
    const robotHeading = document.getElementById('robot-heading');
    const robotCovariance = document.getElementById('robot-covariance');
    const robotOfflineBadge = document.getElementById('robot-offline-badge');
    const gpsCoordsDisplay = document.getElementById('gps-coords');
    const pointList = document.getElementById('point-list');
    const clearTrajectoryBtn = document.getElementById('clear-trajectory');
    const removeLastPointBtn = document.getElementById('remove-last-point');
    const toggleEditModeBtn = document.getElementById('toggle-edit-mode');
    const clearForbiddenAreasBtn = document.getElementById('clear-forbidden-areas');
    const resetForbiddenAreasBtn = document.getElementById('reset-forbidden-areas');
    const saveTrajectoryBtn = document.getElementById('save-trajectory');
    const trajectoryButtons = document.getElementById('trajectory-buttons');
    const editButtons = document.getElementById('edit-buttons');
    const savedTrajectoriesList = document.getElementById('saved-trajectories-list');
    const zoomInBtn = document.getElementById('zoom-in');
    const zoomOutBtn = document.getElementById('zoom-out');
    const darkModeToggle = document.getElementById('dark-mode-toggle');
    const headerSettingsBtn = document.getElementById('header-settings-btn');
    const headerFontSizeSlider = document.getElementById('header-font-size-slider');
    const headerFontSizeLabel = document.getElementById('header-font-size-label');
    const headerToggleEditModeBtn = document.getElementById('header-toggle-edit-mode');
    const headerClearForbiddenAreasBtn = document.getElementById('header-clear-forbidden-areas');
    const headerResetForbiddenAreasBtn = document.getElementById('header-reset-forbidden-areas');
    const headerEditButtons = document.getElementById('header-edit-buttons');

    // --- ROS Connection ---
    const ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090' // Adjust if your ROS bridge is elsewhere
    });
    ros.on('connection', () => console.log('Connected to websocket server.'));
    ros.on('error', (error) => console.log('Error connecting to websocket server: ', error));
    ros.on('close', () => console.log('Connection to websocket server closed.'));

    // --- ROS Topic ---
    const trajectoryTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/web_trajectory',
        messageType: 'geometry_msgs/PoseArray'
    });
    
    // Topic pour sauvegarder les trajectoires
    const saveTrajectoryPub = new ROSLIB.Topic({
        ros: ros,
        name: '/ui/save_trajectory',
        messageType: 'std_msgs/String'
    });
    
    // Topic pour supprimer une trajectoire
    const deleteTrajectoryPub = new ROSLIB.Topic({
        ros: ros,
        name: '/ui/delete_trajectory',
        messageType: 'std_msgs/String'
    });
    
    // Topic pour recevoir la liste des trajectoires
    const trajListSub = new ROSLIB.Topic({
        ros: ros,
        name: '/ui/trajectory_files',
        messageType: 'std_msgs/String'
    });

    const odometryFilteredSub = new ROSLIB.Topic({
        ros: ros,
        name: '/odometry/filtered',
        messageType: 'nav_msgs/Odometry'
    });

    const missionFeedbackSub = new ROSLIB.Topic({
        ros: ros,
        name: '/ui/mission_feedback',
        messageType: 'std_msgs/String'
    });
    
    // S'abonner à la liste des trajectoires
    trajListSub.subscribe((msg) => {
        try {
            const files = JSON.parse(msg.data);
            updateSavedTrajectoriesList(files);
        } catch (e) {
            console.error('Erreur parsing liste trajectoires:', e);
        }
    });
    // Sortie : envoie un tableau de coordonnées des points
    // format : fraction de { x(pxls) / largeur image, fraction de y(pxls) / hauteur image 
    // --- State ---
    let trajectoryPoints = [];
    let pointIdCounter = 0;
    let scale = 1;
    let panning = false;
    let isDragging = false;
    let view = { x: 0, y: 0 };
    let start = { x: 0, y: 0 };
    let startClick = { x: 0, y: 0 };
    
    // Zones interdites
    let editMode = false;
    let forbiddenAreas = [];
    let tempRectPoint = null;
    
    // START POINT que l'on a fixé (pixels only)
    let startPoint = { x: 0, y: 0, type: 'start' };
    let startMarkerEl = null;
    let robotOrientation = 0;
    let covarianceRadius = 0;
    let robotLastPixel = null;
    let robotTakingPhoto = false;
    let robotLastOdometryMs = Date.now();
    let robotOffline = false;
    let robotRafPending = false;
    const robotDotOnlineFill = '#2ecc71';
    const robotDotOfflineFill = '#FFD700';
    const ROBOT_DOT_RADIUS_PX = 16;
    const ROBOT_COV_MIN_PX = 10;
    const LAST_MISSION_COOKIE_NAME = 'lastMissionTrajectoryName';

    let missionTrajectoryData = null;
    let missionTrajectoryFilename = null;
    let missionWaypointIndex = -1;
    let missionLastFeedbackMs = 0;
    let missionCookieRestoreTried = false;
    let missionUpcomingPath = null;
    let missionPassedPath = null;
    let costmapCanvas = null;
    let costmapCtx = null;
    let costmapImageData = null;
    let costmapHideTimer = null;
    let pointAlertsTimer = null;

    const COSTMAP_PRE_ALERT_MS = 4000;
    const COSTMAP_POST_ALERT_MS = 3000;
    const BLACK_PIXEL_MAX = 50;
    const WHITE_PIXEL_MIN = 252;

    function ensureMissionOverlayElements() {
        if (!trajectoryLayer || !trajectoryPath) return;

        let defs = trajectoryLayer.querySelector('defs');
        if (!defs) {
            defs = document.createElementNS('http://www.w3.org/2000/svg', 'defs');
            trajectoryLayer.insertBefore(defs, trajectoryLayer.firstChild);
        }

        let arrow = trajectoryLayer.querySelector('#trajectory-arrowhead');
        if (!arrow) {
            arrow = document.createElementNS('http://www.w3.org/2000/svg', 'marker');
            arrow.setAttribute('id', 'trajectory-arrowhead');
            arrow.setAttribute('viewBox', '0 0 10 10');
            arrow.setAttribute('refX', '9');
            arrow.setAttribute('refY', '5');
            arrow.setAttribute('markerWidth', '2');
            arrow.setAttribute('markerHeight', '8');
            arrow.setAttribute('orient', 'auto-start-reverse');
            const arrowPath = document.createElementNS('http://www.w3.org/2000/svg', 'path');
            arrowPath.setAttribute('d', 'M 0 0 L 10 5 L 0 10 z');
            arrowPath.setAttribute('fill', 'rgba(255, 165, 0, 0.7)');
            arrow.appendChild(arrowPath);
            defs.appendChild(arrow);
        }

        let missionArrow = trajectoryLayer.querySelector('#mission-arrowhead');
        if (!missionArrow) {
            missionArrow = document.createElementNS('http://www.w3.org/2000/svg', 'marker');
            missionArrow.setAttribute('id', 'mission-arrowhead');
            missionArrow.setAttribute('viewBox', '0 0 10 10');
            missionArrow.setAttribute('refX', '9');
            missionArrow.setAttribute('refY', '5');
            missionArrow.setAttribute('markerWidth', '2');
            missionArrow.setAttribute('markerHeight', '8');
            missionArrow.setAttribute('orient', 'auto-start-reverse');
            const missionArrowPath = document.createElementNS('http://www.w3.org/2000/svg', 'path');
            missionArrowPath.setAttribute('d', 'M 0 0 L 10 5 L 0 10 z');
            missionArrowPath.setAttribute('fill', 'rgba(120, 210, 255, 0.45)');
            missionArrow.appendChild(missionArrowPath);
            defs.appendChild(missionArrow);
        }

        trajectoryPath.setAttribute('marker-mid', 'url(#trajectory-arrowhead)');
        trajectoryPath.setAttribute('marker-end', 'url(#trajectory-arrowhead)');

        if (!missionPassedPath) {
            missionPassedPath = document.createElementNS('http://www.w3.org/2000/svg', 'polyline');
            missionPassedPath.setAttribute('id', 'mission-passed-path');
            missionPassedPath.setAttribute('fill', 'none');
            missionPassedPath.setAttribute('stroke', 'rgba(180, 180, 180, 0.28)');
            missionPassedPath.setAttribute('stroke-width', '22');
            missionPassedPath.setAttribute('stroke-linecap', 'round');
            missionPassedPath.setAttribute('stroke-linejoin', 'round');
            missionPassedPath.setAttribute('marker-mid', 'url(#mission-arrowhead)');
            missionPassedPath.setAttribute('marker-end', 'url(#mission-arrowhead)');
            missionPassedPath.style.display = 'none';
            trajectoryLayer.insertBefore(missionPassedPath, trajectoryPath);
        }

        if (!missionUpcomingPath) {
            missionUpcomingPath = document.createElementNS('http://www.w3.org/2000/svg', 'polyline');
            missionUpcomingPath.setAttribute('id', 'mission-upcoming-path');
            missionUpcomingPath.setAttribute('fill', 'none');
            missionUpcomingPath.setAttribute('stroke', 'rgba(120, 210, 255, 0.34)');
            missionUpcomingPath.setAttribute('stroke-width', '22');
            missionUpcomingPath.setAttribute('stroke-linecap', 'round');
            missionUpcomingPath.setAttribute('stroke-linejoin', 'round');
            missionUpcomingPath.setAttribute('marker-mid', 'url(#mission-arrowhead)');
            missionUpcomingPath.setAttribute('marker-end', 'url(#mission-arrowhead)');
            missionUpcomingPath.style.display = 'none';
            trajectoryLayer.insertBefore(missionUpcomingPath, trajectoryPath);
        }
    }

    function setMissionOverlayVisible(visible) {
        if (missionPassedPath) missionPassedPath.style.display = visible ? 'block' : 'none';
        if (missionUpcomingPath) missionUpcomingPath.style.display = visible ? 'block' : 'none';
    }

    function renderMissionTrajectoryOverlay() {
        ensureMissionOverlayElements();
        if (!missionTrajectoryData || !Array.isArray(missionTrajectoryData.trajectory) || missionTrajectoryData.trajectory.length === 0) {
            setMissionOverlayVisible(false);
            return;
        }

        if (Date.now() - missionLastFeedbackMs > 2500) {
            setMissionOverlayVisible(false);
            return;
        }

        const pts = missionTrajectoryData.trajectory
            .filter(p => typeof p.x === 'number' && typeof p.y === 'number')
            .map(p => `${p.x},${p.y}`);

        if (pts.length === 0) {
            setMissionOverlayVisible(false);
            return;
        }

        const safeStart = Math.min(Math.max(0, (typeof missionWaypointIndex === 'number' ? missionWaypointIndex : 0)), pts.length);
        const passedPts = safeStart > 0 ? pts.slice(0, safeStart) : [];
        const upcomingPts = pts.slice(safeStart);

        if (missionPassedPath) missionPassedPath.setAttribute('points', passedPts.join(' '));
        if (missionUpcomingPath) missionUpcomingPath.setAttribute('points', upcomingPts.join(' '));
        setMissionOverlayVisible(true);
    }

    async function tryRestoreMissionTrajectoryFromCookie() {
        if (missionTrajectoryData || missionCookieRestoreTried) return;
        missionCookieRestoreTried = true;

        const filename = getCookie(LAST_MISSION_COOKIE_NAME);
        if (!filename) return;

        try {
            const response = await fetch(`../trajectories/${filename}`);
            if (!response.ok) return;
            const data = await response.json();
            if (!data || !Array.isArray(data.trajectory)) return;
            missionTrajectoryData = data;
            missionTrajectoryFilename = filename;
            renderMissionTrajectoryOverlay();
        } catch (e) {
            console.warn('[NAVIG][MISSION] restore cookie error:', e);
        }
    }

    function refreshTrajectoryMarkerStyles() {
        const markers = Array.from(mapWorld.querySelectorAll('.map-marker'));
        markers.forEach((marker, index) => {
            const isPhotography = marker.classList.contains('photography');
            if (index === 0) {
                marker.style.width = '48px';
                marker.style.height = '48px';
                marker.style.borderWidth = '12px';
                marker.style.backgroundColor = '#6b2ecc';
                marker.style.zIndex = '7';
            } else {
                marker.style.width = '30px';
                marker.style.height = '30px';
                marker.style.borderWidth = '10px';
                marker.style.backgroundColor = isPhotography ? '#00ff00' : 'red';
                marker.style.zIndex = '';
            }

            if (marker.classList.contains('alert-forbidden')) {
                marker.style.width = '44px';
                marker.style.height = '44px';
                marker.style.borderWidth = '12px';
                marker.style.backgroundColor = '#ff2e2e';
                marker.style.zIndex = '20';
            } else if (marker.classList.contains('alert-danger')) {
                marker.style.width = '44px';
                marker.style.height = '44px';
                marker.style.borderWidth = '12px';
                marker.style.backgroundColor = '#ffd400';
                marker.style.zIndex = '20';
            }
        });
    }

    function prepareCostmapSampling() {
        if (!mapCostmapOverlay || !mapCostmapOverlay.naturalWidth || !mapCostmapOverlay.naturalHeight) {
            return false;
        }
        if (!costmapCanvas) {
            costmapCanvas = document.createElement('canvas');
            costmapCtx = costmapCanvas.getContext('2d', { willReadFrequently: true });
        }
        if (!costmapCtx) return false;

        const w = mapCostmapOverlay.naturalWidth;
        const h = mapCostmapOverlay.naturalHeight;
        costmapCanvas.width = w;
        costmapCanvas.height = h;
        costmapCtx.clearRect(0, 0, w, h);
        costmapCtx.drawImage(mapCostmapOverlay, 0, 0, w, h);
        costmapImageData = costmapCtx.getImageData(0, 0, w, h);
        return true;
    }

    function classifyCostmapPixel(point) {
        if (!costmapImageData || !mapImage || !mapImage.naturalWidth || !mapImage.naturalHeight) return 'unknown';

        const costmapW = mapCostmapOverlay.naturalWidth;
        const costmapH = mapCostmapOverlay.naturalHeight;
        if (!costmapW || !costmapH) return 'unknown';

        const sx = costmapW / mapImage.naturalWidth;
        const sy = costmapH / mapImage.naturalHeight;
        const px = Math.max(0, Math.min(costmapW - 1, Math.round(point.x * sx)));
        const py = Math.max(0, Math.min(costmapH - 1, Math.round(point.y * sy)));

        const idx = (py * costmapW + px) * 4;
        const data = costmapImageData.data;
        const r = data[idx];
        const g = data[idx + 1];
        const b = data[idx + 2];

        const maxRgb = Math.max(r, g, b);
        const minRgb = Math.min(r, g, b);
        if (minRgb >= WHITE_PIXEL_MIN) return 'white';
        if (maxRgb <= BLACK_PIXEL_MAX) return 'black';
        return 'danger';
    }

    function ensureCostmapSamplingReady(timeoutMs = 1800) {
        return new Promise((resolve) => {
            if (!mapCostmapOverlay) {
                resolve(false);
                return;
            }

            if (prepareCostmapSampling()) {
                resolve(true);
                return;
            }

            let settled = false;
            const finalize = (ok) => {
                if (settled) return;
                settled = true;
                clearTimeout(timer);
                mapCostmapOverlay.removeEventListener('load', onLoad);
                mapCostmapOverlay.removeEventListener('error', onError);
                resolve(ok && prepareCostmapSampling());
            };

            const onLoad = () => finalize(true);
            const onError = () => finalize(false);

            mapCostmapOverlay.addEventListener('load', onLoad, { once: true });
            mapCostmapOverlay.addEventListener('error', onError, { once: true });

            const timer = setTimeout(() => finalize(false), timeoutMs);

            // Force a (re)load if needed.
            if (!mapCostmapOverlay.getAttribute('src')) {
                mapCostmapOverlay.setAttribute('src', './map_costmap.jpg');
            }
        });
    }

    function clearTrajectoryPointAlerts() {
        const markers = mapWorld.querySelectorAll('.map-marker.alert-forbidden, .map-marker.alert-danger');
        markers.forEach(marker => marker.classList.remove('alert-forbidden', 'alert-danger'));
        refreshTrajectoryMarkerStyles();
    }

    function stopRiskVisualsNow() {
        if (costmapHideTimer) {
            clearTimeout(costmapHideTimer);
            costmapHideTimer = null;
        }
        if (pointAlertsTimer) {
            clearTimeout(pointAlertsTimer);
            pointAlertsTimer = null;
        }
        if (mapCostmapOverlay) {
            mapCostmapOverlay.style.display = 'none';
            mapCostmapOverlay.style.opacity = '0';
        }
        clearTrajectoryPointAlerts();
    }

    function scheduleStopRiskVisuals(delayMs) {
        if (costmapHideTimer) {
            clearTimeout(costmapHideTimer);
            costmapHideTimer = null;
        }
        if (pointAlertsTimer) {
            clearTimeout(pointAlertsTimer);
            pointAlertsTimer = null;
        }

        costmapHideTimer = setTimeout(() => {
            if (mapCostmapOverlay) {
                mapCostmapOverlay.style.display = 'none';
                mapCostmapOverlay.style.opacity = '0';
            }
            costmapHideTimer = null;
        }, delayMs);

        pointAlertsTimer = setTimeout(() => {
            clearTrajectoryPointAlerts();
            pointAlertsTimer = null;
        }, delayMs);
    }

    function analyzeTrajectoryAgainstCostmap() {
        if (!mapCostmapOverlay) {
            return {
                costmapUnavailable: true,
                forbiddenIds: [],
                dangerIds: []
            };
        }

        if (!prepareCostmapSampling()) {
            console.warn('[NAVIG][COSTMAP] map_costmap.jpg non chargee, verification ignoree.');
            return {
                costmapUnavailable: true,
                forbiddenIds: [],
                dangerIds: []
            };
        }

        const forbiddenIds = [];
        const dangerIds = [];

        trajectoryPoints.forEach(point => {
            const pixelType = classifyCostmapPixel(point);
            if (pixelType === 'black') {
                forbiddenIds.push(point.id);
            } else if (pixelType === 'danger') {
                dangerIds.push(point.id);
            }
        });

        return {
            costmapUnavailable: false,
            forbiddenIds,
            dangerIds
        };
    }

    function showRiskVisuals(analysis, showCostmapOverlay) {
        const forbiddenIdSet = new Set(analysis.forbiddenIds || []);
        const dangerIdSet = new Set(analysis.dangerIds || []);

        if (costmapHideTimer || pointAlertsTimer) {
            stopRiskVisualsNow();
        }

        if (showCostmapOverlay) {
            mapCostmapOverlay.style.display = 'block';
            mapCostmapOverlay.style.opacity = '0.7';
        } else {
            mapCostmapOverlay.style.display = 'none';
            mapCostmapOverlay.style.opacity = '0';
        }

        clearTrajectoryPointAlerts();

        trajectoryPoints.forEach(point => {
            const marker = mapWorld.querySelector(`.map-marker[data-point-id="${point.id}"]`);
            if (!marker) return;

            if (forbiddenIdSet.has(point.id)) {
                marker.classList.add('alert-forbidden');
            } else if (dangerIdSet.has(point.id)) {
                marker.classList.add('alert-danger');
            }
        });

        refreshTrajectoryMarkerStyles();
    }

    function rebuildTrajectoryUIFromState() {
        pointList.innerHTML = '';
        const markers = mapWorld.querySelectorAll('.map-marker');
        markers.forEach(marker => marker.remove());

        trajectoryPoints.forEach(point => {
            addPointToVisualList(point);
            drawPointOnMap(point);
        });

        updateTrajectoryPath();
        redrawSerreSvg();
    }

    function removePointsByIds(idsToRemove) {
        if (!Array.isArray(idsToRemove) || idsToRemove.length === 0) return 0;
        const idSet = new Set(idsToRemove);
        const before = trajectoryPoints.length;
        trajectoryPoints = trajectoryPoints.filter(point => !idSet.has(point.id));
        const removed = before - trajectoryPoints.length;
        if (removed > 0) {
            rebuildTrajectoryUIFromState();
        }
        return removed;
    }

    function askTrajectoryRiskResolution(forbiddenCount, dangerCount) {
        const msg =
            `⚠️ Validation costmap\n` +
            `Points interdits (noir): ${forbiddenCount}\n` +
            `Points dangereux (gris/non-blanc): ${dangerCount}\n\n` +
            `La trajectoire ne sera PAS sauvegardée maintenant.\n` +
            `Choisissez une action :\n` +
            `1 = NON (ne rien supprimer)\n` +
            `2 = Supprimer uniquement les points en zones interdites\n` +
            `3 = Supprimer points interdits + dangereux\n\n` +
            `Entrez 1, 2 ou 3:`;

        const answer = window.prompt(msg, '1');
        const normalized = (answer || '').trim();
        if (normalized === '2' || normalized === '3') return normalized;
        return '1';
    }

    function askDangerOnlyResolution(dangerCount) {
        const msg =
            `⚠️ Attention: certains points sont en zones dangereuses.\n` +
            `Points dangereux (gris/non-blanc): ${dangerCount}\n\n` +
            `Choisissez une action :\n` +
            `1 = Les supprimer puis sauvegarder la trajectoire\n` +
            `2 = Les garder et sauvegarder la trajectoire\n\n` +
            `Entrez 1 ou 2:`;

        const answer = window.prompt(msg, '2');
        const normalized = (answer || '').trim();
        if (normalized === '1') return '1';
        return '2';
    }

    async function handleSaveTrajectoryClick() {
        const ready = await ensureCostmapSamplingReady();
        if (!ready) {
            alert('❌ Costmap indisponible: validation impossible, sauvegarde annulée. Réessayez dans quelques secondes.');
            return;
        }

        const analysis = analyzeTrajectoryAgainstCostmap();
        if (analysis.costmapUnavailable) {
            alert('❌ Costmap indisponible: validation impossible, sauvegarde annulée.');
            return;
        }
        const forbiddenCount = analysis.forbiddenIds.length;
        const dangerCount = analysis.dangerIds.length;

        // Si des points interdits sont detectes en vue serre, revenir tout de suite sur la carte exterieure.
        if (forbiddenCount > 0) {
            switchToExteriorTabIfSerreActive();
        }

        if (forbiddenCount === 0 && dangerCount === 0) {
            stopRiskVisualsNow();
            saveTrajectory();
            return;
        }

        // Pour tout point a risque (interdit ou dangereux): afficher costmap+animations 4s avant les alertes.
        showRiskVisuals(analysis, true);
        await new Promise(resolve => setTimeout(resolve, COSTMAP_PRE_ALERT_MS));

        try {
            if (forbiddenCount > 0) {
                const choice = askTrajectoryRiskResolution(forbiddenCount, dangerCount);
                if (choice === '2') {
                    const removed = removePointsByIds(analysis.forbiddenIds);
                    alert(`🧹 ${removed} point(s) interdit(s) supprimé(s). La trajectoire n'a pas été sauvegardée. Cliquez de nouveau sur Sauvegarder après vérification.`);
                    return;
                }
                if (choice === '3') {
                    const removed = removePointsByIds([...analysis.forbiddenIds, ...analysis.dangerIds]);
                    alert(`🧹 ${removed} point(s) interdit(s)/dangereux supprimé(s). La trajectoire n'a pas été sauvegardée. Cliquez de nouveau sur Sauvegarder après vérification.`);
                    return;
                }

                alert('❌ Sauvegarde annulée: des points sont en zones interdites.');
                return;
            }

            // Ici: aucun point interdit, uniquement des points bons + dangereux.
            const dangerChoice = askDangerOnlyResolution(dangerCount);
            if (dangerChoice === '1') {
                removePointsByIds(analysis.dangerIds);
            }
            saveTrajectory();
        } finally {
            // Une fois toutes les popups fermees, attendre 3s puis couper overlay+animations.
            scheduleStopRiskVisuals(COSTMAP_POST_ALERT_MS);
        }
    }

    ensureMissionOverlayElements();
    if (robotDot) robotDot.setAttribute('r', String(ROBOT_DOT_RADIUS_PX));
    if (robotHalo) robotHalo.setAttribute('r', '24');
    if (robotRipple) robotRipple.setAttribute('r', '28');
    if (robotHeading) robotHeading.setAttribute('points', '0,-30 -10,-14 10,-14');

    // --- Système de coordonnées métriques ---
    // Pixel d'origine sur l'image (correspond aux coordonnées métriques (0, 0))
    const originPixel = { x: 160.1, y: 1104.8 };
    // Échelle : 2.6617 cm/px → m/px
    const metersPerPixel = 2.6617 / 100;
    const origin_map_size = { width: 9966, height: 2622 }; // taille de l'image map.jpg en pixels
    // Angle de rotation entre les axes image et le repère monde (degrés)
    const thetaDegrees = 76.681;

    function meters_to_pixels(x, y) {
        if (!mapImage || !mapImage.naturalWidth || !mapImage.naturalHeight) return null;

        const imageSizeX = mapImage.naturalWidth;
        const imageSizeY = mapImage.naturalHeight;
        const ratioConversionWidth = origin_map_size.width / imageSizeX;
        const ratioConversionHeight = origin_map_size.height / imageSizeY;

        const thetaRad = thetaDegrees * Math.PI / 180;
        const cosTheta = Math.cos(thetaRad);
        const sinTheta = Math.sin(thetaRad);

        const metersYInv = (y * (cosTheta / sinTheta) - x) /
            (sinTheta + cosTheta * cosTheta / sinTheta);
        const metersX = (x + metersYInv * sinTheta) / cosTheta;
        const metersY = -metersYInv;

        const dx = metersX / metersPerPixel;
        const dy = metersY / metersPerPixel;

        const convertedX = dx + originPixel.x;
        const convertedY = dy + originPixel.y;

        return {
            x: convertedX / ratioConversionWidth,
            y: convertedY / ratioConversionHeight
        };
    }

    function updateRobotOfflineState(isOffline) {
        robotOffline = isOffline;
        if (robotOfflineBadge) {
            robotOfflineBadge.classList.toggle('visible', isOffline);
        }
        if (robotDot) {
            robotDot.setAttribute('fill', isOffline ? robotDotOfflineFill : robotDotOnlineFill);
        }
    }

    function renderRobotMarker() {
        if (!robotGroup || !robotLastPixel || !isFinite(robotLastPixel.x) || !isFinite(robotLastPixel.y)) return;

        robotGroup.setAttribute('transform', `translate(${robotLastPixel.x}, ${robotLastPixel.y})`);
        robotGroup.style.opacity = '1';

        if (robotHeading) {
            const headingDeg = (robotOrientation || 0) * 180 / Math.PI + 90;
            robotHeading.setAttribute('transform', `rotate(${headingDeg} 0 0)`);
        }

        if (robotCovariance) {
            const metersPerPixelMinimap = metersPerPixel * (origin_map_size.width / mapImage.naturalWidth);
            let covRadiusPx = ROBOT_COV_MIN_PX;
            if (isFinite(covarianceRadius) && covarianceRadius > 0 &&
                isFinite(metersPerPixelMinimap) && metersPerPixelMinimap > 0) {
                covRadiusPx = Math.max(ROBOT_COV_MIN_PX, covarianceRadius / metersPerPixelMinimap);
            }
            robotCovariance.setAttribute('r', String(covRadiusPx));
        }

        if (robotTakingPhoto) {
            robotHalo?.classList.add('pulsing');
            robotRipple?.classList.add('pulsing');
        } else {
            robotHalo?.classList.remove('pulsing');
            robotRipple?.classList.remove('pulsing');
        }
    }

    function scheduleRobotRender() {
        if (robotRafPending) return;
        robotRafPending = true;
        requestAnimationFrame(() => {
            robotRafPending = false;
            renderRobotMarker();
        });
    }

    odometryFilteredSub.subscribe((msg) => {
        try {
            robotLastOdometryMs = Date.now();
            if (robotOffline) updateRobotOfflineState(false);

            const x = msg.pose.pose.position.x;
            const y = msg.pose.pose.position.y;
            const qw = msg.pose.pose.orientation.w;
            const qz = msg.pose.pose.orientation.z;
            const varX = msg.pose.covariance[0];
            const varY = msg.pose.covariance[7];

            robotOrientation = -Math.atan2(2 * qw * qz, 1 - 2 * qz * qz) + thetaDegrees * Math.PI / 180;
            covarianceRadius = Math.sqrt(Math.max(varX || 0, varY || 0));

            const px = meters_to_pixels(x, y);
            if (!px || !isFinite(px.x) || !isFinite(px.y)) return;
            robotLastPixel = px;
            scheduleRobotRender();
        } catch (e) {
            console.warn('[NAVIG][ODOMETRY] erreur traitement:', e);
        }
    });

    missionFeedbackSub.subscribe((msg) => {
        try {
            const fb = JSON.parse(msg.data);
            robotTakingPhoto = !!fb.is_taking_photo;
            missionLastFeedbackMs = Date.now();
            if (typeof fb.current_waypoint_index === 'number') {
                missionWaypointIndex = fb.current_waypoint_index;
            }

            if (typeof fb.robot_x === 'number' && typeof fb.robot_y === 'number') {
                const px = meters_to_pixels(fb.robot_x, fb.robot_y);
                if (px && isFinite(px.x) && isFinite(px.y)) {
                    robotLastPixel = px;
                }
            }
            if (!missionTrajectoryData) {
                tryRestoreMissionTrajectoryFromCookie();
            }
            renderMissionTrajectoryOverlay();
            scheduleRobotRender();
        } catch (e) {
            console.warn('[NAVIG][MISSION] erreur parsing feedback:', e);
        }
    });

    setInterval(() => {
        const silentForMs = Date.now() - robotLastOdometryMs;
        if (silentForMs >= 5000) {
            if (!robotOffline) updateRobotOfflineState(true);
        } else if (robotOffline) {
            updateRobotOfflineState(false);
        }
        if (Date.now() - missionLastFeedbackMs > 2500) {
            setMissionOverlayVisible(false);
        }
    }, 250);

    // --- Serre (greenhouse) rectangle en coordonnées métriques ---
    // Si le dernier point placé est dans ce rectangle, on ouvre la vue "Serre intérieur".
    // TODO: mettre à jour ces bornes métriques pour correspondre à la serre réelle
    // Serre rectangle in image pixels (fixed)
    const serreRectPixels = { x1: 300.0, y1: 362, x2: 647, y2: 739 };
    // --- Configuration de la serre (repère métrique séparé) ---
    // TODO: mettre à jour ces valeurs pour correspondre à l'image serre.jpg
    // Serre runtime state (scale/offset computed dynamically on each layout update)
    let serreImgDisplayScale = 1;
    let serreImgOffsetX = 0;
    let serreImgOffsetY = 0;
    
    // --- Helper: attach metric coords to a point using user-provided pixel_to_meters(x,y) ---
    // This stores metric coordinates in `gps_x`/`gps_y` (kept for backward compatibility)
    function attachGpsToPoint(point) {
        if (!point) return;
        if (typeof pixel_to_meters !== 'function') return;
        try {
            const m = pixel_to_meters(point.x, point.y);
            if (!m) return;
            if (typeof m.x !== 'undefined' && typeof m.y !== 'undefined') {
                point.gps_x = parseFloat(m.x);
                point.gps_y = parseFloat(m.y);
            } else if (Array.isArray(m) && m.length >= 2) {
                point.gps_x = parseFloat(m[0]);
                point.gps_y = parseFloat(m[1]);
            }
        } catch (e) {
            console.error('pixel_to_meters error:', e);
        }
    }

    function pixel_to_meters(x, y) {
        // Convertir les pixels en mètres (en tenant compte de la rotation)
        // Use the actual loaded image natural size so ratios are correct
        if (!mapImage || !mapImage.naturalWidth || !mapImage.naturalHeight) return null;

        const imgW = mapImage.naturalWidth;
        const imgH = mapImage.naturalHeight;

        const thetaRad = thetaDegrees * Math.PI / 180;
        const cosTheta = Math.cos(thetaRad);
        const sinTheta = Math.sin(thetaRad);

        const ratio_conversion_width = origin_map_size.width / imgW;
        const ratio_conversion_height = origin_map_size.height / imgH;

        const converted_x = x * ratio_conversion_width;
        const converted_y = y * ratio_conversion_height;

        console.log(`pixel_to_meters: input (${x.toFixed(1)}, ${y.toFixed(1)}) px → converted (${converted_x.toFixed(1)}, ${converted_y.toFixed(1)}) px (after ratio)`);

        const dx = converted_x - originPixel.x;
        const dy = converted_y - originPixel.y;

        console.log(`pixel_to_meters: delta from origin (${dx.toFixed(1)}, ${dy.toFixed(1)}) px`);

        const meters_x = dx * metersPerPixel;
        const meters_y = dy * metersPerPixel;

        console.log(`pixel_to_meters: delta in meters before rotation (${meters_x.toFixed(3)}, ${meters_y.toFixed(3)}) m`);
        // inverser signes y pour correspondre à l'orientation souhaitée du repère métrique (x vers la droite, y vers le haut)

        const meters_y_inv = -meters_y;

        console.log(`pixel_to_meters: delta in meters after inversion (${meters_x.toFixed(3)}, ${meters_y_inv.toFixed(3)}) m`);

        // rotation de theta autour de l'origine
        const rotated_x = meters_x * cosTheta - meters_y_inv * sinTheta;
        const rotated_y = meters_x * sinTheta + meters_y_inv * cosTheta;

        console.log(`pixel_to_meters: delta in meters after rotation (${rotated_x.toFixed(3)}, ${rotated_y.toFixed(3)}) m`);

        return { x: rotated_x, y: rotated_y };
    }

    mapImage.onload = () => {
        // Calculate initial scale to cover the container
        const containerRect = mapContainer.getBoundingClientRect();
        const imgWidth = mapImage.naturalWidth;
        const imgHeight = mapImage.naturalHeight;

        const scaleX = containerRect.width / imgWidth;
        const scaleY = containerRect.height / imgHeight;
        
        // "Cover" behavior: take the larger scale
        scale = Math.max(scaleX, scaleY);

        // Center the image
        view.x = (containerRect.width - imgWidth * scale) / 2;
        view.y = (containerRect.height - imgHeight * scale) / 2;

        // startPoint stays in pixel coordinates (no GPS conversion)
        drawPointOnMap(startPoint);

        if (trajectoryLayer) {
            trajectoryLayer.setAttribute('viewBox', `0 0 ${imgWidth} ${imgHeight}`);
            trajectoryLayer.setAttribute('width', String(imgWidth));
            trajectoryLayer.setAttribute('height', String(imgHeight));
        }

        if (mapCostmapOverlay) {
            mapCostmapOverlay.style.width = `${imgWidth}px`;
            mapCostmapOverlay.style.height = `${imgHeight}px`;
        }

        updateTransform();
        loadForbiddenAreas();
        // Ensure tabs are present and persistent
        createTabsIfNeeded();
        // La liste des trajectoires sera chargée via ROS topic
    };

    if (mapCostmapOverlay) {
        mapCostmapOverlay.addEventListener('load', () => {
            prepareCostmapSampling();
        });
    }

    // --- Map Setup --- (src assigné APRÈS onload pour éviter la race condition avec le cache)
    mapImage.src = './map.jpg';


    // --- Zoom and Pan Logic ---
    function updateTransform() {
        // Apply transform to the world wrapper, not individual elements
        const transformValue = `translate(${view.x}px, ${view.y}px) scale(${scale})`;
        mapWorld.style.transform = transformValue;
    }

    function resetView() {
        // Reset to cover logic (same as onload)
        const containerRect = mapContainer.getBoundingClientRect();
        const imgWidth = mapImage.naturalWidth;
        const imgHeight = mapImage.naturalHeight;
        const scaleX = containerRect.width / imgWidth;
        const scaleY = containerRect.height / imgHeight;
        scale = Math.max(scaleX, scaleY);
        view.x = (containerRect.width - imgWidth * scale) / 2;
        view.y = (containerRect.height - imgHeight * scale) / 2;
        updateTransform();
    }

    mapContainer.addEventListener('wheel', (event) => {
        event.preventDefault();
        const rect = mapContainer.getBoundingClientRect();
        // Mouse position relative to container
        const mouseX = event.clientX - rect.left;
        const mouseY = event.clientY - rect.top;

        // Mouse position relative to world (before zoom)
        const worldX = (mouseX - view.x) / scale;
        const worldY = (mouseY - view.y) / scale;

        const delta = -event.deltaY;
        const newScale = scale * (1 + delta / 1000);
        
        // Limit zoom (e.g., 0.1x to 10x)
        scale = Math.min(Math.max(0.1, newScale), 10); 

        // Adjust view so mouse stays on same world point
        view.x = mouseX - worldX * scale;
        view.y = mouseY - worldY * scale;

        updateTransform();
    });

    zoomInBtn.addEventListener('click', () => {
        const rect = mapContainer.getBoundingClientRect();
        const centerX = rect.width / 2;
        const centerY = rect.height / 2;
        
        const worldX = (centerX - view.x) / scale;
        const worldY = (centerY - view.y) / scale;

        scale = Math.min(scale * 1.2, 10);
        
        view.x = centerX - worldX * scale;
        view.y = centerY - worldY * scale;
        updateTransform();
    });

    zoomOutBtn.addEventListener('click', () => {
        const rect = mapContainer.getBoundingClientRect();
        const centerX = rect.width / 2;
        const centerY = rect.height / 2;
        
        const worldX = (centerX - view.x) / scale;
        const worldY = (centerY - view.y) / scale;

        scale = Math.max(scale / 1.2, 0.1);
        
        view.x = centerX - worldX * scale;
        view.y = centerY - worldY * scale;
        updateTransform();
    });

    mapContainer.addEventListener('mousedown', (event) => {
        // Ignore mousedown on settings
        if (event.target.closest('.settings-container')) return;
        
        isDragging = false;
        // Only left click for panning
        if (event.button !== 0) return;
        event.preventDefault();
        panning = true;
        mapContainer.style.cursor = 'grabbing';
        start = { x: event.clientX - view.x, y: event.clientY - view.y };
        startClick = { x: event.clientX, y: event.clientY };
    });

    window.addEventListener('mouseup', () => {
        panning = false;
        mapContainer.style.cursor = 'grab';
    });

    window.addEventListener('mousemove', (event) => {
        if (!panning) return;
        event.preventDefault();

        const moveX = event.clientX - startClick.x;
        const moveY = event.clientY - startClick.y;
        if (Math.sqrt(moveX * moveX + moveY * moveY) > 5) {
            isDragging = true;
        }

        view.x = event.clientX - start.x;
        view.y = event.clientY - start.y;
        updateTransform();
    });

    // --- Keyboard Events ---
    window.addEventListener('keydown', (event) => {
        if (event.key.toLowerCase() === 'a' && editMode) {
            event.preventDefault();
            // Trigger a click at the center of the viewport
            const rect = mapContainer.getBoundingClientRect();
            const centerX = rect.width / 2;
            const centerY = rect.height / 2;
            
            // Calculate image coordinates
            const imageX = (centerX - view.x) / scale;
            const imageY = (centerY - view.y) / scale;
            
            if (imageX >= 0 && imageX <= mapImage.naturalWidth && imageY >= 0 && imageY <= mapImage.naturalHeight) {
                handleRectanglePoint(imageX, imageY);
            }
        }
    });


    // --- Point Selection ---
    // Handle Right Click (Context Menu)
    mapContainer.addEventListener('contextmenu', (event) => {
        event.preventDefault(); // Prevent default context menu
        if (isDragging) return;

        const rect = mapContainer.getBoundingClientRect();
        const mouseX = event.clientX - rect.left;
        const mouseY = event.clientY - rect.top;

        const imageX = (mouseX - view.x) / scale;
        const imageY = (mouseY - view.y) / scale;

        if (imageX < 0 || imageX > mapImage.naturalWidth || imageY < 0 || imageY > mapImage.naturalHeight) {
            return;
        }

        // If in edit mode, check if clicking on a forbidden area to delete it
        if (editMode) {
            const clickedAreaIndex = findForbiddenAreaAt(imageX, imageY);
            if (clickedAreaIndex !== -1) {
                // Remove the area
                forbiddenAreas.splice(clickedAreaIndex, 1);
                // Redraw all areas
                redrawForbiddenAreas();
            }
            return;
        }

        // Add photography point (pixel coords only)
        const point = { id: pointIdCounter++, x: imageX, y: imageY, type: 'photography', photography: 'yes' };
        // Attach GPS if user provided pixel_to_gps
        attachGpsToPoint(point);
        trajectoryPoints.push(point);

        addPointToVisualList(point);
        drawPointOnMap(point);
        checkOpenSerreForPoint(point);
        redrawSerreSvg();
    });

    mapContainer.addEventListener('click', (event) => {
        // Ignore clicks on buttons and settings
        if (event.target.tagName === 'BUTTON' || event.target.closest('.settings-container')) return;
        
        if (isDragging) return;
        
        // If in edit mode, handle rectangle points instead
        if (editMode) {
            const rect = mapContainer.getBoundingClientRect();
            const mouseX = event.clientX - rect.left;
            const mouseY = event.clientY - rect.top;
            const imageX = (mouseX - view.x) / scale;
            const imageY = (mouseY - view.y) / scale;
            
            if (imageX >= 0 && imageX <= mapImage.naturalWidth && imageY >= 0 && imageY <= mapImage.naturalHeight) {
                handleRectanglePoint(imageX, imageY);
            }
            return;
        }
        
        // Let's calculate coordinates
        const rect = mapContainer.getBoundingClientRect();
        const mouseX = event.clientX - rect.left;
        const mouseY = event.clientY - rect.top;

        // Convert to image coordinates
        const imageX = (mouseX - view.x) / scale;
        const imageY = (mouseY - view.y) / scale;

        // Check bounds (optional, if we want to restrict points to image area)
        if (imageX < 0 || imageX > mapImage.naturalWidth || imageY < 0 || imageY > mapImage.naturalHeight) {
            // Clicked outside the image
            return;
        }

        // Display pixel coords only
        gpsCoordsDisplay.textContent = `X: ${imageX.toFixed(1)} px, Y: ${imageY.toFixed(1)} px`;

        const point = { id: pointIdCounter++, x: imageX, y: imageY, type: 'default' };
        // Attach GPS coordinates if available
        attachGpsToPoint(point);
        trajectoryPoints.push(point);

        addPointToVisualList(point);
        drawPointOnMap(point);
        checkOpenSerreForPoint(point);
        redrawSerreSvg();
    });

    // --- Trajectory Management ---
    clearTrajectoryBtn.addEventListener('click', () => {
        trajectoryPoints = [];
        pointList.innerHTML = '';
        gpsCoordsDisplay.textContent = '';
        pointIdCounter = 0;
        
        // Remove all visualized trajectory markers (startpoint is not displayed)
        const markers = mapWorld.querySelectorAll('.map-marker');
        markers.forEach(marker => marker.remove());
        
        // Reset path to just start point
        updateTrajectoryPath();
        redrawSerreSvg();
    });

    removeLastPointBtn.addEventListener('click', () => {
        if (trajectoryPoints.length === 0) {
            alert('Aucun point à supprimer.');
            return;
        }
        
        // Remove the last point
        const lastPoint = trajectoryPoints.pop();
        
        // Remove from list
        const listItems = pointList.querySelectorAll('li');
        if (listItems.length > 0) {
            pointList.removeChild(listItems[listItems.length - 1]);
        }
        
        // Remove marker
        const markers = mapWorld.querySelectorAll('.map-marker');
        if (markers.length > 0) {
            mapWorld.removeChild(markers[markers.length - 1]);
        }
        
        // Update path
        updateTrajectoryPath();
        
        // Update GPS display if no points left
        if (trajectoryPoints.length === 0) {
            gpsCoordsDisplay.textContent = '';
        }
        redrawSerreSvg();
    });

    toggleEditModeBtn.addEventListener('click', () => {
        editMode = !editMode;
        toggleEditModeBtn.textContent = editMode ? 'Mode Édition: ON' : 'Mode Édition: OFF';
        toggleEditModeBtn.classList.toggle('active', editMode);
        toggleEditModeBtn.style.backgroundColor = editMode ? '#28a745' : '#dc3545';
        
        // Toggle visibility of button groups
        trajectoryButtons.style.display = editMode ? 'none' : 'block';
        editButtons.style.display = editMode ? 'block' : 'none';
        
        if (!editMode && tempRectPoint) {
            // Clear temporary point if exiting edit mode
            const tempMarkers = mapWorld.querySelectorAll('.forbidden-area-point');
            tempMarkers.forEach(m => m.remove());
            tempRectPoint = null;
        }
    });

    clearForbiddenAreasBtn.addEventListener('click', () => {
        if (confirm('Voulez-vous vraiment effacer toutes les zones interdites ?')) {
            forbiddenAreas = [];
            const rects = mapWorld.querySelectorAll('.forbidden-area');
            rects.forEach(r => r.remove());
        }
    });

    resetForbiddenAreasBtn.addEventListener('click', () => {
        loadBlankAreas();
    });

    saveTrajectoryBtn.addEventListener('click', () => {
        handleSaveTrajectoryClick();
    });

    // --- UI Update Functions ---
    function addPointToVisualList(point) {
        const li = document.createElement('li');
        let typeStr = "";
        if (point.type === 'photography') {
            typeStr = " [PHOTO]";
            li.style.color = "green";
        }
        li.textContent = `Point ${point.id}${typeStr}: (X: ${point.x.toFixed(1)} px, Y: ${point.y.toFixed(1)} px)`;
        pointList.appendChild(li);
    }

    function drawPointOnMap(point) {
        // Le startPoint est conserve dans les donnees mais n'est pas affiche.
        if (point.type === 'start') {
            if (startMarkerEl) {
                startMarkerEl.remove();
                startMarkerEl = null;
            }
            updateTrajectoryPath();
            return;
        }

        const marker = document.createElement('div');
        marker.className = 'map-marker';
        
        if (point.type === 'photography') {
            marker.classList.add('photography');
        }

        // Position is relative to the map-world wrapper
        marker.style.left = `${point.x}px`;
        marker.style.top = `${point.y}px`;
        marker.dataset.pointId = String(point.id);
        mapWorld.appendChild(marker);

        // Update the trajectory path
        updateTrajectoryPath();
        refreshTrajectoryMarkerStyles();
    }

    function updateTrajectoryPath() {
        // Ne pas relier le startPoint au premier point de trajectoire.
        const pointsStr = trajectoryPoints.length > 0
            ? trajectoryPoints.map(p => `${p.x},${p.y}`).join(' ')
            : '';

        trajectoryPath.setAttribute('points', pointsStr);
        refreshTrajectoryMarkerStyles();
        renderMissionTrajectoryOverlay();
    }

    // --- Forbidden Areas Management ---
    function handleRectanglePoint(x, y) {
        if (!tempRectPoint) {
            // First point
            tempRectPoint = { x, y };
            // Draw temporary point
            const marker = document.createElement('div');
            marker.className = 'forbidden-area-point';
            marker.style.left = `${x}px`;
            marker.style.top = `${y}px`;
            mapWorld.appendChild(marker);
        } else {
            // Second point - create rectangle
            const x1 = Math.min(tempRectPoint.x, x);
            const y1 = Math.min(tempRectPoint.y, y);
            const x2 = Math.max(tempRectPoint.x, x);
            const y2 = Math.max(tempRectPoint.y, y);
            
            // Store forbidden area in pixel coordinates only
            const area = { x1, y1, x2, y2 };
            
            forbiddenAreas.push(area);
            drawForbiddenArea(area);
            
            // Clear temporary point
            const tempMarkers = mapWorld.querySelectorAll('.forbidden-area-point');
            tempMarkers.forEach(m => m.remove());
            tempRectPoint = null;
        }
    }

    function drawForbiddenArea(area) {
        const rect = document.createElement('div');
        rect.className = 'forbidden-area';
        rect.style.left = `${area.x1}px`;
        rect.style.top = `${area.y1}px`;
        rect.style.width = `${area.x2 - area.x1}px`;
        rect.style.height = `${area.y2 - area.y1}px`;
        mapWorld.appendChild(rect);
    }

    function saveTrajectory() {
        // Récupérer le nom de la trajectoire
        const nameInput = document.getElementById('trajectory-name');
        let trajName = nameInput ? nameInput.value.trim() : '';
        
        if (!trajName) {
            alert('⚠️ Veuillez entrer un nom pour la trajectoire');
            return;
        }
        
        // Nettoyer le nom (enlever les caractères spéciaux)
        trajName = trajName.replace(/[^a-zA-Z0-9_-]/g, '_');
        
        // Créer les données de trajectoire
        const data = {
            meta: {
                name: trajName,
                timestamp: new Date().toISOString()
            },
            image: {
                width: mapImage.naturalWidth,
                height: mapImage.naturalHeight
            },
            startPoint: {
                x: startPoint.x,
                y: startPoint.y
            },
            trajectory: trajectoryPoints.map(p => ({
                id: p.id,
                x: p.x,
                y: p.y,
                gps_x: (typeof p.gps_x !== 'undefined') ? p.gps_x : null,
                gps_y: (typeof p.gps_y !== 'undefined') ? p.gps_y : null,
                type: p.type,
                photography: p.photography || 'no'
            })),
            forbiddenAreas: forbiddenAreas.map(area => ({
                topLeft:     { x: area.x1, y: area.y1 },
                bottomRight: { x: area.x2, y: area.y2 }
            }))
        };
        
        // Publier via ROS pour sauvegarde sur le serveur
        const jsonString = JSON.stringify(data);
        const msg = new ROSLIB.Message({
            data: jsonString
        });
        
        saveTrajectoryPub.publish(msg);
        console.log('Trajectoire envoyée pour sauvegarde via ROS');
        alert(`✅ Trajectoire "${trajName}" sauvegardée !`);
        
        // Vider le champ de saisie
        if (nameInput) nameInput.value = '';
    }

    function updateSavedTrajectoriesList(files) {
        savedTrajectoriesList.innerHTML = '';
        
        if (!files || files.length === 0) {
            const li = document.createElement('li');
            li.textContent = 'Aucun trajet sauvegardé';
            li.style.fontStyle = 'italic';
            li.style.color = '#999';
            savedTrajectoriesList.appendChild(li);
            return;
        }

        files.forEach(filename => {
            const li = document.createElement('li');
            const nameSpan = document.createElement('span');
            nameSpan.textContent = filename.replace('.json', '');
            
            const actions = document.createElement('div');
            actions.className = 'trajectory-actions';
            
            const loadBtn = document.createElement('button');
            loadBtn.textContent = 'Charger';
            loadBtn.className = 'btn-load';
            loadBtn.onclick = () => loadTrajectoryFromFile(filename);
            
            const deleteBtn = document.createElement('button');
            deleteBtn.textContent = '🗑️';
            deleteBtn.className = 'btn-delete';
            deleteBtn.onclick = () => deleteTrajectoryFile(filename);
            
            actions.appendChild(loadBtn);
            actions.appendChild(deleteBtn);
            
            li.appendChild(nameSpan);
            li.appendChild(actions);
            savedTrajectoriesList.appendChild(li);
        });
    }

    function loadTrajectoryFromFile(filename) {
        fetch(`../trajectories/${filename}`)
            .then(response => {
                if (!response.ok) throw new Error('Fichier non trouvé');
                return response.json();
            })
            .then(data => {
                // Clear current trajectory
                trajectoryPoints = [];
                pointList.innerHTML = '';
                const markers = mapWorld.querySelectorAll('.map-marker');
                markers.forEach(marker => marker.remove());

                if (data.startPoint && typeof data.startPoint.x === 'number' && typeof data.startPoint.y === 'number') {
                    startPoint = { x: data.startPoint.x, y: data.startPoint.y, type: 'start' };
                    drawPointOnMap(startPoint);
                }
                
                // Load trajectory
                pointIdCounter = 0;
                if (data.trajectory && Array.isArray(data.trajectory)) {
                    data.trajectory.forEach(point => {
                        const newPoint = {
                            id: pointIdCounter++,
                            x: point.x,
                            y: point.y,
                            type: point.type || 'normal',
                            photography: point.photography || 'no',
                            gps_x: (typeof point.gps_x !== 'undefined') ? point.gps_x : null,
                            gps_y: (typeof point.gps_y !== 'undefined') ? point.gps_y : null
                        };
                        // If file contained gps values, preserve them; otherwise compute via pixel_to_gps
                        if (newPoint.gps_x === null || newPoint.gps_y === null) {
                            attachGpsToPoint(newPoint);
                        }
                        trajectoryPoints.push(newPoint);
                        addPointToVisualList(newPoint);
                        drawPointOnMap(newPoint);
                    });
                }
                
                // Update trajectory line
                updateTrajectoryPath();
                redrawSerreSvg();

                missionTrajectoryData = data;
                missionTrajectoryFilename = filename;
                missionCookieRestoreTried = true;
                renderMissionTrajectoryOverlay();
                
                const trajName = data.meta && data.meta.name ? data.meta.name : filename.replace('.json', '');
                alert(`✅ Trajet "${trajName}" chargé (${trajectoryPoints.length} points)!`);
            })
            .catch(err => {
                console.error('Erreur chargement trajectoire:', err);
                alert('❌ Erreur lors du chargement du trajet');
            });
    }

    function deleteTrajectoryFile(filename) {
        if (!confirm(`Supprimer le trajet "${filename.replace('.json', '')}" ?`)) return;
        
        // Publier via ROS pour suppression
        const msg = new ROSLIB.Message({
            data: filename
        });
        
        deleteTrajectoryPub.publish(msg);
        console.log('Demande de suppression envoyée:', filename);
    }

    function loadForbiddenAreas() {
        // For now, just initialize empty
        // In a real application, you would load from a server or local storage
    }

    function loadBlankAreas() {
        // Load blank_area.json file
        fetch('./blank_area.json')
            .then(response => {
                if (!response.ok) {
                    throw new Error('Fichier blank_area.json introuvable');
                }
                return response.json();
            })
            .then(data => {
                // Clear existing areas
                forbiddenAreas = [];
                const rects = mapWorld.querySelectorAll('.forbidden-area');
                rects.forEach(r => r.remove());
                
                // Load areas from file
                if (data.forbiddenAreas && Array.isArray(data.forbiddenAreas)) {
                    data.forbiddenAreas.forEach(areaData => {
                        const area = {
                            x1: areaData.pixel.topLeft.x,
                            y1: areaData.pixel.topLeft.y,
                            x2: areaData.pixel.bottomRight.x,
                            y2: areaData.pixel.bottomRight.y
                        };
                        forbiddenAreas.push(area);
                        drawForbiddenArea(area);
                    });
                    console.log(`${forbiddenAreas.length} zones interdites chargées depuis blank_area.json`);
                }
            })
            .catch(error => {
                console.error('Erreur lors du chargement de blank_area.json:', error);
                alert('Impossible de charger le fichier blank_area.json');
            });
    }

    function findForbiddenAreaAt(x, y) {
        // Find if a point is inside a forbidden area
        for (let i = 0; i < forbiddenAreas.length; i++) {
            const area = forbiddenAreas[i];
            if (x >= area.x1 && x <= area.x2 && y >= area.y1 && y <= area.y2) {
                return i;
            }
        }
        return -1;
    }

    // --- Serre: helper functions to detect and open the serre view ---
    function isPointInPixelRect(px, py, rect) {
        return px >= rect.x1 && px <= rect.x2 && py >= rect.y1 && py <= rect.y2;
    }

    function checkOpenSerreForPoint(point) {
        if (!point) return;
        if (isPointInPixelRect(point.x, point.y, serreRectPixels)) {
            // open serre tab (no metric coordinates needed here)
            openSerreTab({ x: null, y: null });
        }
    }

    // Create a small 2-tab UI in the top-left corner of the map container
    function createTabsIfNeeded() {
        if (document.getElementById('map-tabs')) return;

        const tabs = document.createElement('div');
        tabs.id = 'map-tabs';
        Object.assign(tabs.style, {
            position: 'absolute',
            top: '8px',
            left: '8px',
            zIndex: 10000,
            display: 'flex',
            gap: '4px',
            background: 'transparent'
        });

        const btnExterior = document.createElement('button');
        btnExterior.id = 'tab-exterior';
        btnExterior.textContent = 'Extérieur';
        Object.assign(btnExterior.style, {
            padding: '6px 10px',
            background: '#5e95cf',
            border: '1px solid #ccc',
            borderRadius: '4px'
        });

        const btnSerre = document.createElement('button');
        btnSerre.id = 'tab-serre';
        btnSerre.textContent = 'Serre intérieur';
        Object.assign(btnSerre.style, {
            padding: '6px 10px',
            background: '#5e95cf',
            border: '1px solid #ccc',
            borderRadius: '4px'
        });

        btnExterior.addEventListener('click', () => activateTab('exterior'));
        btnSerre.addEventListener('click', () => activateTab('serre'));

        tabs.appendChild(btnExterior);
        tabs.appendChild(btnSerre);

        mapContainer.appendChild(tabs);

        // Create serre tab content – fills the container below the tab bar
        const serreContent = document.createElement('div');
        serreContent.id = 'serre-tab-content';
        Object.assign(serreContent.style, {
            position: 'absolute',
            top: '44px',
            left: '0',
            right: '0',
            bottom: '0',
            zIndex: 9999,
            background: '#ffffff',
            overflow: 'hidden',
            display: 'none'
        });

        // Use an <img> so we can map clicks to image pixels reliably (cover-like behaviour)
        const serreImg = document.createElement('img');
        serreImg.id = 'serre-image';
        serreImg.src = './serre.jpg';
        Object.assign(serreImg.style, {
            position: 'absolute',
            top: '0',
            left: '0',
            objectFit: 'fill'
        });

        // SVG overlay to draw points and connecting polyline (fills the whole content div)
        const serreSvg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
        serreSvg.id = 'serre-svg';
        Object.assign(serreSvg.style, {
            position: 'absolute',
            top: '0',
            left: '0',
            width: '100%',
            height: '100%',
            pointerEvents: 'none'
        });

        serreContent.appendChild(serreImg);
        serreContent.appendChild(serreSvg);
        mapContainer.appendChild(serreContent);

        // When the serre image loads, compute the display layout
        serreImg.addEventListener('load', () => {
            updateSerreImgLayout();
        });

        // Helper: convert a click inside serreContent to an exterior map pixel
        function serreClickToPoint(ev, type) {
            const rect = serreContent.getBoundingClientRect();
            const clickX = ev.clientX - rect.left;
            const clickY = ev.clientY - rect.top;
            const img = document.getElementById('serre-image');
            if (!img || !img.naturalWidth) return null;
            // display coords → serre image pixel
            const sImgX = (clickX - serreImgOffsetX) / serreImgDisplayScale;
            const sImgY = (clickY - serreImgOffsetY) / serreImgDisplayScale;
            // serre image pixel → exterior map pixel
            const rectW = serreRectPixels.x2 - serreRectPixels.x1;
            const rectH = serreRectPixels.y2 - serreRectPixels.y1;
            const extX = serreRectPixels.x1 + sImgX / img.naturalWidth  * rectW;
            const extY = serreRectPixels.y1 + sImgY / img.naturalHeight * rectH;
            const point = { id: pointIdCounter++, x: extX, y: extY, type, photography: type === 'photography' ? 'yes' : 'no' };
            attachGpsToPoint(point);
            return point;
        }

        // Left-click in serre → default point
        serreContent.addEventListener('click', (ev) => {
            if (ev.target.closest('#map-tabs')) return;
            ev.stopPropagation();
            const point = serreClickToPoint(ev, 'default');
            if (!point) return;
            trajectoryPoints.push(point);
            addPointToVisualList(point);
            drawPointOnMap(point);
            redrawSerreSvg();
        });

        // Right-click in serre → photography point (mirror of exterior contextmenu)
        serreContent.addEventListener('contextmenu', (ev) => {
            ev.preventDefault();
            ev.stopPropagation();
            const point = serreClickToPoint(ev, 'photography');
            if (!point) return;
            trajectoryPoints.push(point);
            addPointToVisualList(point);
            drawPointOnMap(point);
            checkOpenSerreForPoint(point);
            redrawSerreSvg();
        });

        // Ensure initial state shows exterior
        activateTab('exterior');
    }

    function activateTab(name) {
        const btnExterior = document.getElementById('tab-exterior');
        const btnSerre = document.getElementById('tab-serre');
        const serreContent = document.getElementById('serre-tab-content');

        if (!btnExterior || !btnSerre || !serreContent) return;

        // Exclusive view: either show the exterior map (`mapWorld`) or the serre content
        if (name === 'serre') {
            btnSerre.style.background = '#0056b3';
            btnExterior.style.background = '#5e95cf';
            serreContent.style.display = 'block';
            if (mapWorld) mapWorld.style.display = 'none';
            // Recompute layout now that the content div is visible
            updateSerreImgLayout();
        } else {
            btnSerre.style.background = '#5e95cf';
            btnExterior.style.background = '#0056b3';
            serreContent.style.display = 'none';
            if (mapWorld) mapWorld.style.display = 'block';
        }
    }

    function isSerreTabActive() {
        const serreContent = document.getElementById('serre-tab-content');
        if (!serreContent) return false;

        const computedDisplay = window.getComputedStyle(serreContent).display;
        const mapWorldDisplay = mapWorld ? window.getComputedStyle(mapWorld).display : 'unknown';
        return computedDisplay !== 'none' || mapWorldDisplay === 'none';
    }

    function switchToExteriorTabIfSerreActive() {
        if (isSerreTabActive()) {
            activateTab('exterior');
        }
    }

    function openSerreTab(metricCoords) {
        createTabsIfNeeded();
        activateTab('serre');
        // metricCoords is an object {x,y} in metres (previously called 'gps')
    }

    function redrawSerreSvg() {
        const svg = document.getElementById('serre-svg');
        if (!svg) return;
        svg.innerHTML = ''; // clear

        if (!trajectoryPoints || trajectoryPoints.length === 0) return;

        // Map each exterior pixel → serre image pixel → serre display coords
        const imgEl = document.getElementById('serre-image');
        const serreNatW = imgEl ? imgEl.naturalWidth  : 1;
        const serreNatH = imgEl ? imgEl.naturalHeight : 1;
        const rectW = serreRectPixels.x2 - serreRectPixels.x1;
        const rectH = serreRectPixels.y2 - serreRectPixels.y1;
        const displayPts = trajectoryPoints.map(p => {
            const sImgX = (p.x - serreRectPixels.x1) / rectW * serreNatW;
            const sImgY = (p.y - serreRectPixels.y1) / rectH * serreNatH;
            return {
                cx: sImgX * serreImgDisplayScale + serreImgOffsetX,
                cy: sImgY * serreImgDisplayScale + serreImgOffsetY,
                type: p.type
            };
        });

        const ns = 'http://www.w3.org/2000/svg';

        // Polyline – mirror style from main trajectoryPath but thinner
        const poly = document.createElementNS(ns, 'polyline');
        poly.setAttribute('points', displayPts.map(pt => `${pt.cx},${pt.cy}`).join(' '));
        poly.setAttribute('fill', 'none');
        try {
            if (trajectoryPath) {
                const cs = getComputedStyle(trajectoryPath);
                const stroke = cs.getPropertyValue('stroke') || '#e74c3c';
                poly.setAttribute('stroke',       stroke.trim() || '#e74c3c');
                poly.setAttribute('stroke-width', '4');
                poly.setAttribute('stroke-linecap',  cs.getPropertyValue('stroke-linecap')  || 'round');
                poly.setAttribute('stroke-linejoin', cs.getPropertyValue('stroke-linejoin') || 'round');
            } else {
                poly.setAttribute('stroke', '#e74c3c');
                poly.setAttribute('stroke-width', '4');
            }
        } catch (e) {
            poly.setAttribute('stroke', '#e74c3c');
            poly.setAttribute('stroke-width', '4');
        }
        svg.appendChild(poly);

        // Point markers – red for normal, green for photography (matches exterior .map-marker CSS)
        displayPts.forEach(pt => {
            const isPhoto = pt.type === 'photography';
            const c = document.createElementNS(ns, 'circle');
            c.setAttribute('cx', pt.cx);
            c.setAttribute('cy', pt.cy);
            c.setAttribute('r',  '10');
            c.setAttribute('fill',         isPhoto ? '#00ff00' : '#e74c3c');
            c.setAttribute('stroke',       '#fff');
            c.setAttribute('stroke-width', '2.5');
            svg.appendChild(c);
        });
    }

    function redrawForbiddenAreas() {
        // Remove all existing rectangles
        const rects = mapWorld.querySelectorAll('.forbidden-area');
        rects.forEach(r => r.remove());
        
        // Redraw all areas
        forbiddenAreas.forEach(area => drawForbiddenArea(area));
    }

    // Compute serre image display layout: contain-fit at 70% of the tab content area, centred.
    // serreImgDisplayScale / Offset are used by redrawSerreSvg and serreClickToPoint.
    function updateSerreImgLayout() {
        const img = document.getElementById('serre-image');
        const svg = document.getElementById('serre-svg');
        if (!img || !img.naturalWidth) return;

        const tabH  = 44;
        const contW = mapContainer.clientWidth;
        const contH = Math.max(1, mapContainer.clientHeight - tabH);

        // Contain-fit, scaled to 70% of the available area
        const fillRatio = 0.70;
        const sc = Math.min(contW / img.naturalWidth, contH / img.naturalHeight) * fillRatio;
        const renderedW = Math.round(img.naturalWidth  * sc);
        const renderedH = Math.round(img.naturalHeight * sc);
        const offX = Math.round((contW - renderedW) / 2);
        const offY = Math.round((contH - renderedH) / 2);

        img.style.position = 'absolute';
        img.style.left   = `${offX}px`;
        img.style.top    = `${offY}px`;
        img.style.width  = `${renderedW}px`;
        img.style.height = `${renderedH}px`;

        if (svg) {
            svg.setAttribute('width',  contW);
            svg.setAttribute('height', contH);
            svg.style.width  = `${contW}px`;
            svg.style.height = `${contH}px`;
        }

        serreImgDisplayScale = sc;
        serreImgOffsetX      = offX;
        serreImgOffsetY      = offY;
        redrawSerreSvg();
    }

    // --- Collapsible Sections ---
    window.toggleSection = function(sectionId) {
        const section = document.getElementById(sectionId);
        section.classList.toggle('collapsed');
        
        // Update arrow
        const header = section.previousElementSibling;
        const arrow = header.querySelector('span');
        if (arrow) {
            arrow.textContent = section.classList.contains('collapsed') ? '▶ ' + arrow.textContent.substring(2) : '▼ ' + arrow.textContent.substring(2);
        }
    };

    // Font size slider - sync with header slider
    const fontSizes = ['small', 'medium', 'large'];
    const fontSizeLabels = ['Petit', 'Moyen', 'Grand'];
    
    if (headerFontSizeSlider) {
        headerFontSizeSlider.addEventListener('input', () => {
            const value = parseInt(headerFontSizeSlider.value);
            const size = fontSizes[value - 1];
            
            // Remove all font classes
            document.body.classList.remove('font-small', 'font-medium', 'font-large');
            
            // Add selected class
            document.body.classList.add(`font-${size}`);
            
            // Update labels
            headerFontSizeLabel.textContent = fontSizeLabels[value - 1];
            
            // Save preference
            setCookie('fontSize', size, 365);
        });
    }

    // Dark mode toggle
    if (darkModeToggle) {
        darkModeToggle.addEventListener('click', () => {
            document.body.classList.toggle('dark-mode');
            const isDarkMode = document.body.classList.contains('dark-mode');
            darkModeToggle.textContent = isDarkMode ? '☀️' : '🌙';
            
            // Save preference
            setCookie('darkMode', isDarkMode ? 'on' : 'off', 365);
        });
    }

    // Load saved font size preference
    const savedFontSize = getCookie('fontSize');
    if (savedFontSize) {
        document.body.classList.add(`font-${savedFontSize}`);
        const sizeIndex = fontSizes.indexOf(savedFontSize);
        if (sizeIndex !== -1) {
            if (headerFontSizeSlider) headerFontSizeSlider.value = sizeIndex + 1;
            if (headerFontSizeLabel) headerFontSizeLabel.textContent = fontSizeLabels[sizeIndex];
        }
    } else {
        // Default to medium
        document.body.classList.add('font-medium');
        if (headerFontSizeSlider) headerFontSizeSlider.value = 2;
        if (headerFontSizeLabel) headerFontSizeLabel.textContent = 'Moyen';
    }

    // Load saved dark mode preference
    const savedDarkMode = getCookie('darkMode');
    if (savedDarkMode === 'on') {
        document.body.classList.add('dark-mode');
        if (darkModeToggle) darkModeToggle.textContent = '☀️';
    }

    clearForbiddenAreasBtn.addEventListener('click', () => {
        if (confirm('Voulez-vous vraiment effacer toutes les zones interdites ?')) {
            forbiddenAreas = [];
            const rects = mapWorld.querySelectorAll('.forbidden-area');
            rects.forEach(r => r.remove());
        }
    });

    resetForbiddenAreasBtn.addEventListener('click', () => {
        loadBlankAreas();
    });

    // Header settings button - make it rotate
    if (headerSettingsBtn) {
        headerSettingsBtn.addEventListener('mouseover', () => {
            headerSettingsBtn.style.transform = 'rotate(180deg)';
        });
        headerSettingsBtn.addEventListener('mouseout', () => {
            headerSettingsBtn.style.transform = 'rotate(0deg)';
        });
    }

    // Header toggle edit mode - sync with main button
    if (headerToggleEditModeBtn) {
        headerToggleEditModeBtn.addEventListener('click', () => {
            toggleEditModeBtn.click();
        });
    }

    // Header clear forbidden areas - sync with main button
    if (headerClearForbiddenAreasBtn) {
        headerClearForbiddenAreasBtn.addEventListener('click', () => {
            clearForbiddenAreasBtn.click();
        });
    }

    // Header reset forbidden areas - sync with main button
    if (headerResetForbiddenAreasBtn) {
        headerResetForbiddenAreasBtn.addEventListener('click', () => {
            resetForbiddenAreasBtn.click();
        });
    }

    // Update header buttons when main toggleEditModeBtn changes
    const originalToggleEditModeClick = toggleEditModeBtn.onclick;
    const updateHeaderEditMode = () => {
        if (headerToggleEditModeBtn) {
            headerToggleEditModeBtn.textContent = toggleEditModeBtn.textContent;
            headerToggleEditModeBtn.style.backgroundColor = toggleEditModeBtn.style.backgroundColor;
            headerEditButtons.style.display = editButtons.style.display;
        }
    };
    
    // Observe changes to editButtons visibility
    const observer = new MutationObserver(updateHeaderEditMode);
    observer.observe(editButtons, { attributes: true, attributeFilter: ['style'] });
});

// =======================================================================
// FONCTIONS GLOBALES
// =======================================================================

function toggleHeaderSettings() {
    const modal = document.getElementById('header-settings-modal');
    if (modal) {
        modal.style.display = (modal.style.display === 'block') ? 'none' : 'block';
    }
}

// Fermer la modale quand on clique en dehors
document.addEventListener('click', (e) => {
    const modal = document.getElementById('header-settings-modal');
    if (modal && e.target === modal) {
        modal.style.display = 'none';
    }
});

// Charger les préférences au démarrage
window.addEventListener('load', () => {
    const fontSizeSlider = document.getElementById('header-font-size-slider');
    const fontSizeLabels = ['Petit', 'Moyen', 'Grand'];
    const fontSizes = ['small', 'medium', 'large'];
    
    const savedFontSize = getCookie('fontSize');
    if (savedFontSize && fontSizeSlider) {
        const sizeIndex = fontSizes.indexOf(savedFontSize);
        if (sizeIndex !== -1) {
            fontSizeSlider.value = sizeIndex + 1;
        }
    }
    
    const darkMode = getCookie('darkMode');
    if (darkMode === 'on') {
        document.body.classList.add('dark-mode');
        const btn = document.getElementById('dark-mode-toggle');
        if (btn) {
            btn.textContent = '☀️';
        }
    }
});
