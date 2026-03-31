
// Arrêt d'urgence
function emergencyStop() {
    // Publie l'arrêt d'urgence vers ROS2
    emergencyPub.publish(new ROSLIB.Message({ data: true }));
    
    // Arrête tout mouvement du robot
    sendCmd('stop');
    sendPtz('stop');
    
    // Arrête la mission si active
    if (missionActive) {
        toggleMission();
    }
    
    // Arrête l'enregistrement vidéo si actif
    if (isRecording) {
        toggleVideo();
    }
    
    // Affiche une alerte visuelle
    const btn = document.getElementById('btnEmergency');
    btn.style.animation = 'pulse 0.5s 3';
    setTimeout(() => {
        btn.style.animation = '';
    }, 1500);
    
    console.log('ARRÊT D\'URGENCE ACTIVÉ');
    logEvent('ARRÊT D\'URGENCE ACTIVÉ', 'error');
}


// Throttle RAF : limite le rendu de l'odométrie à ~60 Hz max
let _odomRafPending = false;
// Références DOM cachées (remplies au DOMContentLoaded, évite getElementById à ~60 Hz)
let _mapAreaEl    = null;
let _robotGroupEl = null;
let _robotHaloEl  = null;
let _robotRippleEl = null;
let _robotDotEl = null;
let _robotDotDefaultFill = null;
// État prise de photo courant (mis à jour par mission feedback, lu par l'odométrie)
let _currentlyTakingPhoto = false;
// Suivi freshness odometrie
let _lastOdometryMs = Date.now()-3000;  // initialisé à "il y a longtemps" pour afficher offline au démarrage
let _odometryOffline = false;
let _missionFeedbackTrajectoryRestoreTried = false;


// Topics mission (remplacement du ActionClient incompatible avec rosbridge ROS2)
const missionStartPub = new ROSLIB.Topic({ ros, name: '/ui/start_mission',  messageType: 'std_msgs/String' });
const missionCancelPub = new ROSLIB.Topic({ ros, name: '/ui/cancel_mission', messageType: 'std_msgs/String' });

const missionFeedbackSub = new ROSLIB.Topic({ ros, name: '/ui/mission_feedback', messageType: 'std_msgs/String' });
missionFeedbackSub.subscribe(async (msg) => {
    try {
        const fb    = JSON.parse(msg.data);
        console.info('[MISSION][FEEDBACK] received', {
            hasTrajectoryInMemory: !!currentTrajectoryData,
            restoreAlreadyTried: _missionFeedbackTrajectoryRestoreTried,
            waypointIndex: fb.current_waypoint_index
        });

        // Si aucun trajet n'est chargé, tenter une seule restauration depuis le cookie.
        if (!currentTrajectoryData && !_missionFeedbackTrajectoryRestoreTried) {
            _missionFeedbackTrajectoryRestoreTried = true;
            console.warn('[MISSION][FEEDBACK] currentTrajectoryData absent, tentative de restauration depuis cookie');
            if (typeof tryLoadLastMissionTrajectoryFromCookie === 'function') {
                const restored = await tryLoadLastMissionTrajectoryFromCookie();
                console.info('[MISSION][FEEDBACK] resultat restauration cookie', {
                    restored,
                    hasTrajectoryAfterRestore: !!currentTrajectoryData
                });
                if (!restored) {
                    logEvent('Mission feedback reçu sans trajet chargé (restauration cookie échouée)', 'warn');
                }
            } else {
                console.error('[MISSION][FEEDBACK] tryLoadLastMissionTrajectoryFromCookie indisponible');
            }
        }

        idx   = (fb.current_waypoint_index !== undefined ? fb.current_waypoint_index + 1 : '?');
        const total = currentTrajectoryData && currentTrajectoryData.trajectory
                      ? currentTrajectoryData.trajectory.length : '?';
        const dist  = fb.distance_remaining !== undefined
                      ? ' — ' + fb.distance_remaining.toFixed(1) + ' m' : '';
        const info  = document.getElementById('trajInfo');
        const isTakingPhoto = !!fb.is_taking_photo;
        _currentlyTakingPhoto = isTakingPhoto;  // mémoriser pour l'odométrie RAF
        const robot_actual_pos_x = fb.robot_x !== undefined ? fb.robot_x : null;
        const robot_actual_pos_y = fb.robot_y !== undefined ? fb.robot_y : null;
        
        // Mettre à jour l'index du waypoint actuel
        if (fb.current_waypoint_index !== undefined) {
            currentWaypointIndex = fb.current_waypoint_index;
        }
        if (info) info.innerText = `🧭 Mission en cours – waypoint ${idx}/${total}${dist}`;
        // console.log('[MISSION] feedback reçu :', fb);
        logEvent(`Mission en cours – waypoint ${idx}/${total}`, 'info');
        if (isTakingPhoto) {
            logEvent(`📸 Prise de photo au waypoint ${idx}`, 'info');
        }
        //[MISSION] erreur parsing feedback: ReferenceError: origin_map_size is not defined
        // Mise à jour de la position du robot sur la carte
        if (robot_actual_pos_x !== null && robot_actual_pos_y !== null) {
            const mapArea = document.getElementById('mapArea');
            // Convertir en pixels naturels de l'image (espace 0→origin_map_size)
            const pos = meters_to_pixels(robot_actual_pos_x, robot_actual_pos_y, map_size.width, map_size.height);
            const pixel_x = pos.pixel_x;
            const pixel_y = pos.pixel_y;
            // store last known pixel for lock mode
            lastKnownRobotPixel = { x: pixel_x, y: pixel_y };
            lastKnownRobotPosition = { x: robot_actual_pos_x, y: robot_actual_pos_y };
            // console.log(`Position robot (pixels, lastknownrobotpixel): (${lastKnownRobotPixel.x.toFixed(1)}, ${lastKnownRobotPixel.y.toFixed(1)})`);  
            // console.log(`Position robot (mètres, lastknownrobotposition): (${lastKnownRobotPosition.x.toFixed(2)}, ${lastKnownRobotPosition.y.toFixed(2)})`);
            updateRobotDotOnMap(pixel_x, pixel_y, isTakingPhoto);
            try {
                const controller = mapArea && mapArea._panController;
                if (controller && controller.mode === 'lock') {
                    // update stored lock pixel and recenter the displayed background
                    mapArea.dataset.lockPixel = `${pixel_x},${pixel_y}`;
                    await controller.lockToPixel(pixel_x, pixel_y);
                }
            } catch (e) {
                console.warn('Erreur recentrage carte en mode lock :', e);
            }
        }
    } catch(e) { console.error('[MISSION] erreur parsing feedback:', e); }
});

const missionResultSub = new ROSLIB.Topic({ ros, name: '/ui/mission_result', messageType: 'std_msgs/String' });
missionResultSub.subscribe((msg) => {
    try {
        const result = JSON.parse(msg.data);
        const btn = document.getElementById('btnMission');
        missionActive = false;
        currentWaypointIndex = -1;
        _currentlyTakingPhoto = false;
        hideRobotDot();
        if (btn) { btn.innerText = '🚀 Lancer la mission'; btn.className = 'mission-btn start'; }
        const info = document.getElementById('trajInfo');
        if (result.success) {
            if (info) info.innerText = '✅ Mission terminée !';
            showToast('✅ Mission terminée avec succès !', 'success');
            logEvent('✅ Mission terminée', 'success');
        } else {
            if (info) info.innerText = `❌ Mission échouée : ${result.message}`;
            showToast(`❌ Mission échouée : ${result.message}`, 'error');
            logEvent(`Mission échouée : ${result.message}`, 'error');
        }
        missionPub.publish(new ROSLIB.Message({ data: false }));
        if (typeof unloadCurrentTrajectory === 'function') {
            unloadCurrentTrajectory({
                clearSelection: true,
                clearCookie: true,
                clearInfo: false,
            });
        } else {
            const mapArea = _mapAreaEl || document.getElementById('mapArea');
            if (mapArea) {
                updateTrajectoryDisplay(mapArea).catch((e) => {
                    console.warn('Erreur redraw trajectoire après fin de mission :', e);
                });
            }
        }
        console.log('[MISSION] résultat reçu :', result);
    } catch(e) { console.error('[MISSION] erreur parsing result:', e); }
});

// Abonnement permanent à l'odométrie filtrée pour suivi position robot même hors mission
const odometryFilteredSub = new ROSLIB.Topic({
    ros: ros,
    name: '/odometry/filtered',
    messageType: 'nav_msgs/Odometry'
});
odometryFilteredSub.subscribe((msg) => {
    try {
        _lastOdometryMs = Date.now();
        if (_odometryOffline) {
            setRobotOfflineState(false);
        }
        const x = msg.pose.pose.position.x;
        const y = msg.pose.pose.position.y;
        const rot_w = msg.pose.pose.orientation.w;
        const rot_z = msg.pose.pose.orientation.z;

        robot_orientation = -Math.atan2(2 * rot_w * rot_z, 1 - 2 * rot_z * rot_z) + thetaDegrees * Math.PI / 180;
        
        const var_x = msg.pose.covariance[0];
        const var_y = msg.pose.covariance[7];
        covariance_radius = Math.sqrt(Math.max(var_x || 0, var_y || 0));


        const pos = meters_to_pixels(x, y, map_size.width, map_size.height);
        
        if (!isFinite(pos.pixel_x) || !isFinite(pos.pixel_y)) return;
        // Mise à jour synchrone de la position (pas de DOM ici)
        lastKnownRobotPixel = { x: pos.pixel_x, y: pos.pixel_y };
        lastKnownRobotPosition = { x, y };
        if (typeof updateMapAutoZoomByRobotPosition === 'function') {
            updateMapAutoZoomByRobotPosition().catch((e) => {
                console.warn('[AUTO ZOOM] erreur mise a jour:', e);
            });
        }
        // Throttle : un seul rendu par frame d'affichage (~60 Hz max)
        if (_odomRafPending) return;
        _odomRafPending = true;
        counterOdometryFrames++;
        if (counterOdometryFrames % 10 === 0) {
            requestAnimationFrame(async () => {
                _odomRafPending = false;
                if (!lastKnownRobotPixel) return;
                const { x: px, y: py } = lastKnownRobotPixel;
                await updateRobotDotOnMap(px, py, _currentlyTakingPhoto);
                // Ne pas recharger le trajet (drawTrajectoryOnMap attend la structure originale),
                // appeler plutôt le redraw qui applique l'index courant et les offsets.
                const mapArea = _mapAreaEl;
                if (mapArea) await updateTrajectoryDisplay(mapArea);
                
                if (!mapArea) return;
                const controller = mapArea._panController;
                if (controller && controller.mode === 'lock') {
                mapArea.dataset.lockPixel = `${px},${py}`;
                    await controller.lockToPixel(px, py);
                }
            });
        }
    } catch (e) {
        console.warn('[ODOMETRY] erreur traitement:', e);
    }
});

function setRobotOfflineState(isOffline) {
    _odometryOffline = isOffline;
    const badge = document.getElementById('robotOfflineBadge');
    if (badge) {
        badge.classList.toggle('visible', isOffline);
    }
    const dot = _robotDotEl || document.getElementById('robotDot');
    if (dot) {
        if (isOffline) {
            dot.setAttribute('fill', '#727166');
        } else if (_robotDotDefaultFill) {
            dot.setAttribute('fill', _robotDotDefaultFill);
        }
    }
    if (isOffline) {
        covariance_radius = 0;  // ne pas afficher l'incertitude quand offline

        // Si la carte est lockée, la repasser immédiatement en mode pan.
        const mapArea = _mapAreaEl || document.getElementById('mapArea');
        const controller = mapArea && mapArea._panController;
        if (controller && controller.mode === 'lock') {
            controller.enable();
            controller.mode = 'pan';
            if (typeof setMapModeLockedVisual === 'function') {
                setMapModeLockedVisual(false);
            }
            const btn = document.getElementById('mapModeBtn');
            if (btn) btn.title = 'Mode: Pan (clic pour Lock)';
            logEvent('Carte: lock désactivé automatiquement (odométrie hors ligne)', 'warn');
        }
    }
}

// setRobotOfflineState(true);


// Surveiller le silence odometrie (5s)
setInterval(() => {
    const silentForMs = Date.now() - _lastOdometryMs;
    if (silentForMs >= 5000) {
        if (!_odometryOffline) setRobotOfflineState(true);
    } else if (_odometryOffline) {
        setRobotOfflineState(false);
    }
}, 250);

// Mise a jour periodique du startpoint (5 Hz)
setInterval(() => {
    if (_odometryOffline) return;  // ne mettre à jour le startpoint que quand le robot est online
    try {
        if (!currentTrajectoryRawPoints) return;
        const mapArea = _mapAreaEl || document.getElementById('mapArea');
        if (!mapArea) return;
        const { originalW, originalH } = currentTrajectoryRawPoints;
        if (!originalW || !originalH) return;
        const pos = meters_to_pixels(
            lastKnownRobotPosition.x,
            lastKnownRobotPosition.y,
            originalW,
            originalH
        );
        if (!isFinite(pos.pixel_x) || !isFinite(pos.pixel_y)) return;
        currentTrajectoryRawPoints.startPoint = pos;
        updateTrajectoryDisplay(mapArea).catch(() => {});
    } catch (e) {
        console.warn('[STARTPOINT] erreur mise a jour:', e);
    }
}, 200);

// Mise a jour periodique du point robot (5 Hz)
setInterval(() => {
    try {
        if (!lastKnownRobotPixel) return;
        const mapArea = _mapAreaEl || document.getElementById('mapArea');
        if (!mapArea) return;
        updateRobotDotOnMap(
            lastKnownRobotPixel.x,
            lastKnownRobotPixel.y,
            _currentlyTakingPhoto,
        ).catch(() => {});
    } catch (e) {
        console.warn('[ROBOT DOT] erreur mise a jour:', e);
    }
}, 200);

