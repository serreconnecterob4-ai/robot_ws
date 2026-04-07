const MEDIA_DEBUG = false;

function mediaDebug(...args) {
    if (MEDIA_DEBUG) {
        console.log(...args);
    }
}

function toggleMission() {
    mediaDebug('[MISSION] toggleMission() appelé, missionActive avant =', missionActive);
    const btn = document.getElementById('btnMission');


    if (missionActive==false) {
        mediaDebug('[MISSION] currentTrajectoryData =', currentTrajectoryData);
        // Vérifier qu'un trajet est chargé avec des points
        if (!currentTrajectoryData || !currentTrajectoryData.trajectory || currentTrajectoryData.trajectory.length === 0) {
            const info = document.getElementById('trajInfo');
            if (info) {
                info.innerText = '⚠️ Veuillez d\'abord charger un trajet (bouton 👁️).';
                info.style.color = '#e74c3c';
                setTimeout(() => { info.style.color = ''; info.innerText = 'Aucun trajet chargé'; }, 3000);
            }
            const sel = document.getElementById('trajSelect');
            if (sel) {
                sel.style.border = '2px solid #e74c3c';
                sel.style.boxShadow = '0 0 8px #e74c3c';
                setTimeout(() => { sel.style.border = ''; sel.style.boxShadow = ''; }, 2000);
            }
            showToast('⚠️ Aucun trajet chargé ! Sélectionnez et chargez un trajet d\'abord.', 'error');
            logEvent('Mission non lancée : aucun trajet chargé', 'warn');
            missionActive = false;
            return;
        }

        const trajectory = currentTrajectoryData.trajectory;
        const waypoints_x = trajectory.map(p => p.gps_x || 0);
        const waypoints_y = trajectory.map(p => p.gps_y || 0);
        const take_photo = trajectory.map(p => (p.photography === 'yes' || p.type === 'photography'));

        mediaDebug('[MISSION] Waypoints GPS extraits :', waypoints_x, waypoints_y, take_photo);

        // --- Logs UI : coordonnées pixels envoyées au serveur ---
        logEvent(`📡 Envoi mission : ${trajectory.length} waypoint(s) (coordonnées pixels)`, 'info');
        trajectory.forEach((p, i) => {
            const px = p.gps_x;
            const py = p.gps_y;
            const photo = take_photo[i] ? ' 📸' : '';
            logEvent(
                `  [${i}] X: ${px.toFixed(1)} px  Y: ${py.toFixed(1)} px${photo}`,
                'info'
            );
        });

        if(_odometryOffline) {
            showToast('⚠️ Robot hors ligne, mission envoyée mais le robot n\'est pas connecté. Mission non prise en compte', 'warn');
            logEvent('Mission envoyée mais robot hors ligne', 'warn');
            return;
        }

        missionActive = true;
        missionPaused = false;
        if (typeof setEmergencyButtonPausedState === 'function') {
            setEmergencyButtonPausedState(false);
        }
        mediaDebug('[MISSION] missionActive après toggle =', missionActive);


        btn.innerText = '🛑 Arrêter la mission';
        btn.className = 'mission-btn stop';

        activeMissionRequestId = `m_${Date.now()}_${Math.random().toString(36).slice(2, 8)}`;

        missionStartPub.publish(new ROSLIB.Message({
            data: JSON.stringify({ waypoints_x, waypoints_y, take_photo, mission_id: activeMissionRequestId })
        }));

        const selectedTrajectory = document.getElementById('trajSelect')?.value;
        if (typeof saveLastMissionTrajectoryName === 'function' && selectedTrajectory) {
            saveLastMissionTrajectoryName(selectedTrajectory);
        }

        const info = document.getElementById('trajInfo');
        if (info) info.innerText = '🚀 Mission démarrée, en attente de Nav2...';

        missionPub.publish(new ROSLIB.Message({ data: true }));
        logEvent(`Mission lancée : ${trajectory.length} waypoints`, 'success');
    } else {
        if(_odometryOffline) {
            showToast('⚠️ Robot hors ligne, mission ne peut pas être annulée. Si une mission était en cours, elle ne sera pas arrêtée côté robot.', 'warn');
            logEvent('Mission annulée localement mais robot hors ligne', 'warn');
            return;
        }

        missionActive = !missionActive;
        missionPaused = false;
        if (typeof setEmergencyButtonPausedState === 'function') {
            setEmergencyButtonPausedState(false);
        }
        mediaDebug('[MISSION] missionActive après toggle =', missionActive);
        mediaDebug('[MISSION] Annulation de la mission');
        currentWaypointIndex = -1;
        btn.innerText = '🚀 Lancer la mission';
        btn.className = 'mission-btn start';
        _currentlyTakingPhoto = false;
        hideRobotDot();

        missionCancelPub.publish(new ROSLIB.Message({ data: 'cancel' }));
        activeMissionRequestId = null;

        const info = document.getElementById('trajInfo');
        if (info) info.innerText = 'Mission annulée';

        missionPub.publish(new ROSLIB.Message({ data: false }));
        const mapArea = _mapAreaEl || document.getElementById('mapArea');
        if (mapArea) {
            updateTrajectoryDisplay(mapArea).catch((e) => {
                console.warn('Erreur redraw trajectoire après annulation :', e);
            });
        }
        logEvent('Mission arrêtée', 'warn');
    }
}

// Photo & Vidéo
let isRecording = false;
let mediaRecorder = null;
let recordedChunks = [];

async function uploadBlob(blob, endpoint, filename) {
    mediaDebug('[UPLOAD] Début upload:', { endpoint, filename, blobSize: blob.size, blobType: blob.type });
    
    const params = new URLSearchParams({ filename });
    const url = `${endpoint}?${params.toString()}`;
    mediaDebug('[UPLOAD] URL finale:', url);
    
    try {
        const res = await fetch(url, {
            method: 'POST',
            body: blob
        });
        mediaDebug('[UPLOAD] Réponse reçue:', { status: res.status, ok: res.ok });
        
        if (!res.ok) {
            const errorMsg = `Upload failed: ${res.status} ${res.statusText}`;
            console.error('[UPLOAD] Erreur:', errorMsg);
            logEvent(`[UPLOAD] ❌ Erreur ${res.status}: ${filename}`, 'error');
            throw new Error(errorMsg);
        }
        
        mediaDebug('[UPLOAD] ✅ Succès pour:', filename);
        logEvent(`[UPLOAD] ✅ ${filename} envoyé avec succès`, 'success');
    } catch (e) {
        console.error('[UPLOAD] Exception:', e.message);
        logEvent(`[UPLOAD] Exception: ${e.message}`, 'error');
        throw e;
    }
}

function takePhoto() {
    mediaDebug('[PHOTO] takePhoto() appelée');
    
    mediaDebug('[PHOTO] État de la vidéo:', {
        videoElement: !!videoElement,
        srcObject: !!videoElement?.srcObject,
        videoWidth: videoElement?.videoWidth,
        videoHeight: videoElement?.videoHeight
    });
    
    if (videoElement && videoElement.srcObject && videoElement.videoWidth) {
        mediaDebug('[PHOTO] Mode WebRTC: Capture en cours...');
        
        const canvas = document.createElement('canvas');
        canvas.width = videoElement.videoWidth;
        canvas.height = videoElement.videoHeight;
        mediaDebug('[PHOTO] Canvas créé:', { width: canvas.width, height: canvas.height });
        
        const ctx = canvas.getContext('2d');
        ctx.drawImage(videoElement, 0, 0, canvas.width, canvas.height);
        mediaDebug('[PHOTO] Image copiée sur canvas');
        
        canvas.toBlob(async (blob) => {
            mediaDebug('[PHOTO] Blob généré:', { size: blob?.size, type: blob?.type });
            
            if (blob) {
                const filename = `photo_${Date.now()}.jpg`;
                mediaDebug('[PHOTO] Filename généré:', filename);
                
                try {
                    await uploadBlob(blob, '/upload_photo', filename);
                    mediaDebug('[PHOTO] ✅ Upload réussi');
                    logEvent('✅ Photo prise (WebRTC)', 'success');
                } catch (e) {
                    console.error('[PHOTO] ❌ Erreur upload:', e);
                    logEvent(`❌ Erreur upload photo: ${e.message}`, 'error');
                }
            } else {
                console.error('[PHOTO] ❌ Blob est null');
                logEvent('❌ Erreur: Blob est null', 'error');
            }
        }, 'image/jpeg', 0.95);
        return;
    }

    mediaDebug('[PHOTO] Mode ROS2: WebRTC non disponible, appel service ROS2');
    
    photoClient.callService(new ROSLIB.ServiceRequest(), (res) => {
        mediaDebug('[PHOTO] Réponse ROS2:', res);
        logEvent(res.success ? '✅ Photo prise (ROS2)' : '❌ Erreur prise photo (ROS2)', res.success ? 'success' : 'error');
        alert(res.success ? "📸 Prise !" : "Erreur");
    });
}

function toggleVideo() {
    mediaDebug('[VIDEO] toggleVideo() appelée, isRecording:', isRecording);
    
    let btn = document.getElementById('btnRecord');
    
    mediaDebug('[VIDEO] Vérification disponibilité:', {
        videoElement: !!videoElement,
        srcObject: !!videoElement?.srcObject,
        MediaRecorder: !!window.MediaRecorder
    });

    if (videoElement && videoElement.srcObject && window.MediaRecorder) {
        mediaDebug('[VIDEO] Mode WebRTC disponible');
        
        if (!isRecording) {
            mediaDebug('[VIDEO] 🔴 DÉMARRAGE enregistrement WebRTC');
            logEvent('[VIDEO] 🔴 DÉMARRAGE enregistrement (WebRTC)', 'success');
            
            recordedChunks = [];
            mediaDebug('[VIDEO] recordedChunks réinitialisé');
            
            try {
                mediaRecorder = new MediaRecorder(videoElement.srcObject, { mimeType: 'video/webm;codecs=vp8' });
                mediaDebug('[VIDEO] MediaRecorder créé avec état:', mediaRecorder.state);
                
                mediaRecorder.ondataavailable = (event) => {
                    mediaDebug('[VIDEO] ondataavailable:', { size: event.data.size });
                    if (event.data && event.data.size > 0) {
                        recordedChunks.push(event.data);
                        mediaDebug('[VIDEO] Chunk ajouté. Total chunks:', recordedChunks.length);
                    }
                };
                
                mediaRecorder.onstop = async () => {
                    mediaDebug('[VIDEO] onstop appelé. Total chunks:', recordedChunks.length);
                    
                    const totalSize = recordedChunks.reduce((sum, chunk) => sum + chunk.size, 0);
                    mediaDebug('[VIDEO] Taille totale:', { bytes: totalSize, MB: (totalSize/1024/1024).toFixed(2) });
                    
                    const blob = new Blob(recordedChunks, { type: 'video/webm' });
                    mediaDebug('[VIDEO] Blob créé:', { size: blob.size, type: blob.type });
                    
                    recordedChunks = [];
                    mediaDebug('[VIDEO] recordedChunks vidé');
                    
                    const filename = `video_${Date.now()}.webm`;
                    mediaDebug('[VIDEO] Filename:', filename);
                    
                    try {
                        await uploadBlob(blob, '/upload_video', filename);
                        mediaDebug('[VIDEO] ✅ Upload vidéo réussi');
                        logEvent('✅ Vidéo sauvegardée (WebRTC)', 'success');
                    } catch (e) {
                        console.error('[VIDEO] ❌ Erreur upload:', e);
                        logEvent(`❌ Erreur upload vidéo: ${e.message}`, 'error');
                    }
                };
                
                mediaRecorder.onerror = (event) => {
                    console.error('[VIDEO] MediaRecorder error:', event.error);
                    logEvent(`❌ Erreur MediaRecorder: ${event.error}`, 'error');
                };
                
                mediaRecorder.start();
                mediaDebug('[VIDEO] MediaRecorder.start() appelé, état:', mediaRecorder.state);
            } catch (e) {
                console.error('[VIDEO] Erreur création MediaRecorder:', e);
                logEvent(`❌ Erreur MediaRecorder: ${e.message}`, 'error');
                return;
            }
            
            isRecording = true;
            btn.innerText = "⏹ STOP";
            btn.style.backgroundColor = "black";
            mediaDebug('[VIDEO] UI mise à jour');
            logEvent('[VIDEO] Enregistrement vidéo démarré (WebRTC)', 'success');
        } else {
            mediaDebug('[VIDEO] 🛑 ARRÊT enregistrement WebRTC');
            
            mediaDebug('[VIDEO] Avant stop - état:', mediaRecorder?.state);
            mediaRecorder.stop();
            mediaDebug('[VIDEO] stop() appelé');
            
            isRecording = false;
            btn.innerText = "🔴 REC";
            btn.style.backgroundColor = "#e74c3c";
            mediaDebug('[VIDEO] UI mise à jour, en attente de onstop...');
        }
        return;
    }

    mediaDebug('[VIDEO] Mode WebRTC NON disponible, utilisation ROS2');

    if (!isRecording) {
        mediaDebug('[VIDEO] ROS2: Démarrage enregistrement');
        
        startVideoClient.callService(new ROSLIB.ServiceRequest(), (res) => {
            mediaDebug('[VIDEO] ROS2 startVideo réponse:', res);
            if (res.success) {
                isRecording = true;
                btn.innerText = "⏹ STOP";
                btn.style.backgroundColor = "black";
                mediaDebug('[VIDEO] ROS2: Enregistrement démarré');
                logEvent('✅ Enregistrement vidéo démarré (ROS2)', 'success');
            } else {
                console.error('[VIDEO] ROS2: Erreur démarrage');
                logEvent('❌ Erreur démarrage vidéo ROS2', 'error');
            }
        });
    } else {
        mediaDebug('[VIDEO] ROS2: Arrêt enregistrement');
        
        // Mettre à jour l'UI IMMÉDIATEMENT (sans attendre la réponse du service)
        isRecording = false;
        btn.innerText = "🔴 REC";
        btn.style.backgroundColor = "#e74c3c";
        mediaDebug('[VIDEO] UI mise à jour immédiatement');
        
        stopVideoClient.callService(new ROSLIB.ServiceRequest(), (res) => {
            mediaDebug('[VIDEO] ROS2 stopVideo réponse:', res);
            if (res.success) {
                mediaDebug('[VIDEO] ROS2: Enregistrement arrêté');
                logEvent('✅ Enregistrement vidéo arrêté (ROS2)', 'success');
            } else {
                console.error('[VIDEO] ROS2: Erreur arrêt');
                logEvent('❌ Erreur arrêt vidéo ROS2', 'error');
                // Si echec, essayer de réinitialiser
                isRecording = false;
            }
        });
    }
}

// Galerie & Suppression
function updateGallery(files) {
    const grid = document.getElementById('galleryGrid');
    grid.innerHTML = "";
    logEvent(`Galerie mise à jour (${files.length} fichiers)`, 'info');
    files.forEach(file => {
        // Conteneur item
        let div = document.createElement('div');
        div.className = "gallery-item";

        // Vérifier si c'est une vidéo
        const isVideo = file.toLowerCase().endsWith('.mp4') || file.toLowerCase().endsWith('.avi') || file.toLowerCase().endsWith('.mov') || file.toLowerCase().endsWith('.webm');

        if (isVideo) {
            // Créer un élément vidéo avec miniature (même style que gallery.html)
            let video = document.createElement('video');
            video.src = 'gallery/' + file;
            video.controls = false;
            video.muted = true;
            video.preload = "metadata";
            video.playsInline = true;
            video.style.width = "100%";
            video.style.height = "100%";
            video.style.objectFit = "cover";
            video.onclick = () => {
                logEvent('Ouverture galerie (vidéo)', 'info');
                window.location.href = 'galerie/gallery.html';
            }; // Rediriger vers gallery

            // Fallback poster (triangle play)
            if (!window.__videoFallbackPoster) {
                const c = document.createElement('canvas');
                c.width = 320; c.height = 180;
                const cx = c.getContext('2d');
                cx.fillStyle = '#0f1419';
                cx.fillRect(0, 0, c.width, c.height);
                cx.fillStyle = '#4da3d8';
                cx.beginPath();
                cx.moveTo(c.width * 0.4, c.height * 0.3);
                cx.lineTo(c.width * 0.7, c.height * 0.5);
                cx.lineTo(c.width * 0.4, c.height * 0.7);
                cx.closePath();
                cx.fill();
                window.__videoFallbackPoster = c.toDataURL('image/png');
            }
            video.setAttribute('poster', window.__videoFallbackPoster);

            // Générer miniature depuis première frame
            const buildPoster = () => {
                try {
                    const canvas = document.createElement('canvas');
                    canvas.width = video.videoWidth || 320;
                    canvas.height = video.videoHeight || 180;
                    const ctx = canvas.getContext('2d');
                    ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
                    video.setAttribute('poster', canvas.toDataURL('image/jpeg'));
                    video.pause();
                } catch (err) {
                    video.setAttribute('poster', window.__videoFallbackPoster);
                }
            };

            video.addEventListener('loadedmetadata', () => {
                video.currentTime = Math.min(0.1, video.duration || 0.1);
            }, { once: true });

            video.addEventListener('seeked', () => {
                buildPoster();
            }, { once: true });

            video.addEventListener('error', () => {
                video.setAttribute('poster', window.__videoFallbackPoster);
            }, { once: true });

            // Icône play overlay (style gallery.html)
            let playIcon = document.createElement('div');
            playIcon.innerHTML = '▶';
            playIcon.style.position = 'absolute';
            playIcon.style.top = '50%';
            playIcon.style.left = '50%';
            playIcon.style.transform = 'translate(-50%, -50%)';
            playIcon.style.fontSize = '3rem';
            playIcon.style.color = 'white';
            playIcon.style.textShadow = '0 0 10px rgba(0,0,0,0.7)';
            playIcon.style.pointerEvents = 'none';
            playIcon.style.opacity = '0.8';

            div.appendChild(video);
            div.appendChild(playIcon);
        } else {
            // Image
            let img = document.createElement('img');
            img.src = 'gallery/' + file;
            img.onclick = () => {
                logEvent('Ouverture galerie (image)', 'info');
                window.location.href = 'galerie/gallery.html';
            }; // Rediriger vers gallery
            div.appendChild(img);
        }

        // Bouton Suppression (Design amélioré)
        let btnDelete = document.createElement('button');
        btnDelete.innerHTML = "&times;"; // <--- MODIFICATION ICI ("×" au lieu de "X")
        btnDelete.className = "btn-delete";
        btnDelete.title = "Supprimer " + (isVideo ? "la vidéo" : "l'image"); // Infobulle au survol
        
        btnDelete.onclick = (e) => {
            e.stopPropagation();
            if(confirm("Supprimer " + file + " ?")) {
                let msg = new ROSLIB.Message({ data: file });
                deletePub.publish(msg);
                logEvent(`Suppression demandée: ${file}`, 'warn');
            }
        };

        div.appendChild(btnDelete);
        grid.appendChild(div);
    });
}

// Ancienne logique de croix rouge désactivée
function handleMapClick(event) {
    mediaDebug("Clic sur carte (fonctionnalité croix désactivée)");
    logEvent('Clic sur la carte', 'info');
}

function toggleFullscreen() {
    const container = document.getElementById('videoContainer');
    const elem = container || document.getElementById('cameraFeed');
    if (!elem) return;
    if (!document.fullscreenElement) {
        elem.requestFullscreen().catch(err => {});
    } else {
        document.exitFullscreen();
    }
    logEvent('Plein écran vidéo basculé', 'info');
}



