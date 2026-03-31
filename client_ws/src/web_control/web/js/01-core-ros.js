// =======================================================================
// 1. CONFIGURATION & CONNEXION
// =======================================================================
const serverIp = window.location.hostname;  // IP du serveur web
const robotIp = "localhost";          // IP du robot/Raspberry
const videoHost = serverIp === "" ? "localhost" : serverIp;
let externalStreamUrl = "";

// Vidéo WebRTC (faible latence)
const videoElement = document.getElementById('cameraFeed');
const webrtcStatus = document.getElementById('webrtcStatus');

let config = null;

async function loadConfig() {
    try {
        const res = await fetch(`../../../../configuration.json?t=${Date.now()}`);        // le ?t évite le cache 🔥

        config = await res.json();

        console.log("Config chargée:", config);

        applyConfig();

    } catch (err) {
        console.error("Erreur chargement config:", err);

        // fallback (si le fichier plante)
        config = {
            video: {
                host: "localhost",
                port: 8889,
                stream: "mystream"
            }
        };

        applyConfig();
    }
}

function applyConfig() {
    const serverIp = window.location.hostname;

    const videoHost = config.video.host === "auto"
        ? serverIp
        : config.video.host;

    externalStreamUrl = `http://${videoHost}:${config.video.port}/${config.video.stream}/`;

    console.log("Stream URL:", externalStreamUrl);
    console.log('[INIT] Flux externe appliqué depuis config');
    // Initialiser le flux si disponible
    if (externalStreamUrl) {
        initExternalStream();
    } else {
        initWebRTC();
    }
}

// Charger config au démarrage (async, sans bloquer)
if (videoElement) {
    loadConfig();
}
function initExternalStream() {
    const container = videoElement.parentElement;
    if (!container) {
        videoElement.src = externalStreamUrl;
        videoElement.play().catch(() => {});
        if (webrtcStatus) {
            webrtcStatus.textContent = 'Flux externe';
        }
        return;
    }

    const iframe = document.createElement('iframe');
    iframe.src = externalStreamUrl;
    iframe.style.width = '100%';
    iframe.style.height = '100%';
    iframe.style.border = '0';
    iframe.allow = 'autoplay; fullscreen';
    container.replaceChild(iframe, videoElement);
    if (webrtcStatus) {
        if (location.protocol === 'https:' && externalStreamUrl.startsWith('http://')) {
            webrtcStatus.textContent = 'Flux externe (HTTP bloqué en HTTPS)';
        } else {
            webrtcStatus.textContent = 'Flux externe';
        }
    }
}

function initWebRTC() {
    // Si flux externe déjà actif, ne pas initialiser WebRTC
    if (externalStreamUrl && webrtcStatus?.textContent?.includes('Flux externe')) {
        console.log('[WebRTC] Flux externe déjà actif, WebRTC désactivé');
        if (webrtcStatus) webrtcStatus.textContent = 'Flux externe (WebRTC désactivé)';
        return;
    }

    const host = window.location.hostname || "localhost";
    const wsUrl = `ws://${host}:8091/ws`;

    let pc = null;
    let ws = null;
    let currentStream = null;
    let antiBufferInterval = null;

    let audioEnabled = false;
    const audioSlider = document.getElementById('audioSlider');

    const setStatus = (text) => {
        if (webrtcStatus) {
            webrtcStatus.textContent = text;
        }
    };

    // Cleanup complet avant toute reconnexion pour eviter l'accumulation d'etats.
    function cleanup() {
        if (antiBufferInterval) {
            clearInterval(antiBufferInterval);
            antiBufferInterval = null;
        }

        if (pc) {
            pc.getSenders().forEach((sender) => {
                if (sender.track) {
                    sender.track.stop();
                }
            });
            pc.close();
            pc = null;
        }

        if (ws) {
            ws.close();
            ws = null;
        }

        if (currentStream) {
            currentStream.getTracks().forEach((track) => track.stop());
            currentStream = null;
        }

        videoElement.srcObject = null;
    }

    function start() {
        cleanup();

        pc = new RTCPeerConnection({
            iceServers: [],
            bundlePolicy: 'max-bundle',
            rtcpMuxPolicy: 'require'
        });

        ws = new WebSocket(wsUrl);

        videoElement.autoplay = true;
        videoElement.playsInline = true;
        videoElement.muted = !audioEnabled;
        videoElement.setAttribute('webkit-playsinline', 'true');

        pc.ontrack = (event) => {
            console.log('Track reçue:', event.track.kind);
            const stream = event.streams[0];

            if (currentStream === stream) {
                return;
            }

            currentStream = stream;
            videoElement.srcObject = stream;
            videoElement.play().catch(() => {});

            if (!antiBufferInterval) {
                antiBufferInterval = setInterval(() => {
                    if (videoElement.buffered.length > 0) {
                        const liveEdge = videoElement.buffered.end(0);
                        const delay = liveEdge - videoElement.currentTime;

                        if (delay > 0.5) {
                            videoElement.currentTime = liveEdge - 0.1;
                        }
                    }
                }, 1000);
            }

            setStatus('WebRTC: flux reçu');
        };

        pc.onconnectionstatechange = () => {
            setStatus(`WebRTC: ${pc.connectionState}`);

            if (pc.connectionState === 'failed' || pc.connectionState === 'disconnected') {
                console.warn('Reconnexion WebRTC...');
                setTimeout(start, 1000);
            }
        };

        pc.oniceconnectionstatechange = () => {
            setStatus(`ICE: ${pc.iceConnectionState}`);
        };

        ws.onopen = async () => {
            setStatus('WebRTC: signalisation...');

            pc.addTransceiver('video', { direction: 'recvonly' });
            pc.addTransceiver('audio', { direction: 'recvonly' });

            const offer = await pc.createOffer();
            await pc.setLocalDescription(offer);

            ws.send(JSON.stringify({
                type: 'offer',
                sdp: offer.sdp
            }));
        };

        ws.onmessage = async (event) => {
            const data = JSON.parse(event.data);

            if (data.type === 'answer') {
                await pc.setRemoteDescription({
                    type: 'answer',
                    sdp: data.sdp
                });
                setStatus('WebRTC: connecté');
            } else if (data.type === 'error') {
                setStatus(`WebRTC: ${data.message}`);
            }
        };

        ws.onerror = () => setStatus('WebRTC: erreur WebSocket');

        ws.onclose = () => {
            setStatus('WebRTC: WebSocket fermé');
            setTimeout(start, 2000);
        };
    }

    window.toggleAudio = () => {
        audioEnabled = !audioEnabled;
        videoElement.muted = !audioEnabled;
        if (audioEnabled) {
            videoElement.play().catch(() => {});
        }
        const btn = document.getElementById('btnAudio');
        if (btn) {
            btn.textContent = audioEnabled ? '🔊' : '🔇';
        }
        setStatus(audioEnabled ? 'WebRTC: son activé' : 'WebRTC: son coupé');
        if (audioSlider) {
            audioSlider.value = audioEnabled ? Math.round(videoElement.volume * 100) : 0;
        }
    };

    if (audioSlider) {
        audioSlider.addEventListener('input', () => {
            const volume = Math.max(0, Math.min(1, Number(audioSlider.value) / 100));
            videoElement.volume = volume;
            const btn = document.getElementById('btnAudio');
            if (volume > 0 && videoElement.muted) {
                audioEnabled = true;
                videoElement.muted = false;
                if (btn) {
                    btn.textContent = '🔊';
                }
            } else if (volume === 0) {
                audioEnabled = false;
                videoElement.muted = true;
                if (btn) {
                    btn.textContent = '🔇';
                }
            }
        });
    }

    start();
}

// WebSocket 1: Se connecte au serveur local (Zehno) pour galerie/trajets/logs
const ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:9090` });

// WebSocket 2: Se connecte directement à la Raspberry pour /cmd_vel (commandes robot)
const rosRobot = new ROSLIB.Ros({ url: `ws://${robotIp}:9090` });

let rosServerConnected = false;
let rosRobotConnected = false;

ros.on('connection', () => {
    rosServerConnected = true;
    document.getElementById('status').innerText = 'Connecté';
    document.getElementById('status').className = 'connected';
});
ros.on('error', (error) => {
    rosServerConnected = false;
    document.getElementById('status').innerText = 'Erreur';
    document.getElementById('status').className = 'disconnected';
});
ros.on('close', () => {
    rosServerConnected = false;
    document.getElementById('status').innerText = 'Déconnecté';
    document.getElementById('status').className = 'disconnected';
});

rosRobot.on('connection', () => {
    rosRobotConnected = true;
    console.info('ROS Robot: connecté');
});

rosRobot.on('error', (error) => {
    rosRobotConnected = false;
    console.warn('ROS Robot: erreur connexion', error);
});

rosRobot.on('close', () => {
    rosRobotConnected = false;
    console.warn('ROS Robot: déconnecté');
});

function createRobotPublisher(name, messageType, options = {}) {
    const { robotOnly = false } = options;
    const robotTopic = new ROSLIB.Topic({ ros: rosRobot, name, messageType });
    const serverTopic = robotOnly ? null : new ROSLIB.Topic({ ros, name, messageType });

    return {
        publish(message) {
            let robotSent = false;
            let serverSent = false;

            if (rosRobotConnected || robotOnly) {
                try {
                    robotTopic.publish(message);
                    robotSent = true;
                } catch (err) {
                    console.error(`Erreur publish robot ${name}:`, err);
                }
            }

            if (!robotOnly && rosServerConnected && serverTopic) {
                try {
                    serverTopic.publish(message);
                    serverSent = true;
                } catch (err) {
                    console.error(`Erreur publish serveur ${name}:`, err);
                }
            }

            if (!robotSent && !serverSent) {
                console.warn(`Topic ${name} non envoyé (aucune connexion disponible)`);
                // fallback: publish to a local rosbridge at localhost:9090
                try {
                    if (!window._localRos) {
                        window._localRos = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
                        window._localRosConnected = false;
                        window._localRos.on('connection', () => { window._localRosConnected = true; console.info('Local ROS bridge connecté'); });
                        window._localRos.on('error', () => { window._localRosConnected = false; });
                        window._localRos.on('close', () => { window._localRosConnected = false; });
                    }

                    // Best-effort: publish to local rosbridge directly (without using createRobotPublisher/logEvent)
                    const localTopic = new ROSLIB.Topic({ ros: window._localRos, name, messageType });
                    try {
                        localTopic.publish(message);
                    } catch (e) {
                        console.warn('Fallback publish failed', e);
                    }

                    // Also publish a local system log directly (avoid calling logEvent to prevent recursion)
                    try {
                        const logMsg = new ROSLIB.Message({ data: JSON.stringify({ message: `Fallback publish '${name}' sur localhost (offline)`, level: 'warn' }) });
                        const sysTopic = new ROSLIB.Topic({ ros: window._localRos, name: '/ui/system_logs', messageType: 'std_msgs/String' });
                        try { sysTopic.publish(logMsg); } catch (e) { /* ignore */ }
                    } catch (e) {
                        /* ignore logging fallback errors */
                    }

                } catch (e) {
                    console.warn('Erreur lors du fallback local:', e);
                }
            }
        }
    };
}

// =======================================================================
// 2. TOPICS
// =======================================================================

// Robot (TwistStamped - type imposé par un autre node)
const cmdVelPub = createRobotPublisher('/cmd_vel', 'geometry_msgs/TwistStamped');

// PTZ (Point: x, y)
const ptzPub = createRobotPublisher('/camera/ptz', 'geometry_msgs/Point');

// Mission Status
const missionPub = createRobotPublisher('/mission/status', 'std_msgs/Bool');

// Delete Image (Nouveau)
const deletePub = createRobotPublisher('/camera/delete_image', 'std_msgs/String');

const zoomPub = createRobotPublisher('/camera/zoom', 'std_msgs/Float32');
const focusPub = createRobotPublisher('/camera/focus', 'std_msgs/Float32');
const autofocusPub = createRobotPublisher('/camera/autofocus', 'std_msgs/Bool');
const lightPub = createRobotPublisher('/camera/light', 'std_msgs/Bool');
const alertPub = createRobotPublisher('/camera/alert', 'std_msgs/Bool');
const robotVolumePub = createRobotPublisher('/robot/volume', 'std_msgs/Float32');
const armSpeedPub = createRobotPublisher('/robot/arm_speed', 'std_msgs/Float32');
const armPosPub = createRobotPublisher('/robot/arm_position', 'std_msgs/Float32');
const clickPub = createRobotPublisher('/ui/click', 'geometry_msgs/Point');

// Logs UI
const logPub = createRobotPublisher('/ui/system_logs', 'std_msgs/String');

function logEvent(message, level = 'info') {
    try {
        logPub.publish(new ROSLIB.Message({
            data: JSON.stringify({ message, level })
        }));
    } catch (e) {
        console.log(message);
    }
}

// Topic pour l'arrêt d'urgence
const emergencyPub = createRobotPublisher('/robot/emergency_stop', 'std_msgs/Bool');

// Topic pour recevoir la liste des fichiers de trajectoire
const trajListSub = new ROSLIB.Topic({ 
    ros: ros, 
    name: '/ui/trajectory_files', 
    messageType: 'std_msgs/String' 
});

trajListSub.subscribe((msg) => {
    try {
        updateTrajectoryList(JSON.parse(msg.data));
    } catch (e) {
        console.error("Erreur parsing liste trajets", e);
    }
});

const gallerySub = new ROSLIB.Topic({ ros: ros, name: '/ui/gallery_files', messageType: 'std_msgs/String' });
let lastGalleryData = "";
gallerySub.subscribe((msg) => { 
    try { 
        console.log("Galerie reçue:", msg.data);  // DEBUG
        if (msg.data !== lastGalleryData) {
            lastGalleryData = msg.data;
            updateGallery(JSON.parse(msg.data)); 
        }
    } catch (e) { 
        console.error("Erreur galerie:", e);  // DEBUG
    } 
});

// Topic pour recevoir le niveau de batterie
const batterySub = new ROSLIB.Topic({ 
    ros: ros, 
    name: '/robot/battery', 
    messageType: 'std_msgs/Float32' 
});

batterySub.subscribe((msg) => {
    const batteryLevel = Math.round(msg.data);
    const batteryElement = document.getElementById('battery');
    batteryElement.innerText = `🔋 ${batteryLevel}%`;
    
    // Changer la couleur selon le niveau
    batteryElement.classList.remove('battery-low', 'battery-critical');
    if (batteryLevel <= 20) {
        batteryElement.classList.add('battery-critical');
    } else if (batteryLevel <= 50) {
        batteryElement.classList.add('battery-low');
    }
});

// Services
const photoClient = new ROSLIB.Service({ ros: ros, name: '/camera/take_photo', serviceType: 'std_srvs/Trigger' });
const startVideoClient = new ROSLIB.Service({ ros: ros, name: '/camera/start_video', serviceType: 'std_srvs/Trigger' });
const stopVideoClient = new ROSLIB.Service({ ros: ros, name: '/camera/stop_video', serviceType: 'std_srvs/Trigger' });

