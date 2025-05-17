/**
 * DJI Mission Planner Web UI
 * 
 * This JavaScript file provides the client-side functionality for the DJI Mission Planner web UI.
 * It handles:
 * - Connection to the drone
 * - Telemetry updates
 * - Mission planning and execution
 * - Map visualization
 * - UI interactions
 */

// Global variables
let map = null;
let missionMap = null;
let droneMarker = null;
let homeMarker = null;
let waypointMarkers = [];
let missionPath = null;
let waypoints = [];
let isConnected = false;
let isMissionRunning = false;
let telemetryUpdateInterval = null;
let statusUpdateInterval = null;
let connectionParams = {};
let dockParams = {};
let currentMission = null;
let isRecording = false;

// Initialize the application when the DOM is loaded
document.addEventListener('DOMContentLoaded', function() {
    // Initialize maps
    initializeMaps();
    
    // Initialize navigation
    initializeNavigation();
    
    // Initialize connection controls
    initializeConnectionControls();
    
    // Initialize mission controls
    initializeMissionControls();
    
    // Initialize quick controls
    initializeQuickControls();
    
    // Initialize settings forms
    initializeSettingsForms();
    
    // Initialize mission planner
    initializeMissionPlanner();
    
    // Initialize mission list
    initializeMissionList();
    
    // Load saved settings
    loadSettings();
    
    // Start status update interval
    statusUpdateInterval = setInterval(updateStatus, 2000);
});

/**
 * Initialize the maps for dashboard and mission planner
 */
function initializeMaps() {
    // Initialize dashboard map
    map = L.map('map').setView([37.7749, -122.4194], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(map);
    
    // Initialize mission planner map
    missionMap = L.map('mission-map').setView([37.7749, -122.4194], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(missionMap);
    
    // Add click handler to mission map for adding waypoints
    missionMap.on('click', function(e) {
        addWaypoint(e.latlng.lat, e.latlng.lng);
    });
}

/**
 * Initialize navigation between pages
 */
function initializeNavigation() {
    const navLinks = document.querySelectorAll('.nav-link');
    const pages = document.querySelectorAll('.page');
    
    navLinks.forEach(link => {
        link.addEventListener('click', function(e) {
            e.preventDefault();
            
            // Get the page to show
            const pageId = this.getAttribute('data-page') + '-page';
            
            // Hide all pages
            pages.forEach(page => {
                page.style.display = 'none';
            });
            
            // Show the selected page
            document.getElementById(pageId).style.display = 'block';
            
            // Update active link
            navLinks.forEach(navLink => {
                navLink.classList.remove('active');
            });
            this.classList.add('active');
            
            // Refresh maps when shown
            if (pageId === 'dashboard-page') {
                setTimeout(() => map.invalidateSize(), 100);
            } else if (pageId === 'mission-planner-page') {
                setTimeout(() => missionMap.invalidateSize(), 100);
            }
        });
    });
}

/**
 * Initialize connection controls
 */
function initializeConnectionControls() {
    const connectBtn = document.getElementById('connectBtn');
    const disconnectBtn = document.getElementById('disconnectBtn');
    const connectionStatus = document.querySelector('.connection-status');
    
    connectBtn.addEventListener('click', async function() {
        // Show connection modal
        const connectionModal = new bootstrap.Modal(document.getElementById('connectionModal'));
        connectionModal.show();
        
        // Update connection status
        connectionStatus.textContent = 'Connecting...';
        connectionStatus.classList.add('connecting');
        
        try {
            // Send connection request
            const response = await fetch('/api/connect', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(connectionParams)
            });
            
            const data = await response.json();
            
            if (data.success) {
                // Update UI for connected state
                isConnected = true;
                connectionStatus.textContent = 'Connected';
                connectionStatus.classList.remove('connecting', 'disconnected');
                connectionStatus.classList.add('connected');
                connectBtn.style.display = 'none';
                disconnectBtn.style.display = 'inline-block';
                
                // Enable control buttons
                enableControlButtons();
                
                // Start telemetry updates
                startTelemetryUpdates();
            } else {
                // Show error
                connectionStatus.textContent = 'Connection Failed';
                connectionStatus.classList.remove('connecting', 'connected');
                connectionStatus.classList.add('disconnected');
                alert('Failed to connect: ' + (data.error || 'Unknown error'));
            }
        } catch (error) {
            console.error('Connection error:', error);
            connectionStatus.textContent = 'Connection Error';
            connectionStatus.classList.remove('connecting', 'connected');
            connectionStatus.classList.add('disconnected');
            alert('Connection error: ' + error.message);
        } finally {
            // Hide connection modal
            connectionModal.hide();
        }
    });
    
    disconnectBtn.addEventListener('click', async function() {
        try {
            // Send disconnect request
            const response = await fetch('/api/disconnect', {
                method: 'POST'
            });
            
            const data = await response.json();
            
            // Update UI for disconnected state
            isConnected = false;
            connectionStatus.textContent = 'Disconnected';
            connectionStatus.classList.remove('connecting', 'connected');
            connectionStatus.classList.add('disconnected');
            connectBtn.style.display = 'inline-block';
            disconnectBtn.style.display = 'none';
            
            // Disable control buttons
            disableControlButtons();
            
            // Stop telemetry updates
            stopTelemetryUpdates();
            
            if (!data.success) {
                alert('Disconnect warning: ' + (data.error || 'Unknown error'));
            }
        } catch (error) {
            console.error('Disconnect error:', error);
            alert('Disconnect error: ' + error.message);
        }
    });
}

/**
 * Enable control buttons when connected
 */
function enableControlButtons() {
    document.getElementById('takeoffBtn').disabled = false;
    document.getElementById('landBtn').disabled = false;
    document.getElementById('rthBtn').disabled = false;
    document.getElementById('photoBtn').disabled = false;
    document.getElementById('videoBtn').disabled = false;
    document.getElementById('startMissionBtn').disabled = false;
}

/**
 * Disable control buttons when disconnected
 */
function disableControlButtons() {
    document.getElementById('takeoffBtn').disabled = true;
    document.getElementById('landBtn').disabled = true;
    document.getElementById('rthBtn').disabled = true;
    document.getElementById('photoBtn').disabled = true;
    document.getElementById('videoBtn').disabled = true;
    document.getElementById('startMissionBtn').disabled = true;
    document.getElementById('pauseMissionBtn').disabled = true;
    document.getElementById('resumeMissionBtn').disabled = true;
    document.getElementById('stopMissionBtn').disabled = true;
}

/**
 * Start telemetry updates
 */
function startTelemetryUpdates() {
    telemetryUpdateInterval = setInterval(updateTelemetry, 1000);
}

/**
 * Stop telemetry updates
 */
function stopTelemetryUpdates() {
    if (telemetryUpdateInterval) {
        clearInterval(telemetryUpdateInterval);
        telemetryUpdateInterval = null;
    }
}

/**
 * Update telemetry data
 */
async function updateTelemetry() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();
        
        if (data.connected) {
            // Update telemetry display
            updateTelemetryDisplay(data.telemetry);
            
            // Update mission status
            updateMissionStatus(data.mission);
            
            // Update map
            updateMap(data.telemetry);
        }
    } catch (error) {
        console.error('Error updating telemetry:', error);
    }
}

/**
 * Update telemetry display
 */
function updateTelemetryDisplay(telemetry) {
    if (!telemetry) return;
    
    // Update battery
    const batteryLevel = telemetry.battery?.percent || 0;
    document.getElementById('battery-level').textContent = batteryLevel + '%';
    const batteryProgress = document.getElementById('battery-progress');
    batteryProgress.style.width = batteryLevel + '%';
    
    // Set battery color based on level
    batteryProgress.classList.remove('bg-success', 'bg-warning', 'bg-danger');
    if (batteryLevel <= 15) {
        batteryProgress.classList.add('bg-danger');
    } else if (batteryLevel <= 30) {
        batteryProgress.classList.add('bg-warning');
    } else {
        batteryProgress.classList.add('bg-success');
    }
    
    // Update position
    if (telemetry.position) {
        document.getElementById('latitude').textContent = telemetry.position.latitude.toFixed(6);
        document.getElementById('longitude').textContent = telemetry.position.longitude.toFixed(6);
        document.getElementById('altitude').textContent = telemetry.position.altitude.toFixed(1);
    }
    
    // Update attitude
    if (telemetry.attitude) {
        document.getElementById('roll').textContent = telemetry.attitude.roll.toFixed(1);
        document.getElementById('pitch').textContent = telemetry.attitude.pitch.toFixed(1);
        document.getElementById('yaw').textContent = telemetry.attitude.yaw.toFixed(1);
    }
    
    // Update velocity
    if (telemetry.velocity) {
        document.getElementById('horizontal-speed').textContent = telemetry.velocity.horizontal.toFixed(1);
        document.getElementById('vertical-speed').textContent = telemetry.velocity.vertical.toFixed(1);
    }
    
    // Update flight status
    if (telemetry.flight_status) {
        document.getElementById('flight-mode').textContent = telemetry.flight_status.mode;
        document.getElementById('home-distance').textContent = telemetry.flight_status.home_distance.toFixed(1);
    }
    
    // Update GPS
    if (telemetry.gps) {
        document.getElementById('gps-satellites').textContent = telemetry.gps.satellites;
    }
}

/**
 * Update mission status
 */
function updateMissionStatus(mission) {
    if (!mission) return;
    
    document.getElementById('current-mission').textContent = mission.current_mission || 'None';
    document.getElementById('mission-status').textContent = mission.state || 'Idle';
    document.getElementById('mission-progress').textContent = mission.progress + '%';
    document.getElementById('mission-progress-bar').style.width = mission.progress + '%';
    
    if (mission.total_waypoints > 0) {
        document.getElementById('current-waypoint').textContent = 
            (mission.waypoint_index + 1) + '/' + mission.total_waypoints;
    } else {
        document.getElementById('current-waypoint').textContent = '0/0';
    }
    
    // Update mission control buttons
    if (mission.state === 'executing') {
        isMissionRunning = true;
        document.getElementById('startMissionBtn').disabled = true;
        document.getElementById('pauseMissionBtn').disabled = false;
        document.getElementById('resumeMissionBtn').disabled = true;
        document.getElementById('stopMissionBtn').disabled = false;
    } else if (mission.state === 'paused') {
        isMissionRunning = true;
        document.getElementById('startMissionBtn').disabled = true;
        document.getElementById('pauseMissionBtn').disabled = true;
        document.getElementById('resumeMissionBtn').disabled = false;
        document.getElementById('stopMissionBtn').disabled = false;
    } else {
        isMissionRunning = false;
        document.getElementById('startMissionBtn').disabled = !isConnected;
        document.getElementById('pauseMissionBtn').disabled = true;
        document.getElementById('resumeMissionBtn').disabled = true;
        document.getElementById('stopMissionBtn').disabled = true;
    }
}

/**
 * Update map with drone position
 */
function updateMap(telemetry) {
    if (!telemetry || !telemetry.position) return;
    
    const lat = telemetry.position.latitude;
    const lng = telemetry.position.longitude;
    
    // Update drone marker
    if (!droneMarker) {
        // Create drone marker
        const droneIcon = L.divIcon({
            className: 'map-marker-drone',
            iconSize: [16, 16]
        });
        
        droneMarker = L.marker([lat, lng], { icon: droneIcon }).addTo(map);
    } else {
        // Update drone marker position
        droneMarker.setLatLng([lat, lng]);
    }
    
    // Center map on drone
    map.setView([lat, lng]);
    
    // Update home marker if available
    if (telemetry.home_location && !homeMarker) {
        const homeLat = telemetry.home_location.latitude;
        const homeLng = telemetry.home_location.longitude;
        
        const homeIcon = L.divIcon({
            className: 'map-marker-home',
            iconSize: [14, 14]
        });
        
        homeMarker = L.marker([homeLat, homeLng], { icon: homeIcon }).addTo(map);
    }
}

/**
 * Initialize mission controls
 */
function initializeMissionControls() {
    const startMissionBtn = document.getElementById('startMissionBtn');
    const pauseMissionBtn = document.getElementById('pauseMissionBtn');
    const resumeMissionBtn = document.getElementById('resumeMissionBtn');
    const stopMissionBtn = document.getElementById('stopMissionBtn');
    
    // Start mission button
    startMissionBtn.addEventListener('click', async function() {
        if (!isConnected) {
            alert('Please connect to a drone first');
            return;
        }
        
        // Show mission selection dialog
        // For now, we'll just use a prompt
        const missionName = prompt('Enter mission filename to execute:');
        if (!missionName) return;
        
        try {
            const response = await fetch(`/api/execute_mission/${missionName}`, {
                method: 'POST'
            });
            
            const data = await response.json();
            
            if (data.success) {
                alert('Mission execution started');
                currentMission = missionName;
            } else {
                alert('Failed to start mission: ' + (data.error || 'Unknown error'));
            }
        } catch (error) {
            console.error('Error starting mission:', error);
            alert('Error starting mission: ' + error.message);
        }
    });
    
    // Pause mission button
    pauseMissionBtn.addEventListener('click', async function() {
        if (!isConnected || !isMissionRunning) return;
        
        try {
            const response = await fetch('/api/command', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    command: 'pause_waypoint_mission',
                    parameters: {}
                })
            });
            
            const data = await response.json();
            
            if (data.success) {
                alert('Mission paused');
            } else {
                alert('Failed to pause mission');
            }
        } catch (error) {
            console.error('Error pausing mission:', error);
            alert('Error pausing mission: ' + error.message);
        }
    });
    
    // Resume mission button
    resumeMissionBtn.addEventListener('click', async function() {
        if (!isConnected || !isMissionRunning) return;
        
        try {
            const response = await fetch('/api/command', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    command: 'resume_waypoint_mission',
                    parameters: {}
                })
            });
            
            const data = await response.json();
            
            if (data.success) {
                alert('Mission resumed');
            } else {
                alert('Failed to resume mission');
            }
        } catch (error) {
            console.error('Error resuming mission:', error);
            alert('Error resuming mission: ' + error.message);
        }
    });
    
    // Stop mission button
    stopMissionBtn.addEventListener('click', async function() {
        if (!isConnected || !isMissionRunning) return;
        
        try {
            const response = await fetch('/api/stop_mission', {
                method: 'POST'
            });
            
            const data = await response.json();
            
            if (data.success) {
                alert('Mission stopped');
                currentMission = null;
            } else {
                alert('Failed to stop mission: ' + (data.error || 'Unknown error'));
            }
        } catch (error) {
            console.error('Error stopping mission:', error);
            alert('Error stopping mission: ' + error.message);
        }
    });
}

/**
 * Initialize quick control buttons
 */
function initializeQuickControls() {
    const takeoffBtn = document.getElementById('takeoffBtn');
    const landBtn = document.getElementById('landBtn');
    const rthBtn = document.getElementById('rthBtn');
    const photoBtn = document.getElementById('photoBtn');
    const videoBtn = document.getElementById('videoBtn');
    
    // Takeoff button
    takeoffBtn.addEventListener('click', async function() {
        if (!isConnected) return;
        
        try {
            const response = await fetch('/api/command', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    command: 'takeoff',
                    parameters: {}
                })
            });
            
            const data = await response.json();
            
            if (data.success) {
                alert('Takeoff command sent');
            } else {
                alert('Failed to send takeoff command');
            }
        } catch (error) {
            console.error('Error sending takeoff command:', error);
            alert('Error sending takeoff command: ' + error.message);
        }
    });
    
    // Land button
    landBtn.addEventListener('click', async function() {
        if (!isConnected) return;
        
        try {
            const response = await fetch('/api/command', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    command: 'land',
                    parameters: {}
                })
            });
            
            const data = await response.json();
            
            if (data.success) {
                alert('Land command sent');
            } else {
                alert('Failed to send land command');
            }
        } catch (error) {
            console.error('Error sending land command:', error);
            alert('Error sending land command: ' + error.message);
        }
    });
    
    // Return to home button
    rthBtn.addEventListener('click', async function() {
        if (!isConnected) return;
        
        try {
            const response = await fetch('/api/command', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    command: 'return_to_home',
                    parameters: {}
                })
            });
            
            const data = await response.json();
            
            if (data.success) {
                alert('Return to home command sent');
            } else {
                alert('Failed to send return to home command');
            }
        } catch (error) {
            console.error('Error sending return to home command:', error);
            alert('Error sending return to home command: ' + error.message);
        }
    });
    
    // Take photo button
    photoBtn.addEventListener('click', async function() {
        if (!isConnected) return;
        
        try {
            const response = await fetch('/api/command', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    command: 'take_photo',
                    parameters: {}
                })
            });
            
            const data = await response.json();
            
            if (data.success) {
                alert('Photo taken');
            } else {
                alert('Failed to take photo');
            }
        } catch (error) {
            console.error('Error taking photo:', error);
            alert('Error taking photo: ' + error.message);
        }
    });
    
    // Video button
    videoBtn.addEventListener('click', async function() {
        if (!isConnected) return;
        
        const command = isRecording ? 'stop_recording' : 'start_recording';
        
        try {
            const response = await fetch('/api/command', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({
                    command: command,
                    parameters: {}
                })
            });
            
            const data = await response.json();
            
            if (data.success) {
                isRecording = !isRecording;
                videoBtn.textContent = isRecording ? 'Stop Recording' : 'Start Recording';
                alert(isRecording ? 'Recording started' : 'Recording stopped');
            } else {
                alert('Failed to ' + (isRecording ? 'stop' : 'start') + ' recording');
            }
        } catch (error) {
            console.error('Error controlling recording:', error);
            alert('Error controlling recording: ' + error.message);
        }
    });
}
