/**
 * Initialize settings forms
 */
function initializeSettingsForms() {
    const connectionForm = document.getElementById('connection-form');
    const dockForm = document.getElementById('dock-form');
    const connectionType = document.getElementById('connection-type');
    const bridgeSettings = document.querySelector('.bridge-settings');
    
    // Show/hide bridge settings based on connection type
    connectionType.addEventListener('change', function() {
        if (this.value === 'BRIDGE') {
            bridgeSettings.style.display = 'block';
        } else {
            bridgeSettings.style.display = 'none';
        }
    });
    
    // Handle connection form submission
    connectionForm.addEventListener('submit', function(e) {
        e.preventDefault();
        
        // Get form values
        const appId = document.getElementById('app-id').value;
        const appKey = document.getElementById('app-key').value;
        const connectionType = document.getElementById('connection-type').value;
        const droneModel = document.getElementById('drone-model').value;
        const serialNumber = document.getElementById('serial-number').value;
        const bridgeIp = document.getElementById('bridge-ip').value;
        const bridgePort = document.getElementById('bridge-port').value;
        
        // Create connection parameters
        connectionParams = {
            app_id: appId,
            app_key: appKey,
            connection_type: connectionType,
            drone_model: droneModel,
            enable_virtual_stick: true,
            enable_camera: true,
            enable_gimbal: true,
            enable_waypoint: true,
            enable_hotpoint: true,
            enable_follow_me: true,
            enable_timeline: true,
            enable_hd_video: true
        };
        
        if (serialNumber) {
            connectionParams.serial_number = serialNumber;
        }
        
        if (connectionType === 'BRIDGE') {
            if (bridgeIp && bridgePort) {
                connectionParams.bridge_ip = bridgeIp;
                connectionParams.bridge_port = parseInt(bridgePort);
            } else {
                alert('Bridge IP and port are required for BRIDGE connection type');
                return;
            }
        }
        
        // Save settings to local storage
        localStorage.setItem('connectionParams', JSON.stringify(connectionParams));
        
        alert('Connection settings saved');
    });
    
    // Handle dock form submission
    dockForm.addEventListener('submit', function(e) {
        e.preventDefault();
        
        // Get form values
        const apiKey = document.getElementById('dock-api-key').value;
        const apiSecret = document.getElementById('dock-api-secret').value;
        const dockSn = document.getElementById('dock-sn').value;
        const region = document.getElementById('dock-region').value;
        
        // Create dock parameters
        dockParams = {
            api_key: apiKey,
            api_secret: apiSecret,
            dock_sn: dockSn,
            region: region,
            refresh_interval: 30
        };
        
        // Save settings to local storage
        localStorage.setItem('dockParams', JSON.stringify(dockParams));
        
        alert('Dock settings saved');
    });
    
    // Test dock connection button
    document.getElementById('test-dock-btn').addEventListener('click', async function() {
        // This would typically call an API endpoint to test the dock connection
        alert('Dock connection test not implemented in this demo');
    });
}

/**
 * Initialize mission planner
 */
function initializeMissionPlanner() {
    const missionForm = document.getElementById('mission-form');
    const addWaypointBtn = document.getElementById('add-waypoint-btn');
    const clearWaypointsBtn = document.getElementById('clear-waypoints-btn');
    const waypointsList = document.getElementById('waypoints-list');
    const missionType = document.getElementById('mission-type');
    
    // Handle mission type change
    missionType.addEventListener('change', function() {
        // Clear waypoints when changing mission type
        clearWaypoints();
    });
    
    // Add waypoint button
    addWaypointBtn.addEventListener('click', function() {
        // Get map center
        const center = missionMap.getCenter();
        addWaypoint(center.lat, center.lng);
    });
    
    // Clear waypoints button
    clearWaypointsBtn.addEventListener('click', function() {
        clearWaypoints();
    });
    
    // Handle mission form submission
    missionForm.addEventListener('submit', async function(e) {
        e.preventDefault();
        
        if (waypoints.length === 0) {
            alert('Please add at least one waypoint');
            return;
        }
        
        // Get form values
        const missionName = document.getElementById('mission-name').value;
        const missionType = document.getElementById('mission-type').value;
        const speed = parseFloat(document.getElementById('mission-speed').value);
        const finishAction = document.getElementById('finish-action').value;
        const headingMode = document.getElementById('heading-mode').value;
        
        // Create mission data
        const missionData = {
            mission_name: missionName,
            mission_type: missionType,
            speed: speed,
            finish_action: finishAction,
            heading_mode: headingMode,
            waypoints: waypoints
        };
        
        try {
            // Save mission
            const response = await fetch('/api/missions', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(missionData)
            });
            
            const data = await response.json();
            
            if (data.success) {
                alert('Mission saved successfully');
                // Clear form
                missionForm.reset();
                clearWaypoints();
                // Refresh mission list
                loadMissions();
            } else {
                alert('Failed to save mission: ' + (data.error || 'Unknown error'));
            }
        } catch (error) {
            console.error('Error saving mission:', error);
            alert('Error saving mission: ' + error.message);
        }
    });
    
    // Initialize waypoint modal
    initializeWaypointModal();
}

/**
 * Initialize waypoint modal
 */
function initializeWaypointModal() {
    const waypointForm = document.getElementById('waypoint-form');
    const saveWaypointBtn = document.getElementById('save-waypoint-btn');
    const deleteWaypointBtn = document.getElementById('delete-waypoint-btn');
    const actionRotateGimbal = document.getElementById('action-rotate-gimbal');
    const gimbalSettings = document.querySelector('.gimbal-settings');
    
    // Show/hide gimbal settings based on action
    actionRotateGimbal.addEventListener('change', function() {
        gimbalSettings.style.display = this.checked ? 'block' : 'none';
    });
    
    // Save waypoint button
    saveWaypointBtn.addEventListener('click', function() {
        const index = parseInt(document.getElementById('waypoint-index').value);
        const latitude = parseFloat(document.getElementById('waypoint-latitude').value);
        const longitude = parseFloat(document.getElementById('waypoint-longitude').value);
        const altitude = parseFloat(document.getElementById('waypoint-altitude').value);
        const heading = parseInt(document.getElementById('waypoint-heading').value);
        const stayTime = parseInt(document.getElementById('waypoint-stay-time').value);
        
        // Get actions
        const actions = [];
        
        if (document.getElementById('action-take-photo').checked) {
            actions.push({ action: 'take_photo' });
        }
        
        if (document.getElementById('action-start-recording').checked) {
            actions.push({ action: 'start_recording' });
        }
        
        if (document.getElementById('action-stop-recording').checked) {
            actions.push({ action: 'stop_recording' });
        }
        
        if (document.getElementById('action-rotate-gimbal').checked) {
            actions.push({
                action: 'rotate_gimbal',
                pitch: parseFloat(document.getElementById('gimbal-pitch').value),
                roll: parseFloat(document.getElementById('gimbal-roll').value),
                yaw: parseFloat(document.getElementById('gimbal-yaw').value)
            });
        }
        
        // Update waypoint
        waypoints[index] = {
            latitude: latitude,
            longitude: longitude,
            altitude: altitude,
            heading: heading,
            stay_time: stayTime,
            actions: actions
        };
        
        // Update marker
        waypointMarkers[index].setLatLng([latitude, longitude]);
        
        // Update waypoint list
        updateWaypointsList();
        
        // Update mission path
        updateMissionPath();
        
        // Hide modal
        const waypointModal = bootstrap.Modal.getInstance(document.getElementById('waypointModal'));
        waypointModal.hide();
    });
    
    // Delete waypoint button
    deleteWaypointBtn.addEventListener('click', function() {
        const index = parseInt(document.getElementById('waypoint-index').value);
        
        // Remove waypoint
        waypoints.splice(index, 1);
        
        // Remove marker
        missionMap.removeLayer(waypointMarkers[index]);
        waypointMarkers.splice(index, 1);
        
        // Update waypoint numbers
        waypointMarkers.forEach((marker, i) => {
            marker.setTooltipContent(`Waypoint ${i + 1}`);
        });
        
        // Update waypoint list
        updateWaypointsList();
        
        // Update mission path
        updateMissionPath();
        
        // Hide modal
        const waypointModal = bootstrap.Modal.getInstance(document.getElementById('waypointModal'));
        waypointModal.hide();
    });
}
