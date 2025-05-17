/**
 * View a mission
 */
async function viewMission(filename) {
    try {
        const response = await fetch(`/api/missions/${filename}`);
        const data = await response.json();
        
        if (data.success) {
            // Switch to mission planner page
            document.querySelectorAll('.nav-link').forEach(link => {
                if (link.getAttribute('data-page') === 'mission-planner') {
                    link.click();
                }
            });
            
            // Clear existing waypoints
            clearWaypoints();
            
            // Set form values
            document.getElementById('mission-name').value = data.mission.mission_name;
            document.getElementById('mission-type').value = data.mission.mission_type;
            document.getElementById('mission-speed').value = data.mission.speed || 5;
            document.getElementById('finish-action').value = data.mission.finish_action || 'no_action';
            document.getElementById('heading-mode').value = data.mission.heading_mode || 'auto';
            
            // Add waypoints
            data.mission.waypoints.forEach(waypoint => {
                addWaypoint(waypoint.latitude, waypoint.longitude);
                
                // Update waypoint properties
                const index = waypoints.length - 1;
                waypoints[index] = {
                    ...waypoint
                };
            });
            
            // Update waypoint list
            updateWaypointsList();
            
            // Update mission path
            updateMissionPath();
        } else {
            alert('Failed to load mission: ' + (data.error || 'Unknown error'));
        }
    } catch (error) {
        console.error('Error viewing mission:', error);
        alert('Error viewing mission: ' + error.message);
    }
}

/**
 * Execute a mission
 */
async function executeMission(filename) {
    if (!isConnected) {
        alert('Please connect to a drone first');
        return;
    }
    
    try {
        const response = await fetch(`/api/execute_mission/${filename}`, {
            method: 'POST'
        });
        
        const data = await response.json();
        
        if (data.success) {
            alert('Mission execution started');
            
            // Switch to dashboard page
            document.querySelectorAll('.nav-link').forEach(link => {
                if (link.getAttribute('data-page') === 'dashboard') {
                    link.click();
                }
            });
            
            currentMission = filename;
        } else {
            alert('Failed to start mission: ' + (data.error || 'Unknown error'));
        }
    } catch (error) {
        console.error('Error executing mission:', error);
        alert('Error executing mission: ' + error.message);
    }
}

/**
 * Delete a mission
 */
async function deleteMission(filename) {
    if (!confirm('Are you sure you want to delete this mission?')) {
        return;
    }
    
    try {
        const response = await fetch(`/api/missions/${filename}`, {
            method: 'DELETE'
        });
        
        const data = await response.json();
        
        if (data.success) {
            alert('Mission deleted');
            loadMissions();
        } else {
            alert('Failed to delete mission: ' + (data.error || 'Unknown error'));
        }
    } catch (error) {
        console.error('Error deleting mission:', error);
        alert('Error deleting mission: ' + error.message);
    }
}

/**
 * Load settings from local storage
 */
function loadSettings() {
    // Load connection parameters
    const savedConnectionParams = localStorage.getItem('connectionParams');
    if (savedConnectionParams) {
        connectionParams = JSON.parse(savedConnectionParams);
        
        // Set form values
        document.getElementById('app-id').value = connectionParams.app_id || '';
        document.getElementById('app-key').value = connectionParams.app_key || '';
        document.getElementById('connection-type').value = connectionParams.connection_type || 'USB';
        document.getElementById('drone-model').value = connectionParams.drone_model || 'Mavic 3';
        document.getElementById('serial-number').value = connectionParams.serial_number || '';
        
        if (connectionParams.connection_type === 'BRIDGE') {
            document.querySelector('.bridge-settings').style.display = 'block';
            document.getElementById('bridge-ip').value = connectionParams.bridge_ip || '';
            document.getElementById('bridge-port').value = connectionParams.bridge_port || '';
        }
    }
    
    // Load dock parameters
    const savedDockParams = localStorage.getItem('dockParams');
    if (savedDockParams) {
        dockParams = JSON.parse(savedDockParams);
        
        // Set form values
        document.getElementById('dock-api-key').value = dockParams.api_key || '';
        document.getElementById('dock-api-secret').value = dockParams.api_secret || '';
        document.getElementById('dock-sn').value = dockParams.dock_sn || '';
        document.getElementById('dock-region').value = dockParams.region || 'us-east-1';
    }
}

/**
 * Update status
 */
async function updateStatus() {
    try {
        const response = await fetch('/api/status');
        const data = await response.json();
        
        // Update connection status
        isConnected = data.connected;
        const connectionStatus = document.querySelector('.connection-status');
        const connectBtn = document.getElementById('connectBtn');
        const disconnectBtn = document.getElementById('disconnectBtn');
        
        if (isConnected) {
            connectionStatus.textContent = 'Connected';
            connectionStatus.classList.remove('connecting', 'disconnected');
            connectionStatus.classList.add('connected');
            connectBtn.style.display = 'none';
            disconnectBtn.style.display = 'inline-block';
            
            // Enable control buttons
            enableControlButtons();
        } else {
            connectionStatus.textContent = 'Disconnected';
            connectionStatus.classList.remove('connecting', 'connected');
            connectionStatus.classList.add('disconnected');
            connectBtn.style.display = 'inline-block';
            disconnectBtn.style.display = 'none';
            
            // Disable control buttons
            disableControlButtons();
        }
    } catch (error) {
        console.error('Error updating status:', error);
    }
}
