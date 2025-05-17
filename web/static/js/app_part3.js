/**
 * Add a waypoint to the mission
 */
function addWaypoint(lat, lng) {
    // Create waypoint object
    const waypoint = {
        latitude: lat,
        longitude: lng,
        altitude: 30,  // Default altitude
        heading: 0,
        stay_time: 0,
        actions: []
    };
    
    // Add to waypoints array
    waypoints.push(waypoint);
    
    // Create marker
    const waypointIcon = L.divIcon({
        className: 'map-marker-waypoint',
        iconSize: [12, 12]
    });
    
    const marker = L.marker([lat, lng], { 
        icon: waypointIcon,
        draggable: true
    })
    .addTo(missionMap)
    .bindTooltip(`Waypoint ${waypoints.length}`);
    
    // Add drag end event
    marker.on('dragend', function(e) {
        const index = waypointMarkers.indexOf(marker);
        const latlng = marker.getLatLng();
        
        // Update waypoint
        waypoints[index].latitude = latlng.lat;
        waypoints[index].longitude = latlng.lng;
        
        // Update waypoint list
        updateWaypointsList();
        
        // Update mission path
        updateMissionPath();
    });
    
    // Add click event to edit waypoint
    marker.on('click', function(e) {
        const index = waypointMarkers.indexOf(marker);
        editWaypoint(index);
    });
    
    // Add to markers array
    waypointMarkers.push(marker);
    
    // Update waypoint list
    updateWaypointsList();
    
    // Update mission path
    updateMissionPath();
}

/**
 * Edit a waypoint
 */
function editWaypoint(index) {
    const waypoint = waypoints[index];
    
    // Set form values
    document.getElementById('waypoint-index').value = index;
    document.getElementById('waypoint-latitude').value = waypoint.latitude;
    document.getElementById('waypoint-longitude').value = waypoint.longitude;
    document.getElementById('waypoint-altitude').value = waypoint.altitude;
    document.getElementById('waypoint-heading').value = waypoint.heading;
    document.getElementById('waypoint-stay-time').value = waypoint.stay_time;
    
    // Reset actions
    document.getElementById('action-take-photo').checked = false;
    document.getElementById('action-start-recording').checked = false;
    document.getElementById('action-stop-recording').checked = false;
    document.getElementById('action-rotate-gimbal').checked = false;
    document.querySelector('.gimbal-settings').style.display = 'none';
    
    // Set actions
    waypoint.actions.forEach(action => {
        if (action.action === 'take_photo') {
            document.getElementById('action-take-photo').checked = true;
        } else if (action.action === 'start_recording') {
            document.getElementById('action-start-recording').checked = true;
        } else if (action.action === 'stop_recording') {
            document.getElementById('action-stop-recording').checked = true;
        } else if (action.action === 'rotate_gimbal') {
            document.getElementById('action-rotate-gimbal').checked = true;
            document.querySelector('.gimbal-settings').style.display = 'block';
            document.getElementById('gimbal-pitch').value = action.pitch || -30;
            document.getElementById('gimbal-roll').value = action.roll || 0;
            document.getElementById('gimbal-yaw').value = action.yaw || 0;
        }
    });
    
    // Show modal
    const waypointModal = new bootstrap.Modal(document.getElementById('waypointModal'));
    waypointModal.show();
}

/**
 * Clear all waypoints
 */
function clearWaypoints() {
    // Remove all markers
    waypointMarkers.forEach(marker => {
        missionMap.removeLayer(marker);
    });
    
    // Clear arrays
    waypoints = [];
    waypointMarkers = [];
    
    // Remove mission path
    if (missionPath) {
        missionMap.removeLayer(missionPath);
        missionPath = null;
    }
    
    // Update waypoint list
    updateWaypointsList();
}

/**
 * Update the waypoints list display
 */
function updateWaypointsList() {
    const waypointsList = document.getElementById('waypoints-list');
    
    if (waypoints.length === 0) {
        waypointsList.innerHTML = '<p class="text-muted">No waypoints added yet. Click on the map to add waypoints.</p>';
        return;
    }
    
    let html = '';
    
    waypoints.forEach((waypoint, index) => {
        html += `
            <div class="waypoint-item">
                <span class="waypoint-number">Waypoint ${index + 1}</span>
                <div class="waypoint-actions">
                    <button class="btn btn-sm btn-primary edit-waypoint" data-index="${index}">Edit</button>
                </div>
                <div class="waypoint-details">
                    Lat: ${waypoint.latitude.toFixed(6)}, Lon: ${waypoint.longitude.toFixed(6)}, Alt: ${waypoint.altitude}m
                </div>
        `;
        
        if (waypoint.actions.length > 0) {
            html += '<div class="waypoint-actions-list">Actions: ';
            
            waypoint.actions.forEach(action => {
                if (action.action === 'take_photo') {
                    html += '<span>Take Photo</span>';
                } else if (action.action === 'start_recording') {
                    html += '<span>Start Recording</span>';
                } else if (action.action === 'stop_recording') {
                    html += '<span>Stop Recording</span>';
                } else if (action.action === 'rotate_gimbal') {
                    html += `<span>Rotate Gimbal (${action.pitch}°, ${action.roll}°, ${action.yaw}°)</span>`;
                }
            });
            
            html += '</div>';
        }
        
        html += '</div>';
    });
    
    waypointsList.innerHTML = html;
    
    // Add edit button event listeners
    document.querySelectorAll('.edit-waypoint').forEach(button => {
        button.addEventListener('click', function() {
            const index = parseInt(this.getAttribute('data-index'));
            editWaypoint(index);
        });
    });
}

/**
 * Update the mission path on the map
 */
function updateMissionPath() {
    // Remove existing path
    if (missionPath) {
        missionMap.removeLayer(missionPath);
    }
    
    if (waypoints.length < 2) {
        return;
    }
    
    // Create latlngs array
    const latlngs = waypoints.map(waypoint => [waypoint.latitude, waypoint.longitude]);
    
    // Create path
    missionPath = L.polyline(latlngs, {
        color: '#007bff',
        weight: 3,
        opacity: 0.7,
        dashArray: '5, 5'
    }).addTo(missionMap);
    
    // Fit map to path
    missionMap.fitBounds(missionPath.getBounds(), {
        padding: [50, 50]
    });
}

/**
 * Initialize mission list
 */
function initializeMissionList() {
    // Load missions
    loadMissions();
}

/**
 * Load missions from server
 */
async function loadMissions() {
    try {
        const response = await fetch('/api/missions');
        const missions = await response.json();
        
        const tableBody = document.getElementById('missions-table-body');
        
        if (missions.length === 0) {
            tableBody.innerHTML = '<tr><td colspan="5" class="text-center">No missions found</td></tr>';
            return;
        }
        
        let html = '';
        
        missions.forEach(mission => {
            html += `
                <tr>
                    <td>${mission.name}</td>
                    <td>${mission.type}</td>
                    <td>${mission.waypoints}</td>
                    <td>${new Date(mission.created).toLocaleString()}</td>
                    <td class="mission-actions">
                        <button class="btn btn-sm btn-primary view-mission" data-filename="${mission.filename}">View</button>
                        <button class="btn btn-sm btn-success execute-mission" data-filename="${mission.filename}">Execute</button>
                        <button class="btn btn-sm btn-danger delete-mission" data-filename="${mission.filename}">Delete</button>
                    </td>
                </tr>
            `;
        });
        
        tableBody.innerHTML = html;
        
        // Add button event listeners
        document.querySelectorAll('.view-mission').forEach(button => {
            button.addEventListener('click', function() {
                const filename = this.getAttribute('data-filename');
                viewMission(filename);
            });
        });
        
        document.querySelectorAll('.execute-mission').forEach(button => {
            button.addEventListener('click', function() {
                const filename = this.getAttribute('data-filename');
                executeMission(filename);
            });
        });
        
        document.querySelectorAll('.delete-mission').forEach(button => {
            button.addEventListener('click', function() {
                const filename = this.getAttribute('data-filename');
                deleteMission(filename);
            });
        });
    } catch (error) {
        console.error('Error loading missions:', error);
        document.getElementById('missions-table-body').innerHTML = 
            '<tr><td colspan="5" class="text-center text-danger">Error loading missions</td></tr>';
    }
}
