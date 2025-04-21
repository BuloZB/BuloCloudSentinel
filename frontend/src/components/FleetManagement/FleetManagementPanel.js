import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { MapContainer, TileLayer, Marker, Popup, Polyline } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import './FleetManagementPanel.css';

// Fix Leaflet marker icon issue
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon-2x.png',
  iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
  shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png',
});

// Custom marker icons
const droneIcon = new L.Icon({
  iconUrl: '/icons/drone-marker.png',
  iconSize: [32, 32],
  iconAnchor: [16, 16],
  popupAnchor: [0, -16]
});

const leaderIcon = new L.Icon({
  iconUrl: '/icons/leader-drone-marker.png',
  iconSize: [32, 32],
  iconAnchor: [16, 16],
  popupAnchor: [0, -16]
});

const FleetManagementPanel = () => {
  const [fleets, setFleets] = useState([]);
  const [formations, setFormations] = useState([]);
  const [behaviors, setBehaviors] = useState([]);
  const [missions, setMissions] = useState([]);
  const [selectedFleet, setSelectedFleet] = useState(null);
  const [fleetTelemetry, setFleetTelemetry] = useState({});
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  
  // Form states
  const [newFleetName, setNewFleetName] = useState('');
  const [newFleetDescription, setNewFleetDescription] = useState('');
  const [availableDrones, setAvailableDrones] = useState([]);
  const [selectedDrones, setSelectedDrones] = useState([]);
  const [showNewFleetForm, setShowNewFleetForm] = useState(false);
  const [showNewMissionForm, setShowNewMissionForm] = useState(false);
  const [newMissionName, setNewMissionName] = useState('');
  const [selectedFormation, setSelectedFormation] = useState('');
  const [selectedBehavior, setSelectedBehavior] = useState('');
  
  // Load data on component mount
  useEffect(() => {
    fetchData();
    
    // Set up polling for telemetry
    const interval = setInterval(() => {
      if (selectedFleet) {
        fetchFleetTelemetry(selectedFleet.id);
      }
    }, 1000);
    
    return () => clearInterval(interval);
  }, [selectedFleet]);
  
  const fetchData = async () => {
    try {
      setIsLoading(true);
      
      // Fetch fleets
      const fleetsResponse = await axios.get('/api/fleet');
      setFleets(fleetsResponse.data);
      
      // Fetch formations
      const formationsResponse = await axios.get('/api/fleet/formation');
      setFormations(formationsResponse.data);
      
      // Fetch behaviors
      const behaviorsResponse = await axios.get('/api/fleet/behavior');
      setBehaviors(behaviorsResponse.data);
      
      // Fetch missions
      const missionsResponse = await axios.get('/api/fleet/mission');
      setMissions(missionsResponse.data);
      
      // Fetch available drones
      const dronesResponse = await axios.get('/api/device-inventory/drones');
      setAvailableDrones(dronesResponse.data);
      
      setIsLoading(false);
    } catch (err) {
      console.error('Error fetching data:', err);
      setError('Failed to load fleet management data');
      setIsLoading(false);
    }
  };
  
  const fetchFleetTelemetry = async (fleetId) => {
    try {
      const response = await axios.get(`/api/fleet/${fleetId}/telemetry`);
      setFleetTelemetry(response.data);
    } catch (err) {
      console.error('Error fetching fleet telemetry:', err);
    }
  };
  
  const handleCreateFleet = async (e) => {
    e.preventDefault();
    
    try {
      // Prepare drone data
      const drones = selectedDrones.map(droneId => {
        const isLeader = selectedDrones.indexOf(droneId) === 0;
        return {
          drone_id: droneId,
          role: isLeader ? 'leader' : 'follower'
        };
      });
      
      // Create fleet
      const response = await axios.post('/api/fleet/create', {
        name: newFleetName,
        description: newFleetDescription,
        drones
      });
      
      // Update fleets list
      setFleets([...fleets, response.data]);
      
      // Reset form
      setNewFleetName('');
      setNewFleetDescription('');
      setSelectedDrones([]);
      setShowNewFleetForm(false);
      
      // Show success message
      alert('Fleet created successfully!');
    } catch (err) {
      console.error('Error creating fleet:', err);
      setError('Failed to create fleet');
    }
  };
  
  const handleCreateMission = async (e) => {
    e.preventDefault();
    
    if (!selectedFleet) {
      setError('Please select a fleet first');
      return;
    }
    
    try {
      // Create mission
      const response = await axios.post('/api/fleet/mission/create', {
        name: newMissionName,
        fleet_id: selectedFleet.id,
        formation_id: selectedFormation || undefined,
        behavior_id: selectedBehavior || undefined
      });
      
      // Update missions list
      setMissions([...missions, response.data]);
      
      // Reset form
      setNewMissionName('');
      setSelectedFormation('');
      setSelectedBehavior('');
      setShowNewMissionForm(false);
      
      // Show success message
      alert('Mission created successfully!');
    } catch (err) {
      console.error('Error creating mission:', err);
      setError('Failed to create mission');
    }
  };
  
  const handleExecuteMission = async (missionId) => {
    try {
      await axios.post(`/api/fleet/mission/${missionId}/execute`);
      
      // Refresh missions
      const missionsResponse = await axios.get('/api/fleet/mission');
      setMissions(missionsResponse.data);
      
      // Show success message
      alert('Mission execution started!');
    } catch (err) {
      console.error('Error executing mission:', err);
      setError('Failed to execute mission');
    }
  };
  
  const handleAbortMission = async (missionId) => {
    try {
      await axios.post(`/api/fleet/mission/${missionId}/abort`);
      
      // Refresh missions
      const missionsResponse = await axios.get('/api/fleet/mission');
      setMissions(missionsResponse.data);
      
      // Show success message
      alert('Mission aborted!');
    } catch (err) {
      console.error('Error aborting mission:', err);
      setError('Failed to abort mission');
    }
  };
  
  const renderDroneMarkers = () => {
    if (!selectedFleet || !fleetTelemetry) {
      return null;
    }
    
    const markers = [];
    
    // Find leader drone
    const leaderDrone = selectedFleet.drones.find(drone => drone.role === 'leader');
    const leaderDroneId = leaderDrone ? leaderDrone.drone_id : null;
    
    // Render markers for each drone
    for (const drone of selectedFleet.drones) {
      const telemetry = fleetTelemetry[drone.drone_id];
      if (!telemetry || !telemetry.position) {
        continue;
      }
      
      const position = [telemetry.position.latitude, telemetry.position.longitude];
      const isLeader = drone.drone_id === leaderDroneId;
      
      markers.push(
        <Marker
          key={drone.drone_id}
          position={position}
          icon={isLeader ? leaderIcon : droneIcon}
        >
          <Popup>
            <div>
              <strong>{isLeader ? 'Leader' : 'Follower'} Drone: {drone.drone_id}</strong>
              <p>Latitude: {telemetry.position.latitude.toFixed(6)}</p>
              <p>Longitude: {telemetry.position.longitude.toFixed(6)}</p>
              {telemetry.position.altitude && (
                <p>Altitude: {telemetry.position.altitude.toFixed(1)}m</p>
              )}
              {telemetry.battery && (
                <p>Battery: {telemetry.battery.percentage.toFixed(0)}%</p>
              )}
            </div>
          </Popup>
        </Marker>
      );
    }
    
    return markers;
  };
  
  const renderFormationLines = () => {
    if (!selectedFleet || !fleetTelemetry) {
      return null;
    }
    
    const positions = [];
    
    // Collect positions for each drone
    for (const drone of selectedFleet.drones) {
      const telemetry = fleetTelemetry[drone.drone_id];
      if (!telemetry || !telemetry.position) {
        continue;
      }
      
      positions.push([telemetry.position.latitude, telemetry.position.longitude]);
    }
    
    if (positions.length < 2) {
      return null;
    }
    
    return <Polyline positions={positions} color="blue" />;
  };
  
  if (isLoading) {
    return <div className="loading">Loading fleet management data...</div>;
  }
  
  return (
    <div className="fleet-management-panel">
      <div className="fleet-management-sidebar">
        <h2>Fleet Management</h2>
        
        {error && (
          <div className="error-message">
            {error}
            <button onClick={() => setError(null)}>Dismiss</button>
          </div>
        )}
        
        <div className="fleet-section">
          <div className="section-header">
            <h3>Fleets</h3>
            <button 
              className="new-button"
              onClick={() => setShowNewFleetForm(!showNewFleetForm)}
            >
              {showNewFleetForm ? 'Cancel' : 'New Fleet'}
            </button>
          </div>
          
          {showNewFleetForm && (
            <form className="new-fleet-form" onSubmit={handleCreateFleet}>
              <div className="form-group">
                <label>Fleet Name</label>
                <input
                  type="text"
                  value={newFleetName}
                  onChange={(e) => setNewFleetName(e.target.value)}
                  required
                />
              </div>
              
              <div className="form-group">
                <label>Description</label>
                <textarea
                  value={newFleetDescription}
                  onChange={(e) => setNewFleetDescription(e.target.value)}
                />
              </div>
              
              <div className="form-group">
                <label>Select Drones</label>
                <div className="drone-selection">
                  {availableDrones.map(drone => (
                    <div key={drone.id} className="drone-option">
                      <input
                        type="checkbox"
                        id={`drone-${drone.id}`}
                        checked={selectedDrones.includes(drone.id)}
                        onChange={(e) => {
                          if (e.target.checked) {
                            setSelectedDrones([...selectedDrones, drone.id]);
                          } else {
                            setSelectedDrones(selectedDrones.filter(id => id !== drone.id));
                          }
                        }}
                      />
                      <label htmlFor={`drone-${drone.id}`}>{drone.name}</label>
                    </div>
                  ))}
                </div>
              </div>
              
              <button type="submit" className="submit-button">Create Fleet</button>
            </form>
          )}
          
          <div className="fleet-list">
            {fleets.length === 0 ? (
              <p className="empty-message">No fleets available</p>
            ) : (
              fleets.map(fleet => (
                <div
                  key={fleet.id}
                  className={`fleet-item ${selectedFleet && selectedFleet.id === fleet.id ? 'selected' : ''}`}
                  onClick={() => {
                    setSelectedFleet(fleet);
                    fetchFleetTelemetry(fleet.id);
                  }}
                >
                  <div className="fleet-item-header">
                    <span className="fleet-name">{fleet.name}</span>
                    <span className="drone-count">{fleet.drones.length} drones</span>
                  </div>
                  {fleet.description && (
                    <div className="fleet-description">{fleet.description}</div>
                  )}
                </div>
              ))
            )}
          </div>
        </div>
        
        {selectedFleet && (
          <div className="mission-section">
            <div className="section-header">
              <h3>Fleet Missions</h3>
              <button 
                className="new-button"
                onClick={() => setShowNewMissionForm(!showNewMissionForm)}
              >
                {showNewMissionForm ? 'Cancel' : 'New Mission'}
              </button>
            </div>
            
            {showNewMissionForm && (
              <form className="new-mission-form" onSubmit={handleCreateMission}>
                <div className="form-group">
                  <label>Mission Name</label>
                  <input
                    type="text"
                    value={newMissionName}
                    onChange={(e) => setNewMissionName(e.target.value)}
                    required
                  />
                </div>
                
                <div className="form-group">
                  <label>Formation</label>
                  <select
                    value={selectedFormation}
                    onChange={(e) => setSelectedFormation(e.target.value)}
                  >
                    <option value="">None</option>
                    {formations.map(formation => (
                      <option key={formation.id} value={formation.id}>
                        {formation.name} ({formation.type})
                      </option>
                    ))}
                  </select>
                </div>
                
                <div className="form-group">
                  <label>Behavior</label>
                  <select
                    value={selectedBehavior}
                    onChange={(e) => setSelectedBehavior(e.target.value)}
                  >
                    <option value="">None</option>
                    {behaviors.map(behavior => (
                      <option key={behavior.id} value={behavior.id}>
                        {behavior.name} ({behavior.type})
                      </option>
                    ))}
                  </select>
                </div>
                
                <button type="submit" className="submit-button">Create Mission</button>
              </form>
            )}
            
            <div className="mission-list">
              {missions.filter(mission => mission.fleet_id === selectedFleet.id).length === 0 ? (
                <p className="empty-message">No missions for this fleet</p>
              ) : (
                missions
                  .filter(mission => mission.fleet_id === selectedFleet.id)
                  .map(mission => (
                    <div key={mission.id} className="mission-item">
                      <div className="mission-item-header">
                        <span className="mission-name">{mission.name}</span>
                        <span className={`mission-status ${mission.status}`}>{mission.status}</span>
                      </div>
                      
                      <div className="mission-actions">
                        {mission.status === 'pending' && (
                          <button
                            className="execute-button"
                            onClick={() => handleExecuteMission(mission.id)}
                          >
                            Execute
                          </button>
                        )}
                        
                        {(mission.status === 'executing' || mission.status === 'in_progress') && (
                          <button
                            className="abort-button"
                            onClick={() => handleAbortMission(mission.id)}
                          >
                            Abort
                          </button>
                        )}
                      </div>
                    </div>
                  ))
              )}
            </div>
          </div>
        )}
      </div>
      
      <div className="fleet-map-container">
        <MapContainer
          center={[51.505, -0.09]}
          zoom={13}
          style={{ height: '100%', width: '100%' }}
        >
          <TileLayer
            attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          />
          
          {renderDroneMarkers()}
          {renderFormationLines()}
        </MapContainer>
      </div>
    </div>
  );
};

export default FleetManagementPanel;
