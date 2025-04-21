import React, { useState, useEffect, useRef } from 'react';
import { MapContainer, TileLayer, Marker, Popup, Polyline, useMap } from 'react-leaflet';
import L from 'leaflet';
import axios from 'axios';
import TelemetryDisplay from './TelemetryDisplay';
import MissionControls from './MissionControls';
import './MissionExecutionPanel.css';

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

const waypointIcon = new L.Icon({
  iconUrl: '/icons/waypoint-marker.png',
  iconSize: [24, 24],
  iconAnchor: [12, 12],
  popupAnchor: [0, -12]
});

const homeIcon = new L.Icon({
  iconUrl: '/icons/home-marker.png',
  iconSize: [32, 32],
  iconAnchor: [16, 16],
  popupAnchor: [0, -16]
});

// Map component that follows the drone
const DroneFollower = ({ position, follow }) => {
  const map = useMap();
  
  useEffect(() => {
    if (follow && position) {
      map.setView([position.latitude, position.longitude], map.getZoom());
    }
  }, [map, position, follow]);
  
  return null;
};

const MissionExecutionPanel = ({ executionId }) => {
  const [mission, setMission] = useState(null);
  const [executionStatus, setExecutionStatus] = useState(null);
  const [telemetry, setTelemetry] = useState(null);
  const [telemetryHistory, setTelemetryHistory] = useState([]);
  const [waypoints, setWaypoints] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [followDrone, setFollowDrone] = useState(true);
  const [showTelemetryPanel, setShowTelemetryPanel] = useState(true);
  const [showControlsPanel, setShowControlsPanel] = useState(true);
  const [connectionStatus, setConnectionStatus] = useState('disconnected');
  
  const wsRef = useRef(null);
  const mapRef = useRef(null);
  
  // Load mission and execution data
  useEffect(() => {
    const fetchData = async () => {
      try {
        setIsLoading(true);
        
        // Get execution status
        const statusResponse = await axios.get(`/api/mission-execution/${executionId}/status`);
        setExecutionStatus(statusResponse.data);
        
        // Get mission details
        const missionResponse = await axios.get(`/api/missions/${statusResponse.data.mission_id}`);
        setMission(missionResponse.data);
        
        // Set waypoints
        if (missionResponse.data.settings && missionResponse.data.settings.waypoint) {
          setWaypoints(missionResponse.data.settings.waypoint);
        }
        
        setIsLoading(false);
      } catch (err) {
        console.error('Error fetching mission execution data:', err);
        setError('Failed to load mission execution data');
        setIsLoading(false);
      }
    };
    
    fetchData();
  }, [executionId]);
  
  // Connect to WebSocket for telemetry
  useEffect(() => {
    const connectWebSocket = () => {
      const ws = new WebSocket(`ws://${window.location.host}/api/mission-execution/${executionId}/telemetry`);
      
      ws.onopen = () => {
        console.log('WebSocket connected');
        setConnectionStatus('connected');
      };
      
      ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        
        if (data.type === 'telemetry') {
          setTelemetry(data.data);
          setTelemetryHistory(prev => [...prev, data.data].slice(-100)); // Keep last 100 points
        } else if (data.type === 'status') {
          setExecutionStatus(data.data);
        }
      };
      
      ws.onclose = () => {
        console.log('WebSocket disconnected');
        setConnectionStatus('disconnected');
        
        // Attempt to reconnect after 2 seconds
        setTimeout(() => {
          if (wsRef.current === ws) {
            connectWebSocket();
          }
        }, 2000);
      };
      
      ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        setConnectionStatus('error');
      };
      
      wsRef.current = ws;
    };
    
    connectWebSocket();
    
    // Cleanup on unmount
    return () => {
      if (wsRef.current) {
        wsRef.current.close();
        wsRef.current = null;
      }
    };
  }, [executionId]);
  
  // Send command to the server
  const sendCommand = async (command, params = null) => {
    try {
      // Send via WebSocket if connected
      if (wsRef.current && connectionStatus === 'connected') {
        wsRef.current.send(JSON.stringify({
          command,
          params
        }));
      } else {
        // Fallback to REST API
        await axios.post(`/api/mission-execution/${executionId}/control`, {
          command,
          params
        });
        
        // Refresh status
        const response = await axios.get(`/api/mission-execution/${executionId}/status`);
        setExecutionStatus(response.data);
      }
    } catch (err) {
      console.error(`Error sending command ${command}:`, err);
      setError(`Failed to send command: ${command}`);
    }
  };
  
  // Get drone position from telemetry
  const getDronePosition = () => {
    if (telemetry && telemetry.position) {
      return {
        latitude: telemetry.position.latitude,
        longitude: telemetry.position.longitude
      };
    }
    return null;
  };
  
  // Get drone heading from telemetry
  const getDroneHeading = () => {
    if (telemetry && telemetry.attitude && telemetry.attitude.yaw !== undefined) {
      return telemetry.attitude.yaw;
    }
    return 0;
  };
  
  // Format elapsed time
  const formatElapsedTime = (seconds) => {
    if (!seconds) return '00:00:00';
    
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const secs = Math.floor(seconds % 60);
    
    return [
      hours.toString().padStart(2, '0'),
      minutes.toString().padStart(2, '0'),
      secs.toString().padStart(2, '0')
    ].join(':');
  };
  
  // Get status color
  const getStatusColor = (status) => {
    switch (status) {
      case 'in_progress':
        return 'green';
      case 'paused':
        return 'orange';
      case 'completed':
        return 'blue';
      case 'aborted':
      case 'failed':
        return 'red';
      default:
        return 'gray';
    }
  };
  
  // Get telemetry history path
  const getTelemetryPath = () => {
    return telemetryHistory
      .filter(t => t.position)
      .map(t => [t.position.latitude, t.position.longitude]);
  };
  
  if (isLoading) {
    return <div className="loading">Loading mission execution data...</div>;
  }
  
  if (error) {
    return <div className="error">{error}</div>;
  }
  
  if (!mission || !executionStatus) {
    return <div className="error">Mission or execution data not found</div>;
  }
  
  const dronePosition = getDronePosition();
  const droneHeading = getDroneHeading();
  
  return (
    <div className="mission-execution-panel">
      <div className="mission-execution-header">
        <h2>{mission.name}</h2>
        <div className="mission-status">
          <span 
            className="status-indicator" 
            style={{ backgroundColor: getStatusColor(executionStatus.status) }}
          ></span>
          <span className="status-text">{executionStatus.status.replace('_', ' ').toUpperCase()}</span>
        </div>
        <div className="mission-progress">
          <div className="progress-bar">
            <div 
              className="progress-fill"
              style={{ 
                width: `${(executionStatus.current_waypoint / executionStatus.total_waypoints) * 100}%`,
                backgroundColor: getStatusColor(executionStatus.status)
              }}
            ></div>
          </div>
          <span className="progress-text">
            {executionStatus.current_waypoint} / {executionStatus.total_waypoints} waypoints
          </span>
        </div>
        <div className="mission-time">
          <span className="time-label">Elapsed:</span>
          <span className="time-value">{formatElapsedTime(executionStatus.elapsed_time)}</span>
        </div>
        <div className="connection-status">
          <span 
            className={`connection-indicator ${connectionStatus}`}
            title={`WebSocket: ${connectionStatus}`}
          ></span>
        </div>
      </div>
      
      <div className="mission-execution-content">
        <div className="mission-map-container">
          <MapContainer
            center={dronePosition ? [dronePosition.latitude, dronePosition.longitude] : [0, 0]}
            zoom={16}
            style={{ height: '100%', width: '100%' }}
            whenCreated={mapInstance => {
              mapRef.current = mapInstance;
            }}
          >
            <TileLayer
              attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
              url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            />
            
            {/* Render waypoints */}
            {waypoints.map((waypoint, index) => (
              <Marker
                key={`waypoint-${index}`}
                position={[waypoint.position.latitude, waypoint.position.longitude]}
                icon={waypointIcon}
              >
                <Popup>
                  <div>
                    <strong>Waypoint {index + 1}</strong>
                    <p>Altitude: {waypoint.position.altitude}m</p>
                    {waypoint.actions && waypoint.actions.length > 0 && (
                      <p>Actions: {waypoint.actions.map(a => a.action).join(', ')}</p>
                    )}
                  </div>
                </Popup>
              </Marker>
            ))}
            
            {/* Render waypoint path */}
            {waypoints.length > 1 && (
              <Polyline
                positions={waypoints.map(wp => [wp.position.latitude, wp.position.longitude])}
                color="blue"
                dashArray="5, 5"
              />
            )}
            
            {/* Render telemetry path */}
            {telemetryHistory.length > 1 && (
              <Polyline
                positions={getTelemetryPath()}
                color="red"
                weight={3}
              />
            )}
            
            {/* Render drone */}
            {dronePosition && (
              <Marker
                position={[dronePosition.latitude, dronePosition.longitude]}
                icon={droneIcon}
                rotationAngle={droneHeading}
                rotationOrigin="center"
              >
                <Popup>
                  <div>
                    <strong>Drone</strong>
                    <p>Latitude: {dronePosition.latitude.toFixed(6)}</p>
                    <p>Longitude: {dronePosition.longitude.toFixed(6)}</p>
                    {telemetry && telemetry.position && telemetry.position.altitude && (
                      <p>Altitude: {telemetry.position.altitude.toFixed(1)}m</p>
                    )}
                    {telemetry && telemetry.speed && (
                      <p>Speed: {telemetry.speed.horizontal.toFixed(1)}m/s</p>
                    )}
                  </div>
                </Popup>
              </Marker>
            )}
            
            {/* Home position */}
            {mission.homePosition && (
              <Marker
                position={[mission.homePosition.latitude, mission.homePosition.longitude]}
                icon={homeIcon}
              >
                <Popup>
                  <div>
                    <strong>Home Position</strong>
                  </div>
                </Popup>
              </Marker>
            )}
            
            {/* Follow drone */}
            <DroneFollower position={dronePosition} follow={followDrone} />
          </MapContainer>
          
          <div className="map-controls">
            <button
              className={`map-control-btn ${followDrone ? 'active' : ''}`}
              onClick={() => setFollowDrone(!followDrone)}
              title={followDrone ? 'Disable follow drone' : 'Enable follow drone'}
            >
              <i className="fas fa-crosshairs"></i>
            </button>
            
            <button
              className="map-control-btn"
              onClick={() => {
                if (mapRef.current && waypoints.length > 0) {
                  const positions = waypoints.map(wp => [wp.position.latitude, wp.position.longitude]);
                  mapRef.current.fitBounds(positions);
                }
              }}
              title="Fit to mission"
            >
              <i className="fas fa-expand"></i>
            </button>
          </div>
        </div>
        
        <div className="mission-panels">
          {showTelemetryPanel && (
            <div className="telemetry-panel">
              <div className="panel-header">
                <h3>Telemetry</h3>
                <button 
                  className="panel-toggle"
                  onClick={() => setShowTelemetryPanel(false)}
                >
                  <i className="fas fa-times"></i>
                </button>
              </div>
              <TelemetryDisplay telemetry={telemetry} />
            </div>
          )}
          
          {showControlsPanel && (
            <div className="controls-panel">
              <div className="panel-header">
                <h3>Mission Controls</h3>
                <button 
                  className="panel-toggle"
                  onClick={() => setShowControlsPanel(false)}
                >
                  <i className="fas fa-times"></i>
                </button>
              </div>
              <MissionControls 
                status={executionStatus.status}
                onCommand={sendCommand}
              />
            </div>
          )}
          
          <div className="panel-toggles">
            {!showTelemetryPanel && (
              <button 
                className="panel-show-btn"
                onClick={() => setShowTelemetryPanel(true)}
              >
                Show Telemetry
              </button>
            )}
            
            {!showControlsPanel && (
              <button 
                className="panel-show-btn"
                onClick={() => setShowControlsPanel(true)}
              >
                Show Controls
              </button>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default MissionExecutionPanel;
