import React, { useState, useEffect } from 'react';
import DOMPurify from 'dompurify';
import './Widget.css';

const MissionWidget = ({ id, title, settings, onRemove }) => {
  const [missions, setMissions] = useState([]);
  const [selectedMission, setSelectedMission] = useState(settings?.defaultMission || null);
  const [missionDetails, setMissionDetails] = useState(null);
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  
  // Fetch available missions
  useEffect(() => {
    const fetchMissions = async () => {
      try {
        const response = await fetch('/api/missions');
        if (!response.ok) {
          throw new Error(`Failed to fetch missions: ${response.status}`);
        }
        
        const data = await response.json();
        
        // Sanitize mission data
        const sanitizedMissions = data.map(mission => ({
          ...mission,
          name: DOMPurify.sanitize(mission.name),
          description: mission.description ? DOMPurify.sanitize(mission.description) : null,
          type: DOMPurify.sanitize(mission.type),
          status: DOMPurify.sanitize(mission.status)
        }));
        
        setMissions(sanitizedMissions);
        
        // Set default mission if available
        if (sanitizedMissions.length > 0 && !selectedMission) {
          setSelectedMission(settings?.defaultMission || sanitizedMissions[0].id);
        }
        
        setIsLoading(false);
      } catch (error) {
        console.error('Error fetching missions:', error);
        setError('Failed to load missions');
        setIsLoading(false);
      }
    };

    fetchMissions();
  }, [settings?.defaultMission, selectedMission]);

  // Fetch mission details when selected mission changes
  useEffect(() => {
    if (!selectedMission) return;
    
    const fetchMissionDetails = async () => {
      try {
        setIsLoading(true);
        
        const response = await fetch(`/api/missions/${selectedMission}`);
        if (!response.ok) {
          throw new Error(`Failed to fetch mission details: ${response.status}`);
        }
        
        const data = await response.json();
        
        // Sanitize mission details
        const sanitizedDetails = {
          ...data,
          name: DOMPurify.sanitize(data.name),
          description: data.description ? DOMPurify.sanitize(data.description) : null,
          type: DOMPurify.sanitize(data.type),
          status: DOMPurify.sanitize(data.status),
          assignedDrones: data.assignedDrones.map(drone => ({
            ...drone,
            name: DOMPurify.sanitize(drone.name),
            status: DOMPurify.sanitize(drone.status)
          })),
          waypoints: data.waypoints.map(waypoint => ({
            ...waypoint,
            name: waypoint.name ? DOMPurify.sanitize(waypoint.name) : null,
            action: waypoint.action ? DOMPurify.sanitize(waypoint.action) : null
          }))
        };
        
        setMissionDetails(sanitizedDetails);
        setIsLoading(false);
      } catch (error) {
        console.error('Error fetching mission details:', error);
        setError('Failed to load mission details');
        setIsLoading(false);
      }
    };

    fetchMissionDetails();
  }, [selectedMission]);

  // Handle mission selection
  const handleMissionChange = (e) => {
    setSelectedMission(e.target.value);
  };

  // Format mission duration
  const formatDuration = (minutes) => {
    const hours = Math.floor(minutes / 60);
    const mins = Math.round(minutes % 60);
    
    if (hours > 0) {
      return `${hours}h ${mins}m`;
    } else {
      return `${mins}m`;
    }
  };

  // Format date
  const formatDate = (dateString) => {
    const date = new Date(dateString);
    return date.toLocaleString();
  };

  // Get status color
  const getStatusColor = (status) => {
    switch (status.toLowerCase()) {
      case 'active':
        return '#4CAF50'; // Green
      case 'completed':
        return '#2196F3'; // Blue
      case 'scheduled':
        return '#FF9800'; // Orange
      case 'failed':
        return '#F44336'; // Red
      case 'paused':
        return '#9E9E9E'; // Gray
      default:
        return '#9E9E9E'; // Gray
    }
  };

  return (
    <div className="widget mission-widget">
      <div className="widget-header">
        <h3>{DOMPurify.sanitize(title)}</h3>
        {onRemove && (
          <button className="widget-remove-btn" onClick={onRemove}>
            ✕
          </button>
        )}
      </div>
      
      <div className="widget-content">
        <div className="mission-controls">
          <select 
            value={selectedMission || ''} 
            onChange={handleMissionChange}
            className="mission-select"
          >
            <option value="" disabled>Select mission</option>
            {missions.map(mission => (
              <option key={mission.id} value={mission.id}>
                {mission.name} - {mission.status}
              </option>
            ))}
          </select>
        </div>
        
        {isLoading ? (
          <div className="widget-loading">Loading mission data...</div>
        ) : error ? (
          <div className="widget-error">{error}</div>
        ) : !missionDetails ? (
          <div className="widget-empty">No mission data available</div>
        ) : (
          <div className="mission-details">
            <div className="mission-header">
              <div className="mission-title">
                <h4>{missionDetails.name}</h4>
                <span 
                  className="mission-status"
                  style={{ backgroundColor: getStatusColor(missionDetails.status) }}
                >
                  {missionDetails.status}
                </span>
              </div>
              
              {missionDetails.description && (
                <p className="mission-description">{missionDetails.description}</p>
              )}
            </div>
            
            <div className="mission-info">
              <div className="info-item">
                <span className="info-label">Type:</span>
                <span className="info-value">{missionDetails.type}</span>
              </div>
              <div className="info-item">
                <span className="info-label">Start:</span>
                <span className="info-value">{formatDate(missionDetails.startTime)}</span>
              </div>
              {missionDetails.endTime && (
                <div className="info-item">
                  <span className="info-label">End:</span>
                  <span className="info-value">{formatDate(missionDetails.endTime)}</span>
                </div>
              )}
              <div className="info-item">
                <span className="info-label">Duration:</span>
                <span className="info-value">{formatDuration(missionDetails.durationMinutes)}</span>
              </div>
              <div className="info-item">
                <span className="info-label">Progress:</span>
                <div className="progress-bar">
                  <div 
                    className="progress-fill"
                    style={{ width: `${missionDetails.progressPercentage}%` }}
                  ></div>
                </div>
                <span className="progress-text">{missionDetails.progressPercentage}%</span>
              </div>
            </div>
            
            <div className="mission-sections">
              <div className="mission-section">
                <h5>Assigned Drones ({missionDetails.assignedDrones.length})</h5>
                <ul className="drone-list">
                  {missionDetails.assignedDrones.map(drone => (
                    <li key={drone.id} className="drone-item">
                      <span className="drone-name">{drone.name}</span>
                      <span 
                        className="drone-status"
                        style={{ backgroundColor: getStatusColor(drone.status) }}
                      >
                        {drone.status}
                      </span>
                    </li>
                  ))}
                </ul>
              </div>
              
              <div className="mission-section">
                <h5>Waypoints ({missionDetails.waypoints.length})</h5>
                <ul className="waypoint-list">
                  {missionDetails.waypoints.map((waypoint, index) => (
                    <li key={index} className={`waypoint-item ${waypoint.completed ? 'completed' : ''}`}>
                      <span className="waypoint-number">{index + 1}</span>
                      <div className="waypoint-details">
                        <span className="waypoint-name">
                          {waypoint.name || `Waypoint ${index + 1}`}
                        </span>
                        {waypoint.action && (
                          <span className="waypoint-action">{waypoint.action}</span>
                        )}
                        <span className="waypoint-coords">
                          {waypoint.latitude.toFixed(6)}, {waypoint.longitude.toFixed(6)}, {waypoint.altitude}m
                        </span>
                      </div>
                      <span className="waypoint-status">
                        {waypoint.completed ? '✓' : '○'}
                      </span>
                    </li>
                  ))}
                </ul>
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default MissionWidget;
