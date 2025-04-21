import React, { useState, useEffect } from 'react';
import DOMPurify from 'dompurify';
import './Widget.css';

const DroneStatusWidget = ({ id, title, settings, onRemove }) => {
  const [drones, setDrones] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [selectedDroneId, setSelectedDroneId] = useState(settings?.defaultDroneId || null);

  // Fetch drones on component mount
  useEffect(() => {
    const fetchDrones = async () => {
      try {
        const response = await fetch('/api/drones');
        if (!response.ok) {
          throw new Error(`Error: ${response.status}`);
        }
        const data = await response.json();

        // Sanitize drone data
        const sanitizedDrones = data.map(drone => ({
          ...drone,
          name: DOMPurify.sanitize(drone.name),
          type: DOMPurify.sanitize(drone.type),
          status: DOMPurify.sanitize(drone.status),
          mission: drone.mission ? DOMPurify.sanitize(drone.mission) : null,
          location: drone.location ? DOMPurify.sanitize(drone.location) : null
        }));

        setDrones(sanitizedDrones);

        // Set first drone as selected if none is selected
        if (!selectedDroneId && sanitizedDrones.length > 0) {
          setSelectedDroneId(sanitizedDrones[0].id);
        }

        setIsLoading(false);
      } catch (error) {
        console.error('Error fetching drones:', error);
        setError('Failed to load drones');
        setIsLoading(false);
      }
    };

    fetchDrones();

    // Set up polling for drone status updates
    const intervalId = setInterval(fetchDrones, 10000); // Poll every 10 seconds

    return () => clearInterval(intervalId);
  }, [selectedDroneId, settings?.defaultDroneId]);

  // Get selected drone
  const selectedDrone = drones.find(drone => drone.id === selectedDroneId);

  // Get status color
  const getStatusColor = (status) => {
    switch (status) {
      case 'online':
      case 'ready':
        return 'green';
      case 'flying':
      case 'mission':
        return 'blue';
      case 'warning':
        return 'orange';
      case 'offline':
      case 'error':
        return 'red';
      default:
        return 'gray';
    }
  };

  return (
    <div className="widget drone-status-widget">
      <div className="widget-header">
        <h3 className="widget-title">{DOMPurify.sanitize(title)}</h3>
        {onRemove && (
          <button className="widget-remove-btn" onClick={onRemove}>
            &times;
          </button>
        )}
      </div>

      <div className="widget-content">
        {isLoading ? (
          <div className="widget-loading">Loading...</div>
        ) : error ? (
          <div className="widget-error">{error}</div>
        ) : drones.length === 0 ? (
          <div className="widget-empty">No drones available</div>
        ) : (
          <>
            <div className="drone-selector">
              <select
                value={selectedDroneId || ''}
                onChange={(e) => setSelectedDroneId(e.target.value)}
              >
                <option value="" disabled>Select Drone</option>
                {drones.map(drone => (
                  <option key={drone.id} value={drone.id}>
                    {drone.name}
                  </option>
                ))}
              </select>
            </div>

            {selectedDrone ? (
              <div className="drone-details">
                <div className="drone-status">
                  <span
                    className="status-indicator"
                    style={{ backgroundColor: getStatusColor(selectedDrone.status) }}
                  ></span>
                  <span className="status-text">{selectedDrone.status}</span>
                </div>

                <div className="drone-info">
                  <div className="info-item">
                    <span className="info-label">Model:</span>
                    <span className="info-value">{selectedDrone.model}</span>
                  </div>

                  <div className="info-item">
                    <span className="info-label">Serial:</span>
                    <span className="info-value">{selectedDrone.serial_number}</span>
                  </div>

                  <div className="info-item">
                    <span className="info-label">Battery:</span>
                    <span className="info-value">
                      {selectedDrone.battery_level ? `${selectedDrone.battery_level}%` : 'N/A'}
                    </span>
                  </div>

                  {selectedDrone.location && (
                    <div className="info-item">
                      <span className="info-label">Location:</span>
                      <span className="info-value">
                        {selectedDrone.location.latitude.toFixed(6)}, {selectedDrone.location.longitude.toFixed(6)}
                      </span>
                    </div>
                  )}
                </div>

                <div className="drone-actions">
                  <button className="action-btn">Connect</button>
                  <button className="action-btn">RTH</button>
                  <button className="action-btn">Emergency</button>
                </div>
              </div>
            ) : (
              <div className="widget-empty">Select a drone to view status</div>
            )}
          </>
        )}
      </div>
    </div>
  );
};

export default DroneStatusWidget;
