import React from 'react';
import './TelemetryDisplay.css';

const TelemetryDisplay = ({ telemetry }) => {
  if (!telemetry) {
    return (
      <div className="telemetry-display">
        <div className="telemetry-waiting">Waiting for telemetry data...</div>
      </div>
    );
  }
  
  const formatValue = (value, precision = 2, unit = '') => {
    if (value === undefined || value === null) {
      return 'N/A';
    }
    return `${parseFloat(value).toFixed(precision)}${unit}`;
  };
  
  const formatTimestamp = (timestamp) => {
    if (!timestamp) return 'N/A';
    
    try {
      const date = new Date(timestamp);
      return date.toLocaleTimeString();
    } catch (e) {
      return timestamp;
    }
  };
  
  const getBatteryColor = (percentage) => {
    if (percentage === undefined || percentage === null) return '#ccc';
    if (percentage <= 20) return '#e74c3c';
    if (percentage <= 40) return '#f39c12';
    return '#2ecc71';
  };
  
  return (
    <div className="telemetry-display">
      <div className="telemetry-timestamp">
        Last update: {formatTimestamp(telemetry.timestamp)}
      </div>
      
      <div className="telemetry-section">
        <h4>Position</h4>
        <div className="telemetry-grid">
          <div className="telemetry-label">Latitude:</div>
          <div className="telemetry-value">
            {telemetry.position ? formatValue(telemetry.position.latitude, 6, '°') : 'N/A'}
          </div>
          
          <div className="telemetry-label">Longitude:</div>
          <div className="telemetry-value">
            {telemetry.position ? formatValue(telemetry.position.longitude, 6, '°') : 'N/A'}
          </div>
          
          <div className="telemetry-label">Altitude:</div>
          <div className="telemetry-value">
            {telemetry.position ? formatValue(telemetry.position.altitude, 1, 'm') : 'N/A'}
          </div>
          
          <div className="telemetry-label">Relative Alt:</div>
          <div className="telemetry-value">
            {telemetry.position && telemetry.position.relative_altitude ? 
              formatValue(telemetry.position.relative_altitude, 1, 'm') : 'N/A'}
          </div>
        </div>
      </div>
      
      <div className="telemetry-section">
        <h4>Attitude</h4>
        <div className="telemetry-grid">
          <div className="telemetry-label">Pitch:</div>
          <div className="telemetry-value">
            {telemetry.attitude ? formatValue(telemetry.attitude.pitch, 1, '°') : 'N/A'}
          </div>
          
          <div className="telemetry-label">Roll:</div>
          <div className="telemetry-value">
            {telemetry.attitude ? formatValue(telemetry.attitude.roll, 1, '°') : 'N/A'}
          </div>
          
          <div className="telemetry-label">Yaw:</div>
          <div className="telemetry-value">
            {telemetry.attitude ? formatValue(telemetry.attitude.yaw, 1, '°') : 'N/A'}
          </div>
        </div>
      </div>
      
      <div className="telemetry-section">
        <h4>Speed</h4>
        <div className="telemetry-grid">
          <div className="telemetry-label">Horizontal:</div>
          <div className="telemetry-value">
            {telemetry.speed ? formatValue(telemetry.speed.horizontal, 1, 'm/s') : 'N/A'}
          </div>
          
          <div className="telemetry-label">Vertical:</div>
          <div className="telemetry-value">
            {telemetry.speed ? formatValue(telemetry.speed.vertical, 1, 'm/s') : 'N/A'}
          </div>
        </div>
      </div>
      
      <div className="telemetry-section">
        <h4>Battery</h4>
        <div className="battery-display">
          <div className="battery-bar-container">
            <div 
              className="battery-bar-fill"
              style={{ 
                width: `${telemetry.battery ? telemetry.battery.percentage : 0}%`,
                backgroundColor: getBatteryColor(telemetry.battery ? telemetry.battery.percentage : null)
              }}
            ></div>
          </div>
          <div className="battery-text">
            {telemetry.battery ? formatValue(telemetry.battery.percentage, 0, '%') : 'N/A'}
          </div>
        </div>
        <div className="telemetry-grid">
          <div className="telemetry-label">Voltage:</div>
          <div className="telemetry-value">
            {telemetry.battery ? formatValue(telemetry.battery.voltage, 2, 'V') : 'N/A'}
          </div>
          
          <div className="telemetry-label">Current:</div>
          <div className="telemetry-value">
            {telemetry.battery ? formatValue(telemetry.battery.current, 2, 'A') : 'N/A'}
          </div>
        </div>
      </div>
      
      <div className="telemetry-section">
        <h4>GPS</h4>
        <div className="telemetry-grid">
          <div className="telemetry-label">Fix Type:</div>
          <div className="telemetry-value">
            {telemetry.gps ? telemetry.gps.fix_type : 'N/A'}
          </div>
          
          <div className="telemetry-label">Satellites:</div>
          <div className="telemetry-value">
            {telemetry.gps ? telemetry.gps.satellites_visible : 'N/A'}
          </div>
          
          <div className="telemetry-label">HDOP:</div>
          <div className="telemetry-value">
            {telemetry.gps ? formatValue(telemetry.gps.hdop, 1) : 'N/A'}
          </div>
        </div>
      </div>
      
      {telemetry.gimbal && (
        <div className="telemetry-section">
          <h4>Gimbal</h4>
          <div className="telemetry-grid">
            <div className="telemetry-label">Pitch:</div>
            <div className="telemetry-value">
              {formatValue(telemetry.gimbal.pitch, 1, '°')}
            </div>
            
            <div className="telemetry-label">Roll:</div>
            <div className="telemetry-value">
              {formatValue(telemetry.gimbal.roll, 1, '°')}
            </div>
            
            <div className="telemetry-label">Yaw:</div>
            <div className="telemetry-value">
              {formatValue(telemetry.gimbal.yaw, 1, '°')}
            </div>
          </div>
        </div>
      )}
      
      {telemetry.camera && (
        <div className="telemetry-section">
          <h4>Camera</h4>
          <div className="telemetry-grid">
            <div className="telemetry-label">Mode:</div>
            <div className="telemetry-value">
              {telemetry.camera.mode || 'N/A'}
            </div>
            
            <div className="telemetry-label">Recording:</div>
            <div className="telemetry-value">
              {telemetry.camera.recording ? 'Yes' : 'No'}
            </div>
            
            <div className="telemetry-label">Photos:</div>
            <div className="telemetry-value">
              {telemetry.camera.photos_taken || '0'}
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default TelemetryDisplay;
