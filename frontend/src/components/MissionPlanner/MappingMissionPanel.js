import React, { useState } from 'react';
import './MappingMissionPanel.css';

const MappingMissionPanel = ({ boundaryPoints, settings, onSettingsChange }) => {
  const [showAdvanced, setShowAdvanced] = useState(false);

  const handleSettingChange = (e) => {
    const { name, value, type, checked } = e.target;
    onSettingsChange({
      ...settings,
      [name]: type === 'checkbox' ? checked : type === 'number' ? parseFloat(value) : value
    });
  };

  return (
    <div className="mapping-mission-panel">
      <h3>Mapping Mission</h3>
      
      {boundaryPoints.length < 3 ? (
        <div className="boundary-instructions">
          <p>Define the area to map by adding at least 3 boundary points on the map.</p>
          <p>Current points: {boundaryPoints.length}/3</p>
        </div>
      ) : (
        <>
          <div className="form-group">
            <label htmlFor="areaMode">Area Mode</label>
            <select
              id="areaMode"
              name="areaMode"
              value={settings.areaMode}
              onChange={handleSettingChange}
            >
              <option value="polygon">Polygon</option>
              <option value="rectangle">Rectangle</option>
              <option value="corridor">Corridor</option>
            </select>
          </div>
          
          <div className="form-group">
            <label htmlFor="flightAltitude">Flight Altitude (m)</label>
            <input
              type="number"
              id="flightAltitude"
              name="flightAltitude"
              value={settings.flightAltitude}
              onChange={handleSettingChange}
              min="10"
              max="500"
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="flightSpeed">Flight Speed (m/s)</label>
            <input
              type="number"
              id="flightSpeed"
              name="flightSpeed"
              value={settings.flightSpeed}
              onChange={handleSettingChange}
              min="0.1"
              max="20"
              step="0.1"
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="gimbalPitch">Gimbal Pitch (°)</label>
            <input
              type="number"
              id="gimbalPitch"
              name="gimbalPitch"
              value={settings.gimbalPitch}
              onChange={handleSettingChange}
              min="-90"
              max="30"
            />
            <div className="gimbal-presets">
              <button 
                className={`preset-btn ${settings.gimbalPitch === -90 ? 'active' : ''}`}
                onClick={() => onSettingsChange({ ...settings, gimbalPitch: -90 })}
              >
                Nadir (-90°)
              </button>
              <button 
                className={`preset-btn ${settings.gimbalPitch === -60 ? 'active' : ''}`}
                onClick={() => onSettingsChange({ ...settings, gimbalPitch: -60 })}
              >
                Oblique (-60°)
              </button>
              <button 
                className={`preset-btn ${settings.gimbalPitch === -45 ? 'active' : ''}`}
                onClick={() => onSettingsChange({ ...settings, gimbalPitch: -45 })}
              >
                45° (-45°)
              </button>
            </div>
          </div>
          
          <div className="form-group">
            <label htmlFor="captureMode">Capture Mode</label>
            <select
              id="captureMode"
              name="captureMode"
              value={settings.captureMode}
              onChange={handleSettingChange}
            >
              <option value="overlap">Overlap</option>
              <option value="distance">Distance</option>
            </select>
          </div>
          
          {settings.captureMode === 'overlap' && (
            <>
              <div className="form-group">
                <label htmlFor="frontOverlap">Front Overlap (%)</label>
                <input
                  type="number"
                  id="frontOverlap"
                  name="frontOverlap"
                  value={settings.frontOverlap}
                  onChange={handleSettingChange}
                  min="10"
                  max="95"
                />
                <div className="range-slider">
                  <input
                    type="range"
                    min="10"
                    max="95"
                    value={settings.frontOverlap}
                    onChange={handleSettingChange}
                    name="frontOverlap"
                  />
                </div>
              </div>
              
              <div className="form-group">
                <label htmlFor="sideOverlap">Side Overlap (%)</label>
                <input
                  type="number"
                  id="sideOverlap"
                  name="sideOverlap"
                  value={settings.sideOverlap}
                  onChange={handleSettingChange}
                  min="10"
                  max="95"
                />
                <div className="range-slider">
                  <input
                    type="range"
                    min="10"
                    max="95"
                    value={settings.sideOverlap}
                    onChange={handleSettingChange}
                    name="sideOverlap"
                  />
                </div>
              </div>
            </>
          )}
          
          {settings.captureMode === 'distance' && (
            <div className="form-group">
              <label htmlFor="captureDistance">Capture Distance (m)</label>
              <input
                type="number"
                id="captureDistance"
                name="captureDistance"
                value={settings.captureDistance || 5}
                onChange={handleSettingChange}
                min="1"
                max="100"
              />
            </div>
          )}
          
          <div className="form-group">
            <label htmlFor="gridAngle">Grid Angle (°)</label>
            <input
              type="number"
              id="gridAngle"
              name="gridAngle"
              value={settings.gridAngle}
              onChange={handleSettingChange}
              min="0"
              max="359"
            />
            <div className="angle-presets">
              <button 
                className={`preset-btn ${settings.gridAngle === 0 ? 'active' : ''}`}
                onClick={() => onSettingsChange({ ...settings, gridAngle: 0 })}
              >
                0°
              </button>
              <button 
                className={`preset-btn ${settings.gridAngle === 90 ? 'active' : ''}`}
                onClick={() => onSettingsChange({ ...settings, gridAngle: 90 })}
              >
                90°
              </button>
              <button 
                className={`preset-btn ${settings.gridAngle === 45 ? 'active' : ''}`}
                onClick={() => onSettingsChange({ ...settings, gridAngle: 45 })}
              >
                45°
              </button>
            </div>
          </div>
          
          <div className="form-group checkbox-group">
            <label>
              <input
                type="checkbox"
                name="useCrosshatch"
                checked={settings.useCrosshatch}
                onChange={handleSettingChange}
              />
              Use Crosshatch Pattern
            </label>
          </div>
          
          {settings.useCrosshatch && (
            <div className="form-group">
              <label htmlFor="crosshatchAngle">Crosshatch Angle (°)</label>
              <input
                type="number"
                id="crosshatchAngle"
                name="crosshatchAngle"
                value={settings.crosshatchAngle}
                onChange={handleSettingChange}
                min="10"
                max="170"
              />
            </div>
          )}
          
          <button 
            className="btn btn-secondary toggle-advanced"
            onClick={() => setShowAdvanced(!showAdvanced)}
          >
            {showAdvanced ? 'Hide Advanced Settings' : 'Show Advanced Settings'}
          </button>
          
          {showAdvanced && (
            <div className="advanced-settings">
              <div className="form-group">
                <label htmlFor="cameraTrigger">Camera Trigger</label>
                <select
                  id="cameraTrigger"
                  name="cameraTrigger"
                  value={settings.cameraTrigger}
                  onChange={handleSettingChange}
                >
                  <option value="auto">Auto</option>
                  <option value="manual">Manual</option>
                  <option value="distance">Distance</option>
                  <option value="time">Time</option>
                </select>
              </div>
              
              <div className="form-group">
                <label htmlFor="margin">Margin (m)</label>
                <input
                  type="number"
                  id="margin"
                  name="margin"
                  value={settings.margin}
                  onChange={handleSettingChange}
                  min="0"
                  max="100"
                />
              </div>
              
              <div className="form-group checkbox-group">
                <label>
                  <input
                    type="checkbox"
                    name="useTerrainFollowing"
                    checked={settings.useTerrainFollowing}
                    onChange={handleSettingChange}
                  />
                  Use Terrain Following
                </label>
              </div>
              
              {settings.useTerrainFollowing && (
                <div className="form-group">
                  <label htmlFor="terrainFollowingMode">Terrain Following Mode</label>
                  <select
                    id="terrainFollowingMode"
                    name="terrainFollowingMode"
                    value={settings.terrainFollowingMode || 'agl'}
                    onChange={handleSettingChange}
                  >
                    <option value="agl">Above Ground Level (AGL)</option>
                    <option value="terrain_follow">Terrain Follow</option>
                  </select>
                </div>
              )}
            </div>
          )}
        </>
      )}
    </div>
  );
};

export default MappingMissionPanel;
