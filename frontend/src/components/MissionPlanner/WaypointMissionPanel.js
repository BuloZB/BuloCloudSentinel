import React, { useState } from 'react';
import './WaypointMissionPanel.css';

const WaypointMissionPanel = ({ waypoints, settings, onSettingsChange, onWaypointChange, onDeleteWaypoint }) => {
  const [expandedWaypoint, setExpandedWaypoint] = useState(null);
  const [showAdvanced, setShowAdvanced] = useState(false);

  const handleSettingChange = (e) => {
    const { name, value, type, checked } = e.target;
    onSettingsChange({
      ...settings,
      [name]: type === 'checkbox' ? checked : type === 'number' ? parseFloat(value) : value
    });
  };

  const handleWaypointChange = (index, field, value) => {
    const waypoint = { ...waypoints[index] };
    
    if (field.includes('.')) {
      const [parent, child] = field.split('.');
      waypoint[parent] = {
        ...waypoint[parent],
        [child]: value
      };
    } else {
      waypoint[field] = value;
    }
    
    onWaypointChange(index, waypoint);
  };

  const toggleWaypointExpand = (index) => {
    setExpandedWaypoint(expandedWaypoint === index ? null : index);
  };

  const addCameraAction = (waypointIndex) => {
    const waypoint = { ...waypoints[waypointIndex] };
    waypoint.actions = [
      ...waypoint.actions,
      {
        action: 'take_photo',
        trigger: 'on_arrival'
      }
    ];
    onWaypointChange(waypointIndex, waypoint);
  };

  const removeCameraAction = (waypointIndex, actionIndex) => {
    const waypoint = { ...waypoints[waypointIndex] };
    waypoint.actions = waypoint.actions.filter((_, i) => i !== actionIndex);
    onWaypointChange(waypointIndex, waypoint);
  };

  const handleCameraActionChange = (waypointIndex, actionIndex, field, value) => {
    const waypoint = { ...waypoints[waypointIndex] };
    waypoint.actions = [...waypoint.actions];
    
    if (field.includes('.')) {
      const [parent, child] = field.split('.');
      waypoint.actions[actionIndex] = {
        ...waypoint.actions[actionIndex],
        [parent]: {
          ...waypoint.actions[actionIndex][parent],
          [child]: value
        }
      };
    } else {
      waypoint.actions[actionIndex] = {
        ...waypoint.actions[actionIndex],
        [field]: value
      };
    }
    
    onWaypointChange(waypointIndex, waypoint);
  };

  return (
    <div className="waypoint-mission-panel">
      <h3>Waypoint Mission</h3>
      
      <div className="form-group">
        <label htmlFor="defaultAltitude">Default Altitude (m)</label>
        <input
          type="number"
          id="defaultAltitude"
          name="defaultAltitude"
          value={settings.defaultAltitude}
          onChange={handleSettingChange}
          min="0"
          max="500"
        />
      </div>
      
      <div className="form-group">
        <label htmlFor="speed">Default Speed (m/s)</label>
        <input
          type="number"
          id="speed"
          name="speed"
          value={settings.speed}
          onChange={handleSettingChange}
          min="0.1"
          max="20"
          step="0.1"
        />
      </div>
      
      <button 
        className="btn btn-secondary toggle-advanced"
        onClick={() => setShowAdvanced(!showAdvanced)}
      >
        {showAdvanced ? 'Hide Advanced Settings' : 'Show Advanced Settings'}
      </button>
      
      {showAdvanced && (
        <div className="advanced-settings">
          <div className="form-group">
            <label htmlFor="finishAction">Finish Action</label>
            <select
              id="finishAction"
              name="finishAction"
              value={settings.finishAction || 'return_home'}
              onChange={handleSettingChange}
            >
              <option value="return_home">Return to Home</option>
              <option value="hover">Hover in Place</option>
              <option value="land">Land at Last Waypoint</option>
            </select>
          </div>
          
          <div className="form-group">
            <label htmlFor="headingMode">Heading Mode</label>
            <select
              id="headingMode"
              name="headingMode"
              value={settings.headingMode || 'auto'}
              onChange={handleSettingChange}
            >
              <option value="auto">Auto (Follow Path)</option>
              <option value="manual">Manual (Set at Each Waypoint)</option>
              <option value="fixed">Fixed (Maintain Initial)</option>
            </select>
          </div>
          
          <div className="form-group">
            <label>
              <input
                type="checkbox"
                name="pathMode"
                checked={settings.pathMode === 'curved'}
                onChange={(e) => onSettingsChange({
                  ...settings,
                  pathMode: e.target.checked ? 'curved' : 'straight'
                })}
              />
              Use Curved Path
            </label>
          </div>
        </div>
      )}
      
      <h4>Waypoints ({waypoints.length})</h4>
      
      {waypoints.length === 0 ? (
        <p className="no-waypoints">Click on the map to add waypoints</p>
      ) : (
        <div className="waypoint-list">
          {waypoints.map((waypoint, index) => (
            <div key={index} className="waypoint-item">
              <div 
                className="waypoint-header"
                onClick={() => toggleWaypointExpand(index)}
              >
                <div className="waypoint-title">
                  <span className="waypoint-number">{index + 1}</span>
                  <span className="waypoint-coords">
                    {waypoint.position.latitude.toFixed(6)}, {waypoint.position.longitude.toFixed(6)}
                  </span>
                </div>
                <div className="waypoint-actions">
                  <button 
                    className="btn btn-small"
                    onClick={(e) => {
                      e.stopPropagation();
                      onDeleteWaypoint(index);
                    }}
                  >
                    Delete
                  </button>
                  <span className={`expand-icon ${expandedWaypoint === index ? 'expanded' : ''}`}>
                    ▼
                  </span>
                </div>
              </div>
              
              {expandedWaypoint === index && (
                <div className="waypoint-details">
                  <div className="form-group">
                    <label>Altitude (m)</label>
                    <input
                      type="number"
                      value={waypoint.position.altitude}
                      onChange={(e) => handleWaypointChange(index, 'position.altitude', parseFloat(e.target.value))}
                      min="0"
                      max="500"
                    />
                  </div>
                  
                  <div className="form-group">
                    <label>Speed (m/s)</label>
                    <input
                      type="number"
                      value={waypoint.speed}
                      onChange={(e) => handleWaypointChange(index, 'speed', parseFloat(e.target.value))}
                      min="0.1"
                      max="20"
                      step="0.1"
                    />
                  </div>
                  
                  {settings.headingMode === 'manual' && (
                    <div className="form-group">
                      <label>Heading (°)</label>
                      <input
                        type="number"
                        value={waypoint.heading || 0}
                        onChange={(e) => handleWaypointChange(index, 'heading', parseFloat(e.target.value))}
                        min="0"
                        max="359"
                      />
                    </div>
                  )}
                  
                  <div className="form-group">
                    <label>
                      <input
                        type="checkbox"
                        checked={waypoint.loiterTime > 0}
                        onChange={(e) => handleWaypointChange(index, 'loiterTime', e.target.checked ? 5 : 0)}
                      />
                      Loiter at Waypoint
                    </label>
                    
                    {waypoint.loiterTime > 0 && (
                      <div className="loiter-time">
                        <label>Loiter Time (s)</label>
                        <input
                          type="number"
                          value={waypoint.loiterTime}
                          onChange={(e) => handleWaypointChange(index, 'loiterTime', parseFloat(e.target.value))}
                          min="1"
                          max="300"
                        />
                      </div>
                    )}
                  </div>
                  
                  <h5>Camera Actions</h5>
                  
                  {waypoint.actions.length === 0 ? (
                    <p className="no-actions">No camera actions</p>
                  ) : (
                    <div className="camera-actions-list">
                      {waypoint.actions.map((action, actionIndex) => (
                        <div key={actionIndex} className="camera-action-item">
                          <div className="form-group">
                            <label>Action</label>
                            <select
                              value={action.action}
                              onChange={(e) => handleCameraActionChange(index, actionIndex, 'action', e.target.value)}
                            >
                              <option value="none">None</option>
                              <option value="take_photo">Take Photo</option>
                              <option value="start_recording">Start Recording</option>
                              <option value="stop_recording">Stop Recording</option>
                              <option value="tilt_camera">Tilt Camera</option>
                              <option value="point_camera">Point Camera</option>
                              <option value="zoom">Zoom</option>
                            </select>
                          </div>
                          
                          <div className="form-group">
                            <label>Trigger</label>
                            <select
                              value={action.trigger}
                              onChange={(e) => handleCameraActionChange(index, actionIndex, 'trigger', e.target.value)}
                            >
                              <option value="on_arrival">On Arrival</option>
                              <option value="during_movement">During Movement</option>
                              <option value="timed">Timed</option>
                              <option value="distance">Distance</option>
                            </select>
                          </div>
                          
                          {(action.action === 'tilt_camera' || action.action === 'point_camera') && (
                            <div className="form-group">
                              <label>Gimbal Pitch (°)</label>
                              <input
                                type="number"
                                value={action.gimbalPitch || 0}
                                onChange={(e) => handleCameraActionChange(index, actionIndex, 'gimbalPitch', parseFloat(e.target.value))}
                                min="-90"
                                max="30"
                              />
                            </div>
                          )}
                          
                          {action.action === 'zoom' && (
                            <div className="form-group">
                              <label>Zoom Level</label>
                              <input
                                type="number"
                                value={action.zoomLevel || 1}
                                onChange={(e) => handleCameraActionChange(index, actionIndex, 'zoomLevel', parseFloat(e.target.value))}
                                min="1"
                                max="10"
                                step="0.1"
                              />
                            </div>
                          )}
                          
                          <button 
                            className="btn btn-small btn-danger"
                            onClick={() => removeCameraAction(index, actionIndex)}
                          >
                            Remove Action
                          </button>
                        </div>
                      ))}
                    </div>
                  )}
                  
                  <button 
                    className="btn btn-small"
                    onClick={() => addCameraAction(index)}
                  >
                    Add Camera Action
                  </button>
                </div>
              )}
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default WaypointMissionPanel;
