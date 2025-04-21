import React, { useState } from 'react';
import './MissionControls.css';

const MissionControls = ({ status, onCommand }) => {
  const [gimbalPitch, setGimbalPitch] = useState(0);
  const [speed, setSpeed] = useState(5);
  const [showAdvanced, setShowAdvanced] = useState(false);
  
  const isPaused = status === 'paused';
  const isActive = status === 'in_progress' || status === 'paused';
  
  const handlePauseResume = () => {
    onCommand(isPaused ? 'resume' : 'pause');
  };
  
  const handleAbort = () => {
    if (window.confirm('Are you sure you want to abort the mission?')) {
      onCommand('abort');
    }
  };
  
  const handleReturnHome = () => {
    if (window.confirm('Are you sure you want to return to home?')) {
      onCommand('return_home');
    }
  };
  
  const handleSkipWaypoint = () => {
    if (window.confirm('Are you sure you want to skip the current waypoint?')) {
      onCommand('skip_waypoint');
    }
  };
  
  const handleTakePhoto = () => {
    onCommand('take_photo');
  };
  
  const handleStartRecording = () => {
    onCommand('start_recording');
  };
  
  const handleStopRecording = () => {
    onCommand('stop_recording');
  };
  
  const handleSetGimbal = () => {
    onCommand('set_gimbal', { pitch: gimbalPitch });
  };
  
  const handleSetSpeed = () => {
    onCommand('set_speed', { speed });
  };
  
  return (
    <div className="mission-controls">
      <div className="control-section">
        <h4>Mission Control</h4>
        <div className="control-buttons">
          <button 
            className={`control-btn ${isPaused ? 'resume' : 'pause'}`}
            onClick={handlePauseResume}
            disabled={!isActive}
          >
            {isPaused ? 'Resume' : 'Pause'}
          </button>
          
          <button 
            className="control-btn abort"
            onClick={handleAbort}
            disabled={!isActive}
          >
            Abort
          </button>
          
          <button 
            className="control-btn return-home"
            onClick={handleReturnHome}
            disabled={!isActive}
          >
            Return Home
          </button>
          
          <button 
            className="control-btn skip"
            onClick={handleSkipWaypoint}
            disabled={!isActive}
          >
            Skip Waypoint
          </button>
        </div>
      </div>
      
      <div className="control-section">
        <h4>Camera Control</h4>
        <div className="control-buttons">
          <button 
            className="control-btn photo"
            onClick={handleTakePhoto}
            disabled={!isActive}
          >
            Take Photo
          </button>
          
          <button 
            className="control-btn record-start"
            onClick={handleStartRecording}
            disabled={!isActive}
          >
            Start Recording
          </button>
          
          <button 
            className="control-btn record-stop"
            onClick={handleStopRecording}
            disabled={!isActive}
          >
            Stop Recording
          </button>
        </div>
      </div>
      
      <button 
        className="toggle-advanced"
        onClick={() => setShowAdvanced(!showAdvanced)}
      >
        {showAdvanced ? 'Hide Advanced Controls' : 'Show Advanced Controls'}
      </button>
      
      {showAdvanced && (
        <>
          <div className="control-section">
            <h4>Gimbal Control</h4>
            <div className="slider-control">
              <label>Pitch: {gimbalPitch}°</label>
              <div className="slider-with-buttons">
                <button 
                  className="slider-btn"
                  onClick={() => setGimbalPitch(Math.max(gimbalPitch - 5, -90))}
                  disabled={!isActive}
                >
                  -
                </button>
                <input
                  type="range"
                  min="-90"
                  max="30"
                  value={gimbalPitch}
                  onChange={(e) => setGimbalPitch(parseInt(e.target.value))}
                  disabled={!isActive}
                />
                <button 
                  className="slider-btn"
                  onClick={() => setGimbalPitch(Math.min(gimbalPitch + 5, 30))}
                  disabled={!isActive}
                >
                  +
                </button>
              </div>
              <div className="preset-buttons">
                <button 
                  className="preset-btn"
                  onClick={() => setGimbalPitch(-90)}
                  disabled={!isActive}
                >
                  Nadir (-90°)
                </button>
                <button 
                  className="preset-btn"
                  onClick={() => setGimbalPitch(-45)}
                  disabled={!isActive}
                >
                  45° (-45°)
                </button>
                <button 
                  className="preset-btn"
                  onClick={() => setGimbalPitch(0)}
                  disabled={!isActive}
                >
                  Horizon (0°)
                </button>
              </div>
              <button 
                className="control-btn apply"
                onClick={handleSetGimbal}
                disabled={!isActive}
              >
                Apply Gimbal
              </button>
            </div>
          </div>
          
          <div className="control-section">
            <h4>Speed Control</h4>
            <div className="slider-control">
              <label>Speed: {speed} m/s</label>
              <div className="slider-with-buttons">
                <button 
                  className="slider-btn"
                  onClick={() => setSpeed(Math.max(speed - 0.5, 0.5))}
                  disabled={!isActive}
                >
                  -
                </button>
                <input
                  type="range"
                  min="0.5"
                  max="15"
                  step="0.5"
                  value={speed}
                  onChange={(e) => setSpeed(parseFloat(e.target.value))}
                  disabled={!isActive}
                />
                <button 
                  className="slider-btn"
                  onClick={() => setSpeed(Math.min(speed + 0.5, 15))}
                  disabled={!isActive}
                >
                  +
                </button>
              </div>
              <div className="preset-buttons">
                <button 
                  className="preset-btn"
                  onClick={() => setSpeed(2)}
                  disabled={!isActive}
                >
                  Slow (2 m/s)
                </button>
                <button 
                  className="preset-btn"
                  onClick={() => setSpeed(5)}
                  disabled={!isActive}
                >
                  Normal (5 m/s)
                </button>
                <button 
                  className="preset-btn"
                  onClick={() => setSpeed(10)}
                  disabled={!isActive}
                >
                  Fast (10 m/s)
                </button>
              </div>
              <button 
                className="control-btn apply"
                onClick={handleSetSpeed}
                disabled={!isActive}
              >
                Apply Speed
              </button>
            </div>
          </div>
        </>
      )}
    </div>
  );
};

export default MissionControls;
