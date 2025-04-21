import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { Pie } from 'react-chartjs-2';
import { Chart as ChartJS, ArcElement, Tooltip, Legend } from 'chart.js';
import './EnergyPredictionPanel.css';

// Register ChartJS components
ChartJS.register(ArcElement, Tooltip, Legend);

const EnergyPredictionPanel = () => {
  const [drones, setDrones] = useState([]);
  const [missions, setMissions] = useState([]);
  const [selectedDrone, setSelectedDrone] = useState(null);
  const [selectedMission, setSelectedMission] = useState(null);
  const [customWaypoints, setCustomWaypoints] = useState([]);
  const [predictionResult, setPredictionResult] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [isPredicting, setIsPredicting] = useState(false);
  const [error, setError] = useState(null);
  const [predictionParams, setPredictionParams] = useState({
    payload_weight: 0.0,
    model_type: 'linear'
  });
  const [weatherData, setWeatherData] = useState(null);
  
  // Load drones and missions on component mount
  useEffect(() => {
    fetchDrones();
    fetchMissions();
  }, []);
  
  // Fetch weather data when a mission is selected
  useEffect(() => {
    if (selectedMission) {
      fetchWeatherForMission(selectedMission.id);
    }
  }, [selectedMission]);
  
  const fetchDrones = async () => {
    try {
      const response = await axios.get('/api/device-inventory/drones');
      setDrones(response.data);
      setIsLoading(false);
    } catch (err) {
      console.error('Error fetching drones:', err);
      setError('Failed to load drones');
      setIsLoading(false);
    }
  };
  
  const fetchMissions = async () => {
    try {
      const response = await axios.get('/api/mission-planner/missions');
      setMissions(response.data);
    } catch (err) {
      console.error('Error fetching missions:', err);
      setError('Failed to load missions');
    }
  };
  
  const fetchWeatherForMission = async (missionId) => {
    try {
      // This is a placeholder - in a real implementation, you would get the mission location
      // and fetch weather data for that location
      const response = await axios.get('/api/weather/current', {
        params: {
          mission_id: missionId
        }
      });
      
      setWeatherData(response.data);
      
      // Update prediction params with weather data
      setPredictionParams(prev => ({
        ...prev,
        wind_speed: response.data.wind_speed,
        wind_direction: response.data.wind_direction,
        temperature: response.data.temperature
      }));
    } catch (err) {
      console.error('Error fetching weather data:', err);
      // Don't set error, as weather data is optional
      setWeatherData(null);
    }
  };
  
  const handlePredictEnergy = async () => {
    if (!selectedDrone) {
      setError('Please select a drone');
      return;
    }
    
    if (!selectedMission && customWaypoints.length === 0) {
      setError('Please select a mission or add custom waypoints');
      return;
    }
    
    try {
      setIsPredicting(true);
      
      const requestData = {
        drone_id: selectedDrone.id,
        ...predictionParams
      };
      
      if (selectedMission) {
        requestData.mission_id = selectedMission.id;
      } else {
        requestData.waypoints = customWaypoints;
      }
      
      const response = await axios.post('/api/power/energy/predict', requestData);
      setPredictionResult(response.data);
      setIsPredicting(false);
    } catch (err) {
      console.error('Error predicting energy consumption:', err);
      setError('Failed to predict energy consumption');
      setIsPredicting(false);
    }
  };
  
  const handleAddWaypoint = () => {
    // This is a placeholder - in a real implementation, you would add a waypoint
    // based on user input or map selection
    const newWaypoint = {
      latitude: 47.6062,
      longitude: -122.3321,
      altitude: 50
    };
    
    setCustomWaypoints([...customWaypoints, newWaypoint]);
  };
  
  const handleRemoveWaypoint = (index) => {
    const newWaypoints = [...customWaypoints];
    newWaypoints.splice(index, 1);
    setCustomWaypoints(newWaypoints);
  };
  
  const renderEnergyFactorsChart = () => {
    if (!predictionResult || !predictionResult.factors) {
      return <div className="no-data">No prediction data available</div>;
    }
    
    // Filter out factors with very small contributions
    const significantFactors = Object.entries(predictionResult.factors)
      .filter(([_, value]) => value > 0.5)
      .sort((a, b) => b[1] - a[1]);
    
    const labels = significantFactors.map(([key, _]) => {
      // Convert key to readable label
      return key.charAt(0).toUpperCase() + key.slice(1).replace(/_/g, ' ');
    });
    
    const data = significantFactors.map(([_, value]) => value);
    
    // Generate colors
    const backgroundColors = [
      'rgba(255, 99, 132, 0.6)',
      'rgba(54, 162, 235, 0.6)',
      'rgba(255, 206, 86, 0.6)',
      'rgba(75, 192, 192, 0.6)',
      'rgba(153, 102, 255, 0.6)',
      'rgba(255, 159, 64, 0.6)',
      'rgba(199, 199, 199, 0.6)'
    ];
    
    const chartData = {
      labels,
      datasets: [
        {
          data,
          backgroundColor: backgroundColors.slice(0, data.length),
          borderWidth: 1
        }
      ]
    };
    
    const options = {
      responsive: true,
      maintainAspectRatio: false,
      plugins: {
        legend: {
          position: 'right'
        },
        title: {
          display: true,
          text: 'Energy Consumption Factors'
        }
      }
    };
    
    return <Pie data={chartData} options={options} />;
  };
  
  if (isLoading) {
    return <div className="loading">Loading energy prediction data...</div>;
  }
  
  return (
    <div className="energy-prediction-panel">
      <div className="energy-prediction-sidebar">
        <h2>Energy Prediction</h2>
        
        {error && (
          <div className="error-message">
            {error}
            <button onClick={() => setError(null)}>Dismiss</button>
          </div>
        )}
        
        <div className="prediction-form">
          <div className="form-section">
            <h3>Select Drone</h3>
            <div className="drone-list">
              {drones.length === 0 ? (
                <p className="empty-message">No drones available</p>
              ) : (
                drones.map(drone => (
                  <div
                    key={drone.id}
                    className={`drone-item ${selectedDrone && selectedDrone.id === drone.id ? 'selected' : ''}`}
                    onClick={() => setSelectedDrone(drone)}
                  >
                    <span className="drone-name">{drone.name}</span>
                    {drone.status && <span className={`drone-status ${drone.status}`}>{drone.status}</span>}
                  </div>
                ))
              )}
            </div>
          </div>
          
          <div className="form-section">
            <h3>Select Mission</h3>
            <div className="mission-list">
              {missions.length === 0 ? (
                <p className="empty-message">No missions available</p>
              ) : (
                missions.map(mission => (
                  <div
                    key={mission.id}
                    className={`mission-item ${selectedMission && selectedMission.id === mission.id ? 'selected' : ''}`}
                    onClick={() => {
                      setSelectedMission(mission);
                      setCustomWaypoints([]);
                    }}
                  >
                    <span className="mission-name">{mission.name}</span>
                    <span className="mission-type">{mission.mission_type}</span>
                  </div>
                ))
              )}
              
              <div 
                className={`mission-item custom ${!selectedMission ? 'selected' : ''}`}
                onClick={() => setSelectedMission(null)}
              >
                <span className="mission-name">Custom Waypoints</span>
                <span className="waypoint-count">{customWaypoints.length} points</span>
              </div>
            </div>
            
            {!selectedMission && (
              <div className="custom-waypoints">
                <div className="waypoints-header">
                  <h4>Custom Waypoints</h4>
                  <button 
                    className="add-waypoint-button"
                    onClick={handleAddWaypoint}
                  >
                    Add Waypoint
                  </button>
                </div>
                
                {customWaypoints.length === 0 ? (
                  <p className="empty-message">No waypoints added</p>
                ) : (
                  <div className="waypoint-list">
                    {customWaypoints.map((waypoint, index) => (
                      <div key={index} className="waypoint-item">
                        <span className="waypoint-coords">
                          {waypoint.latitude.toFixed(6)}, {waypoint.longitude.toFixed(6)}, {waypoint.altitude}m
                        </span>
                        <button 
                          className="remove-waypoint-button"
                          onClick={() => handleRemoveWaypoint(index)}
                        >
                          &times;
                        </button>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            )}
          </div>
          
          <div className="form-section">
            <h3>Prediction Parameters</h3>
            
            <div className="form-group">
              <label>Payload Weight (kg)</label>
              <input
                type="number"
                value={predictionParams.payload_weight}
                onChange={(e) => setPredictionParams({...predictionParams, payload_weight: parseFloat(e.target.value)})}
                min="0"
                max="10"
                step="0.1"
              />
            </div>
            
            <div className="form-group">
              <label>Prediction Model</label>
              <select
                value={predictionParams.model_type}
                onChange={(e) => setPredictionParams({...predictionParams, model_type: e.target.value})}
              >
                <option value="linear">Linear Model</option>
                <option value="polynomial">Polynomial Model</option>
                <option value="neural_network">Neural Network</option>
                <option value="ensemble">Ensemble Model</option>
              </select>
            </div>
            
            {weatherData ? (
              <div className="weather-data">
                <h4>Weather Conditions</h4>
                <div className="weather-info">
                  <div className="weather-item">
                    <span className="weather-label">Temperature</span>
                    <span className="weather-value">{weatherData.temperature}°C</span>
                  </div>
                  
                  <div className="weather-item">
                    <span className="weather-label">Wind Speed</span>
                    <span className="weather-value">{weatherData.wind_speed} m/s</span>
                  </div>
                  
                  <div className="weather-item">
                    <span className="weather-label">Wind Direction</span>
                    <span className="weather-value">{weatherData.wind_direction}°</span>
                  </div>
                  
                  <div className="weather-item">
                    <span className="weather-label">Conditions</span>
                    <span className="weather-value">{weatherData.condition}</span>
                  </div>
                </div>
              </div>
            ) : (
              <div className="weather-data">
                <h4>Weather Conditions</h4>
                <p className="empty-message">No weather data available</p>
                
                <div className="form-group">
                  <label>Wind Speed (m/s)</label>
                  <input
                    type="number"
                    value={predictionParams.wind_speed || 0}
                    onChange={(e) => setPredictionParams({...predictionParams, wind_speed: parseFloat(e.target.value)})}
                    min="0"
                    max="30"
                    step="0.1"
                  />
                </div>
                
                <div className="form-group">
                  <label>Wind Direction (degrees)</label>
                  <input
                    type="number"
                    value={predictionParams.wind_direction || 0}
                    onChange={(e) => setPredictionParams({...predictionParams, wind_direction: parseFloat(e.target.value)})}
                    min="0"
                    max="359"
                    step="1"
                  />
                </div>
                
                <div className="form-group">
                  <label>Temperature (°C)</label>
                  <input
                    type="number"
                    value={predictionParams.temperature || 20}
                    onChange={(e) => setPredictionParams({...predictionParams, temperature: parseFloat(e.target.value)})}
                    min="-20"
                    max="50"
                    step="0.1"
                  />
                </div>
              </div>
            )}
          </div>
          
          <button 
            className="predict-button"
            onClick={handlePredictEnergy}
            disabled={isPredicting || (!selectedMission && customWaypoints.length === 0) || !selectedDrone}
          >
            {isPredicting ? 'Predicting...' : 'Predict Energy Consumption'}
          </button>
        </div>
      </div>
      
      <div className="energy-prediction-content">
        {predictionResult ? (
          <div className="prediction-results">
            <div className="results-header">
              <h3>Energy Prediction Results</h3>
              <div className={`feasibility-badge ${predictionResult.is_feasible ? 'feasible' : 'not-feasible'}`}>
                {predictionResult.is_feasible ? 'Mission Feasible' : 'Mission Not Feasible'}
              </div>
            </div>
            
            <div className="results-summary">
              <div className="summary-item">
                <span className="summary-label">Estimated Consumption</span>
                <span className="summary-value">{predictionResult.estimated_consumption.toFixed(1)}%</span>
              </div>
              
              <div className="summary-item">
                <span className="summary-label">Estimated Duration</span>
                <span className="summary-value">
                  {Math.floor(predictionResult.estimated_duration / 60)}m {predictionResult.estimated_duration % 60}s
                </span>
              </div>
              
              <div className="summary-item">
                <span className="summary-label">Estimated Range</span>
                <span className="summary-value">{predictionResult.estimated_range.toFixed(1)} km</span>
              </div>
              
              <div className="summary-item">
                <span className="summary-label">Confidence</span>
                <span className="summary-value">{(predictionResult.confidence * 100).toFixed(0)}%</span>
              </div>
              
              <div className="summary-item">
                <span className="summary-label">Margin of Error</span>
                <span className="summary-value">±{predictionResult.margin_of_error.toFixed(1)}%</span>
              </div>
              
              <div className="summary-item">
                <span className="summary-label">Model Type</span>
                <span className="summary-value">{predictionResult.model_type}</span>
              </div>
            </div>
            
            <div className="results-details">
              <div className="factors-chart">
                {renderEnergyFactorsChart()}
              </div>
              
              <div className="recommendations">
                <h4>Recommendations</h4>
                {predictionResult.recommendations && predictionResult.recommendations.length > 0 ? (
                  <ul className="recommendation-list">
                    {predictionResult.recommendations.map((recommendation, index) => (
                      <li key={index}>{recommendation}</li>
                    ))}
                  </ul>
                ) : (
                  <p className="empty-message">No recommendations available</p>
                )}
              </div>
            </div>
          </div>
        ) : (
          <div className="no-prediction">
            <p>Select a drone and mission, then click "Predict Energy Consumption"</p>
          </div>
        )}
      </div>
    </div>
  );
};

export default EnergyPredictionPanel;
