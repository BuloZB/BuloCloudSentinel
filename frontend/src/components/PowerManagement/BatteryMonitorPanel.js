import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import { Line } from 'react-chartjs-2';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  Filler
} from 'chart.js';
import './BatteryMonitorPanel.css';

// Register ChartJS components
ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  Filler
);

const BatteryMonitorPanel = () => {
  const [drones, setDrones] = useState([]);
  const [selectedDrone, setSelectedDrone] = useState(null);
  const [batteryMetrics, setBatteryMetrics] = useState(null);
  const [batteryHistory, setBatteryHistory] = useState([]);
  const [thresholds, setThresholds] = useState(null);
  const [isEditingThresholds, setIsEditingThresholds] = useState(false);
  const [newThresholds, setNewThresholds] = useState({
    warning_threshold: 30.0,
    critical_threshold: 15.0,
    temperature_max: 60.0,
    voltage_min: null,
    auto_return_threshold: 20.0
  });
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  
  const batteryChartRef = useRef(null);
  const temperatureChartRef = useRef(null);
  const updateInterval = useRef(null);
  
  // Load drones on component mount
  useEffect(() => {
    fetchDrones();
    
    // Clean up on unmount
    return () => {
      if (updateInterval.current) {
        clearInterval(updateInterval.current);
      }
    };
  }, []);
  
  // Set up polling when a drone is selected
  useEffect(() => {
    if (selectedDrone) {
      // Fetch initial data
      fetchBatteryMetrics(selectedDrone.id);
      fetchBatteryHistory(selectedDrone.id);
      fetchBatteryThresholds(selectedDrone.id);
      
      // Set up polling
      updateInterval.current = setInterval(() => {
        fetchBatteryMetrics(selectedDrone.id);
      }, 5000);
    } else {
      // Clear interval if no drone is selected
      if (updateInterval.current) {
        clearInterval(updateInterval.current);
        updateInterval.current = null;
      }
    }
    
    // Clean up on change
    return () => {
      if (updateInterval.current) {
        clearInterval(updateInterval.current);
      }
    };
  }, [selectedDrone]);
  
  const fetchDrones = async () => {
    try {
      setIsLoading(true);
      const response = await axios.get('/api/device-inventory/drones');
      setDrones(response.data);
      setIsLoading(false);
    } catch (err) {
      console.error('Error fetching drones:', err);
      setError('Failed to load drones');
      setIsLoading(false);
    }
  };
  
  const fetchBatteryMetrics = async (droneId) => {
    try {
      const response = await axios.get(`/api/power/battery/${droneId}/current`);
      setBatteryMetrics(response.data);
    } catch (err) {
      console.error('Error fetching battery metrics:', err);
      setError('Failed to load battery metrics');
    }
  };
  
  const fetchBatteryHistory = async (droneId) => {
    try {
      const response = await axios.get(`/api/power/battery/${droneId}/history`);
      setBatteryHistory(response.data.data || []);
    } catch (err) {
      console.error('Error fetching battery history:', err);
      setError('Failed to load battery history');
    }
  };
  
  const fetchBatteryThresholds = async (droneId) => {
    try {
      const response = await axios.get(`/api/power/battery/thresholds/${droneId}`);
      setThresholds(response.data);
      setNewThresholds({
        warning_threshold: response.data.warning_threshold,
        critical_threshold: response.data.critical_threshold,
        temperature_max: response.data.temperature_max,
        voltage_min: response.data.voltage_min,
        auto_return_threshold: response.data.auto_return_threshold
      });
    } catch (err) {
      console.error('Error fetching battery thresholds:', err);
      setError('Failed to load battery thresholds');
    }
  };
  
  const handleSaveThresholds = async () => {
    try {
      await axios.post(`/api/power/battery/thresholds/${selectedDrone.id}`, newThresholds);
      fetchBatteryThresholds(selectedDrone.id);
      setIsEditingThresholds(false);
    } catch (err) {
      console.error('Error saving battery thresholds:', err);
      setError('Failed to save battery thresholds');
    }
  };
  
  const handleTriggerRTH = async () => {
    if (!selectedDrone) return;
    
    if (!window.confirm(`Are you sure you want to trigger Return-to-Home for ${selectedDrone.name}?`)) {
      return;
    }
    
    try {
      await axios.post(`/api/power/battery/trigger-rth/${selectedDrone.id}`, {
        reason: 'Manual RTH trigger from Battery Monitor'
      });
      alert('Return-to-Home triggered successfully');
    } catch (err) {
      console.error('Error triggering RTH:', err);
      setError('Failed to trigger Return-to-Home');
    }
  };
  
  const getBatteryStatusClass = (status) => {
    switch (status) {
      case 'critical':
        return 'battery-critical';
      case 'warning':
        return 'battery-warning';
      default:
        return 'battery-normal';
    }
  };
  
  const getBatteryLevelClass = (level) => {
    if (level <= (thresholds?.critical_threshold || 15)) {
      return 'battery-critical';
    } else if (level <= (thresholds?.warning_threshold || 30)) {
      return 'battery-warning';
    } else {
      return 'battery-normal';
    }
  };
  
  const formatDuration = (seconds) => {
    if (!seconds) return 'N/A';
    
    const minutes = Math.floor(seconds / 60);
    const remainingSeconds = seconds % 60;
    
    return `${minutes}m ${remainingSeconds}s`;
  };
  
  const renderBatteryChart = () => {
    if (!batteryHistory || batteryHistory.length === 0) {
      return <div className="no-data">No historical data available</div>;
    }
    
    const labels = batteryHistory.map(item => {
      const date = new Date(item.timestamp);
      return date.toLocaleTimeString();
    });
    
    const capacityData = batteryHistory.map(item => item.capacity_percent);
    const voltageData = batteryHistory.map(item => item.voltage);
    
    const data = {
      labels,
      datasets: [
        {
          label: 'Battery Level (%)',
          data: capacityData,
          borderColor: 'rgba(75, 192, 192, 1)',
          backgroundColor: 'rgba(75, 192, 192, 0.2)',
          fill: true,
          tension: 0.4
        },
        {
          label: 'Voltage (V)',
          data: voltageData,
          borderColor: 'rgba(153, 102, 255, 1)',
          backgroundColor: 'rgba(153, 102, 255, 0.2)',
          fill: false,
          tension: 0.4,
          yAxisID: 'y1'
        }
      ]
    };
    
    const options = {
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        y: {
          beginAtZero: true,
          max: 100,
          title: {
            display: true,
            text: 'Battery Level (%)'
          }
        },
        y1: {
          position: 'right',
          title: {
            display: true,
            text: 'Voltage (V)'
          },
          grid: {
            drawOnChartArea: false
          }
        }
      },
      plugins: {
        legend: {
          position: 'top'
        },
        title: {
          display: true,
          text: 'Battery History'
        }
      }
    };
    
    return <Line ref={batteryChartRef} data={data} options={options} />;
  };
  
  const renderTemperatureChart = () => {
    if (!batteryHistory || batteryHistory.length === 0) {
      return <div className="no-data">No historical data available</div>;
    }
    
    const labels = batteryHistory.map(item => {
      const date = new Date(item.timestamp);
      return date.toLocaleTimeString();
    });
    
    const temperatureData = batteryHistory.map(item => item.temperature);
    const currentData = batteryHistory.map(item => item.current);
    
    const data = {
      labels,
      datasets: [
        {
          label: 'Temperature (°C)',
          data: temperatureData,
          borderColor: 'rgba(255, 99, 132, 1)',
          backgroundColor: 'rgba(255, 99, 132, 0.2)',
          fill: true,
          tension: 0.4
        },
        {
          label: 'Current (A)',
          data: currentData,
          borderColor: 'rgba(54, 162, 235, 1)',
          backgroundColor: 'rgba(54, 162, 235, 0.2)',
          fill: false,
          tension: 0.4,
          yAxisID: 'y1'
        }
      ]
    };
    
    const options = {
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        y: {
          title: {
            display: true,
            text: 'Temperature (°C)'
          }
        },
        y1: {
          position: 'right',
          title: {
            display: true,
            text: 'Current (A)'
          },
          grid: {
            drawOnChartArea: false
          }
        }
      },
      plugins: {
        legend: {
          position: 'top'
        },
        title: {
          display: true,
          text: 'Temperature & Current History'
        }
      }
    };
    
    return <Line ref={temperatureChartRef} data={data} options={options} />;
  };
  
  if (isLoading) {
    return <div className="loading">Loading battery monitoring data...</div>;
  }
  
  return (
    <div className="battery-monitor-panel">
      <div className="battery-monitor-sidebar">
        <h2>Battery Monitor</h2>
        
        {error && (
          <div className="error-message">
            {error}
            <button onClick={() => setError(null)}>Dismiss</button>
          </div>
        )}
        
        <div className="drone-selection">
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
        
        {selectedDrone && batteryMetrics && (
          <div className="battery-metrics">
            <h3>Battery Status</h3>
            
            <div className="battery-indicator">
              <div className="battery-outer">
                <div 
                  className={`battery-inner ${getBatteryLevelClass(batteryMetrics.capacity_percent)}`}
                  style={{ width: `${batteryMetrics.capacity_percent}%` }}
                ></div>
              </div>
              <div className={`battery-percentage ${getBatteryLevelClass(batteryMetrics.capacity_percent)}`}>
                {batteryMetrics.capacity_percent.toFixed(1)}%
              </div>
            </div>
            
            <div className="metrics-grid">
              <div className="metric-item">
                <span className="metric-label">Voltage</span>
                <span className="metric-value">{batteryMetrics.voltage.toFixed(2)} V</span>
              </div>
              
              <div className="metric-item">
                <span className="metric-label">Current</span>
                <span className="metric-value">{batteryMetrics.current.toFixed(2)} A</span>
              </div>
              
              <div className="metric-item">
                <span className="metric-label">Temperature</span>
                <span className="metric-value">{batteryMetrics.temperature.toFixed(1)} °C</span>
              </div>
              
              <div className="metric-item">
                <span className="metric-label">Status</span>
                <span className={`metric-value ${getBatteryStatusClass(batteryMetrics.status)}`}>
                  {batteryMetrics.status.charAt(0).toUpperCase() + batteryMetrics.status.slice(1)}
                </span>
              </div>
              
              {batteryMetrics.discharge_rate && (
                <div className="metric-item">
                  <span className="metric-label">Discharge Rate</span>
                  <span className="metric-value">{batteryMetrics.discharge_rate.toFixed(2)} mAh/min</span>
                </div>
              )}
              
              {batteryMetrics.estimated_time_remaining && (
                <div className="metric-item">
                  <span className="metric-label">Time Remaining</span>
                  <span className="metric-value">{formatDuration(batteryMetrics.estimated_time_remaining)}</span>
                </div>
              )}
              
              {batteryMetrics.cycle_count && (
                <div className="metric-item">
                  <span className="metric-label">Cycle Count</span>
                  <span className="metric-value">{batteryMetrics.cycle_count}</span>
                </div>
              )}
              
              {batteryMetrics.health_percent && (
                <div className="metric-item">
                  <span className="metric-label">Battery Health</span>
                  <span className="metric-value">{batteryMetrics.health_percent.toFixed(1)}%</span>
                </div>
              )}
            </div>
            
            <div className="battery-actions">
              <button 
                className="rth-button"
                onClick={handleTriggerRTH}
              >
                Trigger RTH
              </button>
              
              <button 
                className="refresh-button"
                onClick={() => fetchBatteryMetrics(selectedDrone.id)}
              >
                Refresh
              </button>
            </div>
          </div>
        )}
        
        {selectedDrone && thresholds && (
          <div className="battery-thresholds">
            <div className="thresholds-header">
              <h3>Battery Thresholds</h3>
              <button 
                className="edit-button"
                onClick={() => setIsEditingThresholds(!isEditingThresholds)}
              >
                {isEditingThresholds ? 'Cancel' : 'Edit'}
              </button>
            </div>
            
            {isEditingThresholds ? (
              <div className="thresholds-form">
                <div className="form-group">
                  <label>Warning Threshold (%)</label>
                  <input
                    type="number"
                    value={newThresholds.warning_threshold}
                    onChange={(e) => setNewThresholds({...newThresholds, warning_threshold: parseFloat(e.target.value)})}
                    min="20"
                    max="50"
                    step="1"
                  />
                </div>
                
                <div className="form-group">
                  <label>Critical Threshold (%)</label>
                  <input
                    type="number"
                    value={newThresholds.critical_threshold}
                    onChange={(e) => setNewThresholds({...newThresholds, critical_threshold: parseFloat(e.target.value)})}
                    min="5"
                    max="20"
                    step="1"
                  />
                </div>
                
                <div className="form-group">
                  <label>Auto-Return Threshold (%)</label>
                  <input
                    type="number"
                    value={newThresholds.auto_return_threshold}
                    onChange={(e) => setNewThresholds({...newThresholds, auto_return_threshold: parseFloat(e.target.value)})}
                    min="10"
                    max="40"
                    step="1"
                  />
                </div>
                
                <div className="form-group">
                  <label>Max Temperature (°C)</label>
                  <input
                    type="number"
                    value={newThresholds.temperature_max}
                    onChange={(e) => setNewThresholds({...newThresholds, temperature_max: parseFloat(e.target.value)})}
                    min="40"
                    max="80"
                    step="1"
                  />
                </div>
                
                <div className="form-group">
                  <label>Min Voltage (V, optional)</label>
                  <input
                    type="number"
                    value={newThresholds.voltage_min || ''}
                    onChange={(e) => setNewThresholds({...newThresholds, voltage_min: e.target.value ? parseFloat(e.target.value) : null})}
                    min="3"
                    max="20"
                    step="0.1"
                  />
                </div>
                
                <button 
                  className="save-button"
                  onClick={handleSaveThresholds}
                >
                  Save Thresholds
                </button>
              </div>
            ) : (
              <div className="thresholds-display">
                <div className="threshold-item">
                  <span className="threshold-label">Warning Level</span>
                  <span className="threshold-value battery-warning">{thresholds.warning_threshold}%</span>
                </div>
                
                <div className="threshold-item">
                  <span className="threshold-label">Critical Level</span>
                  <span className="threshold-value battery-critical">{thresholds.critical_threshold}%</span>
                </div>
                
                <div className="threshold-item">
                  <span className="threshold-label">Auto-Return</span>
                  <span className="threshold-value">{thresholds.auto_return_threshold}%</span>
                </div>
                
                <div className="threshold-item">
                  <span className="threshold-label">Max Temperature</span>
                  <span className="threshold-value">{thresholds.temperature_max}°C</span>
                </div>
                
                {thresholds.voltage_min && (
                  <div className="threshold-item">
                    <span className="threshold-label">Min Voltage</span>
                    <span className="threshold-value">{thresholds.voltage_min}V</span>
                  </div>
                )}
              </div>
            )}
          </div>
        )}
      </div>
      
      <div className="battery-monitor-content">
        {selectedDrone ? (
          <div className="battery-charts">
            <div className="chart-container">
              {renderBatteryChart()}
            </div>
            
            <div className="chart-container">
              {renderTemperatureChart()}
            </div>
          </div>
        ) : (
          <div className="no-selection">
            <p>Select a drone to view battery information</p>
          </div>
        )}
      </div>
    </div>
  );
};

export default BatteryMonitorPanel;
