import React, { useState, useEffect } from 'react';
import DOMPurify from 'dompurify';
import { Doughnut } from 'react-chartjs-2';
import {
  Chart as ChartJS,
  ArcElement,
  Tooltip,
  Legend
} from 'chart.js';
import './Widget.css';

// Register Chart.js components
ChartJS.register(
  ArcElement,
  Tooltip,
  Legend
);

const BatteryWidget = ({ id, title, settings, onRemove }) => {
  const [batteryData, setBatteryData] = useState(null);
  const [drones, setDrones] = useState([]);
  const [selectedDrone, setSelectedDrone] = useState(settings?.defaultDrone || null);
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  
  // Fetch available drones
  useEffect(() => {
    const fetchDrones = async () => {
      try {
        const response = await fetch('/api/drones');
        if (!response.ok) {
          throw new Error(`Failed to fetch drones: ${response.status}`);
        }
        
        const data = await response.json();
        
        // Sanitize drone data
        const sanitizedDrones = data.map(drone => ({
          ...drone,
          name: DOMPurify.sanitize(drone.name),
          type: DOMPurify.sanitize(drone.type)
        }));
        
        setDrones(sanitizedDrones);
        
        // Set default drone if available
        if (sanitizedDrones.length > 0 && !selectedDrone) {
          setSelectedDrone(settings?.defaultDrone || sanitizedDrones[0].id);
        }
        
        setIsLoading(false);
      } catch (error) {
        console.error('Error fetching drones:', error);
        setError('Failed to load drones');
        setIsLoading(false);
      }
    };

    fetchDrones();
  }, [settings?.defaultDrone, selectedDrone]);

  // Fetch battery data when drone changes
  useEffect(() => {
    if (!selectedDrone) return;
    
    const fetchBatteryData = async () => {
      try {
        setIsLoading(true);
        
        const response = await fetch(`/api/drones/${selectedDrone}/battery`);
        if (!response.ok) {
          throw new Error(`Failed to fetch battery data: ${response.status}`);
        }
        
        const data = await response.json();
        setBatteryData(data);
        setIsLoading(false);
      } catch (error) {
        console.error('Error fetching battery data:', error);
        setError('Failed to load battery data');
        setIsLoading(false);
      }
    };

    fetchBatteryData();
    
    // Set up polling for real-time updates
    const intervalId = setInterval(fetchBatteryData, 10000);
    
    // Cleanup
    return () => clearInterval(intervalId);
  }, [selectedDrone]);

  // Prepare chart data
  const chartData = batteryData ? {
    labels: ['Remaining', 'Used'],
    datasets: [
      {
        data: [batteryData.percentage, 100 - batteryData.percentage],
        backgroundColor: [
          getBatteryColor(batteryData.percentage),
          'rgba(200, 200, 200, 0.2)'
        ],
        borderColor: [
          getBatteryColor(batteryData.percentage, 0.8),
          'rgba(200, 200, 200, 0.8)'
        ],
        borderWidth: 1,
      },
    ],
  } : null;

  // Chart options
  const chartOptions = {
    responsive: true,
    maintainAspectRatio: false,
    cutout: '70%',
    plugins: {
      legend: {
        display: false
      },
      tooltip: {
        callbacks: {
          label: function(context) {
            return `${context.label}: ${context.raw}%`;
          }
        }
      }
    }
  };

  // Get color based on battery percentage
  function getBatteryColor(percentage, alpha = 0.6) {
    if (percentage <= 20) {
      return `rgba(255, 0, 0, ${alpha})`; // Red
    } else if (percentage <= 40) {
      return `rgba(255, 165, 0, ${alpha})`; // Orange
    } else {
      return `rgba(0, 255, 0, ${alpha})`; // Green
    }
  }

  // Get estimated flight time
  function getEstimatedFlightTime(batteryData) {
    if (!batteryData) return 'N/A';
    
    const minutes = Math.floor(batteryData.estimatedMinutesRemaining);
    const seconds = Math.round((batteryData.estimatedMinutesRemaining - minutes) * 60);
    
    return `${minutes}m ${seconds}s`;
  }

  // Handle drone selection
  const handleDroneChange = (e) => {
    setSelectedDrone(e.target.value);
  };

  return (
    <div className="widget battery-widget">
      <div className="widget-header">
        <h3>{DOMPurify.sanitize(title)}</h3>
        {onRemove && (
          <button className="widget-remove-btn" onClick={onRemove}>
            ✕
          </button>
        )}
      </div>
      
      <div className="widget-content">
        <div className="battery-controls">
          <select 
            value={selectedDrone || ''} 
            onChange={handleDroneChange}
            className="drone-select"
          >
            <option value="" disabled>Select drone</option>
            {drones.map(drone => (
              <option key={drone.id} value={drone.id}>
                {drone.name} ({drone.type})
              </option>
            ))}
          </select>
        </div>
        
        {isLoading ? (
          <div className="widget-loading">Loading battery data...</div>
        ) : error ? (
          <div className="widget-error">{error}</div>
        ) : !batteryData ? (
          <div className="widget-empty">No battery data available</div>
        ) : (
          <div className="battery-display">
            <div className="battery-chart">
              <Doughnut data={chartData} options={chartOptions} />
              <div className="battery-percentage">
                <span>{batteryData.percentage}%</span>
              </div>
            </div>
            
            <div className="battery-stats">
              <div className="stat-item">
                <span className="stat-label">Voltage:</span>
                <span className="stat-value">{batteryData.voltage.toFixed(2)}V</span>
              </div>
              <div className="stat-item">
                <span className="stat-label">Current:</span>
                <span className="stat-value">{batteryData.current.toFixed(2)}A</span>
              </div>
              <div className="stat-item">
                <span className="stat-label">Temperature:</span>
                <span className="stat-value">{batteryData.temperature.toFixed(1)}°C</span>
              </div>
              <div className="stat-item">
                <span className="stat-label">Est. Flight Time:</span>
                <span className="stat-value">{getEstimatedFlightTime(batteryData)}</span>
              </div>
              <div className="stat-item">
                <span className="stat-label">Cycles:</span>
                <span className="stat-value">{batteryData.cycles}</span>
              </div>
              <div className="stat-item">
                <span className="stat-label">Health:</span>
                <span className="stat-value">{batteryData.health}%</span>
              </div>
            </div>
            
            {batteryData.percentage <= 20 && (
              <div className="battery-warning">
                <span>⚠️ Low Battery Warning</span>
                <p>Battery level critical. Consider landing soon.</p>
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

export default BatteryWidget;
