import React, { useState, useEffect } from 'react';
import DOMPurify from 'dompurify';
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
} from 'chart.js';
import './Widget.css';

// Register Chart.js components
ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

const TelemetryWidget = ({ id, title, settings, onRemove }) => {
  const [telemetryData, setTelemetryData] = useState(null);
  const [drones, setDrones] = useState([]);
  const [selectedDrone, setSelectedDrone] = useState(settings?.defaultDrone || null);
  const [selectedMetric, setSelectedMetric] = useState(settings?.defaultMetric || 'altitude');
  const [timeRange, setTimeRange] = useState(settings?.defaultTimeRange || '1h');
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  
  const metrics = [
    { id: 'altitude', name: 'Altitude', unit: 'm' },
    { id: 'speed', name: 'Speed', unit: 'm/s' },
    { id: 'battery', name: 'Battery', unit: '%' },
    { id: 'signal', name: 'Signal Strength', unit: 'dBm' },
    { id: 'temperature', name: 'Temperature', unit: '°C' }
  ];
  
  const timeRanges = [
    { id: '15m', name: '15 minutes' },
    { id: '1h', name: '1 hour' },
    { id: '6h', name: '6 hours' },
    { id: '24h', name: '24 hours' }
  ];

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

  // Fetch telemetry data when drone, metric or time range changes
  useEffect(() => {
    if (!selectedDrone) return;
    
    const fetchTelemetryData = async () => {
      try {
        setIsLoading(true);
        
        const response = await fetch(`/api/telemetry/${selectedDrone}?metric=${selectedMetric}&timeRange=${timeRange}`);
        if (!response.ok) {
          throw new Error(`Failed to fetch telemetry data: ${response.status}`);
        }
        
        const data = await response.json();
        setTelemetryData(data);
        setIsLoading(false);
      } catch (error) {
        console.error('Error fetching telemetry data:', error);
        setError('Failed to load telemetry data');
        setIsLoading(false);
      }
    };

    fetchTelemetryData();
    
    // Set up polling for real-time updates
    const intervalId = setInterval(fetchTelemetryData, 5000);
    
    // Cleanup
    return () => clearInterval(intervalId);
  }, [selectedDrone, selectedMetric, timeRange]);

  // Prepare chart data
  const chartData = telemetryData ? {
    labels: telemetryData.timestamps.map(ts => {
      const date = new Date(ts);
      return date.toLocaleTimeString();
    }),
    datasets: [
      {
        label: metrics.find(m => m.id === selectedMetric)?.name || selectedMetric,
        data: telemetryData.values,
        borderColor: 'rgba(75, 192, 192, 1)',
        backgroundColor: 'rgba(75, 192, 192, 0.2)',
        tension: 0.4
      }
    ]
  } : null;

  // Chart options
  const chartOptions = {
    responsive: true,
    maintainAspectRatio: false,
    scales: {
      y: {
        beginAtZero: selectedMetric === 'battery' || selectedMetric === 'signal',
        title: {
          display: true,
          text: metrics.find(m => m.id === selectedMetric)?.unit || ''
        }
      },
      x: {
        title: {
          display: true,
          text: 'Time'
        }
      }
    },
    plugins: {
      legend: {
        position: 'top',
      },
      tooltip: {
        mode: 'index',
        intersect: false,
      },
    },
  };

  // Handle drone selection
  const handleDroneChange = (e) => {
    setSelectedDrone(e.target.value);
  };

  // Handle metric selection
  const handleMetricChange = (e) => {
    setSelectedMetric(e.target.value);
  };

  // Handle time range selection
  const handleTimeRangeChange = (e) => {
    setTimeRange(e.target.value);
  };

  return (
    <div className="widget telemetry-widget">
      <div className="widget-header">
        <h3>{DOMPurify.sanitize(title)}</h3>
        {onRemove && (
          <button className="widget-remove-btn" onClick={onRemove}>
            ✕
          </button>
        )}
      </div>
      
      <div className="widget-content">
        <div className="telemetry-controls">
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
          
          <select 
            value={selectedMetric} 
            onChange={handleMetricChange}
            className="metric-select"
          >
            {metrics.map(metric => (
              <option key={metric.id} value={metric.id}>
                {metric.name}
              </option>
            ))}
          </select>
          
          <select 
            value={timeRange} 
            onChange={handleTimeRangeChange}
            className="time-range-select"
          >
            {timeRanges.map(range => (
              <option key={range.id} value={range.id}>
                {range.name}
              </option>
            ))}
          </select>
        </div>
        
        <div className="telemetry-chart-container">
          {isLoading ? (
            <div className="widget-loading">Loading telemetry data...</div>
          ) : error ? (
            <div className="widget-error">{error}</div>
          ) : !telemetryData ? (
            <div className="widget-empty">No telemetry data available</div>
          ) : (
            <Line data={chartData} options={chartOptions} />
          )}
        </div>
        
        {telemetryData && (
          <div className="telemetry-stats">
            <div className="stat-item">
              <span className="stat-label">Current:</span>
              <span className="stat-value">
                {telemetryData.values[telemetryData.values.length - 1]} 
                {metrics.find(m => m.id === selectedMetric)?.unit}
              </span>
            </div>
            <div className="stat-item">
              <span className="stat-label">Min:</span>
              <span className="stat-value">
                {Math.min(...telemetryData.values)} 
                {metrics.find(m => m.id === selectedMetric)?.unit}
              </span>
            </div>
            <div className="stat-item">
              <span className="stat-label">Max:</span>
              <span className="stat-value">
                {Math.max(...telemetryData.values)} 
                {metrics.find(m => m.id === selectedMetric)?.unit}
              </span>
            </div>
            <div className="stat-item">
              <span className="stat-label">Avg:</span>
              <span className="stat-value">
                {(telemetryData.values.reduce((a, b) => a + b, 0) / telemetryData.values.length).toFixed(2)} 
                {metrics.find(m => m.id === selectedMetric)?.unit}
              </span>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default TelemetryWidget;
