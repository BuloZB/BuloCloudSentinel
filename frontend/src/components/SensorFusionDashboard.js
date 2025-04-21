import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';

const SensorStatusIndicator = ({ status }) => {
  const getStatusColor = () => {
    switch (status) {
      case 'active':
        return 'bg-green-500';
      case 'stale':
        return 'bg-yellow-500';
      case 'inactive':
        return 'bg-red-500';
      default:
        return 'bg-gray-500';
    }
  };

  return (
    <div className={`w-3 h-3 rounded-full ${getStatusColor()} mr-2`}></div>
  );
};

const SensorList = ({ sensors, onSelectSensor }) => {
  return (
    <div className="bg-white shadow rounded-lg p-4 mb-4">
      <h3 className="text-lg font-semibold mb-2">Sensors</h3>
      <div className="max-h-60 overflow-y-auto">
        {Object.entries(sensors).length === 0 ? (
          <p className="text-gray-500">No sensors registered</p>
        ) : (
          <ul className="divide-y divide-gray-200">
            {Object.entries(sensors).map(([sensorId, sensorInfo]) => (
              <li 
                key={sensorId} 
                className="py-2 cursor-pointer hover:bg-gray-50"
                onClick={() => onSelectSensor(sensorId)}
              >
                <div className="flex items-center">
                  <SensorStatusIndicator status={sensorInfo.status} />
                  <div>
                    <p className="font-medium">{sensorId}</p>
                    <p className="text-sm text-gray-500">{sensorInfo.type}</p>
                  </div>
                </div>
              </li>
            ))}
          </ul>
        )}
      </div>
    </div>
  );
};

const PositionDisplay = ({ position }) => {
  if (!position || Object.keys(position).length === 0) {
    return (
      <div className="bg-white shadow rounded-lg p-4 mb-4">
        <h3 className="text-lg font-semibold mb-2">Position</h3>
        <p className="text-gray-500">No position data available</p>
      </div>
    );
  }

  return (
    <div className="bg-white shadow rounded-lg p-4 mb-4">
      <h3 className="text-lg font-semibold mb-2">Position</h3>
      <div className="grid grid-cols-2 gap-2">
        <div>
          <p className="text-sm text-gray-500">X</p>
          <p className="font-medium">{position.x?.toFixed(2) || 'N/A'}</p>
        </div>
        <div>
          <p className="text-sm text-gray-500">Y</p>
          <p className="font-medium">{position.y?.toFixed(2) || 'N/A'}</p>
        </div>
        <div>
          <p className="text-sm text-gray-500">Speed</p>
          <p className="font-medium">{position.speed?.toFixed(2) || 'N/A'} m/s</p>
        </div>
        <div>
          <p className="text-sm text-gray-500">Heading</p>
          <p className="font-medium">{position.heading?.toFixed(1) || 'N/A'}°</p>
        </div>
      </div>
    </div>
  );
};

const DetectionsList = ({ detections }) => {
  if (!detections || detections.length === 0) {
    return (
      <div className="bg-white shadow rounded-lg p-4 mb-4">
        <h3 className="text-lg font-semibold mb-2">Detections</h3>
        <p className="text-gray-500">No detections available</p>
      </div>
    );
  }

  return (
    <div className="bg-white shadow rounded-lg p-4 mb-4">
      <h3 className="text-lg font-semibold mb-2">Detections ({detections.length})</h3>
      <div className="max-h-60 overflow-y-auto">
        <table className="min-w-full divide-y divide-gray-200">
          <thead className="bg-gray-50">
            <tr>
              <th className="px-3 py-2 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">Type</th>
              <th className="px-3 py-2 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">Position</th>
              <th className="px-3 py-2 text-left text-xs font-medium text-gray-500 uppercase tracking-wider">Confidence</th>
            </tr>
          </thead>
          <tbody className="bg-white divide-y divide-gray-200">
            {detections.map((detection, index) => (
              <tr key={index}>
                <td className="px-3 py-2 whitespace-nowrap">
                  <div className="text-sm font-medium text-gray-900">{detection.type}</div>
                </td>
                <td className="px-3 py-2 whitespace-nowrap">
                  <div className="text-sm text-gray-500">
                    X: {detection.x.toFixed(1)}, Y: {detection.y.toFixed(1)}
                  </div>
                </td>
                <td className="px-3 py-2 whitespace-nowrap">
                  <div className="text-sm text-gray-500">{(detection.confidence * 100).toFixed(0)}%</div>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
};

const SensorDetail = ({ sensorId, sensorInfo }) => {
  if (!sensorInfo) {
    return null;
  }

  return (
    <div className="bg-white shadow rounded-lg p-4 mb-4">
      <div className="flex justify-between items-center mb-4">
        <h3 className="text-lg font-semibold">Sensor Details</h3>
        <div className="flex items-center">
          <SensorStatusIndicator status={sensorInfo.status} />
          <span className="text-sm capitalize">{sensorInfo.status}</span>
        </div>
      </div>
      
      <div className="mb-4">
        <p className="text-sm text-gray-500">ID</p>
        <p className="font-medium">{sensorId}</p>
      </div>
      
      <div className="mb-4">
        <p className="text-sm text-gray-500">Type</p>
        <p className="font-medium">{sensorInfo.type}</p>
      </div>
      
      <div className="mb-4">
        <p className="text-sm text-gray-500">Capabilities</p>
        {sensorInfo.capabilities && sensorInfo.capabilities.length > 0 ? (
          <div className="flex flex-wrap gap-1 mt-1">
            {sensorInfo.capabilities.map((capability, index) => (
              <span key={index} className="px-2 py-1 bg-blue-100 text-blue-800 text-xs rounded">
                {capability}
              </span>
            ))}
          </div>
        ) : (
          <p className="text-gray-500">None</p>
        )}
      </div>
      
      <div>
        <p className="text-sm text-gray-500">Last Update</p>
        <p className="font-medium">
          {sensorInfo.last_update ? new Date(sensorInfo.last_update).toLocaleString() : 'Never'}
        </p>
      </div>
    </div>
  );
};

export default function SensorFusionDashboard() {
  const [fusedData, setFusedData] = useState({});
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [selectedSensor, setSelectedSensor] = useState(null);
  const [refreshInterval, setRefreshInterval] = useState(1000);
  const intervalRef = useRef(null);

  const fetchFusedData = async () => {
    try {
      setLoading(true);
      const response = await axios.get('/api/sensor-fusion/fused-data');
      setFusedData(response.data);
      setError(null);
    } catch (err) {
      console.error('Error fetching fused data:', err);
      setError('Failed to fetch sensor fusion data');
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    // Initial fetch
    fetchFusedData();

    // Set up interval for periodic updates
    intervalRef.current = setInterval(fetchFusedData, refreshInterval);

    // Clean up interval on component unmount
    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
    };
  }, [refreshInterval]);

  const handleSelectSensor = (sensorId) => {
    setSelectedSensor(sensorId);
  };

  const handleRefreshIntervalChange = (e) => {
    const newInterval = parseInt(e.target.value, 10);
    setRefreshInterval(newInterval);
    
    // Reset the interval with the new value
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
    }
    intervalRef.current = setInterval(fetchFusedData, newInterval);
  };

  const sensors = fusedData.sensors || {};
  const selectedSensorInfo = selectedSensor ? sensors[selectedSensor] : null;

  return (
    <div className="p-4">
      <div className="flex justify-between items-center mb-4">
        <h2 className="text-2xl font-bold">Sensor Fusion Dashboard</h2>
        <div className="flex items-center">
          <label htmlFor="refreshInterval" className="mr-2 text-sm">Refresh:</label>
          <select
            id="refreshInterval"
            value={refreshInterval}
            onChange={handleRefreshIntervalChange}
            className="border rounded p-1 text-sm"
          >
            <option value={500}>0.5s</option>
            <option value={1000}>1s</option>
            <option value={2000}>2s</option>
            <option value={5000}>5s</option>
          </select>
          <button
            onClick={fetchFusedData}
            className="ml-2 bg-blue-500 text-white px-3 py-1 rounded hover:bg-blue-600 text-sm"
          >
            Refresh Now
          </button>
        </div>
      </div>

      {loading && !fusedData.timestamp && (
        <div className="text-center py-4">
          <p>Loading sensor fusion data...</p>
        </div>
      )}

      {error && (
        <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded mb-4">
          <p>{error}</p>
        </div>
      )}

      {fusedData.timestamp && (
        <div className="text-sm text-gray-500 mb-4">
          Last updated: {new Date(fusedData.timestamp).toLocaleString()}
        </div>
      )}

      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        <div className="md:col-span-1">
          <SensorList 
            sensors={sensors} 
            onSelectSensor={handleSelectSensor} 
          />
          {selectedSensor && (
            <SensorDetail 
              sensorId={selectedSensor} 
              sensorInfo={selectedSensorInfo} 
            />
          )}
        </div>
        
        <div className="md:col-span-2">
          <PositionDisplay position={fusedData.position || {}} />
          <DetectionsList detections={fusedData.detections || []} />
          
          {/* Additional visualization components could be added here */}
          <div className="bg-white shadow rounded-lg p-4">
            <h3 className="text-lg font-semibold mb-2">Operational Picture</h3>
            <div className="h-64 bg-gray-100 rounded flex items-center justify-center">
              <p className="text-gray-500">Visualization placeholder</p>
              {/* This would be replaced with an actual visualization component */}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
