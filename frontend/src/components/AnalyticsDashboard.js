import React, { useEffect, useState } from 'react';
import axios from 'axios';

export default function AnalyticsDashboard() {
  const [data, setData] = useState(null);

  useEffect(() => {
    fetchOverview();
  }, []);

  const fetchOverview = async () => {
    try {
      const response = await axios.get('/api/analytics/overview');
      setData(response.data);
    } catch (error) {
      console.error('Failed to fetch analytics overview', error);
    }
  };

  if (!data) return <div>Loading analytics...</div>;

  return (
    <div className="p-4 bg-white rounded shadow">
      <h2 className="text-xl font-bold mb-4">Analytics Overview</h2>
      <ul className="space-y-2">
        <li>Active Missions: <strong>{data.active_missions}</strong></li>
        <li>Alerts Last 24h: <strong>{data.alerts_last_24h}</strong></li>
        <li>Device Health:</li>
        <ul className="ml-4 list-disc">
          <li>Cameras Online: {data.device_health.cameras_online}</li>
          <li>Drones Online: {data.device_health.drones_online}</li>
          <li>Sensors Online: {data.device_health.sensors_online}</li>
        </ul>
        <li>Average Response Time: <strong>{data.average_response_time_ms.toFixed(2)} ms</strong></li>
      </ul>
    </div>
  );
}
