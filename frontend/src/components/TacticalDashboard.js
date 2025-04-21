import React, { useEffect, useState } from 'react';

function TacticalDashboard() {
  const [alerts, setAlerts] = useState([]);
  const [missions, setMissions] = useState([]);
  const [systemHealth, setSystemHealth] = useState(null);

  useEffect(() => {
    // Fetch real-time alerts
    fetch('/api/alerts')
      .then(res => res.json())
      .then(data => setAlerts(data))
      .catch(() => setAlerts([]));

    // Fetch mission status updates
    fetch('/api/missions')
      .then(res => res.json())
      .then(data => setMissions(data))
      .catch(() => setMissions([]));

    // Fetch system health indicators
    fetch('/api/health')
      .then(res => res.json())
      .then(data => setSystemHealth(data))
      .catch(() => setSystemHealth({ status: 'unreachable' }));
  }, []);

  return (
    <div>
      <h2>Real-time Alerts</h2>
      <ul>
        {alerts.map((alert, index) => (
          <li key={index}>{alert.message}</li>
        ))}
      </ul>

      <h2>Mission Status</h2>
      <ul>
        {missions.map((mission) => (
          <li key={mission.id}>{mission.name}: {mission.status}</li>
        ))}
      </ul>

      <h2>System Health</h2>
      <pre>{JSON.stringify(systemHealth, null, 2)}</pre>
    </div>
  );
}

export default TacticalDashboard;
