import React, { useEffect, useState } from 'react';

function AdminDashboard() {
  const [health, setHealth] = useState(null);
  const [auditLogs, setAuditLogs] = useState([]);
  const [droneStatus, setDroneStatus] = useState(null);

  useEffect(() => {
    fetch('/api/health')
      .then(res => res.json())
      .then(data => setHealth(data))
      .catch(() => setHealth({ status: 'unreachable' }));

    fetch('/api/audit-log')
      .then(res => res.json())
      .then(data => setAuditLogs(data))
      .catch(() => setAuditLogs([]));

    // Placeholder for drone status fetch
    fetch('/api/drone/status')
      .then(res => res.json())
      .then(data => setDroneStatus(data))
      .catch(() => setDroneStatus({ status: 'unknown' }));
  }, []);

  return (
    <div>
      <h2>System Health</h2>
      <pre>{JSON.stringify(health, null, 2)}</pre>

      <h2>Drone Status</h2>
      <pre>{JSON.stringify(droneStatus, null, 2)}</pre>

      <h2>Audit Logs</h2>
      <ul>
        {auditLogs.map((log, index) => (
          <li key={index}>
            [{new Date(log.timestamp).toLocaleString()}] {log.username} performed {log.action} on {log.resource}: {log.details}
          </li>
        ))}
      </ul>
    </div>
  );
}

export default AdminDashboard;
