import React, { useState, useEffect } from 'react';

function DroneCommandHub() {
  const [telemetry, setTelemetry] = useState({});
  const [droneId, setDroneId] = useState('');
  const [command, setCommand] = useState('');
  const [parameters, setParameters] = useState('');

  useEffect(() => {
    if (!droneId) return;
    async function fetchTelemetry() {
      const response = await fetch(`/api/drone-hub/telemetry/${droneId}`, {
        headers: {
          Authorization: 'Bearer your-jwt-token-here'
        }
      });
      const data = await response.json();
      setTelemetry(data || {});
    }
    fetchTelemetry();
  }, [droneId]);

  async function sendCommand() {
    const cmd = {
      drone_id: droneId,
      command: command,
      parameters: parameters ? JSON.parse(parameters) : {}
    };
    await fetch('/api/drone-hub/command', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Authorization: 'Bearer your-jwt-token-here'
      },
      body: JSON.stringify(cmd)
    });
    alert('Command sent');
  }

  return (
    <div className="p-4">
      <h2 className="text-xl font-bold mb-4">Drone Command & Telemetry Hub</h2>
      <div className="mb-4">
        <input
          type="text"
          placeholder="Drone ID"
          value={droneId}
          onChange={(e) => setDroneId(e.target.value)}
          className="border p-2 mr-2"
        />
      </div>
      <div className="mb-4">
        <div>Telemetry Data:</div>
        <pre>{JSON.stringify(telemetry, null, 2)}</pre>
      </div>
      <div className="mb-4">
        <input
          type="text"
          placeholder="Command (takeoff, waypoint, return_home)"
          value={command}
          onChange={(e) => setCommand(e.target.value)}
          className="border p-2 mr-2"
        />
        <input
          type="text"
          placeholder="Parameters (JSON)"
          value={parameters}
          onChange={(e) => setParameters(e.target.value)}
          className="border p-2"
        />
      </div>
      <button onClick={sendCommand} className="bg-blue-500 text-white px-4 py-2 rounded">
        Send Command
      </button>
    </div>
  );
}

export default DroneCommandHub;
