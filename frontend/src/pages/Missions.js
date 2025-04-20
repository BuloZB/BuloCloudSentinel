import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker, Polyline, useMapEvents } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import axios from 'axios';
import Mission3DPreview from '../components/Mission3DPreview';
import MissionControl from '../components/MissionControl';

function WaypointMarker({ position, index, onMove }) {
  const map = useMapEvents({
    dragend() {
      const marker = map.getCenter();
      onMove(index, marker);
    }
  });
  return <Marker position={position} draggable={true} eventHandlers={{
    dragend: (e) => {
      onMove(index, e.target.getLatLng());
    }
  }} />;
}

export default function Missions() {
  const [missions, setMissions] = useState([]);
  const [selectedMission, setSelectedMission] = useState(null);
  const [waypoints, setWaypoints] = useState([]);

  useEffect(() => {
    fetchMissions();
  }, []);

  const fetchMissions = async () => {
    const res = await axios.get('/api/missions');
    setMissions(res.data);
  };

  const selectMission = async (mission) => {
    setSelectedMission(mission);
    setWaypoints(mission.waypoints.map(wp => ({
      latitude: wp.latitude,
      longitude: wp.longitude,
      altitude: wp.altitude || 0
    })));
  };

  const moveWaypoint = (index, latlng) => {
    const newWaypoints = [...waypoints];
    newWaypoints[index] = { latitude: latlng.lat, longitude: latlng.lng, altitude: newWaypoints[index].altitude };
    setWaypoints(newWaypoints);
  };

  return (
    <div className="flex h-full">
      <div className="w-1/4 p-4 border-r overflow-auto">
        <h2 className="text-xl font-bold mb-4">Missions</h2>
        <ul>
          {missions.map(mission => (
            <li key={mission.id} className="cursor-pointer mb-2" onClick={() => selectMission(mission)}>
              {mission.name}
            </li>
          ))}
        </ul>
      </div>
      <div className="flex-1 p-4">
        {selectedMission ? (
          <>
            <h2 className="text-xl font-bold mb-4">{selectedMission.name}</h2>
            <MapContainer center={[waypoints[0].latitude, waypoints[0].longitude] || [0, 0]} zoom={13} style={{ height: '300px' }}>
              <TileLayer
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
              />
              {waypoints.map((pos, idx) => (
                <WaypointMarker key={idx} position={[pos.latitude, pos.longitude]} index={idx} onMove={moveWaypoint} />
              ))}
              <Polyline positions={waypoints.map(wp => [wp.latitude, wp.longitude])} color="blue" />
            </MapContainer>
            <div className="mt-4">
              <Mission3DPreview waypoints={waypoints} />
            </div>
            <div className="mt-4">
              <MissionControl websocketUrl="ws://localhost:8000/ws/missions/" />
            </div>
          </>
        ) : (
          <p>Select a mission to view details</p>
        )}
      </div>
    </div>
  );
}
