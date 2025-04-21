import React, { useState, useEffect, useRef } from 'react';
import DOMPurify from 'dompurify';
import { MapContainer, TileLayer, Marker, Popup, useMap } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import './Widget.css';

// Fix for Leaflet marker icons
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: 'https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon-2x.png',
  iconUrl: 'https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon.png',
  shadowUrl: 'https://unpkg.com/leaflet@1.7.1/dist/images/marker-shadow.png',
});

// Custom drone icon
const droneIcon = new L.Icon({
  iconUrl: '/assets/drone-icon.png',
  iconSize: [32, 32],
  iconAnchor: [16, 16],
  popupAnchor: [0, -16],
});

// Component to update map view when center changes
const MapUpdater = ({ center, zoom }) => {
  const map = useMap();
  
  useEffect(() => {
    if (center) {
      map.setView(center, zoom);
    }
  }, [center, zoom, map]);
  
  return null;
};

const MapWidget = ({ id, title, settings, onRemove }) => {
  const [drones, setDrones] = useState([]);
  const [selectedDrone, setSelectedDrone] = useState(null);
  const [mapCenter, setMapCenter] = useState(settings?.defaultCenter || [51.505, -0.09]);
  const [mapZoom, setMapZoom] = useState(settings?.defaultZoom || 13);
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [mapLayers, setMapLayers] = useState([
    { id: 'osm', name: 'OpenStreetMap', url: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png' },
    { id: 'satellite', name: 'Satellite', url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}' }
  ]);
  const [selectedLayer, setSelectedLayer] = useState(settings?.defaultLayer || 'osm');
  
  // Fetch drone locations
  useEffect(() => {
    const fetchDroneLocations = async () => {
      try {
        const response = await fetch('/api/drones/locations');
        if (!response.ok) {
          throw new Error(`Failed to fetch drone locations: ${response.status}`);
        }
        
        const data = await response.json();
        
        // Sanitize drone data
        const sanitizedDrones = data.map(drone => ({
          ...drone,
          name: DOMPurify.sanitize(drone.name),
          status: DOMPurify.sanitize(drone.status),
          mission: drone.mission ? DOMPurify.sanitize(drone.mission) : null
        }));
        
        setDrones(sanitizedDrones);
        
        // If we have drones and no selected drone, select the first one
        if (sanitizedDrones.length > 0 && !selectedDrone) {
          setSelectedDrone(sanitizedDrones[0].id);
          setMapCenter([sanitizedDrones[0].latitude, sanitizedDrones[0].longitude]);
        }
        
        setIsLoading(false);
      } catch (error) {
        console.error('Error fetching drone locations:', error);
        setError('Failed to load drone locations');
        setIsLoading(false);
      }
    };

    // Initial fetch
    fetchDroneLocations();
    
    // Set up polling for updates
    const intervalId = setInterval(fetchDroneLocations, 5000);
    
    // Cleanup
    return () => clearInterval(intervalId);
  }, [selectedDrone]);

  // Handle drone selection
  const handleDroneSelect = (droneId) => {
    setSelectedDrone(droneId);
    
    const drone = drones.find(d => d.id === droneId);
    if (drone) {
      setMapCenter([drone.latitude, drone.longitude]);
      setMapZoom(16); // Zoom in when selecting a drone
    }
  };

  // Handle map layer change
  const handleLayerChange = (layerId) => {
    setSelectedLayer(layerId);
  };

  return (
    <div className="widget map-widget">
      <div className="widget-header">
        <h3>{DOMPurify.sanitize(title)}</h3>
        {onRemove && (
          <button className="widget-remove-btn" onClick={onRemove}>
            ✕
          </button>
        )}
      </div>
      
      <div className="widget-content">
        {isLoading ? (
          <div className="widget-loading">Loading map...</div>
        ) : error ? (
          <div className="widget-error">{error}</div>
        ) : (
          <>
            <div className="map-controls">
              <select 
                value={selectedDrone || ''} 
                onChange={(e) => handleDroneSelect(e.target.value)}
                className="drone-select"
              >
                <option value="" disabled>Select drone</option>
                {drones.map(drone => (
                  <option key={drone.id} value={drone.id}>
                    {drone.name} - {drone.status}
                  </option>
                ))}
              </select>
              
              <div className="map-layer-controls">
                {mapLayers.map(layer => (
                  <button
                    key={layer.id}
                    className={`map-layer-btn ${selectedLayer === layer.id ? 'active' : ''}`}
                    onClick={() => handleLayerChange(layer.id)}
                  >
                    {layer.name}
                  </button>
                ))}
              </div>
            </div>
            
            <div className="map-container">
              <MapContainer
                center={mapCenter}
                zoom={mapZoom}
                style={{ height: '100%', width: '100%' }}
              >
                <MapUpdater center={mapCenter} zoom={mapZoom} />
                
                <TileLayer
                  url={mapLayers.find(layer => layer.id === selectedLayer)?.url}
                  attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                />
                
                {drones.map(drone => (
                  <Marker
                    key={drone.id}
                    position={[drone.latitude, drone.longitude]}
                    icon={droneIcon}
                  >
                    <Popup>
                      <div>
                        <h4>{drone.name}</h4>
                        <p>Status: {drone.status}</p>
                        <p>Battery: {drone.battery}%</p>
                        {drone.mission && <p>Mission: {drone.mission}</p>}
                        <p>Altitude: {drone.altitude}m</p>
                        <p>Speed: {drone.speed}m/s</p>
                      </div>
                    </Popup>
                  </Marker>
                ))}
              </MapContainer>
            </div>
          </>
        )}
      </div>
    </div>
  );
};

export default MapWidget;
