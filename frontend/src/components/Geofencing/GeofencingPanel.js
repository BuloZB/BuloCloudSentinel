import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import { MapContainer, TileLayer, Marker, Popup, Circle, Polygon, useMapEvents } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import './GeofencingPanel.css';

// Fix Leaflet marker icon issue
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon-2x.png',
  iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
  shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png',
});

// Custom marker icon
const waypointIcon = new L.Icon({
  iconUrl: '/icons/waypoint-marker.png',
  iconSize: [24, 24],
  iconAnchor: [12, 12],
  popupAnchor: [0, -12]
});

// Map click handler component
const MapClickHandler = ({ onMapClick }) => {
  useMapEvents({
    click: onMapClick
  });
  return null;
};

const GeofencingPanel = () => {
  const [geofenceZones, setGeofenceZones] = useState([]);
  const [selectedZone, setSelectedZone] = useState(null);
  const [waypoints, setWaypoints] = useState([]);
  const [validationResult, setValidationResult] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);
  const [mapCenter, setMapCenter] = useState([51.505, -0.09]);
  const [mapZoom, setMapZoom] = useState(13);
  const [drawMode, setDrawMode] = useState(null); // null, 'polygon', 'circle'
  const [drawPoints, setDrawPoints] = useState([]);
  const [circleRadius, setCircleRadius] = useState(1000); // meters
  const [showNewZoneForm, setShowNewZoneForm] = useState(false);
  const [newZone, setNewZone] = useState({
    name: '',
    description: '',
    restriction_level: 'no_fly',
    geometry_type: 'polygon',
    geometry: {},
    altitude_min: null,
    altitude_max: null
  });
  
  const mapRef = useRef(null);
  
  // Load geofence zones on component mount
  useEffect(() => {
    fetchGeofenceZones();
  }, []);
  
  const fetchGeofenceZones = async () => {
    try {
      setIsLoading(true);
      const response = await axios.get('/api/geofence/zones');
      setGeofenceZones(response.data);
      setIsLoading(false);
    } catch (err) {
      console.error('Error fetching geofence zones:', err);
      setError('Failed to load geofence zones');
      setIsLoading(false);
    }
  };
  
  const handleMapClick = (e) => {
    const { lat, lng } = e.latlng;
    
    if (drawMode === 'polygon') {
      // Add point to polygon
      setDrawPoints([...drawPoints, [lat, lng]]);
    } else if (drawMode === 'circle') {
      // Set circle center
      setDrawPoints([[lat, lng]]);
    } else {
      // Add waypoint
      setWaypoints([...waypoints, { latitude: lat, longitude: lng }]);
    }
  };
  
  const handleValidateMission = async () => {
    if (waypoints.length === 0) {
      setError('Please add at least one waypoint');
      return;
    }
    
    try {
      const response = await axios.post('/api/geofence/validate-mission', {
        waypoints: waypoints
      });
      
      setValidationResult(response.data);
      
      // If not valid, highlight violations on map
      if (!response.data.valid) {
        // Center map on first violation
        const firstViolation = response.data.violations[0];
        if (firstViolation && firstViolation.latitude && firstViolation.longitude) {
          mapRef.current.setView([firstViolation.latitude, firstViolation.longitude], 14);
        }
      }
    } catch (err) {
      console.error('Error validating mission:', err);
      setError('Failed to validate mission');
    }
  };
  
  const handleClearWaypoints = () => {
    setWaypoints([]);
    setValidationResult(null);
  };
  
  const handleCreateZone = async () => {
    if (!newZone.name) {
      setError('Please enter a name for the zone');
      return;
    }
    
    if (drawMode === 'polygon' && drawPoints.length < 3) {
      setError('Please draw a polygon with at least 3 points');
      return;
    }
    
    if (drawMode === 'circle' && drawPoints.length === 0) {
      setError('Please select a center point for the circle');
      return;
    }
    
    try {
      // Prepare geometry
      let geometry = {};
      
      if (drawMode === 'polygon') {
        // Close the polygon
        const closedPolygon = [...drawPoints, drawPoints[0]];
        
        geometry = {
          coordinates: [[closedPolygon.map(point => [point[1], point[0]])]]
        };
      } else if (drawMode === 'circle') {
        geometry = {
          center: [drawPoints[0][1], drawPoints[0][0]],
          radius: circleRadius
        };
      }
      
      // Create zone
      const response = await axios.post('/api/geofence/zones/custom', {
        name: newZone.name,
        description: newZone.description,
        restriction_level: newZone.restriction_level,
        geometry_type: drawMode,
        geometry: geometry,
        altitude_min: newZone.altitude_min,
        altitude_max: newZone.altitude_max
      });
      
      // Reset form
      setNewZone({
        name: '',
        description: '',
        restriction_level: 'no_fly',
        geometry_type: 'polygon',
        geometry: {},
        altitude_min: null,
        altitude_max: null
      });
      setDrawMode(null);
      setDrawPoints([]);
      setShowNewZoneForm(false);
      
      // Refresh zones
      fetchGeofenceZones();
      
      // Show success message
      alert('Geofence zone created successfully!');
    } catch (err) {
      console.error('Error creating geofence zone:', err);
      setError('Failed to create geofence zone');
    }
  };
  
  const handleDeleteZone = async (zoneId) => {
    if (!window.confirm('Are you sure you want to delete this zone?')) {
      return;
    }
    
    try {
      await axios.delete(`/api/geofence/zones/custom/${zoneId}`);
      
      // Refresh zones
      fetchGeofenceZones();
      
      // Clear selected zone if it was deleted
      if (selectedZone && selectedZone.id === zoneId) {
        setSelectedZone(null);
      }
      
      // Show success message
      alert('Geofence zone deleted successfully!');
    } catch (err) {
      console.error('Error deleting geofence zone:', err);
      setError('Failed to delete geofence zone');
    }
  };
  
  const renderGeofenceZones = () => {
    return geofenceZones.map(zone => {
      if (zone.geometry_type === 'circle') {
        const center = [zone.geometry.center[1], zone.geometry.center[0]];
        const radius = zone.geometry.radius;
        
        // Determine color based on restriction level
        let color = '#3388ff';
        if (zone.restriction_level === 'no_fly') {
          color = '#ff3333';
        } else if (zone.restriction_level === 'restricted') {
          color = '#ff9933';
        } else if (zone.restriction_level === 'caution') {
          color = '#ffcc33';
        }
        
        return (
          <Circle
            key={zone.id}
            center={center}
            radius={radius}
            pathOptions={{
              color: color,
              fillColor: color,
              fillOpacity: 0.2
            }}
            eventHandlers={{
              click: () => setSelectedZone(zone)
            }}
          >
            <Popup>
              <div>
                <h3>{zone.name}</h3>
                {zone.description && <p>{zone.description}</p>}
                <p>Type: {zone.zone_type}</p>
                <p>Restriction: {zone.restriction_level}</p>
                {zone.altitude_min && <p>Min Altitude: {zone.altitude_min}m</p>}
                {zone.altitude_max && <p>Max Altitude: {zone.altitude_max}m</p>}
                {zone.source === 'custom' && (
                  <button
                    className="delete-button"
                    onClick={(e) => {
                      e.stopPropagation();
                      handleDeleteZone(zone.id);
                    }}
                  >
                    Delete
                  </button>
                )}
              </div>
            </Popup>
          </Circle>
        );
      } else if (zone.geometry_type === 'polygon') {
        const positions = zone.geometry.coordinates[0].map(coord => [coord[1], coord[0]]);
        
        // Determine color based on restriction level
        let color = '#3388ff';
        if (zone.restriction_level === 'no_fly') {
          color = '#ff3333';
        } else if (zone.restriction_level === 'restricted') {
          color = '#ff9933';
        } else if (zone.restriction_level === 'caution') {
          color = '#ffcc33';
        }
        
        return (
          <Polygon
            key={zone.id}
            positions={positions}
            pathOptions={{
              color: color,
              fillColor: color,
              fillOpacity: 0.2
            }}
            eventHandlers={{
              click: () => setSelectedZone(zone)
            }}
          >
            <Popup>
              <div>
                <h3>{zone.name}</h3>
                {zone.description && <p>{zone.description}</p>}
                <p>Type: {zone.zone_type}</p>
                <p>Restriction: {zone.restriction_level}</p>
                {zone.altitude_min && <p>Min Altitude: {zone.altitude_min}m</p>}
                {zone.altitude_max && <p>Max Altitude: {zone.altitude_max}m</p>}
                {zone.source === 'custom' && (
                  <button
                    className="delete-button"
                    onClick={(e) => {
                      e.stopPropagation();
                      handleDeleteZone(zone.id);
                    }}
                  >
                    Delete
                  </button>
                )}
              </div>
            </Popup>
          </Polygon>
        );
      }
      
      return null;
    });
  };
  
  const renderWaypoints = () => {
    return waypoints.map((waypoint, index) => (
      <Marker
        key={`waypoint-${index}`}
        position={[waypoint.latitude, waypoint.longitude]}
        icon={waypointIcon}
      >
        <Popup>
          <div>
            <strong>Waypoint {index + 1}</strong>
            <p>Latitude: {waypoint.latitude.toFixed(6)}</p>
            <p>Longitude: {waypoint.longitude.toFixed(6)}</p>
            <button
              onClick={() => {
                const newWaypoints = [...waypoints];
                newWaypoints.splice(index, 1);
                setWaypoints(newWaypoints);
              }}
            >
              Delete
            </button>
          </div>
        </Popup>
      </Marker>
    ));
  };
  
  const renderDrawing = () => {
    if (drawMode === 'polygon' && drawPoints.length > 0) {
      return (
        <Polygon
          positions={drawPoints}
          pathOptions={{
            color: '#3388ff',
            fillColor: '#3388ff',
            fillOpacity: 0.2,
            dashArray: '5, 5'
          }}
        />
      );
    } else if (drawMode === 'circle' && drawPoints.length > 0) {
      return (
        <Circle
          center={drawPoints[0]}
          radius={circleRadius}
          pathOptions={{
            color: '#3388ff',
            fillColor: '#3388ff',
            fillOpacity: 0.2,
            dashArray: '5, 5'
          }}
        />
      );
    }
    
    return null;
  };
  
  const renderViolations = () => {
    if (!validationResult || !validationResult.violations || validationResult.violations.length === 0) {
      return null;
    }
    
    return validationResult.violations.map((violation, index) => {
      if (violation.latitude && violation.longitude) {
        return (
          <Marker
            key={`violation-${index}`}
            position={[violation.latitude, violation.longitude]}
            icon={new L.Icon({
              iconUrl: '/icons/violation-marker.png',
              iconSize: [32, 32],
              iconAnchor: [16, 16],
              popupAnchor: [0, -16]
            })}
          >
            <Popup>
              <div>
                <strong>Violation at Waypoint {violation.waypoint_index + 1}</strong>
                <p>{violation.description}</p>
                <p>Zone: {violation.zone_name}</p>
                <p>Type: {violation.zone_type}</p>
                <p>Restriction: {violation.restriction_level}</p>
              </div>
            </Popup>
          </Marker>
        );
      }
      
      return null;
    });
  };
  
  if (isLoading) {
    return <div className="loading">Loading geofence data...</div>;
  }
  
  return (
    <div className="geofencing-panel">
      <div className="geofencing-sidebar">
        <h2>Geofencing</h2>
        
        {error && (
          <div className="error-message">
            {error}
            <button onClick={() => setError(null)}>Dismiss</button>
          </div>
        )}
        
        <div className="section">
          <div className="section-header">
            <h3>Geofence Zones</h3>
            <button 
              className="new-button"
              onClick={() => {
                setShowNewZoneForm(!showNewZoneForm);
                if (!showNewZoneForm) {
                  setDrawMode(null);
                  setDrawPoints([]);
                }
              }}
            >
              {showNewZoneForm ? 'Cancel' : 'New Zone'}
            </button>
          </div>
          
          {showNewZoneForm && (
            <div className="new-zone-form">
              <div className="form-group">
                <label>Name</label>
                <input
                  type="text"
                  value={newZone.name}
                  onChange={(e) => setNewZone({...newZone, name: e.target.value})}
                  required
                />
              </div>
              
              <div className="form-group">
                <label>Description</label>
                <textarea
                  value={newZone.description}
                  onChange={(e) => setNewZone({...newZone, description: e.target.value})}
                />
              </div>
              
              <div className="form-group">
                <label>Restriction Level</label>
                <select
                  value={newZone.restriction_level}
                  onChange={(e) => setNewZone({...newZone, restriction_level: e.target.value})}
                >
                  <option value="no_fly">No Fly</option>
                  <option value="restricted">Restricted</option>
                  <option value="caution">Caution</option>
                  <option value="notice">Notice</option>
                </select>
              </div>
              
              <div className="form-group">
                <label>Geometry Type</label>
                <div className="button-group">
                  <button
                    className={`geometry-button ${drawMode === 'polygon' ? 'active' : ''}`}
                    onClick={() => {
                      setDrawMode('polygon');
                      setDrawPoints([]);
                    }}
                  >
                    Polygon
                  </button>
                  <button
                    className={`geometry-button ${drawMode === 'circle' ? 'active' : ''}`}
                    onClick={() => {
                      setDrawMode('circle');
                      setDrawPoints([]);
                    }}
                  >
                    Circle
                  </button>
                </div>
              </div>
              
              {drawMode === 'circle' && (
                <div className="form-group">
                  <label>Radius (meters)</label>
                  <input
                    type="number"
                    value={circleRadius}
                    onChange={(e) => setCircleRadius(Number(e.target.value))}
                    min="100"
                    max="10000"
                  />
                </div>
              )}
              
              <div className="form-group">
                <label>Min Altitude (meters, optional)</label>
                <input
                  type="number"
                  value={newZone.altitude_min || ''}
                  onChange={(e) => setNewZone({...newZone, altitude_min: e.target.value ? Number(e.target.value) : null})}
                  min="0"
                />
              </div>
              
              <div className="form-group">
                <label>Max Altitude (meters, optional)</label>
                <input
                  type="number"
                  value={newZone.altitude_max || ''}
                  onChange={(e) => setNewZone({...newZone, altitude_max: e.target.value ? Number(e.target.value) : null})}
                  min="0"
                />
              </div>
              
              <div className="drawing-instructions">
                {drawMode === 'polygon' ? (
                  <p>Click on the map to add points to the polygon. Add at least 3 points.</p>
                ) : drawMode === 'circle' ? (
                  <p>Click on the map to set the center of the circle.</p>
                ) : (
                  <p>Select a geometry type to start drawing.</p>
                )}
                
                {drawPoints.length > 0 && (
                  <button
                    className="clear-button"
                    onClick={() => setDrawPoints([])}
                  >
                    Clear Drawing
                  </button>
                )}
              </div>
              
              <button
                className="create-button"
                onClick={handleCreateZone}
                disabled={
                  !newZone.name || 
                  (drawMode === 'polygon' && drawPoints.length < 3) ||
                  (drawMode === 'circle' && drawPoints.length === 0)
                }
              >
                Create Zone
              </button>
            </div>
          )}
          
          <div className="zone-list">
            {geofenceZones.length === 0 ? (
              <p className="empty-message">No geofence zones available</p>
            ) : (
              geofenceZones.map(zone => (
                <div
                  key={zone.id}
                  className={`zone-item ${selectedZone && selectedZone.id === zone.id ? 'selected' : ''}`}
                  onClick={() => {
                    setSelectedZone(zone);
                    
                    // Center map on zone
                    if (zone.geometry_type === 'circle') {
                      const center = [zone.geometry.center[1], zone.geometry.center[0]];
                      mapRef.current.setView(center, 14);
                    } else if (zone.geometry_type === 'polygon') {
                      const bounds = zone.geometry.coordinates[0].map(coord => [coord[1], coord[0]]);
                      mapRef.current.fitBounds(bounds);
                    }
                  }}
                >
                  <div className="zone-item-header">
                    <span className="zone-name">{zone.name}</span>
                    <span className={`zone-level ${zone.restriction_level}`}>{zone.restriction_level}</span>
                  </div>
                  {zone.description && (
                    <div className="zone-description">{zone.description}</div>
                  )}
                  <div className="zone-details">
                    <span className="zone-type">{zone.zone_type}</span>
                    <span className="zone-source">{zone.source}</span>
                  </div>
                </div>
              ))
            )}
          </div>
        </div>
        
        <div className="section">
          <div className="section-header">
            <h3>Mission Validation</h3>
          </div>
          
          <div className="validation-controls">
            <p>Click on the map to add waypoints for your mission.</p>
            
            <div className="button-group">
              <button
                className="validate-button"
                onClick={handleValidateMission}
                disabled={waypoints.length === 0}
              >
                Validate Mission
              </button>
              
              <button
                className="clear-button"
                onClick={handleClearWaypoints}
                disabled={waypoints.length === 0}
              >
                Clear Waypoints
              </button>
            </div>
            
            {validationResult && (
              <div className={`validation-result ${validationResult.valid ? 'valid' : 'invalid'}`}>
                <h4>Validation Result</h4>
                
                {validationResult.valid ? (
                  <p className="valid-message">Mission is valid! No geofence violations detected.</p>
                ) : (
                  <>
                    <p className="invalid-message">Mission has {validationResult.violations.length} geofence violation(s):</p>
                    <ul className="violations-list">
                      {validationResult.violations.map((violation, index) => (
                        <li key={index}>
                          {violation.waypoint_index !== undefined ? (
                            <span>Waypoint {violation.waypoint_index + 1}: </span>
                          ) : (
                            <span>Path: </span>
                          )}
                          {violation.description}
                        </li>
                      ))}
                    </ul>
                  </>
                )}
              </div>
            )}
          </div>
        </div>
      </div>
      
      <div className="geofencing-map-container">
        <MapContainer
          center={mapCenter}
          zoom={mapZoom}
          style={{ height: '100%', width: '100%' }}
          whenCreated={mapInstance => {
            mapRef.current = mapInstance;
          }}
        >
          <TileLayer
            attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          />
          
          <MapClickHandler onMapClick={handleMapClick} />
          
          {renderGeofenceZones()}
          {renderWaypoints()}
          {renderDrawing()}
          {renderViolations()}
        </MapContainer>
      </div>
    </div>
  );
};

export default GeofencingPanel;
