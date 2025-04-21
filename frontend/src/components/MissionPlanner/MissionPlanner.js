import React, { useState, useEffect, useRef } from 'react';
import { MapContainer, TileLayer, Marker, Popup, Polyline, useMapEvents } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import axios from 'axios';
import MissionTypeSelector from './MissionTypeSelector';
import WaypointMissionPanel from './WaypointMissionPanel';
import MappingMissionPanel from './MappingMissionPanel';
import OrbitMissionPanel from './OrbitMissionPanel';
import FacadeMissionPanel from './FacadeMissionPanel';
import MissionSettings from './MissionSettings';
import './MissionPlanner.css';

// Fix Leaflet marker icon issue
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon-2x.png',
  iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
  shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png',
});

// Custom marker icons
const waypointIcon = new L.Icon({
  iconUrl: '/icons/waypoint-marker.png',
  iconSize: [24, 24],
  iconAnchor: [12, 12],
  popupAnchor: [0, -12]
});

const homeIcon = new L.Icon({
  iconUrl: '/icons/home-marker.png',
  iconSize: [32, 32],
  iconAnchor: [16, 16],
  popupAnchor: [0, -16]
});

const MissionPlanner = () => {
  const [missionType, setMissionType] = useState('waypoint');
  const [waypoints, setWaypoints] = useState([]);
  const [boundaryPoints, setBoundaryPoints] = useState([]);
  const [homePosition, setHomePosition] = useState(null);
  const [missionSettings, setMissionSettings] = useState({
    name: 'New Mission',
    description: '',
    maxAltitude: 120,
    maxDistance: 1000,
    autoTakeoff: true,
    autoLand: true,
    failSafeReturnHome: true,
    terrainFollowing: 'none'
  });
  const [missionTypeSettings, setMissionTypeSettings] = useState({
    waypoint: {
      speed: 5,
      defaultAltitude: 50,
      cameraActions: []
    },
    mapping: {
      areaMode: 'polygon',
      captureMode: 'overlap',
      frontOverlap: 75,
      sideOverlap: 65,
      flightSpeed: 5,
      flightAltitude: 50,
      gimbalPitch: -90,
      cameraTrigger: 'auto',
      gridAngle: 0,
      useCrosshatch: false,
      crosshatchAngle: 90,
      margin: 0,
      useTerrainFollowing: false
    },
    orbit: {
      radius: 50,
      altitude: 50,
      headingMode: 'poi',
      gimbalMode: 'follow',
      gimbalPitch: -20,
      speed: 2,
      rotationDirection: 'clockwise',
      startAngle: 0,
      endAngle: 360,
      numRotations: 1,
      captureMode: 'none'
    },
    facade: {
      height: 30,
      layers: 3,
      layerHeight: 10,
      distanceFromFacade: 15,
      flightSpeed: 2,
      captureMode: 'overlap',
      overlap: 70,
      gimbalPitch: 0
    }
  });
  const [editMode, setEditMode] = useState('waypoints'); // waypoints, boundary, home
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [savedMissions, setSavedMissions] = useState([]);
  const [templates, setTemplates] = useState([]);
  const [previewWaypoints, setPreviewWaypoints] = useState([]);
  const mapRef = useRef(null);

  // Load saved missions and templates on component mount
  useEffect(() => {
    fetchSavedMissions();
    fetchTemplates();
  }, []);

  // Generate preview waypoints when mission type or settings change
  useEffect(() => {
    generatePreviewWaypoints();
  }, [missionType, boundaryPoints, missionTypeSettings]);

  const fetchSavedMissions = async () => {
    try {
      const response = await axios.get('/api/missions');
      setSavedMissions(response.data);
    } catch (err) {
      console.error('Error fetching saved missions:', err);
      setError('Failed to load saved missions');
    }
  };

  const fetchTemplates = async () => {
    try {
      const response = await axios.get('/api/missions/templates');
      setTemplates(response.data);
    } catch (err) {
      console.error('Error fetching templates:', err);
      setError('Failed to load mission templates');
    }
  };

  const generatePreviewWaypoints = async () => {
    if (missionType === 'waypoint' || boundaryPoints.length < 3) {
      setPreviewWaypoints([]);
      return;
    }

    try {
      setIsLoading(true);
      const response = await axios.post('/api/missions/preview', {
        missionType,
        boundaryPoints,
        settings: missionTypeSettings[missionType]
      });
      setPreviewWaypoints(response.data.waypoints);
    } catch (err) {
      console.error('Error generating preview waypoints:', err);
      setError('Failed to generate preview waypoints');
    } finally {
      setIsLoading(false);
    }
  };

  const handleMapClick = (e) => {
    const { lat, lng } = e.latlng;
    
    if (editMode === 'waypoints') {
      const newWaypoint = {
        position: {
          latitude: lat,
          longitude: lng,
          altitude: missionTypeSettings.waypoint.defaultAltitude
        },
        speed: missionTypeSettings.waypoint.speed,
        actions: [],
        order: waypoints.length
      };
      setWaypoints([...waypoints, newWaypoint]);
    } else if (editMode === 'boundary') {
      const newBoundaryPoint = {
        latitude: lat,
        longitude: lng
      };
      setBoundaryPoints([...boundaryPoints, newBoundaryPoint]);
    } else if (editMode === 'home') {
      const newHomePosition = {
        latitude: lat,
        longitude: lng,
        altitude: 0
      };
      setHomePosition(newHomePosition);
    }
  };

  const handleWaypointDrag = (index, e) => {
    const { lat, lng } = e.target.getLatLng();
    const updatedWaypoints = [...waypoints];
    updatedWaypoints[index].position.latitude = lat;
    updatedWaypoints[index].position.longitude = lng;
    setWaypoints(updatedWaypoints);
  };

  const handleBoundaryPointDrag = (index, e) => {
    const { lat, lng } = e.target.getLatLng();
    const updatedBoundaryPoints = [...boundaryPoints];
    updatedBoundaryPoints[index].latitude = lat;
    updatedBoundaryPoints[index].longitude = lng;
    setBoundaryPoints(updatedBoundaryPoints);
  };

  const handleHomePositionDrag = (e) => {
    const { lat, lng } = e.target.getLatLng();
    setHomePosition({
      ...homePosition,
      latitude: lat,
      longitude: lng
    });
  };

  const handleDeleteWaypoint = (index) => {
    const updatedWaypoints = waypoints.filter((_, i) => i !== index);
    // Update order of remaining waypoints
    updatedWaypoints.forEach((wp, i) => {
      wp.order = i;
    });
    setWaypoints(updatedWaypoints);
  };

  const handleDeleteBoundaryPoint = (index) => {
    const updatedBoundaryPoints = boundaryPoints.filter((_, i) => i !== index);
    setBoundaryPoints(updatedBoundaryPoints);
  };

  const handleMissionTypeChange = (type) => {
    setMissionType(type);
    setWaypoints([]);
    setBoundaryPoints([]);
    setPreviewWaypoints([]);
    
    // Set appropriate edit mode based on mission type
    if (type === 'waypoint') {
      setEditMode('waypoints');
    } else {
      setEditMode('boundary');
    }
  };

  const handleMissionSettingsChange = (settings) => {
    setMissionSettings({
      ...missionSettings,
      ...settings
    });
  };

  const handleMissionTypeSettingsChange = (type, settings) => {
    setMissionTypeSettings({
      ...missionTypeSettings,
      [type]: {
        ...missionTypeSettings[type],
        ...settings
      }
    });
  };

  const saveMission = async () => {
    try {
      setIsLoading(true);
      
      // Prepare mission data
      const missionData = {
        name: missionSettings.name,
        description: missionSettings.description,
        missionType,
        settings: {
          [missionType]: missionTypeSettings[missionType],
          waypoint: missionType === 'waypoint' ? waypoints : previewWaypoints
        },
        boundaryPoints,
        homePosition,
        maxAltitude: missionSettings.maxAltitude,
        maxDistance: missionSettings.maxDistance,
        autoTakeoff: missionSettings.autoTakeoff,
        autoLand: missionSettings.autoLand,
        failSafeReturnHome: missionSettings.failSafeReturnHome,
        terrainFollowing: missionSettings.terrainFollowing
      };
      
      // Send to API
      const response = await axios.post('/api/missions', missionData);
      
      // Update saved missions list
      fetchSavedMissions();
      
      // Show success message
      alert(`Mission "${missionSettings.name}" saved successfully!`);
    } catch (err) {
      console.error('Error saving mission:', err);
      setError('Failed to save mission');
    } finally {
      setIsLoading(false);
    }
  };

  const loadMission = async (missionId) => {
    try {
      setIsLoading(true);
      
      const response = await axios.get(`/api/missions/${missionId}`);
      const mission = response.data;
      
      // Update state with loaded mission
      setMissionType(mission.missionType);
      setMissionSettings({
        name: mission.name,
        description: mission.description,
        maxAltitude: mission.maxAltitude,
        maxDistance: mission.maxDistance,
        autoTakeoff: mission.autoTakeoff,
        autoLand: mission.autoLand,
        failSafeReturnHome: mission.failSafeReturnHome,
        terrainFollowing: mission.terrainFollowing
      });
      
      // Update mission type specific settings
      setMissionTypeSettings({
        ...missionTypeSettings,
        [mission.missionType]: mission.settings[mission.missionType]
      });
      
      // Update waypoints, boundary points, and home position
      setWaypoints(mission.settings.waypoint || []);
      setBoundaryPoints(mission.boundaryPoints || []);
      setHomePosition(mission.homePosition);
      
      // Set appropriate edit mode
      if (mission.missionType === 'waypoint') {
        setEditMode('waypoints');
      } else {
        setEditMode('boundary');
      }
      
      // Center map on mission area
      if (mission.boundaryPoints && mission.boundaryPoints.length > 0) {
        const bounds = mission.boundaryPoints.map(point => [point.latitude, point.longitude]);
        mapRef.current.fitBounds(bounds);
      } else if (mission.settings.waypoint && mission.settings.waypoint.length > 0) {
        const bounds = mission.settings.waypoint.map(wp => [wp.position.latitude, wp.position.longitude]);
        mapRef.current.fitBounds(bounds);
      }
    } catch (err) {
      console.error('Error loading mission:', err);
      setError('Failed to load mission');
    } finally {
      setIsLoading(false);
    }
  };

  const createFromTemplate = async (templateId) => {
    try {
      setIsLoading(true);
      
      const response = await axios.post(`/api/missions/from-template/${templateId}`, {
        name: `New ${missionType} Mission`,
        description: `Created from template`
      });
      
      // Load the newly created mission
      loadMission(response.data.id);
    } catch (err) {
      console.error('Error creating mission from template:', err);
      setError('Failed to create mission from template');
    } finally {
      setIsLoading(false);
    }
  };

  const clearMission = () => {
    if (window.confirm('Are you sure you want to clear the current mission?')) {
      setWaypoints([]);
      setBoundaryPoints([]);
      setHomePosition(null);
      setPreviewWaypoints([]);
      setMissionSettings({
        name: 'New Mission',
        description: '',
        maxAltitude: 120,
        maxDistance: 1000,
        autoTakeoff: true,
        autoLand: true,
        failSafeReturnHome: true,
        terrainFollowing: 'none'
      });
    }
  };

  const MapClickHandler = () => {
    useMapEvents({
      click: handleMapClick
    });
    return null;
  };

  return (
    <div className="mission-planner">
      <div className="mission-planner-sidebar">
        <MissionTypeSelector 
          selectedType={missionType} 
          onChange={handleMissionTypeChange} 
        />
        
        {missionType === 'waypoint' && (
          <WaypointMissionPanel 
            waypoints={waypoints}
            settings={missionTypeSettings.waypoint}
            onSettingsChange={(settings) => handleMissionTypeSettingsChange('waypoint', settings)}
            onWaypointChange={(index, waypoint) => {
              const updatedWaypoints = [...waypoints];
              updatedWaypoints[index] = waypoint;
              setWaypoints(updatedWaypoints);
            }}
            onDeleteWaypoint={handleDeleteWaypoint}
          />
        )}
        
        {missionType === 'mapping' && (
          <MappingMissionPanel 
            boundaryPoints={boundaryPoints}
            settings={missionTypeSettings.mapping}
            onSettingsChange={(settings) => handleMissionTypeSettingsChange('mapping', settings)}
          />
        )}
        
        {missionType === 'orbit' && (
          <OrbitMissionPanel 
            settings={missionTypeSettings.orbit}
            onSettingsChange={(settings) => handleMissionTypeSettingsChange('orbit', settings)}
          />
        )}
        
        {missionType === 'facade' && (
          <FacadeMissionPanel 
            boundaryPoints={boundaryPoints}
            settings={missionTypeSettings.facade}
            onSettingsChange={(settings) => handleMissionTypeSettingsChange('facade', settings)}
          />
        )}
        
        <MissionSettings 
          settings={missionSettings}
          onChange={handleMissionSettingsChange}
        />
        
        <div className="mission-actions">
          <button 
            className="btn btn-primary"
            onClick={saveMission}
            disabled={isLoading}
          >
            {isLoading ? 'Saving...' : 'Save Mission'}
          </button>
          
          <button 
            className="btn btn-secondary"
            onClick={clearMission}
          >
            Clear Mission
          </button>
          
          <div className="edit-mode-selector">
            <label>Edit Mode:</label>
            <div className="btn-group">
              <button 
                className={`btn ${editMode === 'waypoints' ? 'btn-active' : ''}`}
                onClick={() => setEditMode('waypoints')}
              >
                Waypoints
              </button>
              <button 
                className={`btn ${editMode === 'boundary' ? 'btn-active' : ''}`}
                onClick={() => setEditMode('boundary')}
              >
                Boundary
              </button>
              <button 
                className={`btn ${editMode === 'home' ? 'btn-active' : ''}`}
                onClick={() => setEditMode('home')}
              >
                Home
              </button>
            </div>
          </div>
        </div>
        
        {error && (
          <div className="error-message">
            {error}
          </div>
        )}
      </div>
      
      <div className="mission-planner-map">
        <MapContainer 
          center={[51.505, -0.09]} 
          zoom={13} 
          style={{ height: '100%', width: '100%' }}
          whenCreated={mapInstance => {
            mapRef.current = mapInstance;
          }}
        >
          <TileLayer
            attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          />
          
          <MapClickHandler />
          
          {/* Render waypoints */}
          {waypoints.map((waypoint, index) => (
            <Marker
              key={`waypoint-${index}`}
              position={[waypoint.position.latitude, waypoint.position.longitude]}
              icon={waypointIcon}
              draggable={true}
              eventHandlers={{
                dragend: (e) => handleWaypointDrag(index, e)
              }}
            >
              <Popup>
                <div>
                  <strong>Waypoint {index + 1}</strong>
                  <p>Altitude: {waypoint.position.altitude}m</p>
                  <p>Speed: {waypoint.speed}m/s</p>
                  <button onClick={() => handleDeleteWaypoint(index)}>Delete</button>
                </div>
              </Popup>
            </Marker>
          ))}
          
          {/* Render waypoint path */}
          {waypoints.length > 1 && (
            <Polyline
              positions={waypoints.map(wp => [wp.position.latitude, wp.position.longitude])}
              color="blue"
            />
          )}
          
          {/* Render boundary points */}
          {boundaryPoints.map((point, index) => (
            <Marker
              key={`boundary-${index}`}
              position={[point.latitude, point.longitude]}
              draggable={true}
              eventHandlers={{
                dragend: (e) => handleBoundaryPointDrag(index, e)
              }}
            >
              <Popup>
                <div>
                  <strong>Boundary Point {index + 1}</strong>
                  <button onClick={() => handleDeleteBoundaryPoint(index)}>Delete</button>
                </div>
              </Popup>
            </Marker>
          ))}
          
          {/* Render boundary polygon */}
          {boundaryPoints.length > 2 && (
            <Polyline
              positions={[
                ...boundaryPoints.map(point => [point.latitude, point.longitude]),
                [boundaryPoints[0].latitude, boundaryPoints[0].longitude] // Close the polygon
              ]}
              color="red"
            />
          )}
          
          {/* Render preview waypoints */}
          {previewWaypoints.length > 0 && (
            <Polyline
              positions={previewWaypoints.map(wp => [wp.position.latitude, wp.position.longitude])}
              color="green"
              dashArray="5, 5"
            />
          )}
          
          {/* Render home position */}
          {homePosition && (
            <Marker
              position={[homePosition.latitude, homePosition.longitude]}
              icon={homeIcon}
              draggable={true}
              eventHandlers={{
                dragend: handleHomePositionDrag
              }}
            >
              <Popup>
                <div>
                  <strong>Home Position</strong>
                </div>
              </Popup>
            </Marker>
          )}
        </MapContainer>
      </div>
      
      <div className="mission-planner-saved">
        <h3>Saved Missions</h3>
        <div className="saved-missions-list">
          {savedMissions.map(mission => (
            <div 
              key={mission.id} 
              className="saved-mission-item"
              onClick={() => loadMission(mission.id)}
            >
              <span>{mission.name}</span>
              <span className="mission-type-badge">{mission.missionType}</span>
            </div>
          ))}
        </div>
        
        <h3>Templates</h3>
        <div className="templates-list">
          {templates.map(template => (
            <div 
              key={template.id} 
              className="template-item"
              onClick={() => createFromTemplate(template.id)}
            >
              <span>{template.name}</span>
              <span className="mission-type-badge">{template.missionType}</span>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};

export default MissionPlanner;
