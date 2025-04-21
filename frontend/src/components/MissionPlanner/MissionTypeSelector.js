import React from 'react';
import './MissionTypeSelector.css';

const MissionTypeSelector = ({ selectedType, onChange }) => {
  const missionTypes = [
    {
      id: 'waypoint',
      name: 'Waypoint',
      description: 'Create a mission with custom waypoints and actions',
      icon: 'waypoint-icon.svg'
    },
    {
      id: 'mapping',
      name: 'Mapping',
      description: 'Automatically generate a grid pattern for mapping an area',
      icon: 'mapping-icon.svg'
    },
    {
      id: 'orbit',
      name: 'Orbit',
      description: 'Create an orbit around a point of interest',
      icon: 'orbit-icon.svg'
    },
    {
      id: 'facade',
      name: 'Facade',
      description: 'Create a mission to scan a vertical structure',
      icon: 'facade-icon.svg'
    },
    {
      id: 'pano',
      name: 'Panorama',
      description: 'Create a panoramic photo at a specific location',
      icon: 'pano-icon.svg'
    },
    {
      id: 'hover',
      name: 'Hover',
      description: 'Hover at a specific location for a set duration',
      icon: 'hover-icon.svg'
    }
  ];

  return (
    <div className="mission-type-selector">
      <h2>Mission Type</h2>
      <div className="mission-types">
        {missionTypes.map(type => (
          <div
            key={type.id}
            className={`mission-type-card ${selectedType === type.id ? 'selected' : ''}`}
            onClick={() => onChange(type.id)}
          >
            <div className="mission-type-icon">
              <img src={`/icons/${type.icon}`} alt={type.name} />
            </div>
            <div className="mission-type-info">
              <h3>{type.name}</h3>
              <p>{type.description}</p>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};

export default MissionTypeSelector;
