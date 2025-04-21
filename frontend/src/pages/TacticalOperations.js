import React, { useState } from 'react';
import SensorFusionDashboard from '../components/SensorFusionDashboard';
import TacticalDashboard from '../components/TacticalDashboard';
import MissionControl from '../components/MissionControl';

export default function TacticalOperations() {
  const [activeTab, setActiveTab] = useState('sensorFusion');

  const renderTabContent = () => {
    switch (activeTab) {
      case 'sensorFusion':
        return <SensorFusionDashboard />;
      case 'tacticalDashboard':
        return <TacticalDashboard />;
      case 'missionControl':
        return (
          <div className="p-4">
            <h2 className="text-2xl font-bold mb-4">Mission Control</h2>
            <MissionControl websocketUrl="ws://localhost:8000/ws/missions/" />
          </div>
        );
      default:
        return <SensorFusionDashboard />;
    }
  };

  return (
    <div className="h-full flex flex-col">
      <div className="bg-gray-100 border-b">
        <div className="container mx-auto px-4">
          <div className="flex overflow-x-auto">
            <button
              className={`px-4 py-3 font-medium text-sm focus:outline-none ${
                activeTab === 'sensorFusion'
                  ? 'border-b-2 border-blue-500 text-blue-600'
                  : 'text-gray-600 hover:text-gray-800'
              }`}
              onClick={() => setActiveTab('sensorFusion')}
            >
              Sensor Fusion
            </button>
            <button
              className={`px-4 py-3 font-medium text-sm focus:outline-none ${
                activeTab === 'tacticalDashboard'
                  ? 'border-b-2 border-blue-500 text-blue-600'
                  : 'text-gray-600 hover:text-gray-800'
              }`}
              onClick={() => setActiveTab('tacticalDashboard')}
            >
              Tactical Dashboard
            </button>
            <button
              className={`px-4 py-3 font-medium text-sm focus:outline-none ${
                activeTab === 'missionControl'
                  ? 'border-b-2 border-blue-500 text-blue-600'
                  : 'text-gray-600 hover:text-gray-800'
              }`}
              onClick={() => setActiveTab('missionControl')}
            >
              Mission Control
            </button>
          </div>
        </div>
      </div>
      
      <div className="flex-1 overflow-auto">
        {renderTabContent()}
      </div>
    </div>
  );
}
