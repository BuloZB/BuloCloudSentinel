import React from 'react';
import { BrowserRouter as Router, Routes, Route, Link } from 'react-router-dom';
import './App.css';

// Import components and pages
import IncidentTimeline from './components/IncidentTimeline';
import DroneCommandHub from './components/DroneCommandHub';
import AIToolsPanel from './components/AIToolsPanel';
import TacticalOperations from './pages/TacticalOperations';
import MeshNetworking from './pages/MeshNetworking';
import Missions from './pages/Missions';
import Dashboard from './pages/Dashboard';

function App() {
  return (
    <Router>
      <div className="flex h-screen">
        {/* Sidebar Navigation */}
        <div className="w-64 bg-gray-800 text-white p-4">
          <h1 className="text-xl font-bold mb-6">Bulo.Cloud Sentinel</h1>
          <nav>
            <ul className="space-y-2">
              <li>
                <Link to="/" className="block py-2 px-4 rounded hover:bg-gray-700">
                  Dashboard
                </Link>
              </li>
              <li>
                <Link to="/tactical" className="block py-2 px-4 rounded hover:bg-gray-700">
                  Tactical Operations
                </Link>
              </li>
              <li>
                <Link to="/mesh" className="block py-2 px-4 rounded hover:bg-gray-700">
                  Mesh Networking
                </Link>
              </li>
              <li>
                <Link to="/missions" className="block py-2 px-4 rounded hover:bg-gray-700">
                  Missions
                </Link>
              </li>
              <li>
                <Link to="/ai-tools" className="block py-2 px-4 rounded hover:bg-gray-700">
                  AI Tools
                </Link>
              </li>
            </ul>
          </nav>
        </div>

        {/* Main Content */}
        <div className="flex-1 overflow-auto">
          <Routes>
            <Route path="/" element={<Dashboard />} />
            <Route path="/tactical" element={<TacticalOperations />} />
            <Route path="/mesh" element={<MeshNetworking />} />
            <Route path="/missions" element={<Missions />} />
            <Route path="/ai-tools" element={<AIToolsPanel />} />
            <Route path="/incident-timeline" element={<IncidentTimeline />} />
            <Route path="/drone-command" element={<DroneCommandHub />} />
          </Routes>
        </div>
      </div>
    </Router>
  );
}

export default App;
