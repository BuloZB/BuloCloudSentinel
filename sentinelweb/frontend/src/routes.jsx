import React from 'react';
import { Navigate } from 'react-router-dom';

// Import pages
import Dashboard from './pages/Dashboard';
import DroneControl from './pages/DroneControl';
import MissionPlanner from './pages/MissionPlanner';
import VideoStreams from './pages/VideoStreams';
import ModelHub from './pages/ModelHub';
import Settings from './pages/Settings';
import NotFound from './pages/NotFound';

// Define routes
const routes = [
  {
    path: '/',
    element: <Navigate to="/dashboard" replace />,
  },
  {
    path: '/dashboard',
    element: <Dashboard />,
  },
  {
    path: '/drones',
    element: <DroneControl />,
  },
  {
    path: '/missions',
    element: <MissionPlanner />,
  },
  {
    path: '/video',
    element: <VideoStreams />,
  },
  {
    path: '/model-hub',
    element: <ModelHub />,
  },
  {
    path: '/settings',
    element: <Settings />,
  },
  {
    path: '*',
    element: <NotFound />,
  },
];

export default routes;
