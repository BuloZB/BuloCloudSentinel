import React, { useState, useEffect } from 'react';
import { BrowserRouter as Router, Routes, Route, Navigate } from 'react-router-dom';
import './App.css';

// Components
import Dashboard from './components/Dashboard/Dashboard';
import Login from './components/Auth/Login';
import Sidebar from './components/Layout/Sidebar';
import Header from './components/Layout/Header';
import DroneList from './components/Drones/DroneList';
import MissionPlanner from './components/Missions/MissionPlanner';
import VideoStreams from './components/Video/VideoStreams';
import Settings from './components/Settings/Settings';
import NotFound from './components/Common/NotFound';

// Context
import { AuthProvider, useAuth } from './context/AuthContext';

// Protected Route component
const ProtectedRoute = ({ children }) => {
  const { isAuthenticated, isLoading } = useAuth();
  
  if (isLoading) {
    return <div className="loading">Loading...</div>;
  }
  
  if (!isAuthenticated) {
    return <Navigate to="/login" />;
  }
  
  return children;
};

// Main App component
const AppContent = () => {
  const { isAuthenticated } = useAuth();
  const [sidebarCollapsed, setSidebarCollapsed] = useState(false);
  
  // Toggle sidebar
  const toggleSidebar = () => {
    setSidebarCollapsed(!sidebarCollapsed);
  };
  
  return (
    <Router>
      <div className={`app-container ${sidebarCollapsed ? 'sidebar-collapsed' : ''}`}>
        {isAuthenticated && (
          <>
            <Sidebar collapsed={sidebarCollapsed} />
            <div className="main-content">
              <Header toggleSidebar={toggleSidebar} />
              <div className="page-content">
                <Routes>
                  <Route path="/" element={
                    <ProtectedRoute>
                      <Dashboard />
                    </ProtectedRoute>
                  } />
                  <Route path="/drones" element={
                    <ProtectedRoute>
                      <DroneList />
                    </ProtectedRoute>
                  } />
                  <Route path="/missions" element={
                    <ProtectedRoute>
                      <MissionPlanner />
                    </ProtectedRoute>
                  } />
                  <Route path="/video" element={
                    <ProtectedRoute>
                      <VideoStreams />
                    </ProtectedRoute>
                  } />
                  <Route path="/settings" element={
                    <ProtectedRoute>
                      <Settings />
                    </ProtectedRoute>
                  } />
                  <Route path="*" element={<NotFound />} />
                </Routes>
              </div>
            </div>
          </>
        )}
        
        {!isAuthenticated && (
          <Routes>
            <Route path="/login" element={<Login />} />
            <Route path="*" element={<Navigate to="/login" />} />
          </Routes>
        )}
      </div>
    </Router>
  );
};

// Wrap with AuthProvider
const App = () => {
  return (
    <AuthProvider>
      <AppContent />
    </AuthProvider>
  );
};

export default App;
