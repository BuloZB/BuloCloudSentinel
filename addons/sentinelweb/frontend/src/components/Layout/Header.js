import React, { useState, useEffect } from 'react';
import { useLocation } from 'react-router-dom';
import './Header.css';

const Header = ({ toggleSidebar }) => {
  const [pageTitle, setPageTitle] = useState('Dashboard');
  const [currentTime, setCurrentTime] = useState(new Date());
  const location = useLocation();
  
  // Update page title based on current route
  useEffect(() => {
    const path = location.pathname;
    
    if (path === '/') {
      setPageTitle('Dashboard');
    } else if (path.startsWith('/drones')) {
      setPageTitle('Drone Management');
    } else if (path.startsWith('/missions')) {
      setPageTitle('Mission Planning');
    } else if (path.startsWith('/video')) {
      setPageTitle('Video Streams');
    } else if (path.startsWith('/settings')) {
      setPageTitle('Settings');
    } else {
      setPageTitle('SentinelWeb');
    }
  }, [location]);
  
  // Update current time every second
  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentTime(new Date());
    }, 1000);
    
    return () => clearInterval(timer);
  }, []);
  
  // Format time as HH:MM:SS
  const formatTime = (date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' });
  };
  
  // Format date as YYYY-MM-DD
  const formatDate = (date) => {
    return date.toLocaleDateString([], { year: 'numeric', month: 'short', day: 'numeric' });
  };
  
  return (
    <header className="app-header">
      <div className="header-left">
        <button className="sidebar-toggle" onClick={toggleSidebar}>
          ☰
        </button>
        <h1 className="page-title">{pageTitle}</h1>
      </div>
      
      <div className="header-right">
        <div className="datetime">
          <div className="time">{formatTime(currentTime)}</div>
          <div className="date">{formatDate(currentTime)}</div>
        </div>
        
        <div className="header-actions">
          <button className="header-action-btn" title="Notifications">
            🔔
          </button>
          <button className="header-action-btn" title="Help">
            ❓
          </button>
        </div>
      </div>
    </header>
  );
};

export default Header;
