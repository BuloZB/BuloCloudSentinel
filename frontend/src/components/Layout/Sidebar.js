import React from 'react';
import { NavLink } from 'react-router-dom';
import { useAuth } from '../../context/AuthContext';
import './Sidebar.css';

const Sidebar = ({ collapsed }) => {
  const { user, logout } = useAuth();
  
  // Navigation items
  const navItems = [
    { path: '/', label: 'Dashboard', icon: '📊' },
    { path: '/drones', label: 'Drones', icon: '🚁' },
    { path: '/missions', label: 'Missions', icon: '📝' },
    { path: '/video', label: 'Video', icon: '📹' },
    { path: '/settings', label: 'Settings', icon: '⚙️' }
  ];
  
  return (
    <div className={`sidebar ${collapsed ? 'collapsed' : ''}`}>
      <div className="sidebar-header">
        <div className="logo">
          {collapsed ? 'SW' : 'SentinelWeb'}
        </div>
      </div>
      
      <div className="sidebar-content">
        <nav className="sidebar-nav">
          <ul>
            {navItems.map((item) => (
              <li key={item.path}>
                <NavLink 
                  to={item.path}
                  className={({ isActive }) => isActive ? 'active' : ''}
                >
                  <span className="nav-icon">{item.icon}</span>
                  {!collapsed && <span className="nav-label">{item.label}</span>}
                </NavLink>
              </li>
            ))}
          </ul>
        </nav>
      </div>
      
      <div className="sidebar-footer">
        {user && (
          <div className="user-info">
            {!collapsed && (
              <div className="user-details">
                <div className="user-name">{user.username}</div>
                <div className="user-role">{user.roles[0]?.name || 'User'}</div>
              </div>
            )}
            
            <button 
              className="logout-button" 
              onClick={logout}
              title="Logout"
            >
              {collapsed ? '🚪' : 'Logout'}
            </button>
          </div>
        )}
      </div>
    </div>
  );
};

export default Sidebar;
