import React, { useState, useEffect } from 'react';
import { Link, useLocation } from 'react-router-dom';
import './Navigation.css';

interface User {
  id: number;
  username: string;
  email: string;
  role: string;
  avatar: string;
  permissions: string[];
}

interface NavigationProps {
  currentUser: User;
  onLogout: () => void;
  onDrawerStateChange?: (isOpen: boolean) => void;
}

const Navigation: React.FC<NavigationProps> = ({ currentUser, onLogout, onDrawerStateChange }) => {
  const location = useLocation();
  const [isDrawerOpen, setIsDrawerOpen] = useState(true);

  const getRoleColor = (role: string): string => {
    switch (role.toLowerCase()) {
      case 'administrator': return '#ff4444';
      case 'robot operator': return '#00aaff';
      case 'systems engineer': return '#ffaa00';
      default: return '#00ff41';
    }
  };

  // Keep drawer open when route changes (removed auto-close behavior)
  // useEffect(() => {
  //   setIsDrawerOpen(false);
  // }, [location.pathname]);

  // Notify parent when drawer state changes
  useEffect(() => {
    if (onDrawerStateChange) {
      onDrawerStateChange(isDrawerOpen);
    }
  }, [isDrawerOpen, onDrawerStateChange]);

  // Close drawer when clicking outside (disabled for now since we want persistent drawer)
  // useEffect(() => {
  //   const handleClickOutside = (event: MouseEvent) => {
  //     const drawer = document.querySelector('.side-drawer');
  //     const hamburger = document.querySelector('.hamburger-menu');
  //     
  //     if (isDrawerOpen && drawer && hamburger && 
  //         !drawer.contains(event.target as Node) && 
  //         !hamburger.contains(event.target as Node)) {
  //       setIsDrawerOpen(false);
  //     }
  //   };

  //   document.addEventListener('mousedown', handleClickOutside);
  //   return () => document.removeEventListener('mousedown', handleClickOutside);
  // }, [isDrawerOpen]);

  const menuItems = [
    {
      path: '/',
      icon: '🎯',
      label: 'Dashboard',
      description: 'Main Control Center'
    },
    {
      path: '/control',
      icon: '🕹️',
      label: 'Robot Control',
      description: 'Manual Operation'
    },
    {
      path: '/battery',
      icon: '⚡',
      label: 'Power Systems',
      description: 'Battery Monitoring'
    },
    {
      path: '/system-monitoring',
      icon: '🧠',
      label: 'System Brain',
      description: 'AI Monitoring'
    },
    {
      path: '/imu-sensor',
      icon: '🧭',
      label: 'Navigation',
      description: 'IMU Sensors'
    },
    {
      path: '/environmental-sensors',
      icon: '🌡️',
      label: 'Environment',
      description: 'Sensor Array'
    },
    {
      path: '/robot-status',
      icon: '🤖',
      label: 'Robot Status',
      description: 'Health Monitor'
    }
  ];

  return (
    <>
      {/* Top Header */}
      <header className="top-header">
        <button 
          className="hamburger-menu"
          onClick={() => setIsDrawerOpen(!isDrawerOpen)}
          aria-label="Toggle menu"
        >
          <div className={`hamburger-line ${isDrawerOpen ? 'open' : ''}`}></div>
          <div className={`hamburger-line ${isDrawerOpen ? 'open' : ''}`}></div>
          <div className={`hamburger-line ${isDrawerOpen ? 'open' : ''}`}></div>
        </button>
        
        <div className="header-brand">
          <h1>🤖 Xavier Control Dashboard</h1>
        </div>
        
        <div className="header-status">
          <div className="status-indicator">
            <span className="status-dot online">●</span>
            <span className="status-text">ROS Bridge</span>
          </div>
        </div>
      </header>

      {/* Overlay removed - no gray overlay needed */}

      {/* Side Drawer */}
      <nav className={`side-drawer ${isDrawerOpen ? 'open' : ''}`}>
        <div className="drawer-header">
          <div className="xavier-logo">
            <div className="logo-icon">🤖</div>
            <div className="logo-text">
              <div className="logo-title">XAVIER</div>
              <div className="logo-subtitle">Control System</div>
            </div>
          </div>
        </div>

        <div className="drawer-menu">
          {menuItems.map((item) => (
            <Link
              key={item.path}
              to={item.path}
              className={`menu-item ${
                location.pathname === item.path || 
                (item.path === '/system-monitoring' && location.pathname === '/enhanced-system-monitoring')
                  ? 'active' : ''
              }`}
            >
              <div className="menu-icon">{item.icon}</div>
              <div className="menu-content">
                <div className="menu-label">{item.label}</div>
                <div className="menu-description">{item.description}</div>
              </div>
              <div className="menu-arrow">→</div>
            </Link>
          ))}
        </div>

        <div className="drawer-footer">
          <div className="user-section">
            <div className="user-avatar-large">{currentUser.avatar}</div>
            <div className="user-details">
              <div className="user-name">{currentUser.username}</div>
              <div 
                className="user-role"
                style={{ color: getRoleColor(currentUser.role) }}
              >
                {currentUser.role}
              </div>
            </div>
          </div>
          
          <div className="footer-actions">
            <Link to="/profile" className="footer-action">
              <span className="action-icon">👤</span>
              <span className="action-text">Profile</span>
            </Link>
            
            <button 
              className="footer-action logout"
              onClick={onLogout}
            >
              <span className="action-icon">🚪</span>
              <span className="action-text">Logout</span>
            </button>
          </div>
        </div>
      </nav>
    </>
  );
};

export default Navigation;
