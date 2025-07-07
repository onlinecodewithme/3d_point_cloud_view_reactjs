import React from 'react';
import { Link, useLocation } from 'react-router-dom';
import './Navigation.css';

const Navigation: React.FC = () => {
  const location = useLocation();

  return (
    <nav className="navigation">
      <div className="nav-brand">
        <h2>🤖 ROS2 Dashboard</h2>
      </div>
      
      <div className="nav-links">
        <Link 
          to="/" 
          className={`nav-link ${location.pathname === '/' ? 'active' : ''}`}
        >
          <span className="nav-icon">🧭</span>
          <span className="nav-text">Enhanced Dashboard</span>
        </Link>
        
        
        <Link 
          to="/control" 
          className={`nav-link ${location.pathname === '/control' ? 'active' : ''}`}
        >
          <span className="nav-icon">🎮</span>
          <span className="nav-text">Robot Control</span>
        </Link>
        
        <Link 
          to="/battery" 
          className={`nav-link ${location.pathname === '/battery' ? 'active' : ''}`}
        >
          <span className="nav-icon">🔋</span>
          <span className="nav-text">Battery Monitor</span>
        </Link>
      </div>
      
      <div className="nav-status">
        <div className="status-indicator">
          <span className="status-dot">●</span>
          <span className="status-text">ROS Bridge</span>
        </div>
      </div>
    </nav>
  );
};

export default Navigation;
