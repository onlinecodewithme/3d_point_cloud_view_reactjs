import React, { useState, useEffect } from 'react';
import { BrowserRouter as Router, Routes, Route, Navigate } from 'react-router-dom';
import './App.css';
import Navigation from './components/Navigation';
import PointCloudVisualization from './components/PointCloudVisualization';
import EnhancedPointCloudVisualization from './components/EnhancedPointCloudVisualization';
import RobotControlDashboard from './components/RobotControlDashboard';
import BatteryDashboard from './components/BatteryDashboard';
import EnhancedSystemMonitoringDashboard from './components/EnhancedSystemMonitoringDashboard';
import IMUSensorDashboard from './components/IMUSensorDashboard';
import EnvironmentalSensorDashboard from './components/EnvironmentalSensorDashboard';
import RobotStatusDashboard from './components/RobotStatusDashboard';
import Login from './components/Login';
import Register from './components/Register';
import Profile from './components/Profile';
import SystemFooter from './components/SystemFooter';

interface User {
  id: number;
  username: string;
  email: string;
  role: string;
  avatar: string;
  permissions: string[];
}

function App() {
  const [currentUser, setCurrentUser] = useState<User | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [isDrawerOpen, setIsDrawerOpen] = useState(true);

  useEffect(() => {
    // Check for existing user session
    const savedUser = localStorage.getItem('currentUser');
    if (savedUser) {
      try {
        setCurrentUser(JSON.parse(savedUser));
      } catch (error) {
        console.error('Error parsing saved user:', error);
        localStorage.removeItem('currentUser');
      }
    }
    setIsLoading(false);
  }, []);

  const handleLogin = (user: User) => {
    setCurrentUser(user);
  };

  const handleLogout = () => {
    setCurrentUser(null);
    localStorage.removeItem('currentUser');
  };

  const handleDrawerStateChange = (isOpen: boolean) => {
    setIsDrawerOpen(isOpen);
  };

  // Dynamic styles based on drawer state (without top header)
  const getMainContentStyle = (additionalStyles: React.CSSProperties = {}) => ({
    padding: 0,
    margin: 0,
    marginLeft: isDrawerOpen ? '320px' : '0',
    width: isDrawerOpen ? 'calc(100% - 320px)' : '100%',
    height: '100vh',
    transition: 'margin-left 0.3s ease-out, width 0.3s ease-out',
    ...additionalStyles
  });

  if (isLoading) {
    return (
      <div className="app-loading">
        <div className="loading-spinner"></div>
        <p>Loading Xavier Control Dashboard...</p>
      </div>
    );
  }

  // If user is not logged in, show login/register pages
  if (!currentUser) {
    return (
      <Router>
        <div className="App">
          <Routes>
            <Route path="/login" element={<Login onLogin={handleLogin} />} />
            <Route path="/register" element={<Register />} />
            <Route path="*" element={<Navigate to="/login" replace />} />
          </Routes>
        </div>
      </Router>
    );
  }

  // User is logged in, show main application
  return (
    <Router>
      <div className="App">
        <Routes>
          <Route path="/login" element={<Navigate to="/" replace />} />
          <Route path="/register" element={<Navigate to="/" replace />} />
          <Route path="/" element={
            <>
              <Navigation currentUser={currentUser} onLogout={handleLogout} onDrawerStateChange={handleDrawerStateChange} />
              <main style={getMainContentStyle()}>
                <EnhancedPointCloudVisualization />
              </main>
              <SystemFooter isDrawerOpen={isDrawerOpen} />
            </>
          } />
          <Route path="/legacy" element={
            <>
              <Navigation currentUser={currentUser} onLogout={handleLogout} onDrawerStateChange={handleDrawerStateChange} />
              <header className="App-header" style={getMainContentStyle()}>
                <h1>3D Point Cloud Visualization Dashboard</h1>
                <p>Real-time RTAB-Map 3D Mapping Visualization</p>
              </header>
              <main style={getMainContentStyle()}>
                <PointCloudVisualization />
              </main>
              <SystemFooter isDrawerOpen={isDrawerOpen} />
            </>
          } />
          <Route path="/control" element={
            <>
              <Navigation currentUser={currentUser} onLogout={handleLogout} onDrawerStateChange={handleDrawerStateChange} />
              <main style={getMainContentStyle({ overflow: 'hidden' })}>
                <RobotControlDashboard />
              </main>
              <SystemFooter isDrawerOpen={isDrawerOpen} />
            </>
          } />
          <Route path="/battery" element={
            <>
              <Navigation currentUser={currentUser} onLogout={handleLogout} onDrawerStateChange={handleDrawerStateChange} />
              <main style={getMainContentStyle({ overflowY: 'auto' })}>
                <BatteryDashboard />
              </main>
              <SystemFooter isDrawerOpen={isDrawerOpen} />
            </>
          } />
          <Route path="/system-monitoring" element={
            <>
              <Navigation currentUser={currentUser} onLogout={handleLogout} onDrawerStateChange={handleDrawerStateChange} />
              <main style={getMainContentStyle({ 
                overflowY: 'auto',
                background: 'linear-gradient(135deg, #0a0a0a 0%, #1a1a1a 100%)'
              })}>
                <EnhancedSystemMonitoringDashboard />
              </main>
              <SystemFooter isDrawerOpen={isDrawerOpen} />
            </>
          } />
          <Route path="/enhanced-system-monitoring" element={
            <>
              <Navigation currentUser={currentUser} onLogout={handleLogout} onDrawerStateChange={handleDrawerStateChange} />
              <main style={getMainContentStyle({ 
                overflowY: 'auto',
                background: 'linear-gradient(135deg, #0a0a0a 0%, #1a1a1a 100%)'
              })}>
                <EnhancedSystemMonitoringDashboard />
              </main>
              <SystemFooter isDrawerOpen={isDrawerOpen} />
            </>
          } />
          <Route path="/imu-sensor" element={
            <>
              <Navigation currentUser={currentUser} onLogout={handleLogout} onDrawerStateChange={handleDrawerStateChange} />
              <main style={getMainContentStyle({ overflowY: 'auto' })}>
                <IMUSensorDashboard />
              </main>
              <SystemFooter isDrawerOpen={isDrawerOpen} />
            </>
          } />
          <Route path="/environmental-sensors" element={
            <>
              <Navigation currentUser={currentUser} onLogout={handleLogout} onDrawerStateChange={handleDrawerStateChange} />
              <main style={getMainContentStyle({ 
                overflowY: 'auto',
                background: 'linear-gradient(135deg, #0a0a0a 0%, #0f1419 30%, #1a1a2e 70%, #0a0a0a 100%)'
              })}>
                <EnvironmentalSensorDashboard />
              </main>
              <SystemFooter isDrawerOpen={isDrawerOpen} />
            </>
          } />
          <Route path="/robot-status" element={
            <>
              <Navigation currentUser={currentUser} onLogout={handleLogout} onDrawerStateChange={handleDrawerStateChange} />
              <main style={getMainContentStyle({ 
                overflowY: 'auto',
                background: 'linear-gradient(135deg, #0a0a0a 0%, #0f1419 30%, #1a1a2e 70%, #0a0a0a 100%)'
              })}>
                <RobotStatusDashboard />
              </main>
              <SystemFooter isDrawerOpen={isDrawerOpen} />
            </>
          } />
          <Route path="/profile" element={
            <>
              <Navigation currentUser={currentUser} onLogout={handleLogout} onDrawerStateChange={handleDrawerStateChange} />
              <main style={getMainContentStyle({ overflowY: 'auto' })}>
                <Profile />
              </main>
              <SystemFooter isDrawerOpen={isDrawerOpen} />
            </>
          } />
          <Route path="*" element={<Navigate to="/" replace />} />
        </Routes>
      </div>
    </Router>
  );
}

export default App;
