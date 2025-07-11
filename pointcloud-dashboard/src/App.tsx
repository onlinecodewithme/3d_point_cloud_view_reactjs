import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
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

function App() {
  return (
    <Router>
      <div className="App">
        <Routes>
          <Route path="/" element={
            <>
              <Navigation />
              <main style={{ padding: 0, margin: 0, height: '100vh' }}>
                <EnhancedPointCloudVisualization />
              </main>
            </>
          } />
          <Route path="/legacy" element={
            <>
              <Navigation />
              <header className="App-header">
                <h1>3D Point Cloud Visualization Dashboard</h1>
                <p>Real-time RTAB-Map 3D Mapping Visualization</p>
              </header>
              <main>
                <PointCloudVisualization />
              </main>
            </>
          } />
          <Route path="/control" element={
            <>
              <Navigation />
              <main className="control-main">
                <RobotControlDashboard />
              </main>
            </>
          } />
          <Route path="/battery" element={
            <>
              <Navigation />
              <main className="battery-main">
                <BatteryDashboard />
              </main>
            </>
          } />
          <Route path="/system-monitoring" element={
            <>
              <Navigation />
              <main className="enhanced-system-monitoring-main">
                <EnhancedSystemMonitoringDashboard />
              </main>
            </>
          } />
          <Route path="/enhanced-system-monitoring" element={
            <>
              <Navigation />
              <main className="enhanced-system-monitoring-main">
                <EnhancedSystemMonitoringDashboard />
              </main>
            </>
          } />
          <Route path="/imu-sensor" element={
            <>
              <Navigation />
              <main className="imu-sensor-main">
                <IMUSensorDashboard />
              </main>
            </>
          } />
          <Route path="/environmental-sensors" element={
            <>
              <Navigation />
              <main className="environmental-sensor-main">
                <EnvironmentalSensorDashboard />
              </main>
            </>
          } />
          <Route path="/robot-status" element={
            <>
              <Navigation />
              <main className="robot-status-main">
                <RobotStatusDashboard />
              </main>
            </>
          } />
        </Routes>
      </div>
    </Router>
  );
}

export default App;
