import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import './App.css';
import Navigation from './components/Navigation';
import PointCloudVisualization from './components/PointCloudVisualization';
import EnhancedPointCloudVisualization from './components/EnhancedPointCloudVisualization';
import RobotControlDashboard from './components/RobotControlDashboard';

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
        </Routes>
      </div>
    </Router>
  );
}

export default App;
