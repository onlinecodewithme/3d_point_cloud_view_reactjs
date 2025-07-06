import React from 'react';
import './App.css';
import PointCloudVisualization from './components/PointCloudVisualization';

function App() {
  return (
    <div className="App">
      <header className="App-header">
        <h1>3D Point Cloud Visualization Dashboard</h1>
        <p>Real-time RTAB-Map 3D Mapping Visualization</p>
      </header>
      <main>
        <PointCloudVisualization />
      </main>
    </div>
  );
}

export default App;
