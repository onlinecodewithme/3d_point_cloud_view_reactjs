import React, { useRef, useEffect, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Stats } from '@react-three/drei';
import * as THREE from 'three';
import PointCloudVisualization from './components/PointCloudVisualization';
import RosConnection from './components/RosConnection';
import Dashboard from './components/Dashboard';
import './App.css';

interface PointCloudData {
  points: Float32Array;
  colors?: Float32Array;
  timestamp: number;
}

const App: React.FC = () => {
  const [pointCloudData, setPointCloudData] = useState<PointCloudData | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState('Disconnected');
  const [pointCount, setPointCount] = useState(0);

  const handlePointCloudUpdate = (data: PointCloudData) => {
    setPointCloudData(data);
    setPointCount(data.points.length / 3); // Each point has x, y, z coordinates
  };

  return (
    <div className="app">
      <div className="dashboard-panel">
        <Dashboard 
          isConnected={isConnected}
          connectionStatus={connectionStatus}
          pointCount={pointCount}
          timestamp={pointCloudData?.timestamp}
        />
      </div>
      
      <div className="visualization-container">
        <Canvas
          camera={{ position: [5, 5, 5], fov: 75 }}
          style={{ background: '#000' }}
        >
          <ambientLight intensity={0.5} />
          <pointLight position={[10, 10, 10]} />
          
          <PointCloudVisualization data={pointCloudData} />
          
          <OrbitControls 
            enablePan={true}
            enableZoom={true}
            enableRotate={true}
          />
          
          <Stats />
          
          {/* Grid helper for reference */}
          <gridHelper args={[20, 20]} />
          
          {/* Axes helper for orientation */}
          <axesHelper args={[5]} />
        </Canvas>
      </div>

      <RosConnection
        onPointCloudUpdate={handlePointCloudUpdate}
        onConnectionStatusChange={setIsConnected}
        onStatusMessageChange={setConnectionStatus}
      />
    </div>
  );
};

export default App;
